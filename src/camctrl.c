#include <pruss_intc_mapping.h>
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
// Driver header file
#include "prussdrv.h"

#include "camctrl.h"
#include "i2c.h"


#define PRU_NUM0 	 0
#define PRU_NUM1	 1
// chunk size in bytes for transfers between pru1 and pru0
#define CHUNKSIZE 64 
// offset in  pru mem for data transfer ack from arm
#define ARM_PRU_ACK_OFFSET 4
#define ACK_PRUMEM_WORD_OFFSET (ARM_PRU_ACK_OFFSET /4)
#define ARM_PRU_ACK 1
#define ARM_PRU_NACK 0

// offset in pru mem to pass number of frames parameter 
#define NUMFRAMES_PRU_WORD_OFFSET 2

#define DDR_BASEADDR     0x80000000

// uint32_t offset in ddr for ack signal from pru
#define DDR_ACK_OFFSET 0 

// uint32_t offset in ddr for ack signal from pru
#define DDR_NUMFRAMES_OFFSET 1 

//to compensate for mmap bug
#define DDR_OFFSET_0	    0x10000000 

// allow some space for non-pixel data at beginning of shared ddr
#define DDR_DATA_OFFSET 8

//equivalent with 0x00002000
#define OFFSET_SHAREDRAM 0		

#define PRUSS0_SHARED_DATARAM    4
#define PRUSS1_SHARED_DATARAM    4

// offset for pixel data in ddr
#define OFFSET_DDR (DDR_OFFSET_0 + DDR_DATA_OFFSET) 

#define FILESIZE_BYTES (MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH  * FRAMES_PER_TRANSFER)


// PRU-shared values to keep track of read status in exposure()
#define READ_VALID 1
#define READ_INVALID 2

static void *ddrMem;
static uint32_t *DDR_physical; // physical device ddr address
static int mem_fd;

// Local function declarations
static void sendDDRbase();
static void ackPru();
static int pru_allocate_ddr_memory();
//static void nackPru();


/* 
Arguments:
    frameptr: Array to copy image data into.
    size: Size, in bytes, of data to be copied.
Reads out FRAMES_PER_TRANSFER number of frames and copies them into frameptr.
*/
void exposure(uint8_t *frameptr, int framesize) {
    uint8_t *pruDdrPtr = ddrMem + OFFSET_DDR;
    // wait for interrupt from pru0 indicating we can start reading
    int datstatus = READ_INVALID;
    while (datstatus != READ_VALID) {
        datstatus = ((uint32_t *) (ddrMem + DDR_OFFSET_0))[DDR_ACK_OFFSET];
        delay_ms(10);
    }
    // set status invalid for the next call of this function
    ((uint32_t *) (ddrMem + DDR_OFFSET_0))[DDR_ACK_OFFSET] = READ_INVALID; 
    memcpy(frameptr, pruDdrPtr, FRAMES_PER_TRANSFER * framesize);
    // acknowledge completion of a read
    ackPru();
}

// initialize communication with the PRUs 
int config_pru(int initial_config, uint32_t numFrames) {
    unsigned int ret;
    if (initial_config) {
        if (system("sudo scripts/pinmux_config") == -1) {
            printf("Pinmux configuration failed\n");
        } else {
            printf("Configured pinmux\n");
        }
    }
    // init i2c comm
    printf("\tINFO: performed sensor initial configuration\r\n");
    printf("configuring pru\n");
    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
    /* Initialize the PRU */
    prussdrv_init ();

    /* Open PRU Interrupt */
    ret = prussdrv_open(PRU_EVTOUT_1);
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret)
    {
        printf("prussdrv_open open failed\n");
        return (ret);
    }

    /* Get the interrupt initialized */
    prussdrv_pruintc_init(&pruss_intc_initdata);

   // initialize DDR_physical and ddrMem
    DDR_physical = malloc(sizeof(uint32_t));
    ddrMem = malloc(sizeof(uint32_t));
    if (DDR_physical == NULL || ddrMem == NULL) {
        fprintf(stderr, "failed to allocate memory.\n");
        return -1;
    }
    
    // initialize ddrMem and DDR_physical
    pru_allocate_ddr_memory();
    // give pru data to initialize its state, which consists of writing the
    // DDR_physical address into pru mem (NOT shared ddr).
    sendDDRbase();
    
    prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, NUMFRAMES_PRU_WORD_OFFSET, (const unsigned int *) &numFrames, 4);
    prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, NUMFRAMES_PRU_WORD_OFFSET, (const unsigned int *) &numFrames, 4);

    if (initial_config) {
        prussdrv_exec_program (PRU_NUM0, "./oe_pru0.bin"); // set OE low
        prussdrv_exec_program (PRU_NUM1, "./pru1clk.bin"); // start running clock
        delay_ms(999);
        printf("running the clock\n");
    }
    return 0;
}


void wait_pru1_complete(void) {
    prussdrv_pru_wait_event (PRU_EVTOUT_1);
    printf("\tINFO: PRU 1 completed transfer.\r\n");
    prussdrv_pru_clear_event (PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
}


// write the DDR physical base address to PRU0's pru mem
static void sendDDRbase() {
    prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0x0, (const unsigned int *) DDR_physical, 4);
}

// acknowledge start of a read by writing a value to ddr of pru1
static void ackPru() {
    int ack = ARM_PRU_ACK;
    prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, ACK_PRUMEM_WORD_OFFSET, (const unsigned int *) &ack, 4);
}

/*
// signal end of a read by writing a value to pru mem
static void nackPru() {
    int nack = ARM_PRU_NACK;
    prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, ACK_PRUMEM_WORD_OFFSET, (const unsigned int *) &nack, 4);
}
*/

// Obtain physical base address of pru device memory, saving that in the provided pointer, 
// and open a memory map, saving the corresponding virtual address in ddrMem
//
// ddr_phys_addr: physical address of memory.
// return: Size of region in bytes.
static int pru_allocate_ddr_memory()
{
   uint32_t extra_offset = 0x10000000;

   FILE *fin = fopen("/sys/class/uio/uio0/maps/map1/addr", "r");
   if (fin == NULL) {
      perror("Unable to open DDR map address");
     return -1;
   }

   if (DDR_physical == NULL) {
        printf("attempt to dereference null pointer"); 
        return -1;
    }
   // load physical address
   fscanf(fin, "%x", DDR_physical);
   fclose(fin);


    /* open the device */
    mem_fd = open("/dev/mem", O_RDWR);
    if (mem_fd < 0) {
        printf("Failed to open /dev/mem (%s)\n", strerror(errno));
        return -1;
    }
    // map memory
    ddrMem = mmap(0, 0x0FFFFFFF, PROT_WRITE | PROT_READ, MAP_SHARED, mem_fd, (uint32_t) (*DDR_physical - extra_offset));
    if (ddrMem == NULL) {
        printf("Failed to map the device (%s)\n", strerror(errno));
        close(mem_fd);
        return -1;
    }
   return 1;
}


void pru_exit(void) {
//    /* Disable PRU and close memory mapping*/
//    prussdrv_pru_disable(PRU_NUM0);
//    prussdrv_pru_disable(PRU_NUM1);

    prussdrv_exit ();
}

void pru_start(void) {
    // Load and run pru programs
    printf("\tINFO: Executing PRU program.\r\n");
    prussdrv_exec_program (PRU_NUM0, "./pru0.bin");
    prussdrv_exec_program (PRU_NUM1, "./pru1.bin");
    // TODO: is this delay necessary?
    delay_ms(100);
}

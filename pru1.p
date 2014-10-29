// *
// * PRU_memAcc_DDR_sharedRAM.p
// *
// * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
// *
// *
// *  Redistribution and use in source and binary forms, with or without
// *  modification, are permitted provided that the following conditions
// *  are met:
// *
// *    Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// *
// *    Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in the
// *    documentation and/or other materials provided with the
// *    distribution.
// *
// *    Neither the name of Texas Instruments Incorporated nor the names of
// *    its contributors may be used to endorse or promote products derived
// *    from this software without specific prior written permission.
// *
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// *

// *
// * ============================================================================
// * Copyright (c) Texas Instruments Inc 2010-12
// *
// * Use of this software is controlled by the terms and conditions found in the
// * license agreement under which this software has been supplied or provided.
// * ============================================================================
// *


// *****************************************************************************/
// file:   PRU_memAcc_DDR_sharedRAM.p
//
// brief:  PRU Example to access DDR and PRU shared Memory.
//
//
//  (C) Copyright 2012, Texas Instruments, Inc
//
//  author     M. Watkins
//
//  version    0.1     Created
// *****************************************************************************/


.origin 0
.entrypoint MEMACCESS_DDR_PRUSHAREDRAM

#include "PRU_memAcc_DDR_sharedRAM.hp"

MEMACCESS_DDR_PRUSHAREDRAM:

    // Enable OCP master port
    LBCO      r0, CONST_PRUCFG, 4, 4
    CLR     r0, r0, 4         // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
    SBCO      r0, CONST_PRUCFG, 4, 4

    // Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
    // field to 0x0100.  This will make C28 point to 0x00010000 (PRU shared RAM).
    MOV     r0, 0x00000100
    MOV       r1, CTPPR_0
    ST32      r0, r1

    // Configure the programmable pointer register for PRU0 by setting c31_pointer[15:0]
    // field to 0x0010.  This will make C31 point to 0x80001000 (DDR memory).
    MOV     r0, 0x00100000
    MOV       r1, CTPPR_1
    ST32      r0, r1

START:



//    //load GPI data
//    .macro GPIREAD
//    GPIREAD:
//            MOV     gpi_read, 0xdeadbeef
//    .endm


//.macro  onepix
//.mparam dst
//    CLR  SYSCLK // falling clock edge
//    NOP  // wait 10 ns for signal to propagate
//    NOP
//    MOV dst, PIX10_2 // move pix[8:0] into destination reg
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//.endm


.macro  onepix
.mparam dst
    CLR  SYSCLK // falling clock edge
// wait for pixclk to go low
//WAITPIXCLK:
//    QBBS    WAITPIXCLK, r30, PIX_N
    NOP
    MOV dst, PIX10_2 // move pix[8:0] into destination reg
    SET SYSCLK // rising edge
    NOP
    NOP
.endm

//.macro  onepix
//.mparam dst
//    CLR  SYSCLK // falling clock edge
//    ADD pixel_counter, pixel_counter, 1
//    MOV dst.w0, pixel_counter.w0
//    SET SYSCLK // rising edge
//    MOV dst.w2, pixel_counter.w2
//    NOP
//.endm
        
    

    
    
INIT:
        //set PRUCFG.SPP.XFR_SHIFT_EN=1
        LBCO r0, CONST_PRUCFG, 0x34, 4                    
        SET r0, 1
        SBCO r0, CONST_PRUCFG, 0x34, 4



        //MOV reads_left, NUMCHUNKS
        MOV r0, 0
        MOV     transfer_ready, 1
        // TODO: inconsistency with number_frames between pru0 and pru1
        MOV number_frames, NUMFRAMES
        MOV frame_counter, 0
        //MOV pixel_counter, 0

//        // initialize read buffer
//        MOV     r9, 0
//        MOV     r10, 1
//        MOV     r11, 2
//        MOV     r12, 3
//        MOV     r13, 4
//        MOV     r14, 5
//        MOV     r15, 6
//        MOV     r16, 7
//        MOV     r17, 8
//        MOV     r18, 9
//        MOV     r19, 10
//        MOV     r20, 11
//        MOV     r21, 12
//        MOV     r22, 13
//        MOV     r23, 14
//        MOV     r24, 15

FLUSH:
    // wait until FV == 0 in case we need to flush out an old frame
    CLR SYSCLK // clock edge
    MOV var1, r31.w0 // read inputs
    NOP
    SET SYSCLK // clock edge
    NOP
    QBBS    FLUSH, var1, FV_N // loop if FV == 0


READFRAME:
    // spin while providing clock if FV == 0
    NOP
READFRAME1:
    CLR SYSCLK // clock edge
    MOV var1, r31.w0 // read inputs
    NOP
    SET SYSCLK // clock edge
    QBBC    READFRAME, var1, FV_N // loop if FV == 0
 //   //LBCO    var1, CONST_PRUSHAREDRAM, ARM_PRU_ACK_OFFSET, 4

//    MOV r1, ARM_PRU_ACK_OFFSET
//    LBBO    var1, r1, 0, 4
//    QBNE    READFRAME1, var1, 1 // loop if ARM hasn't acknowledged start of data read
    
//    // clear ack from ARM
//    MOV var1, 0
//    //SBCO    var1, CONST_PRUSHAREDRAM, ARM_PRU_ACK_OFFSET, 4
//    SBBO    var1, r1, 0, 4

READLINE:

//    MOV var1, reads_left.w0
//    AND var1, var1, 0x1
//    QBEQ    RISINGEDGE, var1.w0, 1
//    QBA     FALLINGEDGE
//
//    RISINGEDGE:
//        SET r30.t11
//        QBA ACKCHECK
//
//    FALLINGEDGE:
//        CLR r30.t11

    // spin while providing clock if LV == 0
    CLR SYSCLK // clock edge
    MOV var1, r31.w0 // read inputs
    NOP
    SET SYSCLK // clock edge
    NOP
    QBBC    READLINE, var1, LV_N


FILLBUFFER: 
    //WBC PIXCLK // latch data on falling edge of PIXCLK
    // read in 32 pixels
    // TODO: could pack data twice as densely if i stick with 8 bits

    onepix  r9.b0
    onepix  r9.b1
    onepix  r9.b2
    onepix  r9.b3
    onepix  r10.b0
    onepix  r10.b1
    onepix  r10.b2
    onepix  r10.b3
    onepix  r11.b0
    onepix  r11.b1
    onepix  r11.b2
    onepix  r11.b3
    onepix  r12.b0
    onepix  r12.b1
    onepix  r12.b2
    onepix  r12.b3
    onepix  r13.b0
    onepix  r13.b1
    onepix  r13.b2
    onepix  r13.b3
    onepix  r14.b0
    onepix  r14.b1
    onepix  r14.b2
    onepix  r14.b3
    onepix  r15.b0
    onepix  r15.b1
    onepix  r15.b2
    onepix  r15.b3
    onepix  r16.b0
    onepix  r16.b1
    onepix  r16.b2
    onepix  r16.b3
    onepix  r17.b0
    onepix  r17.b1
    onepix  r17.b2
    onepix  r17.b3
    onepix  r18.b0
    onepix  r18.b1
    onepix  r18.b2
    onepix  r18.b3
    onepix  r19.b0
    onepix  r19.b1
    onepix  r19.b2
    onepix  r19.b3
    onepix  r20.b0
    onepix  r20.b1
    onepix  r20.b2
    onepix  r20.b3
    onepix  r21.b0
    onepix  r21.b1
    onepix  r21.b2
    onepix  r21.b3
    onepix  r22.b0
    onepix  r22.b1
    onepix  r22.b2
    onepix  r22.b3
    onepix  r23.b0
    onepix  r23.b1
    onepix  r23.b2
    onepix  r23.b3
    onepix  r24.b0
    onepix  r24.b1
    onepix  r24.b2
    onepix  r24.b3

//    onepix  r9.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r9.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r9.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r9.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r10.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r10.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r10.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r10.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r11.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r11.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r11.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r11.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r12.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r12.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r12.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r12.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r13.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r13.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r13.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r13.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r14.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r14.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r14.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r14.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r15.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r15.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r15.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r15.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r16.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r16.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r16.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r16.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r17.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r17.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r17.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r17.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r18.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r18.b1

QBA SKIPSTONE
READFRAME_STEPPINGSTONE:
    QBA READLINE
READLINE_STEPPINGSTONE:
    QBA READLINE
SKIPSTONE:

//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r18.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r18.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r19.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r19.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r19.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r19.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r20.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r20.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r20.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r20.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r21.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r21.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r21.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r21.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r22.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r22.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r22.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r22.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r23.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r23.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r23.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r23.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r24.b0
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r24.b1
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r24.b2
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    onepix  r24.b3
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP
//    NOP

//    onepix  r9
//    onepix  r10
//    onepix  r11
//    onepix  r12
//    onepix  r13
//    onepix  r14
//    onepix  r15
//    onepix  r16
//    onepix  r17
//    onepix  r18
//    onepix  r19
//    onepix  r20
//    onepix  r21
//    onepix  r22
//    onepix  r23
//    onepix  r24



//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r9, r9, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r10, r10, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r11, r11, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r12, r12, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r13, r13, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r14, r14, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r15, r15, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r16, r16, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r17, r17, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r18, r18, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r19, r19, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r20, r20, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r21, r21, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r22, r22, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r23, r23, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP
//        ADD     r24, r24, 16
//    CLR  SYSCLK // falling clock edge
//    NOP
//    NOP
//    SET SYSCLK // rising edge
//    NOP
//    NOP

//    // no NOP at the end for the last pixel value
//    CLR  SYSCLK // falling clock edge
//    NOP  // wait 10 ns for signal to propagate
//    MOV r24.b3, PIX10_2 // move pix[8:0] into destination reg
//    SET SYSCLK // rising edge

ACKCHECK:
//        CLR  SYSCLK // falling clock edge
//        NOP
        // check for ACK for previous read from pru0
        LBCO    pr0ack, CONST_PRUSHAREDRAM, 0, 4
//        SET SYSCLK // rising edge
//        NOP
        QBNE    ACKCHECK, pr0ack, 1


        // reset ack from pru0 
        // TODO combine this with the next SBCO
        MOV     pr0ack, 0
        SBCO    pr0ack, CONST_PRUSHAREDRAM, 0, 4


    // recall transfer_ready is r8 and always == 1
    // simultaneously copy data and set transfer_ready field in pru memory to 1
    SBCO    transfer_ready, CONST_PRUSHAREDRAM, 4, CHUNKSIZE + 8

    // decrement remaining number of chunks to read
    //SUB reads_left, reads_left, 1 

WAIT_LV:
    // continue in the same line if LV == 1
    // TODO: intermmediate variable necessary?
    CLR  SYSCLK // falling clock edge
    NOP
    NOP
    SET SYSCLK // rising edge
    MOV var1, r31.w0 // read inputs
    QBBS    READLINE_STEPPINGSTONE, var1, LV_N
        
    // otherwise return to polling FV, unless this was the last line
    //QBNE READFRAME, reads_left, 0
    CLR  SYSCLK // falling clock edge
    NOP
    NOP
    SET SYSCLK // rising edge
    MOV var1, r31.w0 // read inputs
    QBBS    WAIT_LV, var1, FV_N


REPEAT: // however many more frames are specified
    SUB   number_frames, number_frames, 1     // decrement loop counter

//        // re initialize read buffer
//        MOV     r9, 0
//        MOV     r10, 1
//        MOV     r11, 2
//        MOV     r12, 3
//        MOV     r13, 4
//        MOV     r14, 5
//        MOV     r15, 6
//        MOV     r16, 7
//        MOV     r17, 8
//        MOV     r18, 9
//        MOV     r19, 10
//        MOV     r20, 11
//        MOV     r21, 12
//        MOV     r22, 13
//        MOV     r23, 14
//        MOV     r24, 15


    // done with one frame
    // Notify pru0 of completion
    LDI    R31, PRU1_PRU0_INTERRUPT + 16 

    QBNE    READFRAME_STEPPINGSTONE, number_frames, 0  // repeat loop unless zero

    // Done with all frames!
    // Send notification to Host for program completion
    MOV       r31.b0, PRU1_ARM_INTERRUPT+16

// if you don't do this i2c writes fail the next time the program is run
CLKEND: // provide a clock indefinately
    // TESTING
//    MOV r1, ARM_PRU_ACK_OFFSET
//    LBBO    var1, r1, 0, 4
//
//    MOV var1, 10
//    SBBO    var1, r1, 0, 4

    
    CLR  SYSCLK // falling clock edge
    NOP
    NOP
    SET SYSCLK // rising edge
    NOP
    QBA CLKEND

    // Halt the processor
    HALT

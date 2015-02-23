#define OE r30.t15 // p8_11, must be controlled from pru0
#define PRU0_ARM_INTERRUPT 19

.origin 0 // offset of the start of the code in PRU memory

    CLR OE // pull buffer OE low to enable output
	MOV	R31.b0, PRU0_ARM_INTERRUPT+16
    HALT



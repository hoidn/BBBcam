// *
// * pru0.p
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
// file:   pru0.p
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

#include "pru.hp"

// Address for the Constant table Block Index Register (CTBIR) for PRU0
#define CTBIR_0          0x22020

// Address for the Constant table Programmable Pointer Register 0(CTPPR_0) for PRU0
#define CTPPR_0_0         0x22028

// Address for the Constant table Programmable Pointer Register 1(CTPPR_1) for PRU0
#define CTPPR_1_0         0x2202C

//macros
.macro diag_correction
        // figure out where we are w/respect to the checkerboard pattern, i.e.
        // which row of the readout?
        QBGT    CHECK_ROW, diag_counter, 40
        // reset diag_counter
        MOV     diag_counter, 0
   CHECK_ROW:
        QBLT    ODD_ROW, diag_counter, 19
    EVEN_ROW:
        MOV diag_state, 0
        QBA INCREMENT_DIAG_COUNTER
    ODD_ROW:
        MOV diag_state, 1
    INCREMENT_DIAG_COUNTER:
        ADD diag_counter, diag_counter, 1

// correct the data for checkerboard bias
 QBEQ    DIAG_STATE_1, diag_state, 1
DIAG_STATE_0:
    // in an even row
    SUB r9.b0, r9.b0, DIAG_CORRECTION
    SUB r9.b2, r9.b2, DIAG_CORRECTION
    SUB r10.b0, r10.b0, DIAG_CORRECTION
    SUB r10.b2, r10.b2, DIAG_CORRECTION
    SUB r11.b0, r11.b0, DIAG_CORRECTION
    SUB r11.b2, r11.b2, DIAG_CORRECTION
    SUB r12.b0, r12.b0, DIAG_CORRECTION
    SUB r12.b2, r12.b2, DIAG_CORRECTION
    SUB r13.b0, r13.b0, DIAG_CORRECTION
    SUB r13.b2, r13.b2, DIAG_CORRECTION
    SUB r14.b0, r14.b0, DIAG_CORRECTION
    SUB r14.b2, r14.b2, DIAG_CORRECTION
    SUB r15.b0, r15.b0, DIAG_CORRECTION
    SUB r15.b2, r15.b2, DIAG_CORRECTION
    SUB r16.b0, r16.b0, DIAG_CORRECTION
    SUB r16.b2, r16.b2, DIAG_CORRECTION
    SUB r17.b0, r17.b0, DIAG_CORRECTION
    SUB r17.b2, r17.b2, DIAG_CORRECTION
    SUB r18.b0, r18.b0, DIAG_CORRECTION
    SUB r18.b2, r18.b2, DIAG_CORRECTION
    SUB r19.b0, r19.b0, DIAG_CORRECTION
    SUB r19.b2, r19.b2, DIAG_CORRECTION
    SUB r20.b0, r20.b0, DIAG_CORRECTION
    SUB r20.b2, r20.b2, DIAG_CORRECTION
    SUB r21.b0, r21.b0, DIAG_CORRECTION
    SUB r21.b2, r21.b2, DIAG_CORRECTION
    SUB r22.b0, r22.b0, DIAG_CORRECTION
    SUB r22.b2, r22.b2, DIAG_CORRECTION
    SUB r23.b0, r23.b0, DIAG_CORRECTION
    SUB r23.b2, r23.b2, DIAG_CORRECTION
    SUB r24.b0, r24.b0, DIAG_CORRECTION
    SUB r24.b2, r24.b2, DIAG_CORRECTION

    QBA DONE_DIAG_CORRECTION

DIAG_STATE_1:
    // in an odd row
    SUB r9.b1, r9.b1, DIAG_CORRECTION
    SUB r9.b3, r9.b3, DIAG_CORRECTION
    SUB r10.b1, r10.b1, DIAG_CORRECTION
    SUB r10.b3, r10.b3, DIAG_CORRECTION
    SUB r11.b1, r11.b1, DIAG_CORRECTION
    SUB r11.b3, r11.b3, DIAG_CORRECTION
    SUB r12.b1, r12.b1, DIAG_CORRECTION
    SUB r12.b3, r12.b3, DIAG_CORRECTION
    SUB r13.b1, r13.b1, DIAG_CORRECTION
    SUB r13.b3, r13.b3, DIAG_CORRECTION
    SUB r14.b1, r14.b1, DIAG_CORRECTION
    SUB r14.b3, r14.b3, DIAG_CORRECTION
    SUB r15.b1, r15.b1, DIAG_CORRECTION
    SUB r15.b3, r15.b3, DIAG_CORRECTION
    SUB r16.b1, r16.b1, DIAG_CORRECTION
    SUB r16.b3, r16.b3, DIAG_CORRECTION
    SUB r17.b1, r17.b1, DIAG_CORRECTION
    SUB r17.b3, r17.b3, DIAG_CORRECTION
    SUB r18.b1, r18.b1, DIAG_CORRECTION
    SUB r18.b3, r18.b3, DIAG_CORRECTION
    SUB r19.b1, r19.b1, DIAG_CORRECTION
    SUB r19.b3, r19.b3, DIAG_CORRECTION
    SUB r20.b1, r20.b1, DIAG_CORRECTION
    SUB r20.b3, r20.b3, DIAG_CORRECTION
    SUB r21.b1, r21.b1, DIAG_CORRECTION
    SUB r21.b3, r21.b3, DIAG_CORRECTION
    SUB r22.b1, r22.b1, DIAG_CORRECTION
    SUB r22.b3, r22.b3, DIAG_CORRECTION
    SUB r23.b1, r23.b1, DIAG_CORRECTION
    SUB r23.b3, r23.b3, DIAG_CORRECTION
    SUB r24.b1, r24.b1, DIAG_CORRECTION
    SUB r24.b3, r24.b3, DIAG_CORRECTION


    DONE_DIAG_CORRECTION:
.endm


MEMACCESS_DDR_PRUSHAREDRAM:

    // Enable OCP master port
    LBCO      r0, CONST_PRUCFG, 4, 4
    CLR     r0, r0, 4         // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
    SBCO      r0, CONST_PRUCFG, 4, 4

    // Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
    // field to 0x0120.  This will make C28 point to 0x00012000 (PRU shared RAM).
    MOV     r0, 0x00000100
    MOV       r1, CTPPR_0_0
    ST32      r0, r1

    // Configure the programmable pointer register for PRU0 by setting c31_pointer[15:0]
    // field to 0x0010.  This will make C31 point to 0x80001000 (DDR memory).
    MOV     r0, 0x00100000
    MOV       r1, CTPPR_1_0
    ST32      r0, r1



INIT:
    MOV     r0, 0
    MOV     transfer_ready, 0
    MOV frame_counter, 0
    SUB frame_counter, frame_counter, 1 // intentional underflow

    // TODO: what does this do? (configure interrupts?)
    LBCO r0, CONST_PRUCFG, 0x34, 4
    SET r0, 1
    SBCO r0, CONST_PRUCFG, 0x34, 4
    // zero r0 for correct behavior of XIN/XOUT
    MOV r0, 0

    // set ACK field in PRU mem to 1 to get the ball rolling
    LDI     pr0ack, 1
    SBCO    pr0ack, CONST_PRUSHAREDRAM, 0, 4

    // Load DDR addr from arm host into a ddr_base register
    // TODO: can we use constant table instead?
    MOV     r1, 0
    LBBO    ddr_base, r1, 0, 4

    // move a value other than 1 or 2 to first address of ddr to indicate invalid data
    MOV var1, DDR_INVALID
    SBBO    var1, ddr_base, 0, 4

    // initialize number_frames from the value written to DDR by host
    INIT_NUM_FRAMES
    ADD number_frames, number_frames, 1
    // initialize ARM ack to 0
    MOV var1, 0
    SBBO    var1, r1, 0, 4

    CLR OE // pull buffer OE low to enable output

HOST_TRIGGER:
    host_trigger r1
    QBEQ HOST_TRIGGER, r1, 0

    // jump to initialization of ddr, without overwriting DATA_INVALID value
    QBA RESETDDR_1

RESETDDR:

    // to indicate completion of a group of four frames
    MOV var1, 1
    SBBO    var1, ddr_base, 0, 4

WAIT_ARM_ACK_1:
    MOV r1, ARM_PRU_ACK_OFFSET
    LBBO    var1, r1, 0, 4
    QBNE    WAIT_ARM_ACK_1, var1, 1
    // clear ack from ARM
    MOV var1, 0
    SBBO    var1, r1, 0, 4

    // clear the interrupt from pru1
    LDI     var1, 18
    SBCO    var1, C0, 0x24, 4

RESETDDR_1:
    // set DDR pointer to ddr base address
    SBCO    ddr_base, CONST_PRUSHAREDRAM, CHUNKSIZE + 8, 4
    LBCO    ddr_pointer, CONST_PRUSHAREDRAM, CHUNKSIZE + 8, 4
    // reset DDR counter to 0

READ:

    // TODO: change label of this register
    LBCO transfer_ready, CONST_PRUSHAREDRAM, 4, 4 // == 1 if there's a fresh chunk to transfer

    QBNE    WAIT, transfer_ready, 1


    // transfer_ready == 1
    // set transfer_ready back to 0 and copy to corresponding field in PRU shared ram
    MOV transfer_ready, 0
    SBCO    transfer_ready, CONST_PRUSHAREDRAM, 4, 4
    // load data
    XIN 10, data_start, CHUNKSIZE

    //do correction for checkerboard pattern
    diag_correction

    // copy data to DDR
    SBBO    data_start, ddr_pointer, DDR_OFFSET, CHUNKSIZE

    //write ACK to PRU mem
    SBCO    pr0ack, CONST_PRUSHAREDRAM, 0, 4

    // increment DDR memory pointer
    ADD ddr_pointer, ddr_pointer, CHUNKSIZE


WAIT:
    //check if we received the kill signal
    QBBS    FRAME_END, r31, 30
    QBA READ



FRAME_END:

    // clear the interrupt from pru1
    LDI     var1, 18
    SBCO    var1, C0, 0x24, 4 

    // Send notification to to host that one frame has been read out
    //MOV       r31.b0, PRU0_ARM_INTERRUPT+16
    SUB   number_frames, number_frames, 1     // decrement loop counter
    ADD   frame_counter, frame_counter, 1     // increment frame counter
    QBEQ  DONE, number_frames, 0  // repeat loop unless zero
    // number of frames completed FRAMES_PER_TRANSFER == 0? 
    MOV   var1, frame_counter.b0
    AND   var1, var1, (FRAMES_PER_TRANSFER - 1)

    // reset ddr pointer every 4 frames
    QBEQ  RESETDDR, var1, (FRAMES_PER_TRANSFER - 1)

    // resume readout
    QBA   READ


DONE:
    // give host time to process interrupt from pru1 before it a signal
    MOV r1, DELAY_1

    // TODO: in general we should do a power-down sequence here. It turns out
    // not to matter for the imx291.

    // Halt the processor
    HALT

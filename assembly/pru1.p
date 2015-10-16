// *
// * pru1.p
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
// file:   pru1.p
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

#define DELAY_MSECONDS 1000 // adjust this to experiment
#define CLOCK 200000 // PRU is always clocked at 200MHz
#define CLOCKS_PER_LOOP 6 // loop contains two instructions, one clock each
#define DELAY_2 (DELAY_MSECONDS * CLOCK / CLOCKS_PER_LOOP)


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

.macro  onepix_9_2
.mparam dst
    CLR  SYSCLK // falling clock edge
    MOV dst, PIX8_1 // move pix[8:0] into destination reg
    QBBS LEADING_1, r31, PIX9_N // branch depending on 
LEADING_0: // most significant bit is 0
    SET SYSCLK // rising edge
    LSR dst, dst, 1
    QBA END_ONEPIX_9_2
LEADING_1: // most significant bit is 1
    SET SYSCLK // rising edge
    LSR dst, dst, 1
    SET dst, dst, 7
END_ONEPIX_9_2:
.endm

.macro  onepix_8_1
.mparam dst
    CLR  SYSCLK // falling clock edge
    MOV dst, PIX8_1 // move pix[8:0] into destination reg
    QBBS SATURATED, r31, PIX9_N // saturated if most significant bit is 1
UNSATURATED: // pix value is good
    SET SYSCLK // rising edge
    NOP
    QBA END_ONEPIX_8_1
SATURATED: // pix value overflowed
    SET SYSCLK
    MOV dst, 255
    NOP
END_ONEPIX_8_1:
.endm

.macro  onepix_7_0
.mparam dst
    CLR  SYSCLK // falling clock edge
    MOV dst, PIX8_1 // move pix[8:0] into destination reg
    QBBS TRAILING_1, r31, PIX0_N // branch depending on 
TRAILING_0: // least significant bit is 0
    SET SYSCLK // rising edge
    LSL dst, dst, 1
    QBA END_ONEPIX_7_0
TRAILING_1: // least significant bit is 1
    SET SYSCLK // rising edge
    LSL dst, dst, 1
    SET dst, dst, 0
END_ONEPIX_7_0:
.endm

INIT:
    //set PRUCFG.SPP.XFR_SHIFT_EN=1
    LBCO r0, CONST_PRUCFG, 0x34, 4
    SET r0, 1
    SBCO r0, CONST_PRUCFG, 0x34, 4



    //MOV reads_left, NUMCHUNKS
    MOV r0, 0
    MOV     transfer_ready, 1

    // initialize number_frames from the value written to DDR by host
    INIT_NUM_FRAMES

    MOV frame_counter, 0

    // Check initial_config parameter from host side
HOST_TRIGGER:
    CLR SYSCLK // clock edge
    NOP
    NOP
    SET SYSCLK
    host_trigger r1
    QBEQ HOST_TRIGGER, r1, 0

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


READLINE:

    // spin while providing clock if LV == 0
    CLR SYSCLK // clock edge
    MOV var1, r31.w0 // read inputs
    NOP
    SET SYSCLK // clock edge
    NOP
    QBBC    READLINE, var1, LV_N


FILLBUFFER:
    // read in 64 pixels

    onepix_8_1  r9.b0
    onepix_8_1  r9.b1
    onepix_8_1  r9.b2
    onepix_8_1  r9.b3
    onepix_8_1  r10.b0
    onepix_8_1  r10.b1
    onepix_8_1  r10.b2
    onepix_8_1  r10.b3
    onepix_8_1  r11.b0
    onepix_8_1  r11.b1
    onepix_8_1  r11.b2
    onepix_8_1  r11.b3
    onepix_8_1  r12.b0
    onepix_8_1  r12.b1
    onepix_8_1  r12.b2
    onepix_8_1  r12.b3
    onepix_8_1  r13.b0
    onepix_8_1  r13.b1
    onepix_8_1  r13.b2
    onepix_8_1  r13.b3
    onepix_8_1  r14.b0
    onepix_8_1  r14.b1
    onepix_8_1  r14.b2
    onepix_8_1  r14.b3
    onepix_8_1  r15.b0
    onepix_8_1  r15.b1
    onepix_8_1  r15.b2
    onepix_8_1  r15.b3
    onepix_8_1  r16.b0
    onepix_8_1  r16.b1
    onepix_8_1  r16.b2
    onepix_8_1  r16.b3
QBA SKIPSTONE_2
READFRAME_STEPPINGSTONE_2:
    QBA READFRAME
READLINE_STEPPINGSTONE_2:
    QBA READLINE
SKIPSTONE_2:
    onepix_8_1  r17.b0
    onepix_8_1  r17.b1
    onepix_8_1  r17.b2
    onepix_8_1  r17.b3
    onepix_8_1  r18.b0
    onepix_8_1  r18.b1
    onepix_8_1  r18.b2
    onepix_8_1  r18.b3
    onepix_8_1  r19.b0
    onepix_8_1  r19.b1
    onepix_8_1  r19.b2
    onepix_8_1  r19.b3
    onepix_8_1  r20.b0
    onepix_8_1  r20.b1
    onepix_8_1  r20.b2
    onepix_8_1  r20.b3
    onepix_8_1  r21.b0
    onepix_8_1  r21.b1
    onepix_8_1  r21.b2
    onepix_8_1  r21.b3
    onepix_8_1  r22.b0
    onepix_8_1  r22.b1
    onepix_8_1  r22.b2
    onepix_8_1  r22.b3
    onepix_8_1  r23.b0
    onepix_8_1  r23.b1
    onepix_8_1  r23.b2
    onepix_8_1  r23.b3
    onepix_8_1  r24.b0
    onepix_8_1  r24.b1
    onepix_8_1  r24.b2
    onepix_8_1  r24.b3


// Can't jump by more than 255 words at once, hence this:
QBA SKIPSTONE
READFRAME_STEPPINGSTONE:
    QBA READFRAME_STEPPINGSTONE_2
READLINE_STEPPINGSTONE:
    QBA READLINE_STEPPINGSTONE_2
SKIPSTONE:

ACKCHECK:
    // check for ACK for previous read from pru0 
    LBCO    pr0ack, CONST_PRUSHAREDRAM, 0, 4
    QBNE    ACKCHECK, pr0ack, 1

    // reset ack from pru0 
    MOV     pr0ack, 0
    SBCO    pr0ack, CONST_PRUSHAREDRAM, 0, 4

    // recall transfer_ready is r8 and always == 1
    // simultaneously copy data and set transfer_ready field in pru memory to 1
    XOUT    10, data_start, CHUNKSIZE
    SBCO    transfer_ready, CONST_PRUSHAREDRAM, 4, 4

WAIT_LV:
    // continue in the same line if LV == 1
    CLR  SYSCLK // falling clock edge
    NOP
    NOP
    SET SYSCLK // rising edge
    MOV var1, r31.w0 // read inputs
    QBBS    READLINE_STEPPINGSTONE, var1, LV_N

    // otherwise return to polling FV, unless this was the last line
    CLR  SYSCLK // falling clock edge
    NOP
    NOP
    SET SYSCLK // rising edge
    // wait for signal propagation. THIS HAS BEEN SHOWN TO BE NECESSARY
    // WITH V2 and V3 OF THE INTERFACE BOARD
    NOP
    // read out another line if FV is still 1. Check the value
    QBBS    WAIT_LV, r31, FV_N


REPEAT: // however many more frames are specified
    SUB   number_frames, number_frames, 1     // decrement loop counter


    // done with one frame
    // Notify pru0 of completion
    LDI    R31, PRU1_PRU0_INTERRUPT + 16

    QBNE    READFRAME_STEPPINGSTONE, number_frames, 0  // repeat loop unless zero

    // Done with all frames!
    // Send notification to Host for program completion
    MOV       r31.b0, PRU1_ARM_INTERRUPT+16

// Continue providing a clock (necessary for i2c comm outside of this program)
CLKEND: // provide a clock indefinately

    CLR  SYSCLK // falling clock edge
    NOP
    NOP
    SET SYSCLK // rising edge
    NOP
    QBA CLKEND

    // Halt the processor
    HALT

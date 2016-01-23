#!/bin/bash
# output pins are pulled down
#modprobe -r uio_pruss
modprobe uio_pruss extram_pool_sz=0x800000
echo cape-univ-hdmi > /sys/devices/bone_capemgr.9/slots
echo cape-universal > /sys/devices/bone_capemgr.9/slots
echo cape-univ-emmc > /sys/devices/bone_capemgr.9/slots

#inputs
echo pruin > /sys/devices/ocp.*/P8_20_pinmux.*/state #emmc, d9
echo pruin > /sys/devices/ocp.*/P8_21_pinmux.*/state #emmc, d0
echo pruin > /sys/devices/ocp.*/P8_27_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_29_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_30_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_39_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_40_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_41_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_42_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_43_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_44_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_45_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P8_46_pinmux.*/state
#bits 1 and 0 of the pixel data. to enable when I figure out 
#how to pinmux these
echo pruin > /sys/devices/ocp.*/P9_25_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P9_29_pinmux.*/state
echo pruin > /sys/devices/ocp.*/P9_30_pinmux.*/state

#outputs
echo pruout > /sys/devices/ocp.*/P8_11_pinmux.*/state #OE for buffers SET FROM PRU0!
echo pruout > /sys/devices/ocp.*/P8_28_pinmux.*/state #SYSCLK
#echo pruout > /sys/devices/ocp.*/P8_12_pinmux.*/state 

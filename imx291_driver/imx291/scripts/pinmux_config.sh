#!/bin/bash
# output pins are pulled down
#modprobe -r uio_pruss
modprobe uio_pruss extram_pool_sz=0x800000
echo cape-univ-hdmi > /sys/devices/bone_capemgr.9/slots
echo cape-universal > /sys/devices/bone_capemgr.9/slots
echo cape-univ-emmc > /sys/devices/bone_capemgr.9/slots

#port 8 pins
#inputs
echo pruin > /sys/devices/ocp.*/P8_20_pinmux.*/state #Frame Valid
echo pruin > /sys/devices/ocp.*/P8_21_pinmux.*/state #Line Valid

echo pruin > /sys/devices/ocp.*/P8_27_pinmux.*/state #d0
echo pruin > /sys/devices/ocp.*/P8_29_pinmux.*/state #d1

echo pruin > /sys/devices/ocp.*/P8_28_pinmux.*/state #d10
echo pruin > /sys/devices/ocp.*/P8_30_pinmux.*/state #d11
echo pruin > /sys/devices/ocp.*/P8_39_pinmux.*/state #d8
echo pruin > /sys/devices/ocp.*/P8_40_pinmux.*/state #d9
echo pruin > /sys/devices/ocp.*/P8_41_pinmux.*/state #d6
echo pruin > /sys/devices/ocp.*/P8_42_pinmux.*/state #d7
echo pruin > /sys/devices/ocp.*/P8_43_pinmux.*/state #d4
echo pruin > /sys/devices/ocp.*/P8_44_pinmux.*/state #d5
echo pruin > /sys/devices/ocp.*/P8_45_pinmux.*/state #d2
echo pruin > /sys/devices/ocp.*/P8_46_pinmux.*/state #d3
#outputs
echo pruout > /sys/devices/ocp.*/P8_11_pinmux.*/state #OE
echo pruout > /sys/devices/ocp.*/P8_12_pinmux.*/state #Xmaster pin


 
#port 9 
#inputs

#how to pinmux i2c Pins
echo pruin > /sys/devices/ocp.*/P9_19_pinmux.*/statec #SCL
echo pruin > /sys/devices/ocp.*/P9_20_pinmux.*/state #SDA


echo pruout > /sys/devices/ocp.*/P9_27_pinmux.*/state #OE_shifters
echo pruout > /sys/devices/ocp.*/P9_28_pinmux.*/state #XCE
echo pruout > /sys/devices/ocp.*/P9_29_pinmux.*/state #OMODE
echo pruout > /sys/devices/ocp.*/P9_30_pinmux.*/state #XCLR
echo pruout > /sys/devices/ocp.*/P9_25_pinmux.*/state #pwr_en for buffers
#echo pruout > /sys/devices/ocp.*/P8_12_pinmux.*/state 

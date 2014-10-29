
#inputs
cat /sys/devices/ocp.*/P9_29_pinmux.*/state
cat /sys/devices/ocp.*/P9_30_pinmux.*/state
#cat /sys/devices/ocp.*/P8_20_pinmux.*/state #emmc
cat /sys/devices/ocp.*/P8_27_pinmux.*/state
cat /sys/devices/ocp.*/P8_29_pinmux.*/state
cat /sys/devices/ocp.*/P8_40_pinmux.*/state
cat /sys/devices/ocp.*/P8_39_pinmux.*/state
cat /sys/devices/ocp.*/P8_42_pinmux.*/state
cat /sys/devices/ocp.*/P8_41_pinmux.*/state
cat /sys/devices/ocp.*/P8_44_pinmux.*/state
cat /sys/devices/ocp.*/P8_43_pinmux.*/state
cat /sys/devices/ocp.*/P8_46_pinmux.*/state
cat /sys/devices/ocp.*/P8_45_pinmux.*/state



#outputs
cat /sys/devices/ocp.*/P8_11_pinmux.*/state
#temporarily reroute P8_12 to replace P8_21 until a microsd card is available
cat /sys/devices/ocp.*/P8_12_pinmux.*/state 
#cat /sys/devices/ocp.*/P8_21_pinmux.*/state #emmc
cat /sys/devices/ocp.*/P8_28_pinmux.*/state
cat /sys/devices/ocp.*/P8_30_pinmux.*/state

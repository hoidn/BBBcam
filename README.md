### Setup procedure (starting from a fresh debian image on microSD card):

#### OS/boot setup

Plug Beaglebone Black into PC and wait for the SD card's boot partition to mount (most recent version of the Beaglebone Black seems to be configured with higher boot priority for microSD compared to eMMC).
Edit /boot/uEnv.txt to disable eMMC and HDMI if necessary. Verify that those overlays are disabled with: 
```sh
cat /sys/devices/bone_capemgr.*/slots
```

#### Network configuration

If internet over USB is needed run the following commands to set up the USB interface as a gateway:
```sh
ifconfig usb0 192.168.7.2
route add default gw 192.168.7.1
```

If the host is running GNU/Linux, follow the additional instructions here:
https://elementztechblog.wordpress.com/2014/12/22/sharing-internet-using-network-over-usb-in-beaglebone-black/

For Windows, see the following video:
https://www.youtube.com/watch?v=fzRVVtGNfj8

#### Install prussdrv:

```sh
git clone https://github.com/beagleboard/am335x_pru_package.git
cd am335x_pru_package/pru_sw/app_loader/interface
sudo CROSS_COMPILE="" make install
```

#### Install beaglebone-uinversal-io

```sh
git clone https://github.com/cdsteinkuehler/beaglebone-universal-io.git && cd beaglebone-universal-io
make all && sudo make install
```

TODO: complete this

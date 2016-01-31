#ifndef imx291_H
#define imx291_H

#include<stdint.h> 
#include "i2c.h"


#define imx291_GLOBAL_GAIN_STR         "0x35"

//CMOS PARALLEL SERIAL OUTPUT IMAGE SETTINGS
#define imx291_STANDBY			0x3000
#define imx291_REGHOLD			0x3001
#define imx291_XMSTA			0x3002
#define imx291_SW_RESET			0x3003

#define imx291_ADBIT			0x3005
#define imx291_VREVERSE			0x3007 
#define imx291_HREVERSE			0x3007 
#define imx291_WINDMODE			0x3007 
#define imx291_FRSEL			0x3009 
#define imx291_FDG_SEL			0x3009 
#define imx291_BLKLEVEL1		0x300A 
#define imx291_BLKLEVEL2		0x300B
#define imx291_GAIN			0x3014 
#define imx291_VMAX1 			0x3018
#define imx291_VMAX2 			0x3019
#define imx291_VMAX3 			0x3020
#define imx291_HMAX1			0x301C
#define imx291_HMAX2			0x301D
#define imx291_SHS1a			0x3020
#define imx291_SHS1b			0x3021
#define imx291_WINWV_OB			0x303A
#define imx291_WINPV1			0x303C
#define imx291_WINPV2			0x303D
#define imx291_WINWV1			0x303E
#define imx291_WINWV2			0x303F
#define imx291_WINPH1			0x3040
#define imx291_WINPH2			0x3041
#define imx291_WINWH1			0x3042
#define imx291_WINWH2			0x3043
#define imx291_ODBIT			0x3046
#define imx291_OPORTSEL			0x3046
#define imx291_XVSLING			0x3048
#define imx291_XHSLING			0x3049

#define imx291_XVSOUTSEL		0x304B
#define imx291_XHSOUTSEL		0x304B
#define imx291_INCKSEL1			0x305C
#define imx291_INCKSEL2			0x305D
#define imx291_INCKSEL3			0x305E
#define imx291_INCKSEL4			0x305F
#define imx291_ADBIT1			0x3129
#define imx291_INCKSEL5			0x315E
#define imx291_INCKSEL6			0x316E
#define imx291_ADBIT2			0x317C
#define imx291_ADBIT3			0x31EC


#define imx291_MAX_WIDTH               1080
#define imx291_MAX_HEIGHT              1920
#define imx291_ROW_SKIP                12

#define imx291_ADDR_STR "0x34"
#define imx291_ADDR 0x34



int check_camera_running();
int set_camera_lock();
int check_gain();
void imx291_init_readout(uint16_t gain);

#endif

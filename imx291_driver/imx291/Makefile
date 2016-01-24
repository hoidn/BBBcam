#CROSS_COMPILE?=arm-arago-linux-gnueabi-
CROSS_COMPILE=

LIBDIR_APP_LOADER?=../../app_loader/lib
INCDIR_APP_LOADER?=../../app_loader/include
BINDIR?=../bin

#CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -D__DEBUG -g -mtune=cortex-a8 -march=armv7-a
#CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -D__DEBUG -O2 -mtune=cortex-a8 -march=armv7-a
LDFLAGS+=-L$(LIBDIR_APP_LOADER) -lprussdrv
OBJDIR=obj
TARGET=$(BINDIR)/main_imx291


_DEPS = 
DEPS = $(patsubst %,$(INCDIR_APP_LOADER)/%,$(_DEPS))

_OBJ = 
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ))  


all: pru1.bin pru0.bin pru1clk.bin oe_pru0.bin main_imx291 OE

pru1.bin: assembly/pru1.p
	#./pasm81_exec -b $^
	pasm -V3 -b $^

pru1clk.bin: assembly/other/pru1clk.p
	#./pasm81_exec -b $^
	pasm -V3 -b $^

pru0.bin: assembly/pru0.p
	#./pasm81_exec -b $^
	pasm -V3 -b $^

oe_pru0.bin: assembly/other/oe_pru0.p
	pasm -V3 -b $^


i2c.o: src/i2c.c
	gcc  -O3 -g -Wall -o $(OBJDIR)/i2c.o -c src/i2c.c `pkg-config --cflags --libs glib-2.0` 

imx291.o: src/imx291.c
	gcc  -O3 -g -Wall -o $(OBJDIR)/imx291.o -c src/imx291.c `pkg-config --cflags --libs glib-2.0` 

camctrl.o: src/camctrl.c
	gcc  -O3 -Wall -g -o $(OBJDIR)/camctrl.o -c src/camctrl.c `pkg-config --cflags --libs glib-2.0` 

main_imx291.o: 
	gcc  -O3 -g -Wall -std=c99 -o $(OBJDIR)/main_imx291.o -c src/main_imx291.c


#$(OBJDIR)/%.o: %.c $(DEPS) MT9M001_i2c.o
#	@mkdir -p obj
#	$(CROSS_COMPILE)gcc $(CFLAGS) -c -o $@ $<


main_imx291: main_imx291.o camctrl.o imx291.o i2c.o
	gcc  -O3 -g -std=c99 -Wall -I../../app_loader/include -D__DEBUG -g -mtune=cortex-a8 -march=armv7-a  -L../../app_loader/lib -lprussdrv  `pkg-config --cflags --libs glib-2.0` $(OBJDIR)/main_imx291.o $(OBJDIR)/camctrl.o $(OBJDIR)/imx291.o $(OBJDIR)/i2c.o -o main_imx291

OE: 
	gcc  -O3 -g -std=c99 -Wall -I../../app_loader/include -D__DEBUG -g -mtune=cortex-a8 -march=armv7-a  -L../../app_loader/lib -lprussdrv  `pkg-config --cflags --libs glib-2.0` src/OE.c -o OE

.PHONY: clean


clean:
	rm -f main_imx291 build/*.o *.bin *.o

echo:
	@echo $(OBJ) 

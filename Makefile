#CROSS_COMPILE?=arm-arago-linux-gnueabi-
CROSS_COMPILE=

LIBDIR_APP_LOADER?=../../app_loader/lib
INCDIR_APP_LOADER?=../../app_loader/include
BINDIR?=../bin

CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -D__DEBUG -g -mtune=cortex-a8 -march=armv7-a
#CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -D__DEBUG -O2 -mtune=cortex-a8 -march=armv7-a
LDFLAGS+=-L$(LIBDIR_APP_LOADER) -lprussdrv
OBJDIR=obj
TARGET=$(BINDIR)/PRU_memAcc_DDR_sharedRAM


_DEPS = 
DEPS = $(patsubst %,$(INCDIR_APP_LOADER)/%,$(_DEPS))

_OBJ = 
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ))  


all: pru1.bin pru0.bin pru1clk.bin oe_pru0.bin PRU_memAcc_DDR_sharedRAM OE

pru1.bin: pru1.p
	#./pasm81_exec -b $^
	pasm -V3 -b $^

pru1clk.bin: pru1clk.p
	#./pasm81_exec -b $^
	pasm -V3 -b $^

pru0.bin: pru0.p
	#./pasm81_exec -b $^
	pasm -V3 -b $^

oe_pru0.bin: oe_pru0.p
	pasm -V3 -b $^


MT9M001_i2c.o: MT9M001_i2c.c
	gcc  -g  -c MT9M001_i2c.c `pkg-config --cflags --libs glib-2.0` 

PRU_memAcc_DDR_sharedRAM.o: 
	gcc  -g -std=c99 -c PRU_memAcc_DDR_sharedRAM.c


$(OBJDIR)/%.o: %.c $(DEPS) MT9M001_i2c.o
	@mkdir -p obj
	$(CROSS_COMPILE)gcc $(CFLAGS) -c -o $@ $<

#$(OBJ): $(DEPS) 
#	$(CROSS_COMPILE)gcc $(CFLAGS) -c -o $@ $<

#$(TARGET): $(OBJ) MT9M001_i2c.o
#	$(CROSS_COMPILE)gcc $(CFLAGS) $(OBJ) -o $@ $^ $(LDFLAGS)
PRU_memAcc_DDR_sharedRAM: PRU_memAcc_DDR_sharedRAM.o MT9M001_i2c.o 
	gcc  -g -std=c99 -Wall -I../../app_loader/include -D__DEBUG -g -mtune=cortex-a8 -march=armv7-a  -L../../app_loader/lib -lprussdrv  `pkg-config --cflags --libs glib-2.0` PRU_memAcc_DDR_sharedRAM.o MT9M001_i2c.o -o PRU_memAcc_DDR_sharedRAM

OE: 
	gcc  -g -std=c99 -Wall -I../../app_loader/include -D__DEBUG -g -mtune=cortex-a8 -march=armv7-a  -L../../app_loader/lib -lprussdrv  `pkg-config --cflags --libs glib-2.0` OE.c -o OE

.PHONY: clean

#clean:
#	rm -rf $(OBJDIR)/ *~  $(INCDIR_APP_LOADER)/*~  $(TARGET)

clean:
	rm -f PRU_memAcc_DDR_sharedRAM *.o *.bin

echo:
	@echo $(OBJ) 

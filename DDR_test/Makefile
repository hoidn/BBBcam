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

_OBJ = PRU_memAcc_DDR_sharedRAM.o
OBJ = $(patsubst %,$(OBJDIR)/%,$(_OBJ))


all: pru1.bin pru0.bin PRU_memAcc_DDR_sharedRAM

pru1.bin: pru1.p
	#./pasm81_exec -b $^
	pasm -V3 -b $^

pru0.bin: pru0.p
	#./pasm81_exec -b $^
	pasm -V3 -b $^

$(OBJDIR)/%.o: %.c $(DEPS)
	@mkdir -p obj
	$(CROSS_COMPILE)gcc $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJ)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: clean

#clean:
#	rm -rf $(OBJDIR)/ *~  $(INCDIR_APP_LOADER)/*~  $(TARGET)

clean:
	rm -f PRU_memAcc_DDR_sharedRAM *.o *.bin

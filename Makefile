TARGET=gcp2stm32

# Standard arm development tools
CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
SE=arm-none-eabi-size
SF=st-flash

LINKERFILE = ./stm32g030C8.ld

# This needs to point to the roof of your libopencm3 installation
LIBOPENPCMROOT = /home/euler357/libopencm3

CFLAGS  = -std=gnu99 -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m0plus
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -I$(LIBOPENPCMROOT)/include -c -DSTM32G0
 
LDFLAGS = --static -nostartfiles -mthumb -mcpu=cortex-m0plus -msoft-float -Wl,-Map=$(TARGET).map 
LDFLAGS += -Wl,--gc-sections -specs=nosys.specs -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group 
LDFLAGS += -T$(LINKERFILE) -L../libopencm3/lib  

default: $(TARGET).bin 

$(TARGET).bin: $(TARGET).elf
	@echo '***************************'
	@echo '*** Making Bin from Elf ***'
	@echo '***************************'
	$(CP) -O binary $^ $@
	@echo

$(TARGET).o: $(TARGET).c
	@echo '*****************'
	@echo '*** Compiling ***'
	@echo '*****************'
	$(CC) $(INCLUDE) $(CFLAGS) $^ -o $@

$(TARGET).elf: $(TARGET).o 
	
	@echo '***************'
	@echo '*** Linking ***'
	@echo '***************'
	$(LD) $(LDFLAGS) $^ -lopencm3_stm32g0 -o $@

fresh: clean default

clean:
	@echo '****************'
	@echo '*** Cleaning ***'
	@echo '****************'
	rm -f *.o *.elf *.bin *.map

flash: default
	@echo '************************'
	@echo '*** Burning to Flash ***'
	@echo '************************'

	# Write Option Bytes (sets up BOOT0, RESET pins, etc.)
	#$(SF) --area=option write 0xdeffe1aa
	#$(SF) --area=option write 0xdaffffaa
	$(SF) --area=option write 0xcaffffaa

	# Write to FLASH
	$(SF) write $(TARGET).bin 0x8000000

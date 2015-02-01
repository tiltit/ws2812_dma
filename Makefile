BINARY = ws2811_dma

PREFIX	?= arm-none-eabi

CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
CP      = arm-none-eabi-objcopy
OD      = arm-none-eabi-objdump


ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 -msoft-float

CFLAGS		= -Os -g -Wall -Wextra -std=gnu99
CFLAGS		+= -I./libopencm3/include
CFLAGS		+= $(ARCH_FLAGS) -MD -DSTM32F1
CFLAGS		+= -fno-common -ffunction-sections -fdata-sections


LDFLAGS		= -L./libopencm3/lib
LDFLAGS		+= -nostartfiles -Wl,--gc-sections
LDFLAGS		+= $(ARCH_FLAGS) -mfix-cortex-m3-ldrd

LDLIBS		= -lopencm3_stm32f1
LDLIBS		+= -Wl,--start-group -lc -lm -lnosys -Wl,--end-group

SOURCES		= main.c
DEPS		= 
OBJECTS		= $(SOURCES:.c=.o)

all: $(BINARY).hex $(BINARY).bin

%.o: %.c $(DEPS)
	@ echo "[Compiling]"
	$(CC) -g -c $(CFLAGS) -o $@ $<

$(BINARY).elf: $(OBJECTS)
	@ echo "[Linking]"
	$(LD) -T stm32f103c8t6.ld $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $(BINARY).elf


$(BINARY).bin: $(BINARY).elf
	@ echo "[Copying]"
	$(CP) -Obinary  $(BINARY).elf $(BINARY).bin
	$(OD) -S $(BINARY).elf > $(BINARY).lst

$(BINARY).hex: $(BINARY).elf
	$(CP) -Oihex $(BINARY).elf $(BINARY).hex

# Note to self...
# To debug with gdb:

# If openOCD:
# Run "openocd -f openocd.cfg"
# In another terminal run: 
# "arm-none-eabi-gdb -tui myprogram.elf"
# Then in gdb:
# (gdb) target remote localhost:3333
# (gdb) monitor reset halt
# (gdb) load myprogram.elf
# (gdb) continue
# Ctrl + c to break
# (gdb) backtrace

# If st-link:
# Run: "st-util"
# Then on another terminal:
# arm-none-eabi-gdb myprogram.elf
# (gdb) tar extended-remote :4242
# (gdb) continue

#flash: $(BINARY).hex
#	openocd -f openocd.cfg \
#        -c "init" -c "reset init" \
#        -c "stm32f1x mass_erase 0" \
#        -c "flash write_image $(BINARY).hex" \
#        -c "reset" \
#        -c "shutdown" $(NULL)

flash: $(BINARY).bin
		st-flash write $(BINARY).bin 0x08000000

clean:
	rm -f *.lst *.o *.elf *.bin *.hex *.d


PRG			= rpi-sense
OBJ			= main.o variables.o rpi-sense.o

MCU_TARGET		= attiny88

ifeq ($(DEBUG),)
# DEBUG unset or empty
# disable debugging code and optimizations
OPTIMIZE		= -Os -g0 -DNDEBUG
# enable code for LED2472G read
CDEFINES	+= -DUSE_LEDREAD

else
# DEBUG set and not empty
OPTIMIZE		= -Os -g
# enable code for debugging register read/write
CDEFINES	+= -DUSE_REGWRITE
# enable code for LED2472G read/write
CDEFINES	+= -DUSE_LEDWRITE
endif

ifneq ($(USESLEEP),)
# enable code optimization with SLEEP instruction
CDEFINES	+= -DUSE_SLEEP
endif

ifneq ($(I2C_PAGES),)
CDEFINES	+= -DI2C_PAGES=$(I2C_PAGES)
# enable code to validate register address
CDEFINES	+= -DI2C_VALIDATE_ADDRESS
endif

ifneq ($(WRITE),)
CDEFINES	+= -DUSE_REGWRITE
CDEFINES	+= -DUSE_LEDWRITE
endif

ifneq ($(DISABLE_EXTRAS),)
CDEFINES	= -DDISABLE_EXTRAS
endif

ifneq (,$(findstring TWI_VECTOR_S,$(CDEFINES)))
OBJ			+= rpi-sense-twi.o
endif

CDEFINES	+= -DTWI_DATA_RAMPY
CC			= avr-gcc

# Override is only needed by avr-lib build system.
override ASFLAGS	= -Wall $(OPTIMIZE) $(CDEFINES) -mmcu=$(MCU_TARGET)
override CFLAGS		= -Wall $(OPTIMIZE) $(CDEFINES) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS	= -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

# Dependencies
%.o: %.c rpi-sense.h
%.o: %.S rpi-sense.h

.PHONY: clean
clean:
	-rm -rf *.o $(PRG).elf
	-rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }

# Every thing below here is used by avr-libc's build system and can be ignored
# by the casual user.

FIG2DEV                 = fig2dev
EXTRA_CLEAN_FILES       = *.hex *.bin *.srec

EXTRA_CLEAN_FILES		+= *.eps *.png *.pdf
dox: eps png pdf

eps: $(PRG).eps
png: $(PRG).png
pdf: $(PRG).pdf

%.eps: %.fig
	$(FIG2DEV) -L eps $< $@

%.pdf: %.fig
	$(FIG2DEV) -L pdf $< $@

%.png: %.fig
	$(FIG2DEV) -L png $< $@

# Flash
flash: $(PRG).hex
	sudo avrdude -c linuxspi -p t88 -P /dev/spidev0.0 -U flash:w:$(PRG).hex

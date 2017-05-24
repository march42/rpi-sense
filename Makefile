PRG			= rpi-sense
OBJ			= main.o rpi-sense.o

MCU_TARGET		= attiny88

ifeq ($(DEBUG),)
# disable debugging code and optimizations
OPTIMIZE		= -Os -g0 -DNDEBUG
else
OPTIMIZE		= -Os -g
endif

CC			= avr-gcc

CDEFINES	+= -DUSE_SLEEP
CDEFINES	+= -DUSE_LEDREAD

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
main.o: main.c
rpi-sense.o: rpi-sense.S

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

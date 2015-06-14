AVRCC		= avr-gcc -mmcu=attiny2313 -DF_CPU=80000000
AVROBJCOPY      = avr-objcopy
AVRCFLAGS       = -Wall -Os -g -Wall -I.

all: roof.hex

%.o: %.c
	$(AVRCC) $(AVRCFLAGS) -c -o $@ $^

%.o: %.S
	$(AVRCC) $(AVRCFLAGS) -c -o $@ $^

%.hex: %.elf
	$(AVROBJCOPY) -j .text -j .data -O ihex $^ $@

%.bin: %.elf
	$(AVROBJCOPY) -j .text -j .data -O binary $^ $@

roof.elf: roof.o
	$(AVRCC) -o $@ $^

install: roof.hex
	sudo avrdude -pt2313 -cusbtiny -U flash:w:roof.hex:i

clean:
	rm -f *.hex *.bin *.o *.elf

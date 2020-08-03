NAME=light

DEVICE= atmega328p
PROGRAMMER= arduino

# Old Arduino Nano Board's Settings
BAUDRATE= 57600
PORT= /dev/ttyUSB0

# Normal Settings
#BAUDRATE= 115200
#PORT= /dev/ttyACM0

CC=avr-gcc
CFLAGS=-Wall -Os -DF_CPU=16000000UL -mmcu=${DEVICE}

OBJCOPY=avr-objcopy


${NAME}.elf: ${NAME}.c
	${CC} ${CFLAGS} -o $@ $^

${NAME}.hex: ${NAME}.elf
	${OBJCOPY} -O ihex -R .eeprom $< $@

install: ${NAME}.hex
	avrdude -v -c ${PROGRAMMER} -p ${DEVICE} -P ${PORT} -b ${BAUDRATE} -U flash:w:$<

clean:
	rm -f ${NAME}.elf ${NAME}.hex

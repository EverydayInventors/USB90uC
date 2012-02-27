#
# Makefile for USB90µC Minimial Development Board timer test program
# Programmed by David Dahl, EverydayInventors
# Copyright 2010, 2011 Free Software Foundation
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#                                    
CC=avr-gcc
CFLAGS=-mmcu=at90usb162 -Wall -Os -D F_CPU=16000000UL
PRG=testtimer

$(PRG).elf: $(PRG).c
	$(CC) $(CFLAGS) -c $(PRG).c
	$(CC) $(CFLAGS) $(PRG).o -o $(PRG).elf

clean:
	rm -f *.o *.elf *.hex

$(PRG).hex: $(PRG).elf
	avr-objcopy -j .text -j .data -O ihex $(PRG).elf $(PRG).hex

upload: $(PRG).hex
#	dfu-programmer at90usb162 erase
#	dfu-programmer at90usb162 flash $(PRG).hex
#	dfu-programmer at90usb162 start
	./uploaddfu.sh $(PRG).hex

ihex: $(PRG).hex


#!/bin/bash
# dfu-programmer doesn't play nice with Make, so moved commands to script
dfu-programmer at90usb162 erase
dfu-programmer at90usb162 flash $1
dfu-programmer at90usb162 start

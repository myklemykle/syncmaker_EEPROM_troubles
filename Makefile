target := syncmaker
openocd_dir := ../openocd
openocd_script := $(openocd_dir)/tcl
openocd := $(openocd_dir)/src/openocd


v6_build_path := build/v6
v4_build_path := build/v4

v6: v6compile v6load

v6img_uf2 := build/rp2040.rp2040.generic/$(target).ino.uf2
v6compileflags := --fqbn rp2040:rp2040:generic   --board-options usbstack=tinyusb   --build-property "build.extra_flags=-DPI_V6" --build-path $(v6_build_path) # extra flags works here

v6compile: $(v6img_uf2)

v6clean:
	arduino-cli compile -v --clean $(v6compileflags)

$(v6img_uf2): syncmaker.ino *.cpp *.h 
	 arduino-cli compile -v $(v6compileflags)  

# flash fw via the USB-filesystem approach:
v6upload: 
	echo "uploading"
	cp $(v6img_uf2) /Volumes/RPI-RP2/NEW.UF2

# flash fw with openocd:
v6load:
	$(openocd) -f interface/picoprobe.cfg -f target/rp2040.cfg -s $(openocd_script) -c "program $(v6_build_path)/$(target).ino.elf verify reset exit"

rdp:
	$(openocd) -f interface/picoprobe.cfg -f target/rp2040-rescue.cfg -s $(openocd_script) 

v6monitor: 
	arduino-cli monitor -p /dev/tty.usbmodem?????

# start openocd debugger
v6ocd: 
	$(openocd) -f interface/picoprobe.cfg -f target/rp2040-core0.cfg -s $(openocd_script)

v6gdb:
	gdbgui -g 'arm-none-eabi-gdb -iex "target extended-remote localhost:3333"'

#######################

v4: v4compile v4load

v4img_uf2 := build/teensy:avr:teensy31/$(target).ino.uf2
v4compileflags := --fqbn teensy:avr:teensy31   --board-options opt=o1std,speed=48,usb=serialmidi   -e --build-path $(v4_build_path) --build-property "build.extra_flags=-DEVT4"  # extra flags does not work here!

v4clean:
	arduino-cli compile -v --clean $(v4compileflags)

v4compile:
	arduino-cli compile -v $(v4compileflags)

#	arduino-cli upload --fqbn teensy:avr:teensy31 -p /dev/cu.usbmodem??????*
v4load:
	teensy_loader_cli -v --mcu=mk20dx256 $(v4_build_path)/$(target).ino.hex

v4monitor: 
	arduino-cli monitor -p /dev/tty.usbmodem???????*


target := syncmaker
openocd_dir := ../openocd
openocd_script := $(openocd_dir)/tcl
openocd := $(openocd_dir)/src/openocd
gdb := arm-none-eabi-gdb

gdb := arm-none-eabi-gdb

v9_build_path := build/v9
v6_build_path := build/v6
v4_build_path := build/v4

v6: v6compile v6load pause v6monitor

pause:
	sleep 1

v6img_uf2 := $(v6_build_path)/$(target).ino.uf2
v9img_uf2 := $(v9_build_path)/$(target).ino.uf2

v6compileflags := --fqbn rp2040:rp2040:generic   --board-options usbstack=tinyusb,boot2=boot2_generic_03h_4_padded_checksum   --build-property "build.extra_flags=-DPI_V6" --build-path $(v6_build_path) # extra flags works here
v9compileflags := --fqbn rp2040:rp2040:generic   --board-options usbstack=tinyusb,boot2=boot2_generic_03h_4_padded_checksum   --build-property "build.extra_flags=-DPI_V9" --build-path $(v9_build_path) # extra flags works here

v9: v9compile v9load pause v6monitor

v9compile: $(v9img_uf2)

v6compile: $(v6img_uf2)

v6clean:
	arduino-cli compile -v --clean $(v6compileflags)

v9clean:
	arduino-cli compile -v --clean $(v9compileflags)

v6debug:
	arduino-cli compile -v --optimize-for-debug $(v6compileflags)

$(v6img_uf2): syncmaker.ino *.cpp *.h 
	 arduino-cli compile -v $(v6compileflags)  

$(v9img_uf2): syncmaker.ino *.cpp *.h 
	 arduino-cli compile -v $(v9compileflags)  

# flash fw via the USB-filesystem approach:
v6upload: 
	echo "uploading v6"
	cp $(v6img_uf2) /Volumes/RPI-RP2/NEW.UF2

v9upload: 
	echo "uploading v9"
	cp $(v9img_uf2) /Volumes/RPI-RP2/NEW.UF2

# flash fw with openocd:
v6load:
	$(openocd) -f interface/picoprobe.cfg -f target/rp2040.cfg -s $(openocd_script) -c "program $(v6_build_path)/$(target).ino.elf verify reset exit"

v9load:
	$(openocd) -f interface/picoprobe.cfg -f target/rp2040.cfg -s $(openocd_script) -c "program $(v9_build_path)/$(target).ino.elf verify reset exit"

rdp:
	$(openocd) -f interface/picoprobe.cfg -f target/rp2040-rescue.cfg -s $(openocd_script) 

v6monitor: 
	arduino-cli monitor -p /dev/tty.usbmodem???*

# start openocd debugger
v6ocd: 
	$(openocd) -f interface/picoprobe.cfg -f target/rp2040-core0.cfg -s $(openocd_script)

v6gdb:
	gdbgui -g '$(gdb) -iex "target extended-remote localhost:3333" $(v6_build_path)/$(target).ino.elf ' $(v6_build_path)/$(target).ino.elf

#######################

v4: v4compile v4load

v4img_uf2 := build/teensy:avr:teensy31/$(target).ino.uf2
v4compileflags := --fqbn teensy:avr:teensy31   --board-options opt=o1std,speed=48,usb=serialmidi   -e --build-path $(v4_build_path) --build-property "build.extra_flags=-DEVT4"  # extra flags does not work here!

v4clean:
	arduino-cli compile -v --clean $(v4compileflags)

v4compile:
	arduino-cli compile -v $(v4compileflags)

v4load:
	echo sorry, still broken. use Arduino App to program Teensy. 
	# this doesn't work, dunno why. maybe more args?
	#arduino-cli upload --fqbn teensy:avr:teensy31 -p /dev/cu.usbmodem??????*
	# meanwhile, is this loader installed? what's the status of it?
	#teensy_loader_cli -v --mcu=mk20dx256 $(v4_build_path)/$(target).ino.hex 

v4monitor: 
	arduino-cli monitor -p /dev/tty.usbmodem???????*


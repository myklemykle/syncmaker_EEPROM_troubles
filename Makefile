target := syncmaker

v6: v6compile v6upload

v6compile: build/rp2040.rp2040.generic/syncmaker.ino.uf2

v6clean:
	arduino-cli compile -v --clean --fqbn rp2040:rp2040:generic

build/rp2040.rp2040.generic/syncmaker.ino.uf2: syncmaker.ino *.cpp *.h
	 arduino-cli compile -v --board-options usbstack=tinyusb --fqbn rp2040:rp2040:generic -e ../$(target)

v6upload: 
	echo "uploading"
	cp build/rp2040.rp2040.generic/$(target).ino.uf2 /Volumes/RPI-RP2/NEW.UF2

monitor: 
	arduino-cli monitor -p /dev/tty.usbmodem*

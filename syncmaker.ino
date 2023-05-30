//////////////////////////// 
// Bug: EEPROM.put() hangs MCU
//
#include "config.h"
#include "pins.h"
#include "settings.h"
#include <Arduino.h>
#include <elapsedMillis.h>
#include <EEPROM.h>

// our main Settings object:
Settings _settings;

void setup() {

  rp2040.idleOtherCore();

  Serial.begin(115200);  // (Baud rate is ignored on USB serial but Pico core requires it anyway)
	delay(200); // let USB stabilize before speaking ...

	// load persistent settings from NVram:
	EEPROM.begin(SETTINGSEEPROMSIZE); // necessary for the rp2040 EEPROM emulation in Flash

	_settings.get(); // calls EEPROM.get()

	// If I don't write to the settings object the compiler removes something? and we don't see the bug.
	for (int i = 0; i<4; i++)
		_settings.s.outs[i] = 4;

	// if I leave core1 paused, i get the bug.  if I resume it, I don't (at the moment, tho in my larger sketch I did.
	/* rp2040.resumeOthercore(); */
}

void setup1(){
	return;
}
void loop1(){
	return;
}

void loop() {
	static elapsedMillis timeToSave = 0;

	if (timeToSave > 6000) {
		Dbg_println("going to write EEPROM");
		Serial.flush();
		_settings.put(); 								// With the bug, the MCU hangs here.
		Dbg_println("write finished"); // And this is never sent.
		Serial.flush();
		timeToSave -= 6000;
	}
}




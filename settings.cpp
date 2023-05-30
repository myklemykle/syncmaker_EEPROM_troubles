#include "config.h"
#include "settings.h"
#include <EEPROM.h>

bool Settings::init(){
	Dbg_println("init settings");
	s = { 
		SETTINGSFLAG, 
		SETTINGSVERSION, 
		500 * 1000, // uS == 120bpm
		{ OUTMODE_SYNC, OUTMODE_SHAKE, OUTMODE_SYNC, OUTMODE_SHAKE }, //  tip1, ring1, tip2, ring2
		false // imuTested
	};
	return true;
}

bool Settings::get(){
	Dbg_println("get settings");
	EEPROM.get(SETTINGSEEPROMBASE, s);

	Dbg_printf("flag = %x\n", s._flag);
	Dbg_printf("ver  = %x\n", s._version);
	Dbg_printf("len  = %d\n", s.measureLen);
	Dbg_printf("outs:  = %d,%d/%d,%d\n", 
			s.outs[0],
			s.outs[1],
			s.outs[2],
			s.outs[3]
			);
	Dbg_println(s.imuTested ? "imu tested" : "imu NOT tested");
	// For now reject any unknown version
	return (s._flag == SETTINGSFLAG) && (s._version == SETTINGSVERSION);
}

bool Settings::put(){
	EEPROM.put(SETTINGSEEPROMBASE, s);
	Dbg_println("put settings");
	Serial.flush();//TEST

	noInterrupts();

	EEPROM.commit(); // crash/hang here ...
									 
	interrupts();
	Dbg_println("committed settings");
	Serial.flush();//TEST
								 //
	return true;
}


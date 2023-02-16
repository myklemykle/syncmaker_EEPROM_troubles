#include "config.h"
#include "settings.h"
#include <EEPROM.h>

bool Settings::init(){
	Dbg_println("init settings");
	s = { 
		SETTINGSFLAG, 
		SETTINGSVERSION, 
		500 * 1000, // uS
		{ OUTMODE_SYNC, OUTMODE_SHAKE, OUTMODE_SYNC, OUTMODE_SHAKE }
	};
	return true;
}

bool Settings::get(){
	Dbg_println("get settings");
	EEPROM.get(SETTINGSEEPROMBASE, s);
	// For now reject any unknown version
	Dbg_printf("flag = %x\n", s._flag);
	Dbg_printf("ver  = %x\n", s._version);
	Dbg_printf("len  = %d\n", s.measureLen);
	Dbg_printf("outs:  = %d,%d/%d,%d\n", 
			s.outs[0],
			s.outs[1],
			s.outs[2],
			s.outs[3]
			);
	return (s._flag == SETTINGSFLAG) && (s._version == SETTINGSVERSION);
}

bool Settings::put(){
	Dbg_println("put settings");
	// TODO: make sure it's initialized?
	EEPROM.put(SETTINGSEEPROMBASE, s);
#ifdef PI_V6
	EEPROM.commit();
#endif
	return true;
}



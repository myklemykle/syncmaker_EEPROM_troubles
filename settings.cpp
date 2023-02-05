#include "config.h"
#include "settings.h"
#include <EEPROM.h>

bool Settings::init(){
	Dbg_println("init settings");
	s = { 
		SETTINGSFLAG, 
		SETTINGSVERSION, 
		120 * 1000, // uS
		{ {shaker, sync}, {shaker, sync} } 
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
			s.out[0].tipMode,
			s.out[0].ringMode,
			s.out[1].tipMode,
			s.out[0].ringMode
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


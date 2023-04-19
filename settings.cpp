#include "config.h"
#include "settings.h"
#include <EEPROM.h>

bool Settings::init(){
	Dbg_println("init settings");
	s = { 
		SETTINGSFLAG, 
		SETTINGSVERSION, 
		500 * 1000, // uS
		{ OUTMODE_SYNC, OUTMODE_SHAKE, OUTMODE_SYNC, OUTMODE_SHAKE } //  tip1, ring1, tip2, ring2
	};
	return true;
}

bool Settings::get(){
	Dbg_println("get settings");
	EEPROM.get(SETTINGSEEPROMBASE, s);
	// For now reject any unknown version
	// // TODO: use Settings::sprint here?
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
#ifdef MCU_RP2040
	EEPROM.commit();
#endif
	return true;
}

// dump a human-log of the settings into a provided buffer:
// const char* outmodeNames[OUTMODE_COUNT] = OUTMODE_NAMES;
char *Settings::sprint(char *buf, int buflen){
	snprintf(buf, buflen, 
			"flag: %x\nversion: %d\n measurelen: %d\n tip1: %s\n ring1: %s\n tip2: %s\n ring2: %s\n",
			s._flag,
			s._version,
			s.measureLen,
			Settings::outmodeNames[s.outs[0]],
			Settings::outmodeNames[s.outs[1]],
			Settings::outmodeNames[s.outs[2]],
			Settings::outmodeNames[s.outs[3]]
	);

	return buf;
}

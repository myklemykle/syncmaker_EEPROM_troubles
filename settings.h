#ifndef syncmaster_setting_h
#define syncmaster_settings_h

#include "pins.h"

// Note: in RP2040 the total emulated EEPROM size is set in EEPROM.begin().
#define SETTINGSEEPROMSIZE 256
#define SETTINGSEEPROMBASE 0

// This is a clue that we got non-garbled data:
#define SETTINGSFLAG  0x2323

// We'll bump this whenever we change the format of _Settings:
#define SETTINGSVERSION 3

// output channel modes
#define OUTMODE_MIDI 0
#define OUTMODE_SYNC 1
#define OUTMODE_SHAKE 2
#define OUTMODE_NOISE 3
#define OUTMODE_SINE 4
#define OUTMODE_SQUARE 5
#define OUTMODE_OFF 6
#define OUTMODE_LOW 7
#define OUTMODE_HIGH 8

// Settings object
typedef struct {
	uint16_t _flag;
	uint16_t _version; 
	unsigned long measureLen;	
	unsigned char outs[4];
	uint8_t imuTested;
} _Settings;

class Settings { 
	public: 
		_Settings s;
		bool init();
		bool get();
		bool put();
};


#endif

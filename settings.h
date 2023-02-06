#ifndef syncmaster_setting_h
#define syncmaster_settings_h


#ifndef PI_V6
// only for rev1 historical reasons is this number not 0:
#define SETTINGSEEPROMBASE 2000
#else
#define SETTINGSEEPROMBASE 0
#endif


// This is a clue that we got non-garbled data:
#define SETTINGSFLAG  0x2323

// We'll bump this whenever we change the format of _Settings:
#define SETTINGSVERSION 2

#define OUTMODE_MIDI 0
#define OUTMODE_SYNC 1
#define OUTMODE_AUDIO 2

// Settings object
typedef struct {
	uint16_t _flag;
	unsigned int _version; 
	unsigned long measureLen;	
	unsigned char outs[4];
} _Settings;

class Settings { 
	public: 
		_Settings s;
		bool init();
		bool get();
		bool put();
};


#endif

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

// output channel modes
#define OUTMODE_MIDI 0
#define OUTMODE_SYNC 1
#define OUTMODE_SHAKE 2
#define OUTMODE_NOISE 3
#define OUTMODE_SINE 4
#define OUTMODE_SQUARE 5
#define OUTMODE_OFF 6

// these modes are audio, and need the GPIP pin in PWM mode:
#define OUTMODE_IS_AUDIO(o) ( OUTMODE_SHAKE <= o <= OUTMODE_SQUARE )

// indicies of the channel settings in the outs[] array:
#define OUTCHANNEL_TIP1 0
#define OUTCHANNEL_RING1 1
#define OUTCHANNEL_TIP2 2
#define OUTCHANNEL_RING2 3

// possible test tones:
#define TESTTONE_NOISE 0
#define TESTTONE_SINE 1
#define TESTTONE_SQUARE 2
#define TESTTONE_OFF 3

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

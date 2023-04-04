#ifndef syncmaster_setting_h
#define syncmaster_settings_h

#include "pins.h"

#if PI_REV < 6
// long long ago, this was to avoid overwriting SensorFusion library's calibration data:
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
#define OUTMODE_LOW 7
#define OUTMODE_HIGH 8

#define OUTMODE_COUNT 9
#define OUTMODE_NAMES { "MIDI", "sync", "shake", "noise", "sine", "square", "off", "low", "high"}

// all of these modes are audio, and need the GPIO pin in PWM mode:
#define OUTMODE_IS_AUDIO(o) ( OUTMODE_SHAKE <= o <= OUTMODE_SQUARE )

// indicies of the channel settings in the outs[] array:
#define OUTCHANNEL_TIP1 0
#define OUTCHANNEL_RING1 1
#define OUTCHANNEL_TIP2 2
#define OUTCHANNEL_RING2 3

// map between those channel numbers and the actual pin numbers
const int outChannelPins[4] = { tip1, ring1, tip2, ring2 }; // from pins.h

// possible test tones:
#define TESTTONE_OFF 0
#define TESTTONE_SINE 1
#define TESTTONE_SQUARE 2
#define TESTTONE_SAW 3
#define TESTTONE_NOISE 4
#define TESTTONE_NAMES { "off", "sine", "square", "saw", "noise" }
#define TESTTONE_COUNT 5 // including "off"

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
		static constexpr char* outmodeNames[OUTMODE_COUNT] = OUTMODE_NAMES;
		bool init();
		bool get();
		bool put();
		char *sprint(char *buf, int buflen);
};


#endif

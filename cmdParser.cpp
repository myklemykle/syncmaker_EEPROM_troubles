#include <stdint.h>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <Arduino.h>
#include "config.h"
#include "settings.h"
#include "hardware/clocks.h"
#include <CommandParser.h>

#ifdef AUDIO_RP2040
#include "RP2040Audio.h"
#endif

//typedef CommandParser<16,4,10,32,64> MyCommandParser; //default vals
typedef CommandParser<16,4,10,32,128> MyCommandParser; // longer responses

MyCommandParser parser;

extern bool showStats;
extern Settings _settings;
extern void configOutput(int pin, byte mode);
extern void configOutputs(int pinPair, byte tipMode, byte ringMode);

#ifdef AUDIO_RP2040
extern RP2040Audio audio;
// extern short RP2040Audio::transferBuffer[TRANSFER_BUFF_SIZE];
// extern short RP2040Audio::sampleBuffer[SAMPLE_BUFF_SIZE];
#endif

#define strmatch(a, b) ( strcmp(a, b) == 0 )

// void cmd_test(MyCommandParser::Argument *args, char *response) {
//   Serial.print("string: "); Serial.println(args[0].asString);
//   Serial.print("double: "); Serial.println(args[1].asDouble);
//   Serial.print("int64: "); Serial.println(args[2].asInt64);
//   Serial.print("uint64: "); Serial.println(args[3].asUInt64);
//   strlcpy(response, "success", MyCommandParser::MAX_RESPONSE_SIZE);
// }

void cmd_stats(MyCommandParser::Argument *args, char *response) {
	if (showStats) {
		showStats = false;
		strlcpy(response, "stats off", MyCommandParser::MAX_RESPONSE_SIZE);
	} else {
		showStats = true;
		strlcpy(response, "stats on", MyCommandParser::MAX_RESPONSE_SIZE);
	}
}

/////////////////////////
// sleep tests:
//
#ifdef IMU_LSM6DSO32X
#include "lsm6dso32x.h"
extern LSM6DSO32X_IMU imu;  // STMicro IMU used from v6 onward
#elif defined(IMU_ICM42605)
#include "MotionSense.h"
extern MotionSense imu;  // on EVT1, rev2 & rev3 & evt4 the IMU is an ICM42605 MEMS acc/gyro chip
#endif
#include "sleep.h"

void cmd_sleep(MyCommandParser::Argument *args, char *response) {
	// imu.sleep();
	// strlcpy(response, "imu asleep", MyCommandParser::MAX_RESPONSE_SIZE);
	
	powerNap();
	strlcpy(response, "took a nap", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_wake(MyCommandParser::Argument *args, char *response) {
	imu.wake();
	strlcpy(response, "imu awake", MyCommandParser::MAX_RESPONSE_SIZE);
}

// print out the current settings ...
void cmd_dump_settings(MyCommandParser::Argument *args, char *response) {
	_settings.sprint(response, MyCommandParser::MAX_RESPONSE_SIZE);
}


// 
// set tip1|tip2|ring1|ring2 off/midi/sync/shake/noise/sine/square
// 
// extern const char* outmodeNames[OUTMODE_COUNT] = OUTMODE_NAMES;
void cmd_set(MyCommandParser::Argument *args, char *response) {
	byte chan;
	
	// "set all"
	if (strmatch(args[0].asString, "all")){
		if (strmatch(args[1].asString, "save")){
			// save settings object to eeprom
			_settings.put();
			strlcpy(response, "settings saved", MyCommandParser::MAX_RESPONSE_SIZE);
		} else if (strmatch(args[1].asString, "default")){
			// set all four outs to their default config
			configOutputs(tip1, OUTMODE_SYNC, OUTMODE_SHAKE);
			configOutputs(tip2, OUTMODE_SYNC, OUTMODE_SHAKE);
			cmd_dump_settings(args, response);
		} else {
			strlcpy(response, "syntax error: set all [save|default]", MyCommandParser::MAX_RESPONSE_SIZE);
		}
		return;
	}

	// "set [tip|ring][1|2]"
	char mode = -1;
	if (strmatch(args[0].asString, "tip1")){
		chan = OUTCHANNEL_TIP1;
	}
	else if (strmatch(args[0].asString, "ring1")){
		chan = OUTCHANNEL_RING1;
	}
	else if (strmatch(args[0].asString, "tip2")){
		chan = OUTCHANNEL_TIP2;
	}
	else if (strmatch(args[0].asString, "ring2")){
		chan = OUTCHANNEL_RING2;
	} else {
		strlcpy(response, "error: bad channel", MyCommandParser::MAX_RESPONSE_SIZE);
		return;
	}

	
	for (int i=0; i<OUTMODE_COUNT; i++){
		if (strmatch(args[1].asString, Settings::outmodeNames[i])){
			mode = i;
			break;
		}
	}

	// some synonyms:
	if (strmatch(args[1].asString, "midi")){
		mode = OUTMODE_MIDI;
	}
	else if (strmatch(args[1].asString, "hi")){
		mode = OUTMODE_HIGH;
	}
	else if (strmatch(args[1].asString, "lo")){
		mode = OUTMODE_LOW;
	} 
	

	if (mode == -1) { // not found
		strlcpy(response, "error: bad mode", MyCommandParser::MAX_RESPONSE_SIZE);
		return;
	}
	
	// update the settings:
	// _settings.s.outs[chan] = mode;
	// do the actual change:
	configOutput(outChannelPins[chan], mode);
	// _settings.put();

	strlcpy(response, "ok", MyCommandParser::MAX_RESPONSE_SIZE);
}


const char* testtoneNames[TESTTONE_COUNT] = TESTTONE_NAMES ;
extern char testTone;


//void _test_tone(MyCommandParser::Argument *args, char *response) {
void _test_tone(char *type, char *response) {
	for(int i=0;i<TESTTONE_COUNT;i++) {
		if (strmatch(type, testtoneNames[i])){
			if (i != testTone) {
				// maybe update the wave table
				switch (i) {
					case TESTTONE_NOISE:
						if (testTone != TESTTONE_OFF) {
							// fill with noise
#ifdef AUDIO_RP2040
							audio.fillWithNoise();
#endif
						}
						break;
					case TESTTONE_SINE:
#ifdef AUDIO_RP2040
							audio.fillWithSine(110);
#endif
						break;
					case TESTTONE_SQUARE:
#ifdef AUDIO_RP2040
							audio.fillWithSquare(110);
#endif
						break;
					case TESTTONE_SAW:
#ifdef AUDIO_RP2040
							audio.fillWithSaw(110);
#endif
						break;
					case TESTTONE_OFF:
						if (testTone != TESTTONE_NOISE) {
							// refill with noise
#ifdef AUDIO_RP2040
							audio.fillWithNoise();
#endif
						}
				}
				testTone = i;
			}
			return;
		}
	}

	// not found
	strlcpy(response, "error: bad test tone", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_test_tone(MyCommandParser::Argument *args, char *response) {
	_test_tone(args[1].asString, response);
}

void cmd_testtone(MyCommandParser::Argument *args, char *response) {
	_test_tone(args[0].asString, response);
}

extern float testLevel;

void cmd_testlevel(MyCommandParser::Argument *args, char *response) {
	double lvl = args[0].asDouble;
	if ( lvl < 0 || lvl > 1000) {
		strlcpy(response, "error: bad test level", MyCommandParser::MAX_RESPONSE_SIZE);
		return;
	}

	testLevel = lvl / 100.0;
	snprintf(response, MyCommandParser::MAX_RESPONSE_SIZE, "level %f/%f\n", lvl, testLevel);
}

#define CLOCK_SELECT_SYS 0x6
#define CLOCK_SELECT_XOSC 0x5
// TODO: somewhere in SDK these sources are all defined ....
// src must be between 0x0 and 0xa ...
void clock_pin_on(uint src){
	// connect clk_sys to GPIO25 for test/measurement
	// This is based on the CLK_GPOUT0_CTRL register in the datasheet
	// and simplified down from clock_gpio_init_int_frac() from the pico SDK
	uint gpclk = clk_gpout3; // appropriate for gpio 25

	// Set up the gpclk generator
	// AUXSRC_LSB is shifting our source into bits 5:8 (see datasheet)
	clocks_hw->clk[gpclk].ctrl = (src << CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_LSB) |
															 CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS;

	// Set gpio pin to gpclock function
	gpio_set_function(25, GPIO_FUNC_GPCK);
}
void clock_pin_off(){
	// disconnect pin 25
    gpio_set_function(25, GPIO_FUNC_NULL);
}
void cmd_clock(MyCommandParser::Argument *args, char *response) {
	if (strmatch(args[0].asString, "freq")) {
		// clk_sys speed as reported by SDK
		snprintf(response, MyCommandParser::MAX_RESPONSE_SIZE, "clk_sys %d\n", rp2040.f_cpu());

	} else if ( (strmatch(args[0].asString, "on") ||
				strmatch(args[0].asString, "sys")) )
	{
		clock_pin_on(CLOCK_SELECT_SYS);
		strlcpy(response, "clock pin sys", MyCommandParser::MAX_RESPONSE_SIZE);
	} else if (strmatch(args[0].asString, "xosc")){
		clock_pin_on(CLOCK_SELECT_XOSC);
		strlcpy(response, "clock pin xosc", MyCommandParser::MAX_RESPONSE_SIZE);
	} else if (strmatch(args[0].asString, "off")){
		clock_pin_off();
		strlcpy(response, "clock pin off", MyCommandParser::MAX_RESPONSE_SIZE);
	} else {
		strlcpy(response, "syntax err: clock [on|off]", MyCommandParser::MAX_RESPONSE_SIZE);
	}
}

// this more general approach isn't compatible with commandParser's arg handling ...
//
// void cmd_test(MyCommandParser::Argument *args, char *response) {
// 	if (strmatch(args[0].asString, "tone")){
// 		cmd_test_tone(args, response);
// 	// } else if (strmatch(args[0].asString, "level")){
// 	// 	cmd_test_level(args, response);
// 	} else {
// 		strlcpy(response, "error: bad test", MyCommandParser::MAX_RESPONSE_SIZE);
// 	}
// }


void cmd_setup() {
  //parser.registerCommand("TEST", "sdiu", &cmd_test);
  parser.registerCommand("stats", "", &cmd_stats);
  parser.registerCommand("set", "ss", &cmd_set);
  parser.registerCommand("clock", "s", &cmd_clock);
	
  parser.registerCommand("testtone", "s", &cmd_testtone);
  parser.registerCommand("tt", "s", &cmd_testtone);

  parser.registerCommand("testlevel", "u", &cmd_testlevel);
  parser.registerCommand("tl", "d", &cmd_testlevel);

	parser.registerCommand("sleep","",&cmd_sleep);
	parser.registerCommand("wake","",&cmd_wake);

	parser.registerCommand("settings","",&cmd_dump_settings);
}

void cmd_update() {
  if (Serial.available()) {
    char line[128];
		// NOTE: this is going to block unless the serial port was line-buffered;
		// we're presuming it is, maybe we shouldn't  ...
    size_t lineLength = Serial.readBytesUntil('\n', line, 127);
    line[lineLength] = '\0';

    char response[MyCommandParser::MAX_RESPONSE_SIZE];
    parser.processCommand(line, response);
    Serial.println(response);
  }
}


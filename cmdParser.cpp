#include <stdint.h>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <Arduino.h>
#include "config.h"
#include "settings.h"
#include <CommandParser.h>

#ifdef PI_V6
#include "RP2040Audio.h"
#endif

typedef CommandParser<> MyCommandParser;

MyCommandParser parser;

extern bool showStats;
extern Settings _settings;

#ifdef PI_V6
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
	if (args[0].asUInt64 == 0) {
		showStats = false;
		strlcpy(response, "stats off", MyCommandParser::MAX_RESPONSE_SIZE);
	} else {
		showStats = true;
		strlcpy(response, "stats on", MyCommandParser::MAX_RESPONSE_SIZE);
		// todo: show them now? they'll show soon enuf.
	}
}

// 
// set tip1|tip2|ring1|ring2 off/midi/sync/shake/noise/sine/square
// 
void cmd_set(MyCommandParser::Argument *args, char *response) {
	char chan;
	char mode;
	
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

	
	if (strmatch(args[1].asString, "off")){
		mode = OUTMODE_OFF;
	}
	else if (strmatch(args[1].asString, "midi")){
		mode = OUTMODE_MIDI;
		// TODO: anything at all about this.
	}
	else if (strmatch(args[1].asString, "sync")){
		mode = OUTMODE_SYNC;
	}
	else if (strmatch(args[1].asString, "shake")){
		mode = OUTMODE_SHAKE;
	} else {
		strlcpy(response, "error: bad mode", MyCommandParser::MAX_RESPONSE_SIZE);
		return;
	}
	
	// do the actual thing:
	_settings.s.outs[chan] = mode;
	// TODO: generate an event? call a method?
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
#ifdef PI_V6
							audio.fillWithNoise();
#endif
						}
						break;
					case TESTTONE_SINE:
#ifdef PI_V6
							audio.fillWithSine(110);
#endif
						break;
					case TESTTONE_SQUARE:
#ifdef PI_V6
							audio.fillWithSquare(110);
#endif
						break;
					case TESTTONE_SAW:
#ifdef PI_V6
							audio.fillWithSaw(110);
#endif
						break;
					case TESTTONE_OFF:
						if (testTone != TESTTONE_NOISE) {
							// refill with noise
#ifdef PI_V6
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
  parser.registerCommand("stats", "u", &cmd_stats);
  parser.registerCommand("set", "ss", &cmd_set);
	
  parser.registerCommand("testtone", "s", &cmd_testtone);
  parser.registerCommand("tt", "s", &cmd_testtone);

  parser.registerCommand("testlevel", "u", &cmd_testlevel);
  parser.registerCommand("tl", "d", &cmd_testlevel);
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

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
		// TODO: update sample
	// }
	// else if (strmatch(args[1].asString, "noise")){   // obsolete: use 'test tone' instead.
	// 	mode = OUTMODE_NOISE;
	// 	// TODO: update sample
	// }
	// else if (strmatch(args[1].asString, "sine")){
	// 	mode = OUTMODE_SINE;
	// 	// TODO: update sample
	// }
	// else if (strmatch(args[1].asString, "square")){
	// 	mode = OUTMODE_SQUARE;
	// 	// TODO: update sample
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


// indices in this array must match TESTTTONE_* defines in settings.h
const char* testtoneNames[4] = { "noise", "sine", "square", "off" };
extern char testTone;


void _test_tone(MyCommandParser::Argument *args, char *response) {
	for(int i=0;i<4;i++) {
		if (strmatch(args[1].asString, testtoneNames[i])){
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

void cmd_test(MyCommandParser::Argument *args, char *response) {
	if (strmatch(args[0].asString, "tone")){
		_test_tone(args, response);
	} else {
		strlcpy(response, "error: bad test", MyCommandParser::MAX_RESPONSE_SIZE);
	}
}


void cmd_setup() {
  //parser.registerCommand("TEST", "sdiu", &cmd_test);
  parser.registerCommand("stats", "u", &cmd_stats);
  parser.registerCommand("set", "ss", &cmd_set);
  parser.registerCommand("test", "ss", &cmd_test);
}

void cmd_update() {
  if (Serial.available()) {
    char line[128];
		// NOTE: this is going to block unless the serial port was line-buffered;
		// we shouldn't assume that.
    size_t lineLength = Serial.readBytesUntil('\n', line, 127);
    line[lineLength] = '\0';

    char response[MyCommandParser::MAX_RESPONSE_SIZE];
    parser.processCommand(line, response);
    Serial.println(response);
  }
}

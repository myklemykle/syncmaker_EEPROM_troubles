#include <stdint.h>
#include <CommandParser.h>
#include <Arduino.h>
#include "settings.h"

typedef CommandParser<> MyCommandParser;

MyCommandParser parser;

extern bool showStats;
extern Settings _settings;

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
	
	if (strcmp(args[0].asString, "tip1") == 0){
		chan = OUTCHANNEL_TIP1;
	}
	else if (strcmp(args[0].asString, "ring1") == 0){
		chan = OUTCHANNEL_RING1;
	}
	else if (strcmp(args[0].asString, "tip2") == 0){
		chan = OUTCHANNEL_TIP2;
	}
	else if (strcmp(args[0].asString, "ring2") == 0){
		chan = OUTCHANNEL_RING2;
	} else {
		strlcpy(response, "error: bad channel", MyCommandParser::MAX_RESPONSE_SIZE);
		return;
	}

	
	if (strcmp(args[1].asString, "off") == 0){
		mode = OUTMODE_OFF;
	}
	else if (strcmp(args[1].asString, "midi") == 0){
		mode = OUTMODE_MIDI;
		// TODO: anything at all about this.
	}
	else if (strcmp(args[1].asString, "sync") == 0){
		mode = OUTMODE_SYNC;
	}
	else if (strcmp(args[1].asString, "shake") == 0){
		mode = OUTMODE_SHAKE;
		// TODO: update sample
	}
	else if (strcmp(args[1].asString, "noise") == 0){
		mode = OUTMODE_NOISE;
		// TODO: update sample
	}
	else if (strcmp(args[1].asString, "sine") == 0){
		mode = OUTMODE_SINE;
		// TODO: update sample
	}
	else if (strcmp(args[1].asString, "square") == 0){
		mode = OUTMODE_SQUARE;
		// TODO: update sample
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

void cmd_setup() {
  //parser.registerCommand("TEST", "sdiu", &cmd_test);
  parser.registerCommand("stats", "u", &cmd_stats);
  parser.registerCommand("set", "ss", &cmd_set);
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

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
//typedef CommandParser<16,4,10,32,128> MyCommandParser; // longer responses
typedef CommandParser<17,4,10,32,1024> MyCommandParser; // even longer to fit hex dumps

MyCommandParser parser;

extern bool showStats;
extern Settings _settings;
extern void configOutput(int pin, byte mode);
extern void configOutputs(int pinPair, byte tipMode, byte ringMode);

#ifdef AUDIO_RP2040
extern RP2040Audio audio;
#endif

#ifdef SERIAL_MIDI
// PIO serial ports:
extern SerialPIO SSerialRing1;
extern SerialPIO SSerialTip1;
extern SerialPIO SSerialRing2;
extern SerialPIO SSerialTip2;
#endif

#define strmatch(a, b) ( strcmp(a, b) == 0 )

// note: using this macro requires the "response" variable;
#define respond(str) strlcpy(response, str, MyCommandParser::MAX_RESPONSE_SIZE)
#define respondf(...) snprintf(response, MyCommandParser::MAX_RESPONSE_SIZE, __VA_ARGS__)
#define syntaxErr(str) snprintf(response, MyCommandParser::MAX_RESPONSE_SIZE, "syntax err: %s\n", str)

// these are the standard args for command functions:
#define CMDARGS MyCommandParser::Argument *args, char *response

// void cmd_test(CMDARGS) {
//   Serial.print("string: "); Serial.println(args[0].asString);
//   Serial.print("double: "); Serial.println(args[1].asDouble);
//   Serial.print("int64: "); Serial.println(args[2].asInt64);
//   Serial.print("uint64: "); Serial.println(args[3].asUInt64);
//   respond("success");
// }

void cmd_stats(CMDARGS) {
	if (showStats) {
		showStats = false;
		respond("stats off");
	} else {
		showStats = true;
		respond("stats on");
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

void cmd_sleep(CMDARGS) {
	if (strmatch(args[0].asString, "imu")){
		imu.sleep();
		respond("imu asleep");
	} else if (strmatch(args[0].asString, "all")){
		powerNap();
		respond("took a nap");
	} else {
		syntaxErr("sleep [all|imu]");
		return;
	}
}

void cmd_wake(CMDARGS) {
	if (strmatch(args[0].asString, "imu")){
		imu.wake();
		// respond("imu awake");
		respond("woke imu");
	} else {
		syntaxErr("wake [all|imu]");
		return;
	}
}

void cmd_settings(CMDARGS){
	if (strmatch(args[0].asString, "get")){
		_settings.get();
		respond("got settings");
	}else if (strmatch(args[0].asString, "put")){
		_settings.put();
		respond("put settings");
	} else {
		// return the current settings ...
		_settings.sprint(response, MyCommandParser::MAX_RESPONSE_SIZE);
	}
}

// print out the current settings ...
void cmd_dump_settings(CMDARGS) {
	_settings.sprint(response, MyCommandParser::MAX_RESPONSE_SIZE);
}


// 
// set tip1|tip2|ring1|ring2 off/midi/sync/shake/noise/sine/square
// 
// extern const char* outmodeNames[OUTMODE_COUNT] = OUTMODE_NAMES;
void cmd_set(CMDARGS) {
	byte chan;
	SerialPIO *serp;
	bool invert = false;
	char mode = -1;
	
	// parse mode
	for (int i=0; i<OUTMODE_COUNT; i++){
		if (strmatch(args[1].asString, Settings::outmodeNames[i])){
			mode = i;
			break;
		}
	}
	// some synonyms:
	if (strmatch(args[1].asString, "midi")){
		mode = OUTMODE_MIDI;
		// invert is already false
	}
	else if (strmatch(args[1].asString, "idim")){
		mode = OUTMODE_MIDI;
		invert = true;
	}
	else if (strmatch(args[1].asString, "hi")){
		mode = OUTMODE_HIGH;
	}
	else if (strmatch(args[1].asString, "lo")){
		mode = OUTMODE_LOW;
	} 

	// parse channel
	//
	if (strmatch(args[0].asString, "all")){
		// "set all"
		if (strmatch(args[1].asString, "save")){
			// save channel settings to eeprom
			_settings.put();
			respond("settings saved");
		} else if (strmatch(args[1].asString, "default")){
			// set all four channels to their default modes
			configOutputs(tip1, OUTMODE_SYNC, OUTMODE_SHAKE);
			configOutputs(tip2, OUTMODE_SYNC, OUTMODE_SHAKE);
			cmd_dump_settings(args, response);
		} else if (mode != -1) { // other valid mode
			// change all channels to mode
			configOutputs(tip1, mode, mode);
			configOutputs(tip2, mode, mode);
#ifdef SERIAL_MIDI
			if (mode == OUTMODE_MIDI) { 
				// set logic inversion on all
					SSerialTip1.setInverted(invert,false);
					SSerialTip2.setInverted(invert,false);
					SSerialRing1.setInverted(invert,false);
					SSerialRing2.setInverted(invert,false);
			}
#endif
			cmd_dump_settings(args, response);
			return;
		} else {
			syntaxErr("set all [save|default|(mode)]");
			return;
		}
	}
	// "set [tip|ring][1|2]"
	else if (strmatch(args[0].asString, "tip1")){
		chan = OUTCHANNEL_TIP1;
#ifdef SERIAL_MIDI
		serp = &SSerialTip1;
#endif
	}
	else if (strmatch(args[0].asString, "ring1")){
		chan = OUTCHANNEL_RING1;
#ifdef SERIAL_MIDI
		serp = &SSerialRing1;
#endif
	}
	else if (strmatch(args[0].asString, "tip2")){
		chan = OUTCHANNEL_TIP2;
#ifdef SERIAL_MIDI
		serp = &SSerialTip2;
#endif
	}
	else if (strmatch(args[0].asString, "ring2")){
		chan = OUTCHANNEL_RING2;
#ifdef SERIAL_MIDI
		serp = &SSerialRing2;
#endif
	} else {
		respond("error: bad channel");
		return;
	}
	
	// change one channel to mode
	configOutput(outChannelPins[chan], mode);

#ifdef SERIAL_MIDI
	// set logic inversion
	if (mode == OUTMODE_MIDI) {
		serp->end();
		serp->setInverted(invert,false);
		serp->begin(31250); // midi baud rate
	}
#endif

	// _settings.put();

	respond("ok");
}


const char* testtoneNames[TESTTONE_COUNT] = TESTTONE_NAMES ;
extern char testTone;
extern volatile unsigned long iPitch[4];
// runmode defs dup'd from main .ino file:
#define RUNMODE_BOOT 0
#define RUNMODE_PLAY 1
#define RUNMODE_TEST 2
#define RUNMODE_DEBUG 3
extern uint8_t runMode;


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
							audio.fillWithSine(runMode == RUNMODE_TEST ? 1 : 110);
#endif
						break;
					case TESTTONE_SQUARE:
#ifdef AUDIO_RP2040
							audio.fillWithSquare(runMode == RUNMODE_TEST ? 1 : 110);
#endif
						break;
					case TESTTONE_SAW:
#ifdef AUDIO_RP2040
							audio.fillWithSaw(runMode == RUNMODE_TEST ? 1 : 110);
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
	respond("error: bad test tone");
}

void cmd_test_tone(CMDARGS) {
	_test_tone(args[1].asString, response);
}

void cmd_testtone(CMDARGS) {
	_test_tone(args[0].asString, response);
}

extern float testLevel;

void cmd_testlevel(CMDARGS) {
	double lvl = args[0].asDouble;
	if ( lvl < 0 || lvl > 1000) {
		respond("error: bad test level");
		return;
	}

	testLevel = lvl / 100.0;
	respondf("level %f/%f\n", lvl, testLevel);
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
void cmd_clock(CMDARGS){
	if (strmatch(args[0].asString, "freq")) {
		// clk_sys speed as reported by SDK
		respondf("clk_sys %d\n", rp2040.f_cpu());

	} else if ( (strmatch(args[0].asString, "on") ||
				strmatch(args[0].asString, "sys")) )
	{
		clock_pin_on(CLOCK_SELECT_SYS);
		respond("clock pin sys");
	} else if (strmatch(args[0].asString, "xosc")){
		clock_pin_on(CLOCK_SELECT_XOSC);
		respond("clock pin xosc");
	} else if (strmatch(args[0].asString, "off")){
		clock_pin_off();
		respond("clock pin off");
	} else {
		syntaxErr("clock [on|off|sys|xosc]");
	}
}

void cmd_imu_get(CMDARGS){
	uint8_t buf;
	unsigned long addr = args[0].asUInt64;
	imu.read_regs(SPI_cs, addr, &buf, 1);
	respondf("addr %x val %x", addr, buf);
}

#define IMU_SPI_BUFLEN 256
void cmd_imu_getn(CMDARGS){
	uint8_t buf[IMU_SPI_BUFLEN];
	unsigned long addr = args[0].asUInt64;
	unsigned long c = args[1].asUInt64;
	if (c > IMU_SPI_BUFLEN){
		respond("error: max count is #IMU_SPI_BUFLEN");
		return;
	}

	// hmm whats' happening ...
	int cnt = c;
	imu.read_regs(SPI_cs, addr, buf, cnt);

	int rowcnt = 8;
	char *cursor = response;
	for (int i = 0; i<c; i++){
		snprintf(cursor, 4, "%2x ", buf[i]);
		cursor += 3;
		rowcnt--;
		if (rowcnt == 0){
			snprintf(cursor, 2, "\n");
			cursor++;
			rowcnt = 8;
		}
	}
}

void cmd_imu_set(CMDARGS){
	uint8_t buf;
	unsigned long addr = args[0].asUInt64;
	unsigned long val  = args[1].asUInt64;
	imu.set_reg(SPI_cs, addr, val);
	respondf("set imu register %x to %x", addr, val);
}

// // dump all registers of the LSM IMU
// void cmd_imu_dump(CMDARGS){
// 	uint8_t buf[IMU_SPI_BUFLEN];
// 	unsigned long addr = args[0].asUInt64;
// 	unsigned long cnt = args[1].asUInt64;
// 	if (cnt > IMU_SPI_BUFLEN){
// 		respond("error: max count is #IMU_SPI_BUFLEN");
// 		return;
// 	}
//
// 	imu.read_regs(SPI_cs, addr, &buf, cnt);
// 	respondf("addr %x val %x", addr, buf);
// }
//

void cmd_peek(CMDARGS){
	unsigned long *addr;
	addr	= (unsigned long *) args[0].asUInt64;
	unsigned long val = *addr;
	respondf("addr %x == %x", addr, val);
}

void cmd_poke(CMDARGS){
	unsigned long *addr;
	addr = (unsigned long *) args[0].asUInt64;
	unsigned long val  = (unsigned long) args[1].asUInt64;
	*addr = val;
	respondf("set addr %x to %x", addr, val);
}


void cmd_audio(CMDARGS){
	int port = args[0].asUInt64; // 1 or 2, or 0 for both ports.
	if (port < 0 || port > 2){
		syntaxErr("audio [1|2|0] [on|off]");
		return;
	}
		
	if (strmatch(args[1].asString, "on")) {
		if (port < 2) { // 0 or 1
			audio.play(0);
		}
		if (port == 0 || port == 2) {
			audio.play(1);
		}
		respond("playing");
	} else if (strmatch(args[1].asString, "off")) {
		if (port < 2) // 0 or 1
			audio.pause(0);
		if (port == 0 || port == 2)
			audio.pause(1);
		respond("paused");
	} else {
		syntaxErr("audio [1|2|0] [on|off]");
	}
}

void cmd_pitch(CMDARGS){
	int channel = args[0].asUInt64; // 1-4, or 0 for all
	if (channel < 0 || channel > 4) {
		syntaxErr("pitch [1-4|0] [hz]");
	}
	int pitch = args[1].asUInt64;
	for (int i = 0; i<4; i++){
		if (channel - 1 == i || channel == 0)
			iPitch[i] = pitch;
	}
	respond("set pitch");
}

void cmd_setup() {
  //parser.registerCommand("TEST", "sdiu", &cmd_test);
  parser.registerCommand("stats", "", &cmd_stats);
  parser.registerCommand("set", "ss", &cmd_set);
  parser.registerCommand("clock", "s", &cmd_clock);
	
  parser.registerCommand("testtone", "s", &cmd_testtone);
  parser.registerCommand("tt", "s", &cmd_testtone);

  parser.registerCommand("testlevel", "u", &cmd_testlevel);
  parser.registerCommand("tl", "d", &cmd_testlevel);

	parser.registerCommand("sleep","s",&cmd_sleep);
	parser.registerCommand("wake","",&cmd_wake);

	parser.registerCommand("settings","s",&cmd_settings);

	parser.registerCommand("imuget","u",&cmd_imu_get);
	parser.registerCommand("imugetn","uu",&cmd_imu_getn);
	// parser.registerCommand("imudump","",&cmd_imu_dump);
	parser.registerCommand("imuset","uu",&cmd_imu_set);

	parser.registerCommand("peek","u", &cmd_peek);
	parser.registerCommand("poke","uu", &cmd_poke);

	parser.registerCommand("audio","us", &cmd_audio);
	parser.registerCommand("pitch","uu", &cmd_pitch);
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


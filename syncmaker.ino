//////////////////////////// 
// Pocket Integrator (PO daughterboard) firmware (c) 2022-2023 MSL
// This version compatible with EVT4 based on Teensy 3.2, or V6+ boards based on RP2040
// (PI_REV defines an integer: 4 for EVT4, 6 or higher for the RP@)$) series)
//
#include "config.h"
#include "pins.h"
#include "settings.h"
#include <Arduino.h>
#include <elapsedMillis.h>
#include <CircularBuffer.h>
#include "cbMacros.h"
#include "sleep.h"
#include "Bounce2.h"
#include <EEPROM.h>

#include "PI_MIDI.h"

#ifdef AUDIO_RP2040
// rp2040 audio
#include "RP2040Audio.h"
#include "hardware/pwm.h"
#include "pico/bootrom.h"
#elif defined(AUDIO_TEENSY)
#include <Audio.h>
// Teensy audio!
#endif

#ifdef MCU_RP2040
// this chip has 2 UARTs for midi & debug stuff.
#include "hardware/uart.h"
// and also these handy PIO devices that can put serial ports on pins that aren't wired for them:
SerialPIO SSerialTip1( tip1, SerialPIO::NOPIN ); // tx only
SerialPIO SSerialRing1( ring1, SerialPIO::NOPIN ); // tx only
SerialPIO SSerialTip2( tip2, SerialPIO::NOPIN ); // tx only
SerialPIO SSerialRing2( ring2, SerialPIO::NOPIN ); // tx only
#endif

#ifdef IMU_LSM6DSO32X
// ST IMU
#include "lsm6dso32x.h"
#elif defined(IMU_ICM42605)
// TDK IMU
#include "MotionSense.h"
#endif

#ifdef MIDITIMECODE
#include "miditimecode.h"
#endif 

///////////////
// Command Parser
//////////////
#include <CommandParser.h>
extern void cmd_setup();
extern void cmd_update();

// command parser will manipulate these globals for controlling the test tone:
char testTone = TESTTONE_OFF;
float testLevel = 1.0;

////////////////////////////////
// IMU Gesture Detection:
#ifdef IMU_LSM6DSO32X
LSM6DSO32X_IMU imu;  // STMicro IMU used from v6 onward
#elif defined(IMU_ICM42605)
MotionSense imu;  // on EVT1, rev2 & rev3 & evt4 the IMU is an ICM42605 MEMS acc/gyro chip
#endif

//const int shakeThreshold = (IMU_COUNT_PER_G * 100) / 65 ; 			// 0.65 * IMU_COUNT_PER_G
const int shakeThreshold = (IMU_COUNT_PER_G * 100) / 80;  // 0.85 * IMU_COUNT_PER_G ... a little less sensitive
const int tapThreshold = 2 * IMU_COUNT_PER_G;

CircularBuffer<long, 3> inertia;
// shaken should be the start of sound moment,
#define SHAKEN (CBROSETHRU(inertia, shakeThreshold))
//#define TAPPED (CBROSETHRU(inertia, tapThreshold))
// try putting tapped at the apex-G moment; it may have better feel:
#define TAPPED ((inertia[0] > tapThreshold) && (inertia[0] < inertia[1]) && (inertia[1] >= inertia[2])) // when Gs just starting to drop
// TODO: decide if there's a difference? A/B testing?


#ifdef TEENSY32

// Teensy audio!
// GUItool: end automatically generated code
AudioSynthNoiseWhite noise1;  //xy=110,301
AudioAmplifier amp1;          //xy=231,298
AudioOutputAnalog dac1;  //xy=603,359 //DEBUG
AudioConnection patchCord1(noise1, amp1);
AudioConnection patchCord2(amp1, dac1);
// GUItool: end automatically generated code

#else

// rp2040 audio runs on core1
// see setup1(), core1(), RP2040Audio.cpp 
RP2040Audio audio;

// C++ you so crazy!  Why do i have to redefine these?  Read the header file dude!
// This buffer stores our audio sample: white or pink noise for maraca.
extern short RP2040Audio::transferBuffer[TRANSFER_BUFF_SIZE];
// We scale/process audio out of the sample buffer into this small buffer,
// and DMA copies that to the PWM output.
extern short RP2040Audio::sampleBuffer[SAMPLE_BUFF_SIZE];

#endif


// Buttons:
const int debounceLen = 2;  // milliseconds

// side buttons
Bounce btn1 = Bounce();
Bounce btn2 = Bounce(); // Teensy 3.2
bool btn1pressed = false, btn2pressed = false;

// nonstop button
Bounce btn3 = Bounce();
bool btn3pressed = false;


#define BOTHPRESSED (btn1pressed && btn2pressed)
#define EITHERPRESSED (btn1pressed || btn2pressed)
#define EITHERNOTPRESSED (!BOTHPRESSED)

#ifdef BUTTON4
// reset button
Bounce btn4 = Bounce();
bool btn4pressed = false;
#endif

//////
// LEDs:

#include "EZLED.h"
EZLED leds[5] = {
	EZLED(0), // ignored, so that code names here match the schematic names which start from 1, not 0.
	EZLED(led1Pin),
	EZLED(led2Pin),
	EZLED(led3Pin), // aka nonstopPin
	EZLED(led4Pin)
};


// length of a sync pulse
const unsigned long pulseLen = 5 * 1000;
const unsigned long minTapInterval = 100 * 1000;  // Ignore spurious double-taps!  
// NOTE: This enforces a max speed of 10 taps per second == 600 taps per minute == 300bpm.
// That is not even half as fast as the Guiness Book's world drumming speed record:
// 1294 taps per minute on a snare drum by Keita Hattori == 647bpm.
// And that's just single-strokes; Pritish A R of Australia recorded 2370 taps
// in a minute on a snare drum using double-strokes.  So should I let this go faster,
// or admit to a top speed of 300bpm?  


// convert a time interval between beats to BPM:
// float: convert usecs to secs (1M to 1),
// divide by 60 secs per minute
//#define USEC2BPM(interval) ( 60.0 / ((interval / 1000000.0) ) )
#define USEC2BPM(interval) (60000000.0 / interval)  // same thing.  gives float result
#define BPM2USEC(bpm) (60000000 / bpm)              // inverse.  gives float result if passed a float!

// Timers:
elapsedMicros loopTimer_us;
elapsedMicros playPinTimer_us;
//#define MIN_AWAKE_US 5000000 // stay awake at least 5 seconds after a powerNap.
#define MIN_AWAKE_US 500000 // stay awake at least 5 seconds after a powerNap.
#ifdef BUTTON4
elapsedMillis resetTimer_ms; // how long is reset held down?
#endif
elapsedMillis awakeTimer_ms; // ms since boot.

elapsedMillis statsTimer_ms; // how often to report stats?
#define STATSTIME 1000 // 1hz (once per second)

elapsedMillis midiFlushTimer_ms; // how often to flush MIDI input?
#define MIDIFLUSHTIME 20 // 50hz 

elapsedMillis cmdTimer_ms; // how often to check for serial commands
#define CMDTIME 100 // 10hz is plenty, serial input is buffered.

// Loop profiler macros (no-ops unless PROFILE is defined)
#include "LoopProfiler.h"

// our main Settings object:
Settings _settings;


// Human Clock:
// a regular series of downbeats, seperated by a time interval (measureLen).
// User button/tap gestures can change the interval or the location of the downbeat.
typedef struct {
  unsigned long measureLen;
  unsigned long downbeatTime;
  int tapCount = 0;
  unsigned long lastTapTime = 0;
#define TAPBUFLEN 9
  CircularBuffer<long, TAPBUFLEN> tapIntervals;  // for running avg of previous tap intervals, to update measureLen
} HumanClock;
HumanClock hc;


// Circular Clock:
// A clock that tries to stay in sync with a Human Clock,
// but is limited in how fast it can speed/slow, and cannot move backwards.
// So relatively more "continuous" than the Human Clock, but still updated discretely.
// This is the generator of MIDI clock/timecode events.
//
// integer CC loops from 0 to 1B (1000000000)
#define CC_INT_RES 1000000000
//
// This is the very minimum (per loop cycle) forward advance -- one millionth of a measure
const int circleMinFwd = CC_INT_RES / 1000000;
// OTOH, if we advance any faster than this, we could skip a MIDI clock beat
const int circleMaxFwd = CC_INT_RES / 12;  // 83333333
//
typedef struct {
  // Clock position is an int value between 0 and 1B.
  long circlePos = 0, prevCirclePos = 0;
  // Freeze & unfreese behavior is subtle:
  // Freeze when both buttons pressed & no more midi measures remain;
  // unfreeze on the next downbeat after a button is released.
  bool frozen = false;
} CircularClock;
CircularClock cc;

#ifdef MIDICLOCK
// Midi Clock msgs are supposed to be sent 24 times per beat.
#define MIDICLOCKSPERPULSE 24.0
const long circleTicksPerClock = CC_INT_RES / MIDICLOCKSPERPULSE;
#endif

#ifdef MIDITIMECODE
// How often to increment the MTC frame, (1/25 second, the minimum resolution of midi timecode)?
// if we declare that 1 second of MTC equals 1 second of PO time at 120bpm, so 2 "beats" per second,
// which is actually (when you set PO tempo to 120) one entire 16-note pattern per second,
// which is 8 sync pulses per second, spread over 25 frames.
// 25/8 = 3.125
#define MIDIFRAMESPERPULSE 3.125
const long circleTicksPerFrame = CC_INT_RES / MIDIFRAMESPERPULSE;
MidiTimecodeGenerator mtc;
#endif

// State Variables:
CircularBuffer<bool, 2> blinkState;
CircularBuffer<bool, 2> pulseState;
CircularBuffer<bool, 2> playing;
CircularBuffer<bool, 2> decodedPlayLed;
CircularBuffer<bool, 2> nonstop;
//bool awakePinState = HIGH;
unsigned int awakePinState = 0;  // analog

unsigned long decodedPlayOnTime = 0, decodedPlayOffTime = 0;

// These are groups of PO models that have the same/similar Play LED animation behaviour:
// These keep the LED steady on when playing (with caveats):
#define MGRP_A 1  //PO12,PO14,PO16
// These keep the LED mostly off, and just flash it every 4 beats:
#define MGRP_B 2  //PO22,24,28,33
// These keep the LED mostly on, but wink it off every 4 beats:
#define MGRP_C 3  //PO32,35
// Unknown/not detected yet:
#define MGRP_AUTO 0

unsigned char poModelGroup = MGRP_AUTO;

boolean showStats = false;
#ifdef SDEBUG
// tracking of CPU performance
unsigned int loops = 0;  // # of main loop cycles between beats
unsigned int imus = 0;   // # number of inertia checks in same.
#endif

// IMU interrupt handler:
volatile bool imu_ready = false;
void imu_int_handler() {
  imu_ready = true;
}

// configure one of our 2 tip/ring output pairs
// pinpair is the number of a GPIO pin, but it's converted to a pair of pins
// (TODO: normalize numbering of stuff on either 0&1 or 1&2!)
void configOutputs(int pinPair, byte tipMode, byte ringMode) {
	int pins[2];
	int tipPin, ringPin;
	int settingsIdx;

	if (pinPair == ring1 || pinPair == ring2) {
		ringPin = pinPair;
		tipPin = pinPair + 1;
	} else if (pinPair == tip1 || pinPair == tip2) {
		tipPin = pinPair;
		ringPin = pinPair - 1;
	} else {
		Dbg_printf("configOutputs; bogus pin %d\n", pinPair);
		return;
	}

	// index into _settings.s.outs[] for these two pins:
	settingsIdx = (tipPin == tip1 ? 0 : 2);

	// If either pin mode is MIDI, both must be configured.
	// Assuming TRS adapter type A (the MIDI standard): https://minimidi.world/
	//   tip will be send "normal" rs232 logic, while ring sends "inverted" rs232 logic.
	// 
	if (tipMode == OUTMODE_MIDI || ringMode == OUTMODE_MIDI){
		tipMode = OUTMODE_MIDI; 
		ringMode = OUTMODE_MIDI;
	}

	Dbg_print("ring mode: ");
	switch(ringMode) { // OUTMODES are defined in settings.h
case OUTMODE_OFF:  
		Dbg_println("off");
		gpio_set_function(ringPin, GPIO_FUNC_NULL );
		break;
case OUTMODE_LOW:  
		Dbg_println("low");
		gpio_set_function(ringPin, GPIO_FUNC_SIO );
		digitalWrite(ringPin, LOW);
		break;
case OUTMODE_HIGH:  
		gpio_set_function(ringPin, GPIO_FUNC_SIO );
		digitalWrite(ringPin, HIGH);
		break;
case OUTMODE_SHAKE:
case OUTMODE_NOISE:
case OUTMODE_SINE:
case OUTMODE_SQUARE:
		Dbg_println("audio");
		// PWM audio
		gpio_set_function(ringPin, GPIO_FUNC_PWM );
		break;
case OUTMODE_SYNC:
		Dbg_println("sync");
		// plain old output pin
		gpio_set_function(ringPin, GPIO_FUNC_SIO );
		break;
case OUTMODE_MIDI:
		Dbg_println("midi");
		gpio_set_function(ringPin, GPIO_FUNC_PIO0); // sofware serial!
		break;
	}

	Dbg_print("tip mode: ");
	switch(tipMode) {
case OUTMODE_OFF:  
		Dbg_println("off");
		gpio_set_function(tipPin, GPIO_FUNC_NULL );
		break;
case OUTMODE_LOW:  
		Dbg_println("low");
		gpio_set_function(tipPin, GPIO_FUNC_SIO );
		digitalWrite(tipPin, LOW);
		break;
case OUTMODE_HIGH:  
		gpio_set_function(tipPin, GPIO_FUNC_SIO );
		digitalWrite(tipPin, HIGH);
		break;
case OUTMODE_SHAKE:
case OUTMODE_NOISE:
case OUTMODE_SINE:
case OUTMODE_SQUARE:
		Dbg_println("audio");
		// PWM audio
		gpio_set_function(tipPin, GPIO_FUNC_PWM );
		break;
case OUTMODE_SYNC:
		Dbg_println("sync");
		// plain old output pin
		gpio_set_function(tipPin, GPIO_FUNC_SIO );
		break;
case OUTMODE_MIDI:
		Dbg_println("midi");
		gpio_set_function(tipPin, GPIO_FUNC_PIO0); // sofware serial!
		break;
	}

	// update settings:
	_settings.s.outs[settingsIdx++] = tipMode;
	_settings.s.outs[settingsIdx] = ringMode;

// TODO: if anything can be turned off, turn it off.
}

// simpler util, for setting just one pin:
void configOutput(int pin, byte mode){
  switch(pin){
    case ring1:
      configOutputs(pin, _settings.s.outs[OUTCHANNEL_TIP1], mode);
      break;
    case ring2:
      configOutputs(pin, _settings.s.outs[OUTCHANNEL_TIP2], mode);
      break;
		case tip1:
      configOutputs(pin, mode, _settings.s.outs[OUTCHANNEL_RING1]);
      break;
		case tip2:
      configOutputs(pin, mode, _settings.s.outs[OUTCHANNEL_RING2]);
  }

}

// configure/initalize/etc at boot.
void setup() {
#ifdef MCU_RP2040
	  // fix startup weirdness
  rp2040.idleOtherCore();

	  // enable watchdog
	rp2040.wdt_begin(5000);
#endif

	// get settings:
#ifdef MCU_RP2040
	EEPROM.begin(256); // necessary for the rp2040 EEPROM emulation in Flash
#endif
	if (! _settings.get()) {
		_settings.init();
	}

  ///////
  // Pin setup:
  //
	// voltage select for 1.8v operation: set bit 0 of PADS_BANK0 and PADS_QSPI
	uint32_t *pads_voltage_sel = (uint32_t *) PADS_BANK0_BASE;
	*pads_voltage_sel = *pads_voltage_sel | 0b1; 
	pads_voltage_sel = (uint32_t *) PADS_QSPI_BASE;
	*pads_voltage_sel = *pads_voltage_sel | 0b1; 

	// pin modes:
	
	leds[1].init();
	leds[2].init();
#ifdef NONSTOP_HACK
	leds[2].initPWM();
#else
	leds[3].init();
#endif

#ifdef BUTTON4
	leds[4].init();
#endif
#ifdef EVT4
  leds[4].init(); // aka boardLedPin
#endif

#ifdef MCU_RP2040
	// raising the current limit to support TRSMIDI (default is 4ma, MIDI needs >5ma)
  pinMode(tip1, OUTPUT_8MA);              // j1 tip
  pinMode(tip2, OUTPUT_8MA);              // j2 tip
  pinMode(ring1, OUTPUT_8MA);              // j1 ring
  pinMode(ring2, OUTPUT_8MA);              // j2 ring
#else
  pinMode(tip1, OUTPUT);              // j1 tip
  pinMode(tip2, OUTPUT);              // j2 tip
	// Teensy has ring 1&2 hardwired to dac
#endif

  pinMode(button1Pin, INPUT_PULLUP);  // sw1
  pinMode(button2Pin, INPUT_PULLUP);  // sw2
  pinMode(button3Pin, INPUT_PULLUP);  // nonstop

#ifdef EVT4
  leds[4].init(); // aka boardLedPin
#endif

  pinMode(PO_play, INPUT);  // not sure if PULLUP helps here or not?  Flickers on & off anyway ...
  pinMode(PO_wake, INPUT);
  pinMode(PO_reset, INPUT_PULLUP);  // TODO: check v6: do we still get resets when physically connecting the boards? V6 added a pullup here ...
  pinMode(PO_SWCLK, INPUT_PULLUP);
  pinMode(PO_SWDIO, INPUT_PULLUP);
  pinMode(PO_SWO, INPUT_PULLUP);

#if PI_REV >= 9
	// turn on the analog reference regulator:
	pinMode(Aref_enable, OUTPUT);
	digitalWrite(Aref_enable, HIGH);
#endif

#ifdef MCU_RP2040
	/////// CMOS chips must stabilize all unconnected pins, to avoid static glitches
	int i;
	for (i=0; i < std::size(unusedPins); i++){
		/* Dbg_printf("disable pin %d\n", unusedPins[i]); */
		pinMode(i, INPUT_PULLUP);
	}
#endif

	// initialize USB connections:
	myMidi.begin();
  Serial.begin(115200);  // (Baud rate is ignored on USB serial but Pico core requires it anyway)

#ifdef MCU_RP2040
	// configure output jacks -- potentially disconnecting serial for now
	configOutputs(ring1, _settings.s.outs[0], _settings.s.outs[1]);
	configOutputs(ring2, _settings.s.outs[2], _settings.s.outs[3]);
#endif

	// flash LEDs to say good morning, and give USB a moment to stabilize before we use it.
	leds[1].on();
	leds[2].on();
	leds[3].on();
#ifdef BUTTON4
	leds[4].on();
#endif // BUTTON4

  delay(200); // waiting for USB host...

	leds[1].off();
	leds[2].off();
	leds[3].off();
#ifdef BUTTON4
	leds[4].off();
#endif // BUTTON4

  ///////////
  // initialize loop state:
  CBINIT(inertia, 0);
  CBINIT(nonstop, 0);
  CBINIT(playing, 0);
  CBINIT(decodedPlayLed, 0);


#ifdef IMU_SPI
  ///////
  // SPI+IMU setup:

  // raise chipSelect line to select the only chip.
  // (no-op; the library toggles it on & off anyway)
  pinMode(SPI_cs, OUTPUT);

#ifdef TEENSY32
  digitalWrite(SPI_cs, HIGH);
  // Teensy AudioLibrary may have changed these. i change them back!
  SPIPORT.setMOSI(SPI_mosi);
  SPIPORT.setMISO(SPI_miso);
  SPIPORT.setSCK(SPI_clock);
	SPIPORT.begin();
#elif defined(MCU_RP2040)
  // Dunno why the RP2040 version has different api here ... maybe not necessary/depreacted?
  SPIPORT.setRX(SPI_miso);
  SPIPORT.setTX(SPI_mosi);
  SPIPORT.setCS(SPI_cs);  // no-op; the library drives this manually
  SPIPORT.setSCK(SPI_clock);
	SPIPORT.begin(false); // manage CS ourselves
#endif

#endif

  pinMode(IMU_int, INPUT_PULLUP);
  attachInterrupt(IMU_int, imu_int_handler, FALLING);
	
  imu.begin();



  // Button setup:
  //
  btn1.attach(button1Pin);
  btn1.interval(debounceLen);

  btn2.attach(button2Pin);
  btn2.interval(debounceLen);

  btn3.attach(button3Pin);
  btn3.interval(debounceLen);

#ifdef BUTTON4
  btn4.attach(button4Pin);
  btn4.interval(debounceLen);
#endif

	readButtons();

	// TODO: check for initial button states at boot to trigger various
	// mode changes, resets, etc.

  ////////
  // Clock setup:
  //
  hc.downbeatTime = micros();  // now!
  // If side buttons are held down when we boot, reset the default measure length
  if (BOTHPRESSED) {
    _settings.s.measureLen = hc.measureLen = 250 * 1000;  // 120bpm == 2 beats per second, @ 2 pulses per beat == 1/4 second (250ms) per pulse) */
		_settings.put();
  } else {
		hc.measureLen = _settings.s.measureLen;
  }


#ifdef MIDITIMECODE
  mtc.setup();
#endif

  CBINIT(hc.tapIntervals, 0);

#ifdef TEENSY32
  ////////
  // Teensy Audio setup:
  AudioMemory(2);
  dac1.analogReference(EXTERNAL);  // 3.3v p2p (but see below)
  noise1.amplitude(2);	/// umm why not 1? 
  amp1.gain(0);
  /* dc1.amplitude(0); */                     //DEBUG
  /* mixer1.gain(0, 1); // noise1 -> amp1 */  //DEBUG
  /* mixer1.gain(1, 1); // dc1 */             //DEBUG

  // Initialize the DAC output pins
  // https://forum.pjrc.com/threads/25519-Noise-on-DAC-(A14)-output-Teensy-3-1
  analogWriteResolution(12);
  analogWrite(A14, 0);    //Set the DAC output to 0.
  DAC0_C0 &= 0b10111111;  //uses 1.2V reference for DAC instead of 3.3V
                          // does this clash with analogReference() above?  probably ...
                          // anyway I still get background noise when running off battery.
                          // TODO: test without this stuff & instead with INTERNAL in dac1.analogReference
#else
	rp2040.resumeOtherCore();
  // see setup1() for rp2040 audio on second core
#endif

  sleep_setup();

	cmd_setup();

	PROFILE_SETUP();

#if PI_REV == 10
  Dbg_println("flashed for v10 board");
#elif PI_REV == 9
  Dbg_println("flashed for v9 board");
#elif PI_REV == 8
  Dbg_println("flashed for v8 board");
#elif PI_REV == 7
  Dbg_println("flashed for v7 board");
#elif PI_REV == 6
  Dbg_println("flashed for v6 board");
//
// FWIW, rev 5 (with Microchip MCU) never got as far as firmware
//
#elif PI_REV == 4
  Dbg_println("flashed for EVT4 board");
#else
  Dbg_println("flashed for rev3 board");
//
// earlier versions are super-unsupported!
//
#endif

}


// Call the Bounce library functions to read the button state,
// and update the btnPressed booleans.
void readButtons(){
	// check buttons:
	if (btn1.update())
		btn1pressed = (btn1.read() == LOW);
	if (btn2.update())
		btn2pressed = (btn2.read() == LOW);
	if (btn3.update())
		btn3pressed = (btn3.read() == LOW);
#ifdef BUTTON4
	if (btn4.update())
		btn4pressed = (btn4.read() == LOW);
#endif
}

	
int ax, ay, az;
int gx, gy, gz;
uint32_t volumeLevel = 0; 

// Read acc+gyro data from IMU, 
// calculate new shaker volume
// update the audio system with that.

uint32_t processIMU(int& ax, int& ay, int& az, int& gx, int& gy, int& gz){
#ifdef SDEBUG
	imus++;
#endif

	// shift inner loop state:
	CBNEXT(inertia);

	PROFILE_MARK_START("imu");
	imu.readMotionSensor(ax, ay, az, gx, gy, gz);
	PROFILE_MARK_END("imu");

	PROFILE_MARK_START("sqrt");
	CBSET(inertia, (long)sqrt((ax * ax) + (ay * ay) + (az * az)));  // vector amplitude (always positive)
	PROFILE_MARK_END("sqrt");

#ifdef SDEBUG
	/* if (inertia[0] > shakeThreshold)  */
	/* 	Dbg_println(inertia[0]); */
#endif

	// This funny formula gives a float value for volume:
	// it's the difference between momentary intertia and a minimum inertial threshhold (shakeThreshold),
	// both of which are scaled to Gs at the configured resolution of the IMU (IMU_COUNT_PER_G),
	// then divided by 3gs (/3.0 * IMU_COUNT_PER_G) , a volume-attenuating coefficient that I apparently found in my ass.

	// The result can't go below 0, but what's the max?
	// If the IMU is sensitive to 8gs, and the threshhold is 0.8 gs,
	// then the max is 7.2/3  = 2.4
	// (Testing shows i can hear an audible increase in noise volume from overamplifying up to about 3.0,
	// so that could be an ideal threshhold.)
	// (I got to this formula through tweaking and listening, but it's kinda obscure.)

	//volumeLevel = max((inertia[0] - shakeThreshold) / (3.0 * IMU_COUNT_PER_G), 0.0); // always 0 or more
	// volumeLevel = max((inertia[0] - shakeThreshold) / (1.0 * IMU_COUNT_PER_G), 0.0); // more loud please! 
	uint32_t volumeLevel = max((inertia[0] - shakeThreshold)/ 1 , 0.0); // int version: multiplied by IMU_COUNT_PER_G

	PROFILE_MARK_START("audio");
#ifdef AUDIO_RP2040
	// send vol level to core 1:
	if (testTone != TESTTONE_OFF) {
		rp2040.fifo.push_nb((uint32_t)(testLevel * (float)WAV_PWM_RANGE)); 
		//rp2040.fifo.push_nb(min(WAV_PWM_RANGE, az * WAV_PWM_RANGE / IMU_COUNT_PER_G)); //DEBUG: level adjusts with rotation
	} else {
		//rp2040.fifo.push_nb((volumeLevel * WAV_PWM_RANGE));  // core1 saves this to iVolumeLevel
		rp2040.fifo.push_nb((volumeLevel * WAV_PWM_RANGE / IMU_COUNT_PER_G));  // int version: divided by IMU_COUNT_PER_G
	}
#elif defined(AUDIO_TEENSY)
	// use the inertia (minus gravity) to set the volume of the noise generator
	if (testTone != TESTTONE_OFF) {
		amp1.gain(testLevel); // for testing
	} else {
		//amp1.gain(volumeLevel );
		amp1.gain((1.0 * volumeLevel) / IMU_COUNT_PER_G); // converted to float, divided by IMU_COUNT_PER_G
	}

#endif
	PROFILE_MARK_END("audio");

	return volumeLevel;
}

// Not sure this even needs to be a function, but we're gonna try to change the timing so ...
void flushMIDI(PI_MIDI m){
	m.flushInput();
}

// Handle the play-start event (user pressed play button).
void handlePlayStart(unsigned long nowTime){
		// sync clocks 
    hc.downbeatTime = nowTime;
    cc.circlePos = 0;
#ifdef MIDICLOCK
		// send midi start messages:
    /* myMidi.clockTick(); */
		/* myMidi.clockStart(); */

    /* I haven't yet found good guidance for this bit,
			 but it really appears that Live 10 treats the Start and Stop
			 messages as if they were also clock ticks.  Sending Clock 
			 and Start in quick succession was making Live briefly think the tempo 
			 was super-high, leading to an initial stutter.  I changed this
			 and it now sounds much better.
			 FWIW I think there's a matching problem with Stop; we don't sync it to
			 where the MTC clock pulse would be, which is why sometimes when
			 I hit stop Live sets the tempo super high -- but it's stopped, 
			 so it's not really a problem.  Nevertheless,
			 I should find some other test cases than Live ...  */

		myMidi.clockStart();
#endif
#ifdef MIDITIMECODE
    // rewind time to zero
    mtc.rewind();
#endif
}

void handlePlayEnd(unsigned long nowTime){
#ifdef MIDICLOCK
		// send midi stop message:
		myMidi.clockStop();
#endif
}

//////////////////////////////////////////////////
// Decode the state of PO Play from the LED signal.
//
// First we read the value on the Play LED pin,
// but that's PWM, so we have to debounce that to get a boolean
// for decodedPlayLED .
//
// Then we have to figure out what the state of that LED means.
// It's unlit when not playing, but when playing, different PO models still can blink it on & off in various ways.
// So we try to deduce the state of play from the animation of that LED.
//
// Then if the state of play has changed we do starting-play and stopping-play events, for the clocks and MIDI.
//
// RATE: we want minimal lag btwn pressing play & sending midi START,
// so call this on every loop!
//
elapsedMicros pwmOffTimer;
// duration of the off-cycle of the Play button PWM wave
const unsigned long pwmOffTime = 150 * 1000;
#define PO_PLAY_PWMOFFTIME  150 * 1000
void decodePlayState(unsigned long nowTime){

  if (digitalRead(PO_play) == HIGH){
    pwmOffTimer = 0;
    if (!decodedPlayLed[0]) {       // if LED is lit, and wasn't before
      CBSET(decodedPlayLed, HIGH);  // decode as on.
    }

  } else if (pwmOffTimer > PO_PLAY_PWMOFFTIME) {  // if unlit for longer than a PWM off cycle
    if (decodedPlayLed[0]) {
      CBSET(decodedPlayLed, LOW);  // decode as off
    }
  }

  // Decide if we're playing or not, based on decodedPlayLed, buttons and NONSTOP:

  // If the play light just lit, we're playing.  Simple.
  if (CBROSE(decodedPlayLed)) {
    // set PLAYING.
    CBSET(playing, true);
    Dbg_println("START"); // TODO move to event?
    decodedPlayOffTime = playPinTimer_us;
    playPinTimer_us = 0;
    /* Dg_print("play led off for "); */
    /* Dbg_print(decodedPlayOffTime); */
    /* Dbg_print(" during meaure of "); */
    /* Dbg_print(hc.measureLen); */
    /* Dbg_println("us"); */
  }

  // otherwise, if the play light just unlit,
  else if (CBFELL(decodedPlayLed)) {
    decodedPlayOnTime = playPinTimer_us;
    playPinTimer_us = 0;
    /* Dbg_print("play led on for "); */
    /* Dbg_print(decodedPlayOnTime); */
    /* Dbg_print(" during meaure of "); */
    /* Dbg_print(hc.measureLen); */
    /* Dbg_println("us"); */
  }

  // otherwise, if the light has been off for a while, and we're not NONSTOPping
  else if (playing[0] && !decodedPlayLed[0] && !nonstop[0]) {
    // We can read the Play LED volts from the connector, to divine the state of play.
    // However, animation of the Play LED varies a lot(!) between PO models.
    // So how long we wait before stopping depends on which model we've detected.
    // Here's a simplified overview:

    // The po-1x series are very simple, The play LED is "lit" (actually PWM) whenever playing,
    // but unfortunately it unlights during play if you press BPM or Pattern,
    // which one does when changing pattern or volume while playing.
    // So we wait for the LED to be unlit for a full 8 pulses/ 16beats in this case.

    // The Speak & Tonic are lit during play, but blink off every 4 beats,
    // and also flicker at other random times, related to volume of the output.
    // OTOH BPM and Pattern do not unlight it.  That's nice.
    // So we only have wait for the LED to be off for longer than length of one of those off blinks.

    // The KO is basically the inverse of that: the LED is only lit briefly every 2 pulses/4 beats.
    // The office & maybe other PO-2X series are like that too.

    switch (poModelGroup) {
      case MGRP_AUTO:
      case MGRP_A:
        // has to be off for more than 8 pulses/16 beats
        if (playPinTimer_us > (81 * hc.measureLen) / 10) {
          CBSET(playing, false);
          Dbg_println("STOP grp A");
        }
        break;
      case MGRP_B:
        // has to be off for a bit more than 2 pulses/4 beats
        if (playPinTimer_us > (21 * hc.measureLen) / 10) {
          CBSET(playing, false);
          Dbg_println("STOP grp B");
        }
        break;
      case MGRP_C:
        // has to be off for more than the length of a wink ... 100ms?
        if (playPinTimer_us > (100 * 1000)) {
          CBSET(playing, false);
          Dbg_println("STOP grp C");
        }
        break;
    }

    if (CBFELL(playing)) {
      Dbg_println("STOP");
      decodedPlayOnTime = decodedPlayOffTime = 0;  // reset to "not measured yet" state
    }
  }

  // waiting until play_led detection is more reliable ...
  /* // can we detect the PO model group from play LED behavior yet? */
  /* if ((poModelGroup == MGRP_AUTO) && playing[0]) { */
  /*   if (decodedPlayOnTime > 2 * hc.measureLen) {  // only group A keeps the play light on this long without blinking it. */
  /*     poModelGroup = MGRP_A; */
  /*     Dbg_println("########## Detected Model Group A"); */
  /*   } */
  /*   else if (0 < decodedPlayOnTime < 80  // group B never lights the led longer than 60us */
  /*       && ((decodedPlayOnTime + decodedPlayOffTime) >> 3 == (2*hc.measureLen) >> 3)) { // right shifting make these match within a ballpark of +- 12% */
  /*     poModelGroup = MGRP_B; */
  /*     Dbg_println("########## Detected Model Group B"); */
  /*   } */
  /*   else if (0 < decodedPlayOffTime < 80  // group B never turns off the led longer than 60us */
  /*       && ((decodedPlayOnTime + decodedPlayOffTime) >> 3 == (2*hc.measureLen) >> 3)) { // right shifting make these match within a ballpark of +- 12% */
  /*     poModelGroup = MGRP_C; */
  /*     Dbg_println("########## Detected Model Group C"); */
  /*   } */
  /* } */

  if (btn3pressed && btn3.fell()) {  // if NONSTOP button pressed,
    if (nonstop[0]) {
      CBSET(nonstop, false);
      if (!decodedPlayLed[0]) {// if PO not currently playing,
      	CBSET(playing, false); // stop when nonstop is deactivated.
      }
      // get it on the next loop
    } else {
      CBSET(nonstop, true);
    }
  }

  if (CBROSE(playing)) {
		// play has started.
		handlePlayStart(nowTime);

  } else if (CBFELL(playing)) {
    // play has ended.
		handlePlayEnd(nowTime);
  }
}

// Check the buttons and the gyro output,
// and see if the user has changed the beat.
// RATE: this should be called whenever TAPPED changes or the buttons are read.
void checkTaps(unsigned long nowTime){
  static unsigned long tapInterval = 0;
  // Check the buttons:
  //
  if (EITHERPRESSED) { // either btn1 or btn2
    if (TAPPED) {

      // Adjust position of downbeat based on tap/shake:

      // If the next beat is closer than the previous beat
      // (dbT is greater than nowTime, but by less than mL/2))
      // or if we're in the midst of a pulse (dbT is less than nT)
      // ((dbT - (mL/2)) < nT < dbT)
      // Advance downbeat to NOW!
      // otherwise, when the previous beat is still closer (dbT > nt + (mL/2),
      // Retard downbeat to now + hc.measureLen

      if (BOTHPRESSED) {
        hc.downbeatTime = nowTime;
      } else if (hc.downbeatTime > (nowTime + (hc.measureLen / 2))) {
        hc.downbeatTime = nowTime + hc.measureLen;
      } else {
        hc.downbeatTime = nowTime;
      }

      if (++hc.tapCount > 1) {
        // Also adjust the measure length to the time between taps:
        tapInterval = nowTime - hc.lastTapTime;

        if (EITHERNOTPRESSED) {
          // if tapInterval is closer to mL*2 than to mL,
          // assume we are tapping half-time (1/4 notes)
          if (tapInterval > (1.5 * hc.measureLen)) {
            tapInterval /= 2;
          }
          // TODO: have to allow quarter-time & 1/8 time as well ...
          // and in single-button mode there should be a limit to how much the tempo can change,
          // which this section also enforces ...  so some percentage max change in single-button mode, say 25%
#define SINGLEMAXCHANGE 0.25  // 25%
                              // so the window for normal time is measurelen*(1-SINGLEMAXCHANGE) < tapInterval < measurelen*(1 + SINGLEMAXCHANGE),
                              // for half time : measurelen*(2-SINGLEMAXCHANGE) < tapInterval < measurelen*(2 + SINGLEMAXCHANGE),
                              // for quarter time : measurelen*(4-SINGLEMAXCHANGE) < tapInterval < measurelen*(4 + SINGLEMAXCHANGE),
        }
        // Othwerwise assume full time (1/8 notes)

        if (tapInterval < minTapInterval) {
          // Ignore spurious double-taps!  (This enforces a max tempo.)
          Dbg_print(tapInterval);
          Dbg_println(": ignoring bounce");
          hc.tapCount--;
        } else {
          hc.tapIntervals.unshift(tapInterval);

          if (hc.tapCount > 1) {
            // average up all taps in this set.
            // TODO: how we detect if the differences in length between taps are all in one direction,
            // due to the user accellerating/decellerating,
            // and average differently in that case?
            // test more & see if it's an issue ...
            unsigned long tcAvg = 0;
            for (int i = 0;
                 (i < (hc.tapCount - 1)) && (i < TAPBUFLEN);
                 i++) {
              tcAvg += hc.tapIntervals[i];
            }

            if (hc.tapCount > 2)
              hc.measureLen = tcAvg / min(hc.tapCount - 1, TAPBUFLEN);
            else
              hc.measureLen = tcAvg;

          }

          // but there has to be a minimum meaure length.
          if (hc.measureLen < (pulseLen * 2)) {
            hc.measureLen = pulseLen * 2;
          }
        }
      } else {
        tapInterval = 0;
      }

      hc.lastTapTime = nowTime;

    } else {
      // armed but not tapping ...
    }

  } else { 
		// neither btn1 or btn2 is pressed ...
    if (btn1.rose() || btn2.rose()) {          // if we just released a button

			// if taps <= 8 (intervals <= 7), quantize BPM
			if (2 <= hc.tapCount && hc.tapCount <= 8) {
				// But when BPM gets "too low", quantization is probably inappropriate.
				// For now, 30 BPM is the arbitrary threshhold
				// TODO: experiment with this.
				if (USEC2BPM(hc.measureLen) > 30) {
					hc.measureLen = BPM2USEC(round(USEC2BPM(hc.measureLen)));
					Dbg_printf("tapcount %d, ", hc.tapCount);
					Dbg_print("quantized to ");          // DEBUG
					Dbg_print(USEC2BPM(hc.measureLen));  // DEBUG
					Dbg_println(" BPM");                 // DEBUG
				}
			}

			_settings.s.measureLen = hc.measureLen;

			// TODO: for RP2040 this is sort of a hack, flash is not EEPROM & will wear out flash too fast 
			// if I put these settings as often as I have been. (atm, on every button release!)
			// We should have some other strategy for saving the settings less often, but often enough:
			// mark settings as dirty somehow,
			// save, if dirty, on some time interval and/or at stop.
      _settings.put();
		}

    // not armed.
    hc.tapCount = 0;
  }
}

///////////
// Calculate & update LEDs 1-3, including strobing of 1 & 2
//
// Important time intervals, in uS:
// length of a LED strobe:
#define strobeOnLen 3000
// length of a LED antistrobe (a moment of darkness):
#define strobeOffLen 25000
// (Strobe-off intervals, in which we briefly unlight a lit LED,
// need to be much longer than strobe-on intervals.)

void updateLEDs(unsigned long nowTime){
  if (playing[0]) {
    if (btn1pressed) {
      if (nowTime - hc.downbeatTime < strobeOffLen)  
      {
				leds[1].off();
      } else {
				leds[1].on();
      }
    } else {
      if (nowTime - hc.downbeatTime < strobeOnLen) {
				leds[1].on();
      } else {
				leds[1].off();
      }
    }

    if (btn2pressed) {
      if (nowTime - hc.downbeatTime < strobeOffLen) {
				leds[2].off();
      } else {
				leds[2].on();
      }
    } else {
      if (nowTime - hc.downbeatTime < strobeOnLen) {
				leds[2].on();
      } else {
				leds[2].off();
      }
    }
  } else {
		leds[1].set(btn1pressed);
		leds[2].set(btn2pressed);
  }

  if (!nonstop[0]) {
    if (CBDIFF(nonstop)) {
			leds[3].off();

    }
  } else {  // nonstop!
		leds[3].on();
  }
#ifdef NONSTOP_HACK
	leds[3].update(); // TODO: move this to some less tight loop?
#endif
}

//////////
// Calculate & update the sync pulse
//
void updateSync(unsigned long nowTime){
  if (BOTHPRESSED) {
    if (TAPPED) {
      CBSET(pulseState, HIGH);
    } else if (nowTime - hc.downbeatTime >= pulseLen) {
      CBSET(pulseState, LOW);
    }
  } else {
    if (nowTime - hc.downbeatTime < pulseLen) {
      CBSET(pulseState, HIGH);
    } else if (nowTime - hc.downbeatTime >= pulseLen) {
      CBSET(pulseState, LOW);
    }
  }
  // Pulse if PLAYING
  if (CBDIFF(pulseState)) {
    if (playing[0]) {
#ifdef TARGET_RP2040
      // send sync pulse on whatever pins are configured for sync
			for(int i=0;i<4;i++)
				if (_settings.s.outs[i] == OUTMODE_SYNC)
					digitalWrite(outPins[i], pulseState[0]);
#else // Teensy 3.2:
			// send sync pulse on tip1 & tip2
			digitalWrite(tip1, pulseState[0]);
			digitalWrite(tip2, pulseState[0]);
#endif
    }
	}
}

//////////
// Update the circular clock:
// User can make discontinuous changes in the human clock;
// the circular clock attempts to sync up with the human clock,
// but it never skips or moves backwards, only slows down or speeds up.
// (It still gets incrememented discretely, but the increments can never
// be larger than the width of a single MIDI clock interval.)
int midiMeasuresRemaining = 0;
void updateCClock(unsigned long nowTime){
  long measureIdx;
  long instantPos;
  long circleOffset;
  bool targetNear, targetAhead;

  cc.prevCirclePos = cc.circlePos;

  // the instantaneous position of our moment in the current measure is:
  measureIdx = hc.measureLen - (hc.downbeatTime - nowTime);

  // Expressed as an int btwn 0 & 1B:
  instantPos = (CC_INT_RES / 100) * ((100 * measureIdx) / hc.measureLen);

  // the difference between that position & the current circular position:
  circleOffset = abs(cc.circlePos - instantPos);

  // Freeze the circular clock when both buttons pressed & end of MMR reached;
  // unfreeze on the next pulse after a button is released.
  // also unfreeze for 1 measureLen if tapped while bothbuttons pressed

  if (BOTHPRESSED) {
    if (btn1.fell() || btn2.fell()) {  // If we just pressed now entered 2-button mode
      midiMeasuresRemaining = 1;       // midi only to the end of this measure.
    } else if (TAPPED) {               // but if we tapped a beat,
      midiMeasuresRemaining++;         // add a measure to that,
                                       // then unfreeze for just that one measure:
      if (cc.frozen) {
        cc.frozen = false;
#ifdef MIDICLOCK
				myMidi.clockContinue();
#endif
        /* #ifdef MIDITIMECODE */
        // TODO: Shifting the downbeat (without chaning tempo) in MTC mode ...
        /* 			mtc.sendStamp(); */
        /* #endif */
      }
    }
    if (midiMeasuresRemaining == 0) {
      if (!cc.frozen) {
        cc.frozen = true;
#ifdef MIDICLOCK
				myMidi.clockStop();
#endif
      }
    }
  } else if (CBROSE(pulseState)) {
    if (cc.frozen) {
      cc.frozen = false;
#ifdef MIDICLOCK
			myMidi.clockContinue();
#endif

      /* #ifdef MIDITIMECODE */
      /* 			mtc.sendStamp(); */
      /* #endif */
    }
  }

  // Is the cc ahead of the hc & needing to slow down, or behind it & needing to speed up?
  targetNear = circleOffset < (CC_INT_RES / 2);
  targetAhead = (instantPos > cc.circlePos);
  if (!cc.frozen) {
    if ((targetNear && targetAhead) || (!targetNear && !targetAhead)) {
      // speed up!
      cc.circlePos += min(circleOffset, circleMaxFwd);
    } else {
      // slow down!
      cc.circlePos += circleMinFwd;
    }
  }

#ifdef MIDICLOCK
  // if we've crossed a clock boundary, send a MIDI clock.
  if (((cc.circlePos + circleTicksPerClock) / circleTicksPerClock) > ((cc.prevCirclePos + circleTicksPerClock) / circleTicksPerClock)) {
    // debugging noise:
    // send a triangle pulse to audio out
    // dc1.amplitude(1.0 - (cc.circlePos/2)); // DEBUG
    // dc1.amplitude(0, 1); // DEBUG

    if (playing[0]) {
      if (!cc.frozen) {
        // emit MIDI clock!
				myMidi.clockTick();

        /* if (TAPPED) {  */
        /* 	Dbg_print("TMC "); */
        /* } else {  */
        /* 	Dbg_print("MC "); */
        /* } */

        /* Dbg_print((int)(cc.circlePos * MIDICLOCKSPERPULSE)); */
        /* Dbg_print(cc.circlePos, 7); */
        /* Dbg_print(">"); */
        /* Dbg_print(cc.prevCirclePos, 7); */

        /* Dbg_print(", "); */
        /* if (cc.circlePos >= 1.0) { */
        /* 	Dbg_println("."); */
        /* } */
      }
    }
  }
#endif
#ifdef MIDITIMECODE

  // if we've crossed a frame boundary, increment the frame.
  // TODO INTWISE ...
  if (((cc.circlePos + circleTicksPerFrame) / circleTicksPerFrame) > ((cc.prevCirclePos + circleTicksPerFrame) / circleTicksPerFrame)) {
    if (playing[0]) {
      if (!cc.frozen) {
        mtc.incFrame();
      }
    }
  }
#endif

  // wrap!
  if (cc.circlePos > CC_INT_RES) {
    cc.circlePos -= CC_INT_RES;
    cc.prevCirclePos -= CC_INT_RES;
    if (BOTHPRESSED) {
      if (midiMeasuresRemaining > 0)
        midiMeasuresRemaining--;
    }
  }
}

// Print various benchmarks/stats/debuggery on one line.
void printStats(){
    if (showStats) {
      Dbg_print(awakeTimer_ms);
      Dbg_print(':');
      Dbg_print(loopTimer_us);
      Dbg_print(':');

      if (awakePinState >= awakePinThreshold) {
        Dbg_print(playing[0] ? (decodedPlayLed[0] ? "play" : "(play)")
                             : "stop");
        Dbg_print(" wv@");
        Dbg_print(awakePinState);
        Dbg_print(nonstop[0] ? " NONSTOP " : "--->");
      } else {
        Dbg_print("asleep, wv@");
        Dbg_print(awakePinState);
      }
      Dbg_print(", ");
      /* Dbg_print(loops); */
      /* Dbg_print(" loops, "); */
      /* Dbg_print(imus); */
      /* Dbg_print(" IMUs at "); */
      Dbg_print((float)loops / (float)imus);
      Dbg_print(" loops/IMU @ ");
      Dbg_print(USEC2BPM(hc.measureLen));
      Dbg_print(" BPM, ");
#ifdef TEENSY32
      Dbg_print(AudioMemoryUsageMax());
      Dbg_print(" audioMem, ");
      Dbg_print(AudioProcessorUsageMax());
      Dbg_print(" audioCPU, ");
#endif
      Dbg_print(midiMeasuresRemaining);
      Dbg_print(" MMR, accel ");
      /* Dbg_print(instantPos); */
      /* Dbg_print(" instant, "); */
      /* Dbg_print(cc.circlePos); */
      /* Dbg_print(" circular, "); */
      Dbg_print(ax);
      Dbg_print(", ");
      Dbg_print(ay);
      Dbg_print(", ");
      Dbg_print(az);
			Dbg_print("; vol ");
			Dbg_print(volumeLevel);
      //		if (btn3pressed)  // DEBUG */
      //			Dbg_print(" Boink!"); // DEBUG */

      Dbg_println(".");

			PROFILE_PRINT_AVG();

#ifdef SDEBUG
      // reset counters
      loops = imus = 0;
#endif
    }
}


#ifdef AUDIO_RP2040

// rp2040 audio on core 1:
void setup1(){

	// put some noise in the buffer:
	audio.fillWithNoise();

	// Setup PWM outputs & interrupt handler
	audio.init();

  // Start DMA-ing audio from transfer buffer to PWM pins.
  audio.play(0);
  audio.play(1);
}

volatile uint32_t iVolumeLevel;

void loop1(){
	static short reportcount = 1500; 

	// update the interpolater with the latest volume level
	iVolumeLevel = rp2040.fifo.pop(); // this blocks at IMU interrupt rate

	reportcount--;
	if (reportcount ==0){
		reportcount = 1500;
		//Dbg_printf("iVol: %d\n", iVolumeLevel);
		/* Serial.printf("unscaled: %d\n",interp0->base[1]); */
		/* Serial.printf("scale: %d\n",interp0->accum[1]); */
		/* Serial.printf("lane1 result: %d\n",interp0->peek[1]); */
	}

	//audio.tweak(); // only for testing/tuning
}

#endif


void loop() {

  unsigned long loopStartTime = loopTimer_us;
	static unsigned long lastNap = loopStartTime;

	/* PROFILE_START_LOOP(); */
	PROFILE_MARK_START("ALL");

#ifdef MCU_RP2040
	// feed kibble to watchdog
	rp2040.wdt_reset();
#endif


	// check if PO has gone to sleep:
  awakePinState = analogRead(PO_wake);
#ifdef TAKE_NAPS
  // go to sleep if the PO_wake pin is low:
	//if (btn1pressed) { // DEBUG
	/* TODO: This ain't sleeping ever. Why not? did i calculate MIN_AWAKE wrong? do i understand static vars? */
  if ((awakePinState < awakePinThreshold) && ((loopStartTime - lastNap) > MIN_AWAKE_US))  {
		Dbg_println("zzzzz");
    unsigned long napDuration = powerNap();
    //unsigned long napDuration = 666 // DEBUG 
		lastNap = loopTimer_us; // note the time, so we don't sleep again too soon
		Dbg_printf("slept %d us.\n", napDuration);
    return;
  }
#endif

  // shift outer loop states:
	PROFILE_MARK_START("bufscoot");
  CBNEXT(blinkState);
  CBNEXT(decodedPlayLed);
  CBNEXT(nonstop);
  CBNEXT(playing);
  CBNEXT(pulseState);
	// I'm really thinking this should happen on every loop not every IMU, but i'm not fixing what's not broken (yet)
	/* CBNEXT(inertia); */
	PROFILE_MARK_END("bufscoot");

#ifdef SDEBUG
  // count loop rate:
  loops++;
#endif

	// read the buttons
	readButtons();

  // Read IMU data if ready:
  if (imu_ready) {  // on interrupt
    imu_ready = false;
    // IMU inner loop runs once per IMU interrupt
		volumeLevel = processIMU(ax,ay,az,gx,gy,gz);
	}
	
#ifdef MIDITIMECODE
  mtc.frameCheck();
#endif

  // Decode the state of PO Play from the LED signal:
	decodePlayState(loopStartTime);

  // downbeatTime is the absolute time of the start of either the previous or next beat (rising edge of sync pulse).
	// if now is at least a tenth of a second beyond the previous beat, advance it to the next beat.
  while (loopStartTime > (hc.downbeatTime + 100000)) {
    hc.downbeatTime += hc.measureLen;
  }

	// check buttons and taps for beat changes:
	checkTaps(loopStartTime);

#ifdef BUTTON4
	// Manage button 4 behavior: reboots, mode changes, etc.
	if (btn4.fell()){
		leds[4].on();
		resetTimer_ms = 0;
	}
	if (btn4.rose()){
#ifdef   MCU_RP2040
		rp2040.reboot();
#endif  // MCU_RP2040
	}
	// if reset is held down for 3 secs, boot in USB bootloader mode
	if (btn4pressed && (resetTimer_ms > 3000)) {

#ifdef   MCU_RP2040
		// TODO/TOTRY: if we install ISR on btn4 here, will it remain connected after reset?
		// if so, we can use that to reset out of bootloader mode ...
		reset_usb_boot(1 << led4Pin, 0);

#else
		// just turn off the led:
		leds[4].off();
#endif  // MCU_RP2040
	}

#endif// BUTTON4

	// update LEDs
	updateLEDs(loopStartTime);

	// update sync pulse
	updateSync(loopStartTime);

	// update circular clock
	updateCClock(loopStartTime);

	if (midiFlushTimer_ms > MIDIFLUSHTIME) {
		// flush/ignore MIDI input (often but not obessively)
		flushMIDI(myMidi);
		midiFlushTimer_ms -= MIDIFLUSHTIME;
	}

	if (statsTimer_ms > STATSTIME){
		// print debug stats if enabled
		printStats();
		statsTimer_ms -= STATSTIME;
	}

	if (cmdTimer_ms > CMDTIME){
		// read/handle serial commands
		cmd_update();
		cmdTimer_ms -= CMDTIME;
	}

  // Protect against timer overflows
  // We could do this only every 1000 (or more!) measures,
  // but we choose every 8,	so that if it causes any audible bug, it'll be heard often!
	// Basic testing suggests overflows hang the clock somehow ... maybe that's curable, but this is simple & works.
  if (loopTimer_us > 9 * hc.measureLen) {
    //Dbg_println("PROTECTION!");//DEBUG
    loopTimer_us -= 8 * hc.measureLen;
    hc.downbeatTime -= 8 * hc.measureLen;
    hc.lastTapTime -= 8 * hc.measureLen;
  }

	PROFILE_MARK_END("ALL");
}



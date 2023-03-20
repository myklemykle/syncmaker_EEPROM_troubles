//////////////////////////// 
// Pocket Integrator (PO daughterboard) firmware (c) 2022-2023 MSL
// This version compatible with EVT4 based on Teensy 3.2, or V6+ boards based on RP2040
// (You must #define either EVT4 or PI_V6 ; define neither both nor neither.)
//
#include "config.h"
#include "pins.h"
#include "settings.h"
#include <Arduino.h>
#include <elapsedMillis.h>
#include <CircularBuffer.h>
#include "sleep.h"
#include "Bounce2.h"
#include <EEPROM.h>

#ifdef PI_V6 // rp2040 versions V6, V7, V8
///////////////
#include "hardware/interp.h"
// ST IMU
#include "lsm6dso32x.h"
// Adafruit TinyUSB w/midi:
#include <MIDI.h>
#include <Adafruit_TinyUSB.h>
// rp2040 audio
#include "RP2040Audio.h"
#include "hardware/pwm.h"
///////////////
#else // teensy version EVT4 & earlier
// TDK IMU
#include "MotionSense.h"
// Teensy audio!
#include <Audio.h>
///////////////
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

// for controlling the test tone:
char testTone = TESTTONE_OFF;
float testLevel = 1.0;

/////////////////////////////
// Some utils for handling loop variables, which are very short CircularBuffers
// for comparing this loop's value to the previous loop's value.
// The most recent value will be in cb[0], the previous in cb[1], etc.
////////////////////////////
// init the buffer with one value in all positions:
#define CBINIT(cb, val) \
  while (!cb.isFull()) cb.push(val)

// change the current value without touching the previous value:
#define CBSET(cb, val) \
  { \
    cb.shift(); \
    cb.unshift(val); \
  }

// update the value (current becomes previous)
#define CBPUSH(cb, val) (cb.unshift(val))

// update the current value to match the previous value
#define CBNEXT(cb) (cb.unshift(cb[0]))

// get the current value
#define CBGET(cb, val) (cb[0])

// test: current value is less than previous
#define CBFELL(cb) (cb[0] < cb[1])  // also works for bools in C++ because TRUE = 1 and FALSE = 0

// test: current value is below val, and previous value was above it
#define CBFELLTHRU(cb, val) (cb[0] <= val && cb[1] > val)

// test: current value is more than previous
#define CBROSE(cb) (cb[0] > cb[1])

// test: current value is above val, and previous value was below it
#define CBROSETHRU(cb, val) (cb[0] > val && cb[1] <= val)

// test: value changed (current != previous) 
#define CBDIFF(cb) (cb[0] != cb[1])


#ifdef PI_V6

// USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the (generic) Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
// (jeesuz what a hairy-looking macro to do something pretty basic)
// (what is this MIDI now? what type? what symbol? this is just weird.)

#else

// usbMIDI -- defined automatically by Teensy libs.

#endif

// IMU Gesture Detection:
#ifdef PI_V6
LSM6DSO32X_IMU imu;  // STMicro IMU used from v6 onward
#else
MotionSense imu;  // on EVT1, rev2 & rev3 & evt4 the IMU is an ICM42605 MEMS acc/gyro chip
#endif

#define COUNT_PER_G 4096                    // accelerometer units
#define COUNT_PER_DEG_PER_SEC_PER_COUNT 16  // gyro units
//const int shakeThreshold = (COUNT_PER_G * 100) / 65 ; 			// 0.65 * COUNT_PER_G
const int shakeThreshold = (COUNT_PER_G * 100) / 80;  // 0.85 * COUNT_PER_G ... a little less sensitive
const int tapThreshold = 2 * COUNT_PER_G;

CircularBuffer<long, 3> inertia;
// shaken should be the start of sound moment,
#define SHAKEN (CBROSETHRU(inertia, shakeThreshold))
// but try putting tapped at the apex-G moment; it may have better feel.
//#define TAPPED (CBROSETHRU(inertia, tapThreshold))
#define TAPPED ((inertia[0] > tapThreshold) && (inertia[0] < inertia[1]) && (inertia[1] >= inertia[2])) // when Gs just starting to drop


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

#ifdef NONSTOP_HACK
// bit-banging PWM, to dim the nonstop LED on EVT4 boards:
unsigned long nonstopLedPWMClock;
bool nonstopLedPWMState = false;
#define NSLEDPWM_ON 300    // usec
#define NSLEDPWM_OFF 5000  // usec
#endif

#ifdef PI_V6
const int pwmBrightness = 8; // out of 256? looks okay ...
#endif

#define BOTHPRESSED (btn1pressed && btn2pressed)
#define EITHERPRESSED (btn1pressed || btn2pressed)
#define EITHERNOTPRESSED (!BOTHPRESSED)

#ifdef PI_V6
// reset button
Bounce btn4 = Bounce();
bool btn4pressed = false;
#endif

// Important time intervals, in uS:
const unsigned long strobeOnLen = 500; 

const unsigned long strobeOffLen = 100 * 1000;
const unsigned long pulseLen = 5 * 1000;
const unsigned long minTapInterval = 100 * 1000;  // Ignore spurious double-taps!  (This enforces a max tempo.)

// duration of the off-cycle of the Play button PWM wave
const unsigned long pwmOffTime = 150 * 1000;
// But this should really be just 1ms ... once we fix measurement.

// convert a time interval between beats to BPM:
// float: convert usecs to secs (1M to 1),
// divide by 60 secs per minute
//#define USEC2BPM(interval) ( 60.0 / ((interval / 1000000.0) ) )
#define USEC2BPM(interval) (60000000.0 / interval)  // same thing.  gives float result
#define BPM2USEC(bpm) (60000000 / bpm)              // inverse.  gives float result if passed a float!

// Timers:
elapsedMicros loopTimer;
elapsedMicros playPinTimer;
elapsedMicros pwmOffTimer;
elapsedMicros awakeTimer;


#ifdef IMU_8KHZ
const int imuClockTick = 125;  // 8khz data rate
#elif defined(IMU_1_666KHZ)
const int imuClockTick = 600;  // 8khz data rate
#else
const int imuClockTick = 500;  // 2khz data rate
#endif


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
// but is limited in how fast it can speed/slow,
// and cannot move backwards.
// So relatively more "continuous" than the Human Clock, but still updated discretely.
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
CircularBuffer<bool, 2> led1State;
CircularBuffer<bool, 2> led2State;
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

boolean showStats = true;
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

void setup() {
#ifdef PI_V6

	  // fix startup weirdness
  rp2040.idleOtherCore();

	  // turn on this helpful developer feature
  /* rp2040.enableDoubleResetBootloader(); */
#endif

	// get settings:
#ifdef PI_V6
	EEPROM.begin(256); // necessary for the rp2040 EEPROM emulation in Flash
#endif
	if (! _settings.get()) {
		_settings.init();
	}

  ///////
  // Pin setup:
  //
  pinMode(led1Pin, OUTPUT);           // led1
  pinMode(led2Pin, OUTPUT);           // led2
  pinMode(tip1, OUTPUT);              // j1 tip
  pinMode(tip2, OUTPUT);              // j2 tip
#ifndef TEENSY32
	// teensy has both of these connected to the DAC
  pinMode(ring1, OUTPUT);              // j1 ring
  pinMode(ring2, OUTPUT);              // j2 ring
#endif
  pinMode(button1Pin, INPUT_PULLUP);  // sw1
  pinMode(button2Pin, INPUT_PULLUP);  // sw2
  pinMode(button3Pin, INPUT_PULLUP);  // nonstop
  pinMode(nonstopLedPin, OUTPUT);     // nonstop led

#ifdef PI_V6
	/////////////
	// configure our output jacks. (2x stereo = 4 channels)
	// They can be audio (PWM), sync (digital outputs), or paired as MIDI uarts.
	// TODO: handle the midi option
	for(int i=0;i<4;i++)
		//gpio_set_function(outPins[i], (_settings.s.outs[i] == OUTMODE_SHAKE ) ? GPIO_FUNC_PWM : GPIO_FUNC_NULL );
		gpio_set_function(outPins[i], OUTMODE_IS_AUDIO(_settings.s.outs[i]) ? GPIO_FUNC_PWM : GPIO_FUNC_NULL );
#endif
	

	// TODO: why is this even here?
#ifdef PI_V6
	analogWrite(nonstopLedPin, 0);
#else
  digitalWrite(nonstopLedPin, LOW);
#endif

#ifdef NONSTOP_HACK
  nonstopLedPWMClock = loopTimer;
#endif

  pinMode(boardLedPin, OUTPUT);  // nonstop led

  pinMode(PO_play, INPUT);  // not sure if PULLUP helps here or not?  Flickers on & off anyway ...
  pinMode(PO_wake, INPUT);
  pinMode(PO_reset, INPUT_PULLUP);  // TODO: check v6: do we still get resets when physically connecting the boards? V6 added a pullup here ...
  pinMode(PO_SWCLK, INPUT_PULLUP);
  pinMode(PO_SWDIO, INPUT_PULLUP);
  pinMode(PO_SWO, INPUT_PULLUP);

#ifdef PI_V6
	/////// CMOS chips must stabilize all unconnected pins, to avoid static glitches
	int i;
	for (i=0; i < std::size(unusedPins); i++){
		/* Dbg_printf("disable pin %d\n", unusedPins[i]); */
		pinMode(i, INPUT_PULLUP);
	}
#endif


#ifdef PI_V6
  ///////
  // USB MIDI startup
	// 
	// WARNING: apparently MIDI.begin() needs to happen before Serial.begin(), or else it fails silently.
  //
  // Initialize MIDI, and listen to all MIDI channels:
  MIDI_USB.begin(MIDI_CHANNEL_OMNI);
#endif

	/////////////////
	// USB serial port:
  Serial.begin(115200);  // baud rate is ignored on USB serial.

	digitalWrite(led1Pin, HIGH);
	digitalWrite(led2Pin, HIGH);
  delay(2000); // waiting for USB host...
	digitalWrite(led1Pin, LOW);
	digitalWrite(led2Pin, LOW);

#ifdef PI_V6
  Dbg_println("flashed for v6 board");
#elif PI_REV == 4
  Dbg_println("flashed for EVT4 board");
#else
  Dbg_println("flashed for rev3 board");
#endif

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
	/* digitalWrite(led4Pin, HIGH);//DEBUG */
#elif defined(PI_V6)
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
  btn1.update();
  btn1pressed = (btn1.read() == LOW);

  btn2.attach(button2Pin);
  btn2.interval(debounceLen);
  btn2.update();
  btn2pressed = (btn2.read() == LOW);

  btn3.attach(button3Pin);
  btn3.interval(debounceLen);
  btn3.update();
  btn3pressed = (btn3.read() == LOW);

  ////////
  // Clock setup:
  //
  hc.downbeatTime = micros();  // now!
  // If buttons are held down when we boot, reset the default measure length
  if (BOTHPRESSED) {
    _settings.s.measureLen = hc.measureLen = 250 * 1000;  // 120bpm == 2 beats per second, @ 2 pulses per beat == 1/4 second (250ms) per pulse) */
		_settings.put();
  } else {
		hc.measureLen = _settings.s.measureLen;
  }


#ifdef MIDITIMECODE
  mtc.setup();
#endif

  loopTimer = 0;  // why?

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

  //https://forum.pjrc.com/threads/25519-Noise-on-DAC-(A14)-output-Teensy-3-1
  // Initialize the DAC output pins
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
}


#ifndef TEENSY32

// rp2040 audio:

///////////////
// Set up hardware interpolator 0 on core1 in blend mode for volume control:
/*  */
/* void interpSetup1(){ */
/* 	interp_config cfg = interp_default_config(); */
/* 	interp_config_set_blend(&cfg, true); */
/* 	interp_config_set_signed(&cfg, true); */
/* 	interp_set_config(interp0, 0, &cfg); // config lane 0 */
/*  */
/* 	cfg = interp_default_config(); */
/* 	interp_config_set_signed(&cfg, true);  */
/* 	interp_set_config(interp0, 1, &cfg); // config lane 1 */
/*  */
/* 	// Volume levels will be from int 0 to 255: */
/* 	interp0->base[0] = 0; */
/* 	// base[1] will get the sample we're scaling ... */
/* 	interp0->accum[1] = 255; // turn it all the way up for now ... */
/*  */
/* 	// TODO: use interp1 in clamp mode */
/* 	// to shift results into positive (using base2) */
/* 	// and clamp btwn 0 and WAV_PWM_RANGE (using clamp mode) */
/* 	 */
/* 	// ALSO: handle vol levels above 255? */
/*  */
/* } */

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
  static int ax, ay, az;
  static int gx, gy, gz;

  unsigned long tapInterval = 0;
  unsigned long nowTime = loopTimer;

  long instantPos;
  long circleOffset;
  long measureIdx;
  bool targetNear, targetAhead;

  static int midiMeasuresRemaining = 0;

	static float volumeLevel;


#ifdef PI_V6
  // TODO: check the resolution of the rp2040 analogRead
#else
  // go to sleep if the PO_wake pin is low:
  awakePinState = analogRead(PO_wake);

  if (awakePinState < awakePinThreshold) {
    // if (btn1pressed) { // DEBUG
    powerNap();
    return;
  }
#endif

  // shift outer loop states:
  CBNEXT(blinkState);
  CBNEXT(led1State);
  CBNEXT(led2State);
  CBNEXT(nonstop);
  CBNEXT(decodedPlayLed);
  CBNEXT(playing);
  CBNEXT(pulseState);

#ifdef SDEBUG
  // count loop rate:
  loops++;
#endif

	// check buttons:
	if (btn1.update())
		btn1pressed = (btn1.read() == LOW);
	if (btn2.update())
		btn2pressed = (btn2.read() == LOW);
	if (btn3.update())
		btn3pressed = (btn3.read() == LOW);


  // Read IMU data if ready:
  if (imu_ready) {  // on interrupt
    imu_ready = false;

    // IMU inner loop runs once per IMU interrupt, currentlyu 2000hz ...

#ifdef SDEBUG
    imus++;
#endif

    // shift inner loop state:
    CBNEXT(inertia);


    imu.readMotionSensor(ax, ay, az, gx, gy, gz);
    CBSET(inertia, (long)sqrt((ax * ax) + (ay * ay) + (az * az)));  // vector amplitude (always positive)

#ifdef SDEBUG
    /* if (inertia[0] > shakeThreshold)  */
    /* 	Dbg_println(inertia[0]); */
#endif

		// This funny formula gives a float value for volume:
		// it's the difference between momentary intertia and a minimum inertial threshhold (shakeThreshold),
		// both of which are scaled to Gs at the configured resolution of the IMU (COUNT_PER_G),
		// then divided by 3gs (/3.0 * COUNT_PER_G) , a volume-attenuating coefficient that I apparently found in my ass.

		// The result can't go below 0, but what's the max?
		// If the IMU is sensitive to 8gs, and the threshhold is 0.8 gs,
		// then the max is 7.2/3  = 2.4
		// (Testing shows i can hear an audible increase in noise volume from overamplifying up to about 3.0,
		// so that could be an ideal threshhold.)
		// (I got to this formula through tweaking and listening, but it's kinda obscure.)

		//volumeLevel = max((inertia[0] - shakeThreshold) / (3.0 * COUNT_PER_G), 0.0); // always 0 or more
		volumeLevel = max((inertia[0] - shakeThreshold) / (1.0 * COUNT_PER_G), 0.0); // more loud please!

#ifdef PI_V6
		// send vol level to core 1:
		if (testTone != TESTTONE_OFF) {
			rp2040.fifo.push_nb((uint32_t)(testLevel * (float)WAV_PWM_RANGE)); 
			//rp2040.fifo.push_nb(min(WAV_PWM_RANGE, az * WAV_PWM_RANGE / COUNT_PER_G)); //DEBUG: level adjusts with rotation
		} else {
			//rp2040.fifo.push_nb(max((inertia[0] - shakeThreshold) / (3.0 * COUNT_PER_G), 0.0) * WAV_PWM_RANGE); 
			rp2040.fifo.push_nb((uint32_t)(volumeLevel * WAV_PWM_RANGE));  // core1 saves this to iVolumeLevel
		}
#endif
#ifdef TEENSY32
    // use the inertia (minus gravity) to set the volume of the pink noise generator
		if (testTone != TESTTONE_OFF) {
			amp1.gain(testLevel); // for testing
		} else {
			//amp1.gain(max((inertia[0] - shakeThreshold) / (3.0 * COUNT_PER_G), 0.0));
			amp1.gain(volumeLevel);
		}

#endif

    ///////////////////////////////////
    // other things to be done at IMU interrupt rate (2000hz) :

    // MIDI Controllers should discard incoming MIDI messages.
#ifdef PI_V6
    while (MIDI_USB.read(MIDI_CHANNEL_OMNI)) 
			{ // read & ignore incoming messages 
			}
#else
    while (usbMIDI.read()) 
			{ // read & ignore incoming messages 
			}
#endif

  }

#ifdef MIDITIMECODE
  mtc.frameCheck();
#endif

  // Decode the state of PO Play from the LED signal:
  // This pin is low when not playing, but when playing ... it's complicated.

  if (digitalRead(PO_play) == HIGH){
    pwmOffTimer = 0;
    if (!decodedPlayLed[0]) {       // if LED is lit, and wasn't before
      CBSET(decodedPlayLed, HIGH);  // decode as on.
    }

  } else if (pwmOffTimer > pwmOffTime) {  // if unlit for longer than a PWM off cycle
    if (decodedPlayLed[0]) {
      CBSET(decodedPlayLed, LOW);  // decode as off
    }
  }

  // Decide if we're playing or not, based on decodedPlayLed, buttons and NONSTOP:

  // If the play light just lit, we're playing.  Simple.
  if (CBROSE(decodedPlayLed)) {
    // set PLAYING.
    CBSET(playing, true);
    Dbg_println("START");
    decodedPlayOffTime = playPinTimer;
    playPinTimer = 0;
    /* Dg_print("play led off for "); */
    /* Dbg_print(decodedPlayOffTime); */
    /* Dbg_print(" during meaure of "); */
    /* Dbg_print(hc.measureLen); */
    /* Dbg_println("us"); */
  }

  // otherwise, if the play light just unlit,
  else if (CBFELL(decodedPlayLed)) {
    decodedPlayOnTime = playPinTimer;
    playPinTimer = 0;
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
        if (playPinTimer > (81 * hc.measureLen) / 10) {
          CBSET(playing, false);
          Dbg_println("STOP grp A");
        }
        break;
      case MGRP_B:
        // has to be off for a bit more than 2 pulses/4 beats
        if (playPinTimer > (21 * hc.measureLen) / 10) {
          CBSET(playing, false);
          Dbg_println("STOP grp B");
        }
        break;
      case MGRP_C:
        // has to be off for more than the length of a wink ... 100ms?
        if (playPinTimer > (100 * 1000)) {
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
      	CBSET(playing, false); // stop when notstop is deactivated.
      }
      // get it on the next loop
    } else {
      CBSET(nonstop, true);
    }
  }

  /////////
  // Update the Human Clock & MIDI transport
  //
  if (CBROSE(playing)) {
    // user just pressed play button to start PO
    // move downbeat to now!
    hc.downbeatTime = nowTime;
    // sync the circular clock!
    //cc.circlePos = 0.0;
    cc.circlePos = 0;
#ifdef MIDICLOCK

    /* usbMIDI.sendRealTime(usbMIDI.Start); */
    /* usbMIDI.sendRealTime(usbMIDI.Clock); */

    /* I haven't yet found good guidance for this bit,
			 but it really appears that Live 10 treats the Start and Stop
			 messages as if they were also clock beats.  Sending Clock 
			 and Start in quick succession was making Live briefly think the tempo 
			 was super-high, leading to an initial stutter.  I changed this
			 and it now sounds much better.
			 FWIW I think there's a matching problem with Stop; we don't sync it to
			 where the MTC clock pulse would be, which is why sometimes when
			 I hit stop Live sets the tempo super high -- but it's stopped, 
			 so it's not really a problem.  Nevertheless,
			 I should find some other test cases than Live ...  */

#ifdef PI_V6
    MIDI_USB.sendStart();
#else
    usbMIDI.sendRealTime(usbMIDI.Start);
#endif

#endif
#ifdef MIDITIMECODE
    // rewind time to zero
    mtc.rewind();
#endif

  } else if (CBFELL(playing)) {
    // user just pressed play button to stop PO.

#ifdef MIDICLOCK
#ifdef PI_V6
    MIDI_USB.sendStop();
#else
    usbMIDI.sendRealTime(usbMIDI.Stop);
#endif
#endif
  }

  // downbeatTime is the absolute time of the start of the current pulse.
  // if now is at least pulseLen millis beyond the previous beat, advance the beat
  while (nowTime > (hc.downbeatTime + pulseLen)) {
    hc.downbeatTime += hc.measureLen;
  }

  // Check the buttons:
  //
  if (EITHERPRESSED) {
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
		// neither button is pressed ...
    if (btn1.rose() || btn2.rose()) {          // if we just released the buttons,

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

  ///////////
  // Calculate & update LEDs.
  // (Strobe-off times need to be much longer than strobe-on times
  // in order to be clearly visible to the human eye.)
  //
  if (playing[0]) {
    if (btn1pressed) {
      if (nowTime - hc.downbeatTime < strobeOffLen)  // there's some bug here, the interval never gets long enough.
      {
        CBSET(led1State, LOW);
      } else {
        CBSET(led1State, HIGH);
      }
    } else {
      if (nowTime - hc.downbeatTime < strobeOnLen) {
        CBSET(led1State, HIGH);
      } else {
        CBSET(led1State, LOW);
      }
    }

    if (btn2pressed) {
      if (nowTime - hc.downbeatTime < strobeOffLen) {
        CBSET(led2State, LOW);
      } else {
        CBSET(led2State, HIGH);
      }
    } else {
      if (nowTime - hc.downbeatTime < strobeOnLen) {
        CBSET(led2State, HIGH);
      } else {
        CBSET(led2State, LOW);
      }
    }
  } else {
    CBSET(led1State, btn1pressed);
    CBSET(led2State, btn2pressed);
  }


  if (CBDIFF(led1State)) {
    digitalWrite(led1Pin, led1State[0]);
  }
  if (CBDIFF(led2State)) {
    digitalWrite(led2Pin, led2State[0]);
  }

  if (!nonstop[0]) {
    if (CBDIFF(nonstop)) {
#ifdef PI_V6
      analogWrite(nonstopLedPin, 0);
#else
      digitalWrite(nonstopLedPin, LOW);
#endif

    }
  } else {  // nonstop!
#ifdef NONSTOP_HACK
    // ghetto PWM because the LED is too bright on this board ...
    // blink on & off every N usec
    if (nonstopLedPWMState) {                            // is on now
      if (nowTime - nonstopLedPWMClock > NSLEDPWM_ON) {  // on-time expired
        nonstopLedPWMClock += NSLEDPWM_ON;               // adj clock
        nonstopLedPWMState = false;                      // turn off
      }
    } else {                                              // is off now
      if (nowTime - nonstopLedPWMClock > NSLEDPWM_OFF) {  // off-time expired
        nonstopLedPWMClock += NSLEDPWM_OFF;               // adj clock
        nonstopLedPWMState = true;                        // turn on
      }
    }
    digitalWrite(nonstopLedPin, nonstopLedPWMState ? HIGH : LOW);
#elif defined(PI_V6)
    // precise dimming with PWM/analogWrite
    analogWrite(nonstopLedPin, pwmBrightness);
#else
		digitalWrite(nonstopLedPin, HIGH);
		// fix brightness in hardware, do something simple here.
		// and/or: always put LEDs on PWM pins!
#endif
  }

  //////////
  // Calculate & update the sync pulse
  //
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

  //////////
  // Update the circular clock:
  // User can make discontinuous changes in the human clock;
  // the circular clock attempts to sync up with the human clock,
  // but it never skips or moves backwards, only slows down or speeds up.
  // (It still gets incrememented discretely, but the increments can never
  // be larger than the width of a single MIDI clock interval.)

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
#ifdef PI_V6
        MIDI_USB.sendContinue();
#else
        usbMIDI.sendRealTime(usbMIDI.Continue);
#endif
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
#ifdef PI_V6
        MIDI_USB.sendStop();
#else
        usbMIDI.sendRealTime(usbMIDI.Stop);
#endif
#endif
      }
    }
  } else if (CBROSE(pulseState)) {
    if (cc.frozen) {
      cc.frozen = false;
#ifdef MIDICLOCK
#ifdef PI_V6
      MIDI_USB.sendContinue();
#else
      usbMIDI.sendRealTime(usbMIDI.Continue);
#endif
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
#ifdef PI_V6
        MIDI_USB.sendClock();
#else
        usbMIDI.sendRealTime(usbMIDI.Clock);
#endif

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

    // print benchmarks when pulse goes high.
    if (showStats && pulseState[0] == HIGH) {
      /* Dbg_printf("stats: %s  ", (showStats ? "y" : "n")); */
      Dbg_print(awakeTimer);
      Dbg_print(':');
      Dbg_print(loopTimer);
      Dbg_print(':');
#ifdef NONSTOP_HACK
      //Dbg_print(nonstopLedPWMClock);  // DEBUG
      //Dbg_print(':');  // DEBUG
#endif

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

#ifdef SDEBUG
      // reset counters
      loops = imus = 0;
#endif
    }

		// Read serial commands when pulse goes high:
		cmd_update();
  }

  // Protect against timer overflows
  // We could do this only every 1000 (or more!) measures,
  // but we choose every 8,
  //	so that if it causes any audible bug, it'll be heard often!
	// TODO: is this really necessary? for ints? 
  if (loopTimer > 9 * hc.measureLen) {
    //Dbg_println("PROTECTION!");//DEBUG
    loopTimer -= 8 * hc.measureLen;
    hc.downbeatTime -= 8 * hc.measureLen;
    hc.lastTapTime -= 8 * hc.measureLen;
#ifdef NONSTOP_HACK
    nonstopLedPWMClock -= 8 * hc.measureLen;
#endif
  }
}



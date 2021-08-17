// EVT4 (and rev3) Pocket Integrator (PO daughterboard) firmware (c) 2021 mykle systems labs
#include "config.h"

/* #define NONSTOP ijustcantstopit */

#include <CircularBuffer.h>

// IMU Gesture Detection:
#include "NXPMotionSense.h" // hacked version of this Teensy Prop Shield lib;
														// set to higher data rate, 
														// ignores other two IMU chips 
														// Should be renamed, refactored, etc.

NXPMotionSense imu;					// on EVT1, rev2 & rev3 the IMU is an ICM42605 MEMS acc/gyro chip
#define COUNT_PER_G 8192 // accelerometer units
#define COUNT_PER_DEG_PER_SEC_PER_COUNT 16 // gyro units
#define COUNT_PER_UT_COUNT 10 // compass units, not used
const int shakeThreshold = 5324;			// 0.65 * COUNT_PER_G
const int tapThreshold = 2 * COUNT_PER_G;
bool shaken = LOW, tapped = LOW;
long inertia = 0, prevInertia = 0;


// Sleep/hibernation:
#include <Snooze.h>
SnoozeDigital s_digital;
SnoozeTimer s_timer;
SnoozeBlock s_config(s_timer);
// This is the codec power supply that's regulated lower than the main supply.
// i'm reading this as 778 when on usb power & awake ... 
// then down to 300-ish when board sleeps.
const unsigned int awakePinThreshold = 400;

#include <Audio.h>
// GUItool: end automatically generated code
AudioSynthNoiseWhite     noise1;         //xy=110,301
AudioAmplifier           amp1;           //xy=231,298
//AudioSynthWaveformDc     dc1;            //xy=262,375 //DEBUG
//AudioMixer4              mixer1;         //xy=470,359
AudioOutputAnalog        dac1;           //xy=603,359 //DEBUG
AudioConnection          patchCord1(noise1, amp1);
/* AudioConnection          patchCord2(amp1, 0, mixer1, 0); */ //DEBUG
/* AudioConnection          patchCord3(dc1, 0, mixer1, 1); */ //DEBUG
/* AudioConnection          patchCord4(mixer1, dac1); */ //DEBUG
AudioConnection          patchCord2(amp1, dac1); 
// GUItool: end automatically generated code


// Buttons:
#include "Bounce2.h"
const int debounceLen = 2; // must be in MS as required by bounce2 lib
Bounce btn1 = Bounce();
Bounce btn2 = Bounce();
#ifdef EVT4
Bounce btn3 = Bounce();
#endif
#define PRESSED(btn) 				(btn.read() == LOW)
#define BOTHPRESSED(b1,b2) 	(PRESSED(b1) && PRESSED(b2))
#define EITHERPRESSED(b1,b2) 		(PRESSED(b1) || PRESSED(b2))
#define EITHERNOTPRESSED(b1,b2) 		(!PRESSED(b1) || !PRESSED(b2))


// Pins:
#ifdef EVT4
const int button1Pin = 22;   // sw1
const int button2Pin = 21;   // sw2
const int boardLedPin = 13;	// builtin led on Teensy 3.2
const int led1Pin =  15;    // led1
const int led2Pin =  8;    // led2
const int button3Pin = 0;   // nonstop
const int button3LedPin = 1; // nonstop led
const int pulsePin1 = 20; 	// j1 tip / l_sync_out
const int pulsePin2 = 23; 		// j2 tip / r_sync_out
const int PO_play = 16; 		// goes high on PO play (runs to play LED)
const int PO_wake = 14;			// goes low when PO sleeps I think
const int PO_reset = 7;			
const int PO_SWCLK = 6;			// jtag/stlink pins:
const int PO_SWDIO = 5;
const int PO_SWO   = 4;
/* const int IMU_fsync = -1;   // not connected on evt4 */
const int IMU_int = 2;
const int SPI_clock = 13;
const int SPI_cs = 10;
const int SPI_miso = 12;
const int SPI_mosi = 11;
#else
const int button1Pin = 0;   // sw1
const int button2Pin = 9;   // sw2
const int boardLedPin = 13;	// builtin led on Teensy 3.2
const int led1Pin =  21;    // led1
const int led2Pin =  20;    // led2
const int pulsePin1 = 11; 	// j1 tip
const int pulsePin2 = 8; 		// j2 tip
const int PO_play = 16; 		// goes high on PO play (runs to play LED)
const int PO_wake = 17;			// goes low when PO sleeps I think
const int PO_reset = 7;			
const int PO_SWCLK = 6;			// jtag/stlink pins:
const int PO_SWDIO = 5;
const int PO_SWO   = 4;
const int IMU_fsync = 1;   // need to ground this on icm-42605
const int IMU_int = 2;
#endif



// Important time intervals:
const unsigned int TIMESCALE = 1000; // uS
const unsigned long strobeOnLen = 500;

const unsigned long strobeOffLen = 100 * TIMESCALE;
const unsigned long pulseLen = 5 * TIMESCALE; 		
const unsigned long minTapInterval = 100 * TIMESCALE;  // Ignore spurious double-taps!  (This enforces a max tempo.)
/* const unsigned long playFlickerTime = 100 * TIMESCALE;  */

//#define USEC2BPM(interval) ( 60.0 / ((interval / 1000000.0) * 2.0 ) )  // float: convert usecs to secs (1M to 1), pulses to beats (2 to 1), divide by 60 secs per minute
#define USEC2BPM(interval) ( 30000000.0 / interval )  									// same thing
#define BPM2USEC(bpm) ( 30000000.0 / bpm  ) 															// inverse

// Timers:
elapsedMicros loopTimer;
elapsedMicros playPinTimer;
elapsedMicros awakeTimer;


#ifdef IMU_INTERRUPTS
volatile bool imu_ready = false;
#else
elapsedMicros imuClock;

#ifdef IMU_8KHZ
const int imuClockTick = 125; // 8khz data rate 
#else
const int imuClockTick = 500; // 2khz data rate
#endif

#endif



// EEPROM
#include <EEPROM.h>

// only for rev1 historical reasons is this number not 0:
const int eepromBase = 2000;


// Human Clock:
// a regular series of downbeats, seperated by a time interval (measureLen).
// User button/tap gestures can change the interval or the location of the downbeat.
typedef struct {
	unsigned long measureLen;
	unsigned long downbeatTime;
	unsigned int tapCount = 0;
	unsigned long lastTapTime = 0;        
	CircularBuffer<long, 3> tapIntervals; // for running avg of previous tap intervals, to update measureLen
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
#define CC_FLOAT_RES 1000000000.0
//
// This is the very minimum (per loop cycle) forward advance -- one millionth of a loop
const	int circleMinFwd = CC_INT_RES / 1000000;  
// OTOH, if we advance any faster than this, we could skip a MIDI clock beat
const int circleMaxFwd = CC_INT_RES / 12; // 83333333
//
typedef struct {
	// Clock position is an int value between 0 and 1B.
	long circlePos = 0, prevCirclePos = 0;
	// Freeze when both buttons pressed & end of MMR reached;
	// unfreeze on the next downbeat after a button is released.
	bool frozen = false;
} CircularClock;
CircularClock cc;

#ifdef MIDICLOCK
	// Midi Clock msgs are supposed to be sent 24 times per beat.
	// Human Clock sends 1 sync pulse per 2 PO beats, 
	// so we should send 12 midi clocks per sync pulse
#define MIDICLOCKSPERPULSE 12.0
const long circleTicksPerClock = CC_INT_RES / MIDICLOCKSPERPULSE;
#endif

#ifdef MIDITIMECODE
	// How often to increment the MTC frame, (1/25 second, the minimum resolution of midi timecode)?
	// if we declare that 1 second of MTC equals 1 second of PO time at 120bpm, so 2 "beats" per second,
	// which is actually (when you set PO tempo to 120) one entire 16-note pattern per second, 
	// which is 8 sync pulses per second, spread over 25 frames.  
	// 25/8 = 3.125
#define MIDIFRAMESPERPULSE 3.125
const long circleTicksPerFrame = CC_INT_RES / MIDIFRAMESPERPULSE ;
MidiTimecodeGenerator mtc;
#endif

// State Variables:
bool led1State = LOW, newLed1State = LOW;
bool led2State = LOW, newLed2State = LOW;
bool blinkState = LOW, newBlinkState = LOW;
bool pulseState = LOW, newPulseState = LOW;
bool playing = LOW, prevPlaying = LOW, playLedState = LOW, prevPlayLedState = LOW, playPinState = LOW; 
#ifdef NONSTOP
bool nonstop = LOW, nonstopStarted = LOW;
#endif
//bool awakePinState = HIGH; 
unsigned int awakePinState = 0; // analog



#ifdef SDEBUG
// tracking of CPU performance
unsigned int loops = 0; // # of main loop cycles between beats
unsigned int imus = 0; // # number of inertia checks in same.
#endif

#ifdef IMU_INTERRUPTS
// IMU interrupt handler:
void imu_int(){
	imu_ready = true;
}
#endif

void setup()
{
  Serial.begin(115200);
	delay(100);
#ifdef EVT4
	Dbg_println("flashed for EVT4 board");
# else
	Dbg_println("flashed for rev3 board");
#endif

	///////
	// IMU setup:
#ifdef IMU_SPI
	// raise chipSelect line to select the only chip. 
	pinMode(SPI_cs, OUTPUT);
	digitalWrite(SPI_cs, HIGH);
	// AudioLibrary changed these, i change them back!
	SPI.setMOSI(SPI_mosi);
	SPI.setMISO(SPI_miso);
	SPI.setSCK(SPI_clock);
#else
	// slight hack for EVT1/rev1:
	// i changed IMU chips, and this pin (IMU pin 10) is "reserved" on the ICM42605:
	pinMode(IMU_fsync, OUTPUT);
	// imu pin 11 is also reserved but I can't get to it from software ...
	digitalWrite(IMU_fsync, 1); // ground this pin (teensy signals are "active low" so ground == 1)
#endif

#ifdef IMU_INTERRUPTS
	// IMU int pin is open-collector mode
	pinMode(IMU_int, INPUT_PULLUP);
	attachInterrupt(IMU_int, imu_int, FALLING);
#else
	imuClock = 0;
#endif
	
  imu.begin();

	///////
	// Pin setup:
	//
  pinMode(led1Pin, OUTPUT);       // led1
  pinMode(led2Pin, OUTPUT);       // led2
  pinMode(pulsePin1, OUTPUT);       // j1 tip
  pinMode(pulsePin2, OUTPUT);       // j2 tip
  pinMode(button1Pin, INPUT_PULLUP); // sw1
  pinMode(button2Pin, INPUT_PULLUP); // sw2
#ifdef EVT4
  pinMode(button3Pin, INPUT_PULLUP); // nonstop
  pinMode(button3LedPin, OUTPUT);       // nonstop led
	digitalWrite(button3LedPin, HIGH);
#endif

	pinMode(PO_play, INPUT); // not sure if PULLUP helps here or not?  Flickers on & off anyway ...
	pinMode(PO_wake, INPUT);
	pinMode(PO_reset, INPUT_PULLUP); // TODO: why do we still get resets when connecting?  need pullup on the PI board?
	pinMode(PO_SWCLK, INPUT_PULLUP);
	pinMode(PO_SWDIO, INPUT_PULLUP);
	pinMode(PO_SWO, INPUT_PULLUP);

	btn1.attach(button1Pin);
	btn1.interval(debounceLen);
	btn2.attach(button2Pin);
	btn2.interval(debounceLen);
#ifdef EVT4
	btn3.attach(button3Pin);
	btn3.interval(debounceLen);
#endif

	////////
	// Clock setup:
	//
	hc.downbeatTime = micros(); // now!
	// If buttons are held down when we boot, reset the default measure length
	if (BOTHPRESSED(btn1, btn2)) {
		hc.measureLen = 250 * TIMESCALE; 	// default for 120bpm (1 beat per half/second) */
	} else {
		EEPROM.get(eepromBase, hc.measureLen);
	}

#ifdef MIDITIMECODE
	mtc.setup();
#endif

	loopTimer = 0; // why?

	// init this buffer:
	hc.tapIntervals.push(0);
	hc.tapIntervals.push(0);
	hc.tapIntervals.push(0);

	////////
	// Teensy Audio setup:
	AudioMemory(2);
	dac1.analogReference(EXTERNAL); // 3.3v p2p (but see below)
	noise1.amplitude(2);
	amp1.gain(0);
	/* dc1.amplitude(0); */ //DEBUG
	/* mixer1.gain(0, 1); // noise1 -> amp1 */ //DEBUG
	/* mixer1.gain(1, 1); // dc1 */ //DEBUG

	//https://forum.pjrc.com/threads/25519-Noise-on-DAC-(A14)-output-Teensy-3-1
	// Initialize the DAC output pins
  analogWriteResolution(12);
  analogWrite(A14, 0);  //Set the DAC output to 0.
  DAC0_C0 &= 0b10111111;  //uses 1.2V reference for DAC instead of 3.3V
												// does this clash with analogReference() above?  probably ... 
	// anyway I still get background noise when running off battery.
	// TODO: test without this stuff & instead with INTERNAL in dac1.analogReference

	////////////
	// Sleep setup:
	s_digital.pinMode(button2Pin, INPUT_PULLUP, FALLING);  // NO-OP; wake from deepSleep not supported on this pin.
	s_timer.setTimer(5000); 
	/* s_compare.pinMode(PO_wake, HIGH, 1.65); */ // comparison wake from deepSleep not supported on this pin.

}

void loop()
{
	int ax, ay, az;
  int gx, gy, gz;

	unsigned long tapInterval = 0; // could be fewer bits?
	unsigned long nowTime = loopTimer;

	long instantPos; 
	long circleOffset;
	long measureIdx;
	bool targetNear, targetAhead;

	static int midiMeasuresRemaining = 0;

	btn1.update();
	btn2.update();
#ifdef EVT4
	btn3.update();
#endif

#ifdef NONSTOP
	if (EITHERNOTPRESSED(btn1, btn2)){
		// this mode is entered only when we start in NONSTOP mode with both buttons pressed,
		// and cleared as soon as both buttons aren't pressed.  It exists to prevent conflict
		// between these two meanings/uses of both-buttons-pressed.
		nonstopStarted = false;
	}
#endif

#ifdef SDEBUG
	loops++; 
#endif

	// go to sleep if the PO_wake pin is low:
	//awakePinState = digitalRead(PO_wake);
	awakePinState = analogRead(PO_wake);

	// awaiting a jumper fix in EVT4 ...
#ifndef EVT4
	if (awakePinState < awakePinThreshold) {
	// if (PRESSED(btn1)) { // DEBUG 
		powerNap();
		return;
	}
#endif 

	// Check IMU:

	shaken = LOW;
	tapped = LOW;

	// Read IMU data if ready:
#ifdef IMU_INTERRUPTS
	if (imu_ready) { // on interrupt
		imu_ready = false;
#else
	if (imuClock > imuClockTick) { // on interval
		imuClock -= imuClockTick;
#endif

#ifdef SDEBUG
		imus++;
#endif

		prevInertia = inertia;
    imu.readMotionSensor(ax, ay, az, gx, gy, gz);
		inertia = (long)sqrt((ax * ax) + (ay * ay) + (az * az));  // vector amplitude

#ifdef SDEBUG
		/* if (inertia > shakeThreshold)  */
		/* 	Dbg_println(inertia); */
#endif

		// use the inertia (minus gravity) to set the volume of the pink noise generator 
		//amp1.gain(max((inertia - shakeThreshold)/3.0, 0.10)); // DEBUG (always on, to listen for audio dropouts)
		amp1.gain(max((inertia - shakeThreshold)/(3.0 * COUNT_PER_G), 0.0));

		if ( (inertia > shakeThreshold) && (prevInertia <= shakeThreshold) ) {
			shaken = HIGH;
		}
		if ( (inertia > tapThreshold) && (prevInertia <= tapThreshold) ) {
			// TODO: shaken should be the start of sound moment, 
			// but try putting tapped at the apex-G moment; it may have better feel.
			tapped = HIGH;
		}
	}

#ifdef MIDITIMECODE
	mtc.frameCheck();
#endif

	// Decode the state of the PO Play LED from the pin signal:
	// This pin is low when not playing, but when playing it's actually flickering,
	// so we can't just read it as logic.
	prevPlayLedState = playLedState;
	playPinState = digitalRead(PO_play);
	if (playPinState) { // lit
		playPinTimer = 0;
		playLedState = playPinState;
	} else {						// unlit
		// Done flickering? Has play been stopped for 10ms or longer?
		//if (playPinTimer > playFlickerTime) {
		
		// animation of this LED varies a lot(!) between PO models.
		// The KO is the worst: it only strobes it every 2 pulses/4 beats.
		// The Speak is normal on during play, but blinks off for about 2 measures every 4 beats, and also flickers at other random times ...
		// while the KO is the opposite & only flickers on that long.

		// TODO: actually measure the flicker we're seeing during the first 8 pulses after play,
		// and use that to tune this interval (and potentially detect the PO model!)

		//if (playPinTimer > (2 * hc.measureLen)) { // coeff. of 2 theoretically correct but very slightly not long enough due to misc computation delays.
		if (playPinTimer > ( (27 * hc.measureLen) / 10) ) { // integer equivalent of 2.5 coeff.
			playLedState = playPinState;

			// DEBUG
			/* if(prevPlayLedState){ */
			/* 	Dbg_print("play led off for"); */
			/* 	Dbg_print(playPinTimer); */
			/* 	Dbg_print(" during meaure of "); */
			/* 	Dbg_print(hc.measureLen); */
			/* 	Dbg_println("us"); */
			/* } */

		}
		// Most of the rest either hold it completely high (1 & 2 series) 
		// or blink it off every 2 pulses/4 beats.
		// The Speak has the problem that when you press play it doesn't
		// start right away ... waiting for a pulse or IDK?

		// this makes the whole thing way less responsive to the play button,
		// although you can still start/stop accurately using the side buttons.

		// TODO: detect which model we're attached to somehow?
	}

	// calculate if we're playing or not, based on playLedState, buttons and NONSTOP:
	prevPlaying = playing;

	// If the play light just lit,
	if (playLedState && (!prevPlayLedState)) {
		// set PLAYING.
		playing = true;
		Dbg_println("START");
#ifdef NONSTOP
		// if both buttons are held down, 
		if (BOTHPRESSED(btn1, btn2)) {
			// set NONSTOP and NONSTOP-STARTED.
			nonstop = nonstopStarted = true;
		}
#endif

	// otherwise, if the play light just unlit,
	} else if (prevPlayLedState && (! playLedState)) {
#ifdef NONSTOP
		// if both buttons are held down,
		if (BOTHPRESSED(btn1, btn2)) {
			// clear PLAYING, NONSTOP and NONSTOP-STARTED
			playing = nonstop = nonstopStarted = false;
		// else 
			// if NONSTOP
				// don't stop!
			// else
				// clear PLAYING.
		} else if (!nonstop) { 
			playing = false;
		}
#else
		playing = false;
#endif
		Dbg_println("STOP");
	}
	
	/////////
	// Update the Human Clock & MIDI transport
	//
	if (playing && !prevPlaying) {
		// user just pressed play button to start PO
		// move downbeat to now!
		hc.downbeatTime = nowTime;
		// sync the circular clock!
		//cc.circlePos = 0.0;
		cc.circlePos = 0;
#ifdef MIDICLOCK
		usbMIDI.sendRealTime(usbMIDI.Start);
		usbMIDI.sendRealTime(usbMIDI.Clock);
#endif
#ifdef MIDITIMECODE
		// rewind time to zero
		mtc.rewind();
#endif
	} else if (!playing && prevPlaying) {
		// user just pressed play button to stop PO.
#ifdef MIDICLOCK
		usbMIDI.sendRealTime(usbMIDI.Stop);
#endif
	}

	// downbeatTime is the absolute time of the start of the current pulse.
	// if now is at least pulseLen millis beyond the previous beat, advance the beat
  while (nowTime > (hc.downbeatTime + pulseLen)) {  
    hc.downbeatTime += hc.measureLen;
	}

	// Check the buttons:
	//
	if (EITHERPRESSED(btn1, btn2)) {
		if (tapped) { 

			// Adjust position of downbeat based on tap/shake:

			// If the next beat is closer than the previous beat
			// (dbT is greater than nowTime, but by less than mL/2))
			// or if we're in the midst of a pulse (dbT is less than nT)
			// ((dbT - (mL/2)) < nT < dbT)
				// Advance downbeat to NOW!
			// otherwise, when the previous beat is still closer (dbT > nt + (mL/2),
				// Retard downbeat to now + hc.measureLen

			if (BOTHPRESSED(btn1, btn2)) {
				hc.downbeatTime = nowTime;
			} else if (hc.downbeatTime > (nowTime + (hc.measureLen/2))) {
				hc.downbeatTime = nowTime + hc.measureLen;
			} else {
				hc.downbeatTime = nowTime;
			}

			if (++hc.tapCount > 1) {
				// Also adjust the measure length to the time between taps:
				tapInterval = nowTime - hc.lastTapTime;

				if (! BOTHPRESSED(btn1, btn2)) {
					// if tapInterval is closer to mL*2 than to mL, 
					// assume we are tapping half-time (1/4 notes)
					if (tapInterval > (1.5 * hc.measureLen)) {
						tapInterval /= 2;
					}
				}
				// Othwerwise assume full time (1/8 notes)

				if (tapInterval < minTapInterval) {
					// Ignore spurious double-taps!  (This enforces a max tempo.)
					Dbg_println("ignoring bounce");
				} else {
					hc.tapIntervals.unshift(tapInterval);
					if (hc.tapCount == 2) {
						hc.measureLen = tapInterval;
					} else if (hc.tapCount == 3) {
						hc.measureLen = (hc.tapIntervals[1] + hc.tapIntervals[0]) / 2;
					} else if (hc.tapCount >= 4) {
						hc.measureLen = (hc.tapIntervals[2] + hc.tapIntervals[1] + hc.tapIntervals[0]) / 3;
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
		if (btn1.rose() || btn2.rose()) 				// if we just released the buttons,
			EEPROM.put(eepromBase, hc.measureLen); 	// save the new tempo to NVRAM.

		// not armed.
		hc.tapCount = 0;
	}

	///////////
	// Calculate & update LEDs.
	// (Strobe-off times need to be much longer than strobe-on times
	// in order to be clearly visible to the human eye.)
	//
	if (playing ) { 
		if (PRESSED(btn1)) {
			if (nowTime - hc.downbeatTime < strobeOffLen) // there's some bug here, the interval never gets long enough.
				newLed1State = LOW;
			else
				newLed1State = HIGH;
		} else {
			if (nowTime - hc.downbeatTime < strobeOnLen)
				newLed1State = HIGH;
			else
				newLed1State = LOW;
		}

		if (PRESSED(btn2)) {
			if (nowTime - hc.downbeatTime < strobeOffLen)
				newLed2State = LOW;
			else
				newLed2State = HIGH;
		} else {
			if (nowTime - hc.downbeatTime < strobeOnLen)
				newLed2State = HIGH;
			else
				newLed2State = LOW;
		}
	} else {  // !playing
		// don't blink when not playing, just indicate buttons
		newLed1State = PRESSED(btn1);
		newLed2State = PRESSED(btn2);
	}

	
	if (led1State != newLed1State) {
		led1State = newLed1State;
		digitalWrite(led1Pin, led1State);
	}
	if (led2State != newLed2State) {
		led2State = newLed2State;
		digitalWrite(led2Pin, led2State);
	}

	//////////
	// Calculate & update the sync pulse
	//
	if (BOTHPRESSED(btn1, btn2)) { 
		if (tapped) // TODO: simplify? i think tapped is always true if the following if-clause is false:
			newPulseState = HIGH;
		else if (nowTime - hc.downbeatTime >= pulseLen) 
			newPulseState = LOW;
	} else {
		if (nowTime - hc.downbeatTime < pulseLen)
			newPulseState = HIGH;
		else if (nowTime - hc.downbeatTime >= pulseLen) 
			newPulseState = LOW;
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
	instantPos = (CC_FLOAT_RES / 100) * ( (100 * measureIdx) / hc.measureLen ) ;

	// the difference between that position & the current circular position:
	circleOffset = abs(cc.circlePos - instantPos);
	
	// Freeze the circular clock when both buttons pressed & end of MMR reached;
	// unfreeze on the next pulse after a button is released.
	// also unfreeze for 1 measureLen if tapped while bothbuttons pressed

	if (BOTHPRESSED(btn1, btn2)) {
		if (btn1.fell() || btn2.fell()) { // If we just pressed now entered 2-button mode
			midiMeasuresRemaining = 1;  	 // midi only to the end of this measure.
		} else if (tapped) {						// but if we tapped a beat,
			midiMeasuresRemaining++;		// add a measure to that,
				// then unfreeze for just that one measure:
			if (cc.frozen) {
				cc.frozen = false; 					
#ifdef MIDICLOCK
				usbMIDI.sendRealTime(usbMIDI.Continue);
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
				usbMIDI.sendRealTime(usbMIDI.Stop);
#endif
			}
		}
	} else if ((pulseState != newPulseState) && (newPulseState == HIGH) ) {
		if (cc.frozen) {
			cc.frozen = false; 					
#ifdef MIDICLOCK
			usbMIDI.sendRealTime(usbMIDI.Continue);
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
		if ( ( targetNear && targetAhead ) || (!targetNear && !targetAhead) ) {
			// speed up!
			cc.circlePos += min(circleOffset, circleMaxFwd); 
		} else {
			// slow down!
			cc.circlePos += circleMinFwd;
		}
	}

#ifdef MIDICLOCK
	// if we've crossed a clock boundary, send a MIDI clock.
	if (   (  (cc.circlePos + circleTicksPerClock) / circleTicksPerClock  )   >   (  (cc.prevCirclePos + circleTicksPerClock) / circleTicksPerClock  )   ) {
		// debugging noise:
		// send a triangle pulse to audio out
		// dc1.amplitude(1.0 - (cc.circlePos/2)); // DEBUG
		// dc1.amplitude(0, 1); // DEBUG

		if (playing) { 
			if (!cc.frozen) {
				// emit MIDI clock!
				usbMIDI.sendRealTime(usbMIDI.Clock);

				/* if (tapped) {  */
				/* 	Dbg_print("TMC "); */
				/* } else {  */
				/* 	Dbg_print("MC "); */
				/* } */

				/* Dbg_print((int)(cc.circlePos * MIDICLOCKSPERPULSE)); */
				/* Dbg_print2(cc.circlePos, 7); */
				/* Dbg_print(">"); */
				/* Dbg_print2(cc.prevCirclePos, 7); */

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
	if (   (  (cc.circlePos + circleTicksPerFrame) / circleTicksPerFrame  )   >   (  (cc.prevCirclePos + circleTicksPerFrame) / circleTicksPerFrame  )   ) {
		if (playing) { 
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
		if (BOTHPRESSED(btn1, btn2)) {
			if (midiMeasuresRemaining>0)
				midiMeasuresRemaining--;
		}
	}

	// Pulse if PLAYING
	if (pulseState != newPulseState) {
		pulseState = newPulseState;
		if (playing) { 
			// send sync pulse on both pins
			digitalWrite(pulsePin1, pulseState);
			digitalWrite(pulsePin2, pulseState);
		}

		// print benchmarks when pulse goes high.
		if (pulseState == HIGH) {
			Dbg_print(awakeTimer);
			Dbg_print(':');

			if (awakePinState >= awakePinThreshold) {   
				Dbg_print(playing ? "play, wv@" : "stop, wv");
				Dbg_print(awakePinState);
#ifdef NONSTOP
				if (nonstop) 
					Dbg_print("NONSTOP ");
#endif
			} else { 
				Dbg_print("asleep, wv@");
				Dbg_print(awakePinState);
			}
			Dbg_print(", ");
			Dbg_print(loops);
			Dbg_print(" loops, ");
			Dbg_print(imus);
			/* Dbg_print(" imus in "); */
			/* Dbg_print(hc.measureLen); */
			/* Dbg_print(" us, "); */
			Dbg_print(" IMUs at ");
			Dbg_print(USEC2BPM(hc.measureLen));
			Dbg_print(" BPM, ");
			Dbg_print(AudioMemoryUsageMax());
			Dbg_print(" audioMem, ");
			Dbg_print(AudioProcessorUsageMax());
			Dbg_print(" audioCPU, ");
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
			Dbg_println(".");

#ifdef SDEBUG
			// reset counters
			loops = imus = 0;
#endif
		}
	}

	// Protect against overflow of loopTimer.
	// We could do this only every 1000 (or more!) measures,
	// but we choose every 10,
	//	so that if it causes any audible bug, it'll be heard often!
	if (loopTimer > 10 * hc.measureLen) { 
		//Dbg_println("PROTECTION!");//DEBUG
		loopTimer -= 9*hc.measureLen;
		hc.downbeatTime -= 9*hc.measureLen;
		hc.lastTapTime -= 9*hc.measureLen;
	}
}

uint powerNap(){
	uint who = 0;
	// Head towards deep sleep:

#ifdef SDEBUG
	Dbg_println("zzzzz.");
	delay(100);
#endif

	// turn off LEDs
	digitalWrite(led1Pin, LOW);
	digitalWrite(led2Pin, LOW);
	digitalWrite(boardLedPin, LOW);
#ifdef EVT4
	digitalWrite(button3LedPin, LOW);
#endif

	// disable interrupts from accelerometer
	/* detachInterrupt(IMU_int); */
	delay(100);
	// sleep accelerometer
	imu.sleep();

	// attach to buttons for button wakeup
	s_config += s_digital;

	do {
		// sleep N seconds or until right button wakes us
		who = Snooze.deepSleep(s_config);
		awakePinState = analogRead(PO_wake);
		btn2.update();
	//} while (awakePinState == LOW && (! PRESSED(btn2)));
	} while (awakePinState < awakePinThreshold && (! PRESSED(btn2)));

	// detach from buttons
	s_config -= s_digital;

	// wake accelerometer
	imu.wake();

	// reattach interrupts
	/* attachInterrupt(IMU_int, imu_int, FALLING); */

	// reset some counters
	//hc.downbeatTime += (nowTime - thenTime);

	// update button state for next loop
	btn1.update();
	btn2.update();

#ifdef SDEBUG
	if (!Serial) { 
		delay(100); 
	}
	Dbg_println("good morning!");
#endif

	// LEDs will be restored on next loop.

	awakeTimer = 0;
	return who; // loop again!
}


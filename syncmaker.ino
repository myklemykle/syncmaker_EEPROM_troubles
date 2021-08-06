// Rev 1 Pocket Integrator (PO daughterboard) firmware (c) 2021 mykle systems labs
#include <CircularBuffer.h>

// Config:
//
// use interrupts, or poll?
#define INTERRUPTS yeasurewelikeinterrupts  
// use high-resolution timers?
#define MICROS microsmothafuckka!!!   
// send debug output to usb serial?
#define SDEBUG crittersbuggin 
#ifdef SDEBUG
// Teensy will eventually hang on a clogged output buffer if we Serial.print() without USB plugged in.
// is there some more normal solution for this?  Seems like this would be a common problem.
#define Dbg_print(X) if(Serial) Serial.print(X)
#define Dbg_print2(X, Y) if(Serial) Serial.print(X, Y)
#define Dbg_println(X) if(Serial) Serial.println(X)
#define Dbg_flush(X) if(Serial) Serial.flush()
#else
#define Dbg_print(X) {}
#define Dbg_println(X) {}
#define Dbg_flush(X) {}
#endif


// Gesture Detection:
#include "NXPMotionSense.h" // hacked version of this Teensy Prop Shield lib;
														// set to higher data rate, 
														// ignores other two IMU chips 
														// Should be renamed, refactored, etc.
NXPMotionSense imu;		// on EVT1/rev1, the IMU is an ICM42605 MEMS acc/gyro chip
const float shakeThreshold = 0.65; 	
const float tapThreshold = 2.0;
bool shaken = LOW, tapped = LOW;
float inertia = 0, prevInertia = 0;


// Sleep/hibernation:
#include <Snooze.h>
SnoozeDigital s_digital;
SnoozeTimer s_timer;
SnoozeBlock s_config(s_timer);

#include <Audio.h>
// GUItool: end automatically generated code
AudioSynthNoiseWhite     noise1;         //xy=110,301
AudioAmplifier           amp1;           //xy=231,298
AudioSynthWaveformDc     dc1;            //xy=262,375
AudioMixer4              mixer1;         //xy=470,359
AudioOutputAnalog        dac1;           //xy=603,359
AudioConnection          patchCord1(noise1, amp1);
AudioConnection          patchCord2(amp1, 0, mixer1, 0);
AudioConnection          patchCord3(dc1, 0, mixer1, 1);
AudioConnection          patchCord4(mixer1, dac1);
// GUItool: end automatically generated code


// Buttons:
#include "Bounce2.h"
const int debounceLen = 2; // must be in MS as required by bounce2 lib
Bounce btn1 = Bounce();
Bounce btn2 = Bounce();
#define PRESSED(btn) 				(btn.read() == LOW)
#define BOTHPRESSED(b1,b2) 	(PRESSED(b1) && PRESSED(b2))
#define ARMED(b1,b2) 		(PRESSED(b1) || PRESSED(b2))
// useful for preventing two-button mode right after having entered NONSTOP with both buttons pressed.
#define TWOBUTTONMODE(b1,b2) 	(PRESSED(b1) && PRESSED(b2) && (! nonstopStarted))
#define EITHERPRESSED(b1,b2) 		(PRESSED(b1) || PRESSED(b2))
#define EITHERNOTPRESSED(b1,b2) 		(!PRESSED(b1) || !PRESSED(b2))


// Pins:
const int button1Pin = 0;   // sw1
const int button2Pin = 9;   // sw2
const int boardLedPin = 13;	// builtin led on Teensy 3.2
const int led1Pin =  21;    // led1
const int led2Pin =  20;    // led2
const int pulsePin1 = 11; 	// j1 tip
const int pulsePin2 = 8; 		// j2 tip
const int analogOut = 14;   // 12 bit DAC on Teensy 3.2 connected to j1/j2 ring
const int PO_play = 16; 		// goes high on PO play (runs to play LED)
const int PO_wake = 17;			// goes low when PO sleeps I think
const int PO_reset = 7;			
const int PO_SWCLK = 6;			// jtag/stlink pins:
const int PO_SWDIO = 5;
const int PO_SWO   = 4;
const int IMU_fsync = 1;   // need to ground this on icm-42605
const int IMU_int = 2;



// Important time intervals:
#ifdef MICROS
const unsigned int TIMESCALE = 1000; // uS
const unsigned long strobeOnLen = 500;
# else 
const unsigned int TIMESCALE = 1;		// mS
const unsigned long strobeOnLen = 1;
#endif

const unsigned long strobeOffLen = 100 * TIMESCALE;
const unsigned long pulseLen = 5 * TIMESCALE; 		
const unsigned long minTapInterval = 100 * TIMESCALE;  // Ignore spurious double-taps!  (This enforces a max tempo.)
const unsigned long playFlickerTime = 100 * TIMESCALE; 



// Timers:
#ifdef MICROS
elapsedMicros loopTimer;
elapsedMicros playPinTimer;
elapsedMicros awakeTimer;
#else
elapsedMillis loopTimer;
elapsedMillis playPinTimer;
elapsedMillis awakeTimer;
#endif


#ifdef INTERRUPTS
volatile bool imu_ready = false;
#else
elapsedMicros imuClock;
const int clockTick = 500; // 2khz data rate (match what's set in NXPMotionSense
#endif


// EEPROM
#include <EEPROM.h>
// NOTE: the prop shield calibration is stored in the EEPROM,
// we mustn't overwrite that!  
// It's rumored to be 68 bytes starting at address 0x60 -- 
//    https://forum.pjrc.com/threads/33997-EEPROM-usage-table?highlight=prop+shield+eeprom
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
// This is the very minimum (per loop cycle) forward advance
const	float circleMinFwd = 1.0/1000000.0; 
// OTOH, if we advance any faster than this, we could skip a MIDI clock beat
const float circleMaxFwd = 0.08333; // 1/12 to 5 places.
//
typedef struct {
	// Clock position is a float value between 0 and 1.
	// TODO: refactor to ints.
	float circlePos = 0.0, prevCirclePos = 0.0;
	// Freeze when both buttons pressed & end of MMR reached;
	// unfreeze on the next downbeat after a button is released.
	bool frozen = false;
} CircularClock;
CircularClock cc;


// State Variables:
bool led1State = LOW, newLed1State = LOW;
bool led2State = LOW, newLed2State = LOW;
bool blinkState = LOW, newBlinkState = LOW;
bool pulseState = LOW, newPulseState = LOW;
bool playing = LOW, prevPlaying = LOW, playLedState = LOW, prevPlayLedState = LOW, playPinState = LOW; 
bool nonstop = LOW, nonstopStarted = LOW;
bool awakePinState = HIGH; 



#ifdef SDEBUG
// tracking of CPU performance
unsigned int loops = 0; // # of main loop cycles between beats
unsigned int imus = 0; // # number of inertia checks in same.
#endif


#ifdef INTERRUPTS
// IMU interrupt handler:
void imu_int(){
	imu_ready = true;
}
#endif

void setup()
{
  Serial.begin(115200);

	///////
	// IMU setup:
	// slight hack for EVT1/rev1:
	// i changed IMU chips, and this pin (IMU pin 10) is "reserved" on the ICM42605:
	pinMode(IMU_fsync, OUTPUT);
	// imu pin 11 is also reserved but I can't get to it from software ...
	digitalWrite(IMU_fsync, 1); // ground this pin (teensy signals are "active low" so ground == 1)

#ifdef INTERRUPTS
	// IMU int2 pin is open-collector mode
	pinMode(IMU_int, INPUT_PULLUP);
	attachInterrupt(IMU_int, imu_int, FALLING);
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

	pinMode(PO_play, INPUT); // not sure if PULLUP helps here or not?  Flickers on & off anyway ...
	pinMode(PO_wake, INPUT);
	pinMode(PO_reset, INPUT_PULLUP); // TODO: really we want this pulled down, not up.
	pinMode(PO_SWCLK, INPUT_PULLUP);
	pinMode(PO_SWDIO, INPUT_PULLUP);
	pinMode(PO_SWO, INPUT_PULLUP);

	btn1.attach(button1Pin);
	btn1.interval(debounceLen);
	btn2.attach(button2Pin);
	btn2.interval(debounceLen);

	////////
	// Clock setup:
	//
#ifdef MICROS
	hc.downbeatTime = micros(); // now!
#else
	hc.downbeatTime = millis(); // now!
#endif
	// If buttons are held down when we boot, reset the default measure length
	if (BOTHPRESSED(btn1, btn2)) {
		hc.measureLen = 250 * TIMESCALE; 	// default for 120bpm (1 beat per half/second) */
	} else {
		EEPROM.get(eepromBase, hc.measureLen);
	}

#ifndef INTERRUPTS
	imuClock = 0;
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
	dc1.amplitude(0);
	mixer1.gain(0, 1); // noise1 -> amp1
	mixer1.gain(1, 1); // dc1

	//https://forum.pjrc.com/threads/25519-Noise-on-DAC-(A14)-output-Teensy-3-1
	// Initialize the DAC output pins
  analogWriteResolution(12);
  analogWrite(A14, 0);  //Set the DAC output to 0.
  DAC0_C0 &= 0b10111111;  //uses 1.2V reference for DAC instead of 3.3V
												// does this clash with analogReference() above?

	////////////
	// Sleep setup:
	s_digital.pinMode(button2Pin, INPUT_PULLUP, FALLING);  // NO-OP; wake from deepSleep not supported on this pin.
	s_timer.setTimer(5000); 
	/* s_compare.pinMode(PO_wake, HIGH, 1.65); */ // comparison wake from deepSleep not supported on this pin.

}

void loop()
{
	float ax, ay, az;
  float gx, gy, gz;

	unsigned long tapInterval = 0; // could be fewer bits?
	unsigned long nowTime = loopTimer;

	float instantPos; 
	float circleOffset;
	//int measureIdx;
	long measureIdx;
	bool targetNear, targetAhead;

	static int midiMeasuresRemaining = 0;

	btn1.update();
	btn2.update();

	if (EITHERNOTPRESSED(btn1, btn2)){
		// this mode is entered only when we start in NONSTOP mode with both buttons pressed,
		// and cleared as soon as both buttons aren't pressed.  It exists to prevent conflict
		// between these two meanings/uses of both-buttons-pressed.
		nonstopStarted = false;
	}

#ifdef SDEBUG
	loops++; 
#endif

	// go to sleep if the PO_wake pin is low:
	awakePinState = digitalRead(PO_wake);

	if (! awakePinState) {
	/* if (PRESSED(btn1)) { // DEBUG */
		powerNap();
		return;
	}

	// Check IMU:

	shaken = LOW;
	tapped = LOW;

	// Read IMU data if ready:
#ifdef INTERRUPTS
	if (imu_ready) { // on interrupt
		imu_ready = false;
#else
	if (imuClock > clockTick) { // on interval
		imuClock -= clockTick;
#endif

#ifdef SDEBUG
		imus++;
#endif

		prevInertia = inertia;
    imu.readMotionSensor(ax, ay, az, gx, gy, gz);
		inertia = abs(sqrt(pow(ax,2) + pow(ay,2) + pow(az,2)));  // vector amplitude

#ifdef SDEBUG
		/* if (inertia > shakeThreshold)  */
		/* 	Dbg_println(inertia); */
#endif

		// use the inertia (minus gravity) to set the volume of the pink noise generator 
		amp1.gain(max((inertia - shakeThreshold)/3.0, 0.0));

		if ( (inertia > shakeThreshold) && (prevInertia <= shakeThreshold) ) {
			shaken = HIGH;
		}
		if ( (inertia > tapThreshold) && (prevInertia <= tapThreshold) ) {
			// TODO: shaken should be the start of sound moment, 
			// but try putting tapped at the apex-G moment; it may have better feel.
			tapped = HIGH;
		}
	}

	// Decode the state of the PO Play LED from the pin signal:
	// This pin is low when not playing, but when playing it's actually flickering,
	// so we can't just read it as logic.
	prevPlayLedState = playLedState;
	playPinState = digitalRead(PO_play);
	if (playPinState) {
		playPinTimer = 0;
		playLedState = playPinState;
	} else {
		// Done flickering? Has play been stopped for 10ms or longer?
		if (playPinTimer > playFlickerTime) {
			playLedState = playPinState;
		}
	}

	// calculate if we're playing or not, based on playLedState,
	// buttons and NONSTOP flag:
	prevPlaying = playing;

	// If the play light just lit,
	if (playLedState && (!prevPlayLedState)) {
		// set PLAYING.
		playing = true;
		// if both buttons are held down, 
		if (BOTHPRESSED(btn1, btn2)) {
			// set NONSTOP and NONSTOP-STARTED.
			nonstop = nonstopStarted = true;
		}

	// otherwise, if the play light just unlit,
	} else if (prevPlayLedState && (! playLedState)) {
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
	}
	
	/////////
	// Update the Human Clock & MIDI transport
	//
	if (playing && !prevPlaying) {
		// user just pressed play button to start PO
		// move downbeat to now!
		hc.downbeatTime = nowTime;
		usbMIDI.sendRealTime(usbMIDI.Start);
	} else if (!playing && prevPlaying) {
		// user just pressed play button to stop PO.
		usbMIDI.sendRealTime(usbMIDI.Stop);
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
	measureIdx = hc.downbeatTime - nowTime;

	// Expressed as a float btwn 0 & 1:
	instantPos = 1.0 - ( (float)measureIdx / (float)hc.measureLen ) ;

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
				Dbg_println("@");//DEBUG obv
				usbMIDI.sendRealTime(usbMIDI.Continue);
				cc.frozen = false; 					
			}
			Dbg_print(midiMeasuresRemaining);//DEBUG obv
			Dbg_println("!");//DEBUG obv
		}
		if (midiMeasuresRemaining == 0) {
			if (!cc.frozen) 
				usbMIDI.sendRealTime(usbMIDI.Stop);
			cc.frozen = true;
		}
	} else if ((pulseState != newPulseState) && (newPulseState == HIGH) ) {
		if (cc.frozen) {
			Dbg_println("@");//DEBUG obv
			usbMIDI.sendRealTime(usbMIDI.Continue);
			cc.frozen = false; 					
		}
	}

	// Is the cc ahead of the hc & needing to slow down, or behind it & needing to speed up?
	targetNear = (circleOffset < 0.5);
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

	// if we've crossed a 1/12 boundary, send a MIDI clock.
	if ((int)((1+cc.circlePos) * 12.0) > (int)((1+cc.prevCirclePos) * 12.0)) {
		/*
		// debugging noise:
		// send a triangle pulse to audio out
		dc1.amplitude(1.0 - (cc.circlePos/2)); // DEBUG
		dc1.amplitude(0, 1); // DEBUG
		*/


		if (playing) { 
			if (!cc.frozen) {
				// emit MIDI clock!
				usbMIDI.sendRealTime(usbMIDI.Clock);

				if (tapped) { 
					Dbg_print("TMC ");
				} else { 
					Dbg_print("MC ");
				}
				Dbg_print((int)(cc.circlePos * 12.0));
				/* Dbg_print2(cc.circlePos, 7); */
				/* Dbg_print(">"); */
				/* Dbg_print2(cc.prevCirclePos, 7); */
				Dbg_print(", ");
				if (cc.circlePos >= 1.0) {
					Dbg_println(".");
				}
			}
		}
	}

	// wrap!
	if (cc.circlePos > 1.0) {
		cc.circlePos -= 1.0;
		cc.prevCirclePos -= 1.0;
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
			if (awakePinState) { 
				Dbg_print(playing ? "play  " : "stop  ");
				if (nonstop) 
					Dbg_print("NONSTOP ");
			} else { 
				Dbg_print("asleep  ");
			}
			Dbg_print(loops);
			Dbg_print(" loops, ");
			Dbg_print(imus);
			Dbg_print(" imus in ");
			Dbg_print(hc.measureLen);
#ifdef MICROS
			Dbg_print(" us, ");
#else
			Dbg_print(" ms, ");
#endif
			Dbg_print(AudioMemoryUsageMax());
			Dbg_print(" audioMem, ");
			Dbg_print(AudioProcessorUsageMax());
			Dbg_print(" audioCPU, ");
			Dbg_print(midiMeasuresRemaining);
			Dbg_print(" MMR, ");
			/* Dbg_print(instantPos); */
			/* Dbg_print(" instant, "); */
			/* Dbg_print(cc.circlePos); */
			/* Dbg_print(" circular, "); */
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
	delay(10);
#endif

	// turn off LEDs
	digitalWrite(led1Pin, LOW);
	digitalWrite(led2Pin, LOW);
	digitalWrite(boardLedPin, LOW);

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
		awakePinState = digitalRead(PO_wake);
		btn2.update();
	} while (awakePinState == LOW && (! PRESSED(btn2)));

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


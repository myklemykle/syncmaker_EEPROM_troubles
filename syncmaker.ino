// Rev 1 Pocket Integrator (PO daughterboard) firmware (c) 2021 mykle systems labs

#include "Bounce2.h"
#include "NXPMotionSense.h" // hacked version of this Teensy Prop Shield lib;
														// set to higher data rate, 
														// ignores other two IMU chips 
														// Should be renamed, refactored, etc.
#include <Wire.h>
#include <EEPROM.h>
#include <Snooze.h>
#include <CircularBuffer.h>

// use interrupts or poll?
#define INTERRUPTS yeasurewelikeinterrupts  

// high-resolution timers.
#define MICROS microsmothafuckka!!!   

#define SDEBUG crittersbuggin 

#ifdef SDEBUG
// Teensy will eventually hang on a clogged output buffer if we Serial.print() without USB plugged in.
// is there some more normal solution for this?  Seems like this would be a common problem.
#define Dbg_print(X) if(Serial) Serial.print(X)
#define Dbg_println(X) if(Serial) Serial.println(X)
#else
#define Dbg_print(X) {}
#define Dbg_println(X) {}
#endif

NXPMotionSense imu;
SnoozeDigital s_digital;
//SnoozeCompare s_compare;
SnoozeTimer s_timer;
/* SnoozeAlarm s_alarm; */
SnoozeBlock s_config(s_timer);

// GUItool reqs:
#include <Audio.h>

// GUItool: begin automatically generated code
//AudioSynthNoisePink      pink1;          //xy=88,242
AudioSynthNoiseWhite      pink1;          //xy=88,242
AudioAmplifier           amp1;           //xy=257,241
//AudioEffectReverb        reverb1;        //xy=440,243
AudioOutputAnalog        dac1;           //xy=628,246
AudioConnection          patchCord1(pink1, amp1);
//AudioConnection          patchCord2(amp1, reverb1);
//AudioConnection          patchCord3(reverb1, dac1);
AudioConnection          patchCord2(amp1, dac1);
// GUItool: end automatically generated code


// set pin numbers:
const int button1Pin = 0;   // sw1
const int button2Pin = 9;   // sw2
const int boardLedPin = 13;
const int led1Pin =  21;    // led1
const int led2Pin =  20;    // led2
const int pulsePin1 = 11; 	// j1 tip
const int pulsePin2 = 8; 		// j2 tip
const int analogOut = 14;   // j1/j2 ring: 12 bit DAC on Teensy 3.2 !
const int PO_play = 16; 		// goes high on PO play (runs to play LED)
const int PO_wake = 17;		// goes low when PO sleeps I think
const int PO_reset = 7;
const int PO_SWCLK = 6;
const int PO_SWDIO = 5;
const int PO_SWO   = 4;
const int IMU_fsync = 1;   // need to ground this on icm-42605
const int IMU_int = 2;

// default time periods:
const int debounceLen = 2; // must be in MS as required by bounce2 lib

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
unsigned long measureLen = 250 * TIMESCALE; 	// default for 120bpm (1 beat per half/second)

CircularBuffer<long, 3> tapIntervals;

#ifdef MICROS
elapsedMicros loopClock;
elapsedMicros playPinTimer;
elapsedMicros awakeTime;
#else
elapsedMillis loopClock;
elapsedMillis playPinTimer;
elapsedMillis awakeTime;
#endif

#ifdef INTERRUPTS
volatile bool imu_ready = false;
#else
elapsedMicros imuClock;
const int clockTick = 500; // 2khz data rate (match what's set in NXPMotionSense
#endif

// NOTE: the prop shield calibration is stored in the EEPROM,
// we mustn't overwrite that!  
// It's rumored to be 68 bytes starting at address 0x60 -- 
//    https://forum.pjrc.com/threads/33997-EEPROM-usage-table?highlight=prop+shield+eeprom
const int eepromBase = 2000;

// vars
bool led1State = LOW, newLed1State = LOW;
bool led2State = LOW, newLed2State = LOW;
bool blinkState = LOW, newBlinkState = LOW;
bool pulseState = LOW, newPulseState = LOW;
bool playing = LOW, prevPlaying = LOW, playLedState = LOW, prevPlayLedState = LOW, playPinState = LOW; 
bool nonstop = LOW, nonstopStarted = LOW;
bool awakePinState = HIGH; 

unsigned long downbeatTime = 0;        
unsigned long lastTapTime = 0;        

unsigned int tapCount = 0;

float inertia = 0;
float prevInertia = 0;
const float shakeThreshold = 0.65;
const float tapThreshold = 2.0;
bool shaken = LOW;
bool tapped = LOW;

#ifdef SDEBUG
// tracking performance
unsigned int loops = 0; // # of main loop cycles between beats
unsigned int imus = 0; // # number of inertia checks in same.
#endif

// Objects:
Bounce btn1 = Bounce();
Bounce btn2 = Bounce();

// Macros
#define PRESSED(btn) 				(btn.read() == LOW)
#define BOTHPRESSED(b1,b2) 	(PRESSED(b1) && PRESSED(b2))
// useful for preventing two-button mode right after having entered NONSTOP with both buttons pressed.
#define TWOBUTTONMODE(b1,b2) 	(PRESSED(b1) && PRESSED(b2) && (! nonstopStarted))
#define EITHERPRESSED(b1,b2) 		(PRESSED(b1) || PRESSED(b2))
#define EITHERNOTPRESSED(b1,b2) 		(!PRESSED(b1) || !PRESSED(b2))
/* #define TAP(b1,b2) 			( (PRESSED(b1) && b2.fell()) || (PRESSED(b2) && b1.fell()) ) */

#ifdef INTERRUPTS
// IMU interrupt handler:
void imu_int(){
	imu_ready = true;
}
#endif

void setup()
{
  Serial.begin(115200);

	// pins!

	// slight hack:
	// because i changed chips, this pin (imu pin 10) is now "reserved" on the IMU:
	pinMode(IMU_fsync, OUTPUT);
	// imu pin 11 is also reserved but I can't get to it from software ...
	digitalWrite(IMU_fsync, 1); // ground this pin (teensy signals are "active low" so ground == 1)

#ifdef INTERRUPTS
	// IMU int2 pin is open-collector mode
	pinMode(IMU_int, INPUT_PULLUP);
	attachInterrupt(IMU_int, imu_int, FALLING);
#endif
	
  imu.begin();

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

#ifdef MICROS
	downbeatTime = micros(); // now!
#else
	downbeatTime = millis(); // now!
#endif

	// teensy audio setup
	AudioMemory(2);
	dac1.analogReference(EXTERNAL); // 3.3v p2p
	pink1.amplitude(2);
	//reverb1.reverbTime(0.5);
	amp1.gain(0);

	// If buttons are held down when we boot, reset the default measure length
	if (BOTHPRESSED(btn1, btn2)) {
	} else {
		EEPROM.get(eepromBase, measureLen);
	}

	// sleep stuff:
	s_digital.pinMode(button2Pin, INPUT_PULLUP, FALLING);  // DEBUG
	/* s_alarm.setRtcTimer(0, 0, 10);// hour, min, sec */
	s_timer.setTimer(5000); // not working wtf?
	/* s_compare.pinMode(PO_wake, HIGH, 1.65); */ // rev1 uses wrong pin for this.

#ifndef INTERRUPTS
	imuClock = 0;
#endif

	loopClock = 0;

	//https://forum.pjrc.com/threads/25519-Noise-on-DAC-(A14)-output-Teensy-3-1
	// Initialize the DAC output pins
  analogWriteResolution(12);
  analogWrite(A14, 0);  //Set the DAC output to 0.
  DAC0_C0 &= 0b10111111;  //uses 1.2V reference for DAC instead of 3.3V

	tapIntervals.push(0);
	tapIntervals.push(0);
	tapIntervals.push(0);
}

void loop()
{
	float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

	unsigned long tapInterval = 0; // could be fewer bits?
	unsigned long nowTime = loopClock;
	unsigned long thenTime;

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

	// downbeatTime is the absolute time of the start of the current pulse.
	// if now is at least pulseLen millis beyond the previous beat, advance the beat
  while (nowTime > (downbeatTime + pulseLen)) {  
    downbeatTime += measureLen;
	}

	// go to sleep if the PO_wake pin is low:
	awakePinState = digitalRead(PO_wake);

	if (! awakePinState) {
	/* if (PRESSED(btn1)) { //DEBUG */
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

/* #ifdef SDEBUG */
/* 		if (inertia > shakeThreshold)  */
/* 			Dbg_println(inertia); */
/* #endif */

		// use the inertia (minus gravity) to set the volume of the pink noise generator 
		amp1.gain(max((inertia - shakeThreshold)/3.0, 0.0));

		
		if ( (inertia > shakeThreshold) && (prevInertia <= shakeThreshold) ) {
			shaken = HIGH;
		}
		if ( (inertia > tapThreshold) && (prevInertia <= tapThreshold) ) {
			// TODO: shaken should be the start of sound moment, 
			// but try putting tapped at the apex-G moment; it may have better feel.
			tapped = HIGH;
		if (inertia > shakeThreshold) 
			Dbg_println(inertia);
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

	// Now calculate if we're playing or not, based on playLedState,
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
	
	if (playing && !prevPlaying) {
		// user just pressed play;
		// move downbeat to now!
		downbeatTime = nowTime;
	}

	// Check the buttons:

	if (EITHERPRESSED(btn1, btn2)) {
		if (tapped) { 

			// Adjust position of downbeat based on tap/shake:

			// If the next beat is closer than the previous beat
			// (dbT is greater than nowTime, but by less than mL/2))
			// or if we're in the midst of a pulse (dbT is less than nT)
			// ((dbT - (mL/2)) < nT < dbT)
				// Advance downbeat to NOW!
			// otherwise, when the previous beat is still closer (dbT > nt + (mL/2),
				// Retard downbeat to now + measureLen

			if (BOTHPRESSED(btn1, btn2)) {
				downbeatTime = nowTime;
			} else if (downbeatTime > (nowTime + (measureLen/2))) {
				downbeatTime = nowTime + measureLen;
			} else {
				downbeatTime = nowTime;
			}


			if (++tapCount > 1) {
				// Also adjust the measure length to the time between taps:
				tapInterval = nowTime - lastTapTime;

				if (! BOTHPRESSED(btn1, btn2)) {
					// if tapInterval is closer to mL*2 than to mL, 
					// assume we are tapping half-time (1/4 notes)
					if (tapInterval > (1.5 * measureLen)) {
						tapInterval /= 2;
					}
				}
				// Othwerwise assume full time (1/8 notes)

				if (tapInterval < minTapInterval) {
					// Ignore spurious double-taps!  (This enforces a max tempo.)
					Dbg_println("ignoring bounce");
				} else {
					tapIntervals.unshift(tapInterval);
					if (tapCount == 2) {
						measureLen = tapInterval;
					} else if (tapCount == 3) {
						measureLen = (tapIntervals[1] + tapIntervals[0]) / 2;
					} else if (tapCount >= 4) {
						measureLen = (tapIntervals[2] + tapIntervals[1] + tapIntervals[0]) / 3;
					}

					// but there has to be a minimum meaure length.
					if (measureLen < (pulseLen * 2)) {
						measureLen = pulseLen * 2;
					}
				}
			} else {
				tapInterval = 0;
			}

			lastTapTime = nowTime;

		} else {
			// armed but not tapping ...
		}

	} else {
		if (btn1.rose() || btn2.rose()) 				// if we just released the buttons,
			EEPROM.put(eepromBase, measureLen); 	// save the new tempo to NVRAM.

		// not armed.
		tapCount = 0;
	}

	// Calculate blinkage.
	if (playing ) { 

		// (Strobe-off times need to be much longer than strobe-on times
		// in order to be clearly visible to the human eye.)
		if (PRESSED(btn1)) {
			if (nowTime - downbeatTime < strobeOffLen) // there's some bug here, the interval never gets long enough.
				newLed1State = LOW;
			else
				newLed1State = HIGH;
		} else {
			if (nowTime - downbeatTime < strobeOnLen)
				newLed1State = HIGH;
			else
				newLed1State = LOW;
		}

		if (PRESSED(btn2)) {
			if (nowTime - downbeatTime < strobeOffLen)
				newLed2State = LOW;
			else
				newLed2State = HIGH;
		} else {
			if (nowTime - downbeatTime < strobeOnLen)
				newLed2State = HIGH;
			else
				newLed2State = LOW;
		}

	} else { 
		// don't blink when not playing, just indicate buttons
		newLed1State = PRESSED(btn1);
		newLed2State = PRESSED(btn2);
	}

	
	// Update LEDs
	if (led1State != newLed1State) {
		led1State = newLed1State;
		digitalWrite(led1Pin, led1State);
	}
	if (led2State != newLed2State) {
		led2State = newLed2State;
		digitalWrite(led2Pin, led2State);
	}

	// Calculate pulse
	if (BOTHPRESSED(btn1, btn2)) { 
		if (tapped) // TODO: simplify? i think tapped is always true if the following if-clause is false:
			newPulseState = HIGH;
		else if (nowTime - downbeatTime >= pulseLen) 
			newPulseState = LOW;
	} else {
		if (nowTime - downbeatTime < pulseLen)
			newPulseState = HIGH;
		else if (nowTime - downbeatTime >= pulseLen) 
			newPulseState = LOW;
	}

	// Pulse if PLAYING
	if (pulseState != newPulseState) {
		pulseState = newPulseState;
		if (playing) { 
			// send sync pulse on both pins
			digitalWrite(pulsePin1, pulseState);
			digitalWrite(pulsePin2, pulseState);
		}

		if (pulseState == HIGH) {
			// print benchmarks when pulse goes high.
			Dbg_print(awakeTime);
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
			Dbg_print(measureLen);
#ifdef MICROS
			Dbg_print(" us, ");
#else
			Dbg_print(" ms, ");
#endif
			Dbg_print(AudioMemoryUsageMax());
			Dbg_print(" audioMem, ");
			Dbg_print(AudioProcessorUsageMax());
			Dbg_print(" audioCPU");
			Dbg_println(".");

#ifdef SDEBUG
			// reset counters
			loops = imus = 0;
#endif
		}
	}

	// protect against overflow of loopClock;
	if (loopClock > 10 * measureLen) { 
		Dbg_println("PROTECTION!");//DEBUG
		loopClock -= 9*measureLen;
		downbeatTime -= 9*measureLen;
		lastTapTime -= 9*measureLen;
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
	//downbeatTime += (nowTime - thenTime);

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

	awakeTime = 0;
	return who; // loop again!
}


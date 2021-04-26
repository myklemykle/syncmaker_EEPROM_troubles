// Rev 1 Pocket Integrator (PO daughterboard) firmware (c) 2021 mykle systems labs

#include "Bounce2.h"
#include "NXPMotionSense.h" // hacked version of this Teensy Prop Shield lib;
														// set to higher data rate, 
														// ignores other two IMU chips 
#include <Wire.h>
#include <EEPROM.h>

#define INTERRUPTS yeasurewelikeinterrupts.

NXPMotionSense imu;

// GUItool reqs:
#include <Audio.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

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

// measure speed of inner loop:
#define BENCHMARKS 1

// set pin numbers:
const int button1Pin = 0;   // sw1
const int button2Pin = 9;   // sw2
const int led1Pin =  20;    // led1
const int led2Pin =  21;    // led2
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
const int debounceLen = 2;
const int blinkLen = 50; 		// MS
const int pulseLen = 5; 		// MS

#ifndef INTERRUPTS
elapsedMicros imuClock;
const int clockTick = 500;
#endif

// NOTE: the prop shield calibration is stored in the EEPROM,
// we mustn't overwrite that!  
// It's rumored to be 68 bytes starting at address 0x60 -- 
//    https://forum.pjrc.com/threads/33997-EEPROM-usage-table?highlight=prop+shield+eeprom
const int eepromBase = 2000;

// vars
bool led1State = LOW, newLed1State = LOW;
bool led2State = LOW, newLed2State = LOW;
bool pulseState = LOW, newPulseState = LOW;
bool playState = LOW; bool playPinState = LOW; 
long playPinTimer = 0;
bool sleepState = HIGH; bool sleepPinState = HIGH; 

long downbeatTime = 0;        
long lastTapTime = 0;        

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long measureLen = 250; 	// default MS for 120bpm (1 beat per half/second)
unsigned int tapCount = 0;

float inertia = 0;
float prevInertia = 0;
const float shakeThreshold = 1.25;
const float tapThreshold = 2.0;
bool shaken = LOW;
bool tapped = LOW;
volatile bool imu_ready = false;

#ifdef BENCHMARKS
// tracking performance
unsigned int loops = 0; // # of main loop cycles between beats
//volatile unsigned int imus = 0; // # number of inertia checks in same.
unsigned int imus = 0; // # number of inertia checks in same.
#endif

// Objects:
Bounce btn1 = Bounce();
Bounce btn2 = Bounce();

// Macros
#define PRESSED(btn) 				(btn.read() == LOW)
#define ARMED(b1,b2) 		(PRESSED(b1) || PRESSED(b2))
#define TAP(b1,b2) 			( (PRESSED(b1) && b2.fell()) || (PRESSED(b2) && b1.fell()) )

// stolen from NXPMotionSense, where it's a private method (why?)
bool write_reg(uint8_t i2c, uint8_t addr, uint8_t val)
{
	Wire.beginTransmission(i2c);
	Wire.write(addr);
	Wire.write(val);
	return Wire.endTransmission() == 0;
}

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

	// because i changed chips, this pin (imu pin 10) is now "reserved" on the IMU:
	pinMode(IMU_fsync, OUTPUT);
	digitalWrite(IMU_fsync, 1); // ground this pin (teensy signals are "active low" so ground == 1)
	// imu pin 11 is also reserved but I can't get to it from software ...

#ifdef INTERRUPTS
	// IMU int2 pin is open-collector mode
	pinMode(IMU_int, INPUT_PULLUP);
	attachInterrupt(IMU_int, imu_int, FALLING);
#endif
	
  imu.begin();

  pinMode(led1Pin, OUTPUT);       // LED
  pinMode(led2Pin, OUTPUT);       // LED
  pinMode(pulsePin1, OUTPUT);       // LED
  pinMode(pulsePin2, OUTPUT);       // LED
  pinMode(button1Pin, INPUT_PULLUP); // Pushbutton
  pinMode(button2Pin, INPUT_PULLUP); // Pushbutton

	pinMode(PO_play, INPUT); // not sure if PULLUP helps here or not?  Flickers on & off anyway ...
	pinMode(PO_wake, INPUT);
	pinMode(PO_reset, INPUT_PULLUP);
	pinMode(PO_SWCLK, INPUT_PULLUP);
	pinMode(PO_SWDIO, INPUT_PULLUP);
	pinMode(PO_SWO, INPUT_PULLUP);

	btn1.attach(button1Pin);
	btn1.interval(debounceLen);
	btn2.attach(button2Pin);
	btn2.interval(debounceLen);

	downbeatTime = millis(); // now!

	AudioMemory(2);
	dac1.analogReference(EXTERNAL); // 3.3v p2p
	pink1.amplitude(2);
	//reverb1.reverbTime(0.5);
	amp1.gain(0);

	EEPROM.get(eepromBase, measureLen);

#ifndef INTERRUPTS
	imuClock = 0;
#endif
}

void loop()
{
	float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

	// Check the clock:

  unsigned long nowTime = millis(); 
  unsigned long awakeTime;
	unsigned long tapInterval = 0; // could be fewer bits?

#ifdef BENCHMARKS
	loops++; 
#endif

	// downbeatTime is the absolute time of the start of the current pulse.
	// if now is at least pulseLen millis beyond the previous beat, advance the beat
  if (nowTime > (downbeatTime + pulseLen)) {  
    downbeatTime += measureLen;
	}


	// Check IMU:

	shaken = LOW;
	tapped = LOW;
#ifdef INTERRUPTS
	if (imu_ready) { // on interrupt
		imu_ready = false;
#else
	if (imuClock > clockTick) { // on interval
		imuClock -= clockTick;
#endif

		imus++;
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz);

		// absolute amplitude of 3d vector
		prevInertia = inertia;

		inertia = abs(sqrt(pow(ax,2) + pow(ay,2) + pow(az,2)));  // vector amplitude
		if (inertia > shakeThreshold) 

			Serial.println(inertia);

		// use the inertia (minus gravity) to set the volume of the pink noise generator 
		/* amp1.gain(max((inertia - shakeThreshold), 0.0)); */
		//amp1.gain(max((inertia - shakeThreshold)/2.0, 0.0));
		amp1.gain(max((inertia - shakeThreshold)/4.0, 0.0));

		
		if ( (inertia > shakeThreshold) && (prevInertia <= shakeThreshold) ) {
			// TODO: shaken should be the start of sound moment, 
			// but try putting tapped at the apex-G moment; it may have better feel.
			shaken = HIGH;
			// TODO: also need to somehow debounce this, or auto-adjust threshold.
		}
		if ( (inertia > tapThreshold) && (prevInertia <= tapThreshold) ) {
			tapped = HIGH;
		}
	}

	// Check the PO state:
	// This pin is low when not playing, but when playing it's actually flickering.
	playPinState = digitalRead(PO_play);
	if (playPinState) {
		playPinTimer = nowTime;
		//if (playPinState != playState) {
			//Serial.println("play");
		//}
		playState = playPinState;
	} else {
		// Done flickering? Has play been stopped for 10ms or longer?
		if (nowTime - playPinTimer > 100) {
			//if (playPinState != playState) {
				//Serial.println("stop");
			//}
			playState = playPinState;
		}
	}

	sleepPinState = digitalRead(PO_wake);
	sleepState = sleepPinState;
	// only advance this clock when awake; let's see if we ever do fall asleep?
	if (sleepState) {
		awakeTime = nowTime; 
	}

	// Check the buttons:

	btn1.update();
	btn2.update();

	if (ARMED(btn1, btn2)) {
		newLed2State = HIGH;
		if (TAP(btn1, btn2) || tapped) { 

			// Adjust position of downbeat based on tap/shake:

			// If the next beat is closer than the previous beat
			// (dbT is greater than nowTime, but by less than mL/2))
			// or if we're in the midst of a pulse (dbT is less than nT)
			// ((dbT - (mL/2)) < nT < dbT)
				// Advance downbeat to NOW!
			// otherwise, when the previous beat is still closer (dbT > nt + (mL/2),
				// Retard downbeat to now + measureLen

			// solved for simplicity:
			downbeatTime = nowTime;
			if (downbeatTime > (nowTime + (measureLen/2))) {
				downbeatTime += measureLen;
			}

			if (++tapCount > 1) {
				// Also adjust the measure length to the time between taps:
				tapInterval = nowTime - lastTapTime;

				// if tapInterval is closer to mL*2 than to mL, 
				// assume we are tapping half-time (1/4 notes)
				if (tapInterval > (1.5 * measureLen)) {
					tapInterval /= 2;
				}
				// Othwerwise assume full time (1/8 notes)

				if (tapInterval < 100) {
					// Ignore spurious double-taps!  (This enforces a max tempo.)
				} else {
					// Compute a running average over 2 or 3 intervals if available:
					if (tapCount > 4) tapCount = 4;
					measureLen = ( ((tapCount - 2) * measureLen ) + tapInterval) / (tapCount -1) ;

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
		newLed2State = LOW;
	}

	// Calculate pulse and blink signals:

	if ((downbeatTime > 0) && (measureLen > 0)) {
		
		if (nowTime - downbeatTime < blinkLen)
			newLed1State = HIGH;
		else
			newLed1State = LOW;

		if (nowTime - downbeatTime < pulseLen)
			newPulseState = HIGH;
		else
			newPulseState = LOW;

	} else {
	}

	// Update LEDs and pulse pin

	if (led1State != newLed1State) {
		led1State = newLed1State;
		// set the LED with the led1State of the variable:
		digitalWrite(led1Pin, led1State);
	}
	if (led2State != newLed2State) {
		led2State = newLed2State;
		// set the LED with the led2State of the variable:
		digitalWrite(led2Pin, led2State);
	}
	if (pulseState != newPulseState) {
		pulseState = newPulseState;
		// set the LED with the pulseState of the variable:
		digitalWrite(pulsePin1, pulseState);
		digitalWrite(pulsePin2, pulseState);

#ifdef BENCHMARKS
		if (pulseState == HIGH) {
			// print benchmarks
			Serial.print(AudioMemoryUsageMax());
			Serial.print(" audioMem, ");
			Serial.print(AudioProcessorUsageMax());
			Serial.print(" audioCPU, ");
			Serial.print(awakeTime);
			Serial.print(':');
			Serial.print(sleepState ? "awake  " : "asleep  ");
			Serial.print(playState ? "play  " : "stop  ");
			Serial.print(loops);
			Serial.print(" loops, ");
			Serial.print(imus);
			Serial.print(" imus in ");
			Serial.print(measureLen);
			Serial.print(" ms");
			Serial.println(".");

			// reset counters
			loops = imus = 0;
		}
#endif

	}
}


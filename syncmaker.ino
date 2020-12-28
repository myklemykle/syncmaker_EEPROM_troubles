#include "Bounce2.h"
#include <NXPMotionSense.h>
#include <Wire.h>
#include <EEPROM.h>

NXPMotionSense imu;

// GUItool reqs:
#include <Audio.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthNoisePink      pink1;          //xy=355,251
AudioAmplifier           amp1;           //xy=504,249
AudioOutputAnalog        dac1;           //xy=628,246
AudioConnection          patchCord1(pink1, amp1);
AudioConnection          patchCord2(amp1, dac1);
// GUItool: end automatically generated code

// measure speed of inner loop:
#define BENCHMARKS 1

// set pin numbers:
const int button1Pin = 2;     // the number of the pushbutton pin
const int button2Pin = 3;     // the number of the pushbutton pin
const int led1Pin =  13;      	// the number of the LED pin
const int led2Pin =  15;      	// the number of the LED pin
const int pulsePin = 16; 			// sending PO/Korg sync on this pin.
const int analogOut = 14;    // 12 bit DAC on Teensy 3.2 !
const int debounceLen = 2;
const int blinkLen = 50; 		// MS
const int pulseLen = 5; 		// MS

// vars
bool led1State = LOW, newLed1State = LOW;
bool led2State = LOW, newLed2State = LOW;
bool pulseState = LOW, newPulseState = LOW;

long downbeatTime = 0;        
long lastTapTime = 0;        

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long measureLen = 250; 	// default MS for 120bpm (1 beat per half/second)
unsigned int tapCount = 0;

float inertia = 0;
float prevInertia = 0;
const int shakeThreshold = 2;
bool shaken = LOW;

#ifdef BENCHMARKS
// tracking performance
unsigned int loops = 0; // # of main loop cycles between beats
unsigned int imus = 0; // # number of inertia checks in same.
#endif

// Objects:
Bounce btn1 = Bounce();
Bounce btn2 = Bounce();

// Macros
#define PRESSED(btn) 				(btn.read() == LOW)
#define ARMED(b1,b2) 		(PRESSED(b1) || PRESSED(b2))
#define TAP(b1,b2) 			( (PRESSED(b1) && b2.fell()) || (PRESSED(b2) && b1.fell()) )

void setup()
{
  Serial.begin(115200);
  imu.begin();

	// pins!
  pinMode(led1Pin, OUTPUT);       // LED
  pinMode(led2Pin, OUTPUT);       // LED
  pinMode(pulsePin, OUTPUT);       // LED
  pinMode(button1Pin, INPUT_PULLUP); // Pushbutton
  pinMode(button2Pin, INPUT_PULLUP); // Pushbutton
	btn1.attach(button1Pin);
	btn1.interval(debounceLen);
	btn2.attach(button2Pin);
	btn2.interval(debounceLen);

	downbeatTime = millis(); // now!

	AudioMemory(20);
	dac1.analogReference(EXTERNAL); // 3.3v p2p
	pink1.amplitude(1);
	amp1.gain(0);
}

void loop()
{
	float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

	// look at the clock
  unsigned long nowTime = millis(); 
	unsigned long tapInterval = 0; // could be fewer bits?

	shaken = LOW;

#ifdef BENCHMARKS
	loops++; 
#endif

	btn1.update();
	btn2.update();

	if (imu.available()) { // When IMU becomes available (every 10 ms or so)
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);
#ifdef BENCHMARKS
		imus++;
#endif
		// absolute amplitude of 3d vector
		prevInertia = inertia;
		inertia = sqrt(pow(ax,2) + pow(ay,2) + pow(az,2)); // maybe the sqrt can be left out?
		if (inertia > shakeThreshold) 
			Serial.println(inertia);
		// use the inertia (minus gravity) to set the volume of the pink noise generator 
		amp1.gain(max((inertia - shakeThreshold)/2, 0));
		if ( (inertia > shakeThreshold) && (prevInertia <= shakeThreshold) )
			shaken = HIGH;
			// TODO: also need to somehow debounce this, or auto-adjust threshold.
	}

	// dbT is the absolute time of the start of the current pulse.
	// if now is at least pulseLen millis beyond the previous beat, advance the beat
  if (nowTime > (downbeatTime + pulseLen)) {  
    downbeatTime += measureLen;
	}

	if (ARMED(btn1, btn2)) {
		newLed2State = HIGH;
		if (TAP(btn1, btn2) || shaken) { 

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

				// Compute a running average over 2 or 3 intervals if available:
				if (tapCount > 4) tapCount = 4;
				measureLen = ( ((tapCount - 2) * measureLen ) + tapInterval) / (tapCount -1) ;

				// but there has to be a minimum meaure length.
				if (measureLen < (pulseLen * 2)) {
					measureLen = pulseLen * 2;
				}
			} else {
				tapInterval = 0;
			}

			lastTapTime = nowTime;

		} else {
			// armed but not tapping ...
		}

	} else {
		// not armed.
		tapCount = 0;
		newLed2State = LOW;
	}

	// If we have good values, we can pulse.
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
		digitalWrite(pulsePin, pulseState);

#ifdef BENCHMARKS
		if (pulseState == HIGH) {
			// print benchmarks
			Serial.print(AudioMemoryUsageMax());
			Serial.println(" audioMem");
			Serial.print(loops);
			Serial.print(" loops, ");
			Serial.print(imus);
			Serial.print(" imus in ");
			Serial.print(measureLen);
			Serial.println(" ms");

			// reset counters
			loops = imus = 0;
		}
#endif

	}
}


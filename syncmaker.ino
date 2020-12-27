#include "Bounce2.h"

// set pin numbers:
const int button1Pin = 2;     // the number of the pushbutton pin
const int button2Pin = 3;     // the number of the pushbutton pin
const int led1Pin =  13;      	// the number of the LED pin
const int led2Pin =  15;      	// the number of the LED pin
const int pulsePin = 14; 			// sending PO/Korg sync on this pin.
const int debounceLen = 2;
const int blinkLen = 50; 		// MS
const int pulseLen = 5; 		// MS
const int pPM = 2; // pulses per measure

// vars
bool led1State = LOW, newLed1State = LOW;
bool led2State = LOW, newLed2State = LOW;
bool pulseState = LOW, newPulseState = LOW;

long downbeatTime = 0;        
long lastTapTime = 0;        

// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long measureLen = 500; 	// default MS for 120bpm (1 beat per half/second)
unsigned int tapCount = 0;

// Objects:
Bounce btn1 = Bounce();
Bounce btn2 = Bounce();

// Macros
#define PRESSED(btn) 				(btn.read() == LOW)
#define ARMED(b1,b2) 		(PRESSED(b1) || PRESSED(b2))
#define TAP(b1,b2) 			( (PRESSED(b1) && b2.fell()) || (PRESSED(b2) && b1.fell()) )

void setup()
{
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
}

void loop()
{
  unsigned long nowTime = millis(); 

	btn1.update();
	btn2.update();

	// dbT is the absolute time of the start of the current pulse.
	// if now is at least pulseLen millis beyond the previous beat, advance the beat
  if (nowTime > (downbeatTime + pulseLen)) {  // TODO: handle clock wrap
    downbeatTime += (measureLen / pPM);
	}

	if (ARMED(btn1, btn2)) {
		newLed2State = HIGH;
		if (TAP(btn1, btn2)) { 

			// Adjust position of downbeat based on tap:

			// If the next beat is closer than the previous beat
			// (dbT is greater than nowTime, but by less than mL/2))
			// or if we're in the midst of a pulse (dbT is less than nT)
			// ((dbT - (mL/2)) < nT < dbT)
				// Advance time -- shift downbeat to NOW!
			// otherwise, when the previous beat is still closer (dbT > nt + (mL/2),
				// so Retard time: shift the downbeat to now plus 1 beat (measureLen/pPM)

			if (downbeatTime > (nowTime + (measureLen/(2*pPM)))) {
				downbeatTime = nowTime + (measureLen/pPM);
			} else {
				downbeatTime = nowTime;
			}

			if (++tapCount > 1) {
				// Also adjust the measure length to the time between taps
				measureLen = nowTime - lastTapTime;
				// TODO: make sure measureLen is greater than pulseLen!
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
	}
}


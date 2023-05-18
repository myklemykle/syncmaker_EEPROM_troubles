#include "EZLED.h"
#include <Arduino.h> // uint8_t, etc.
#include <elapsedMillis.h>
#include "config.h" // PWM_LED_BRIGHNESS, etc.
	
// constructor
EZLED::EZLED(uint8_t p){
	pin = p;
	pinState = 0;
#ifdef NONSTOP_HACK
	softpwm = false;
#endif
	script = NULL;
	// brightness, duration, script?
}

// call once before other methods, to init hardware.
void EZLED::init(){
	pinMode(pin, OUTPUT);
}
#ifdef NONSTOP_HACK
void EZLED::initPWM(){
	init();
	softpwm = true;
	pwmClock = micros();
}
#endif

// turn either off or full-on
void EZLED::set(bool s){
	if (s * 100 == pinState)
		return;
	if (s)
		on();
	else
		off();
}

// dim to a float percentage
void EZLED::fdim(float pct){
	if (pct == pinState)
		return;
  float newState = max(0.0, min(pct,100.0));
	if (newState == pinState)
		return;
#ifdef PWM_LED_BRIGHNESS
  analogWrite(pin, (newState / 100.0 * PWM_LED_BRIGHNESS));
#else
  // turn it on if it's nonzero.
  if (newState > 0.0)
    on();
  else
    off();
#endif
	pinState = newState;
}

// dim to an int percentage
void EZLED::dim(unsigned int pct){
	if (pct == pinState) 
		return;
  unsigned int newState = max(0, min(pct,100));
	if (newState == pinState) 
		return;
#ifdef PWM_LED_BRIGHNESS
  analogWrite(pin, (newState * PWM_LED_BRIGHNESS / 100));
#else
  // turn it on if it's nonzero.
  if (newState > 0)
    on();
  else
    off();
#endif
	pinState = newState;
}

// turn led on
void EZLED::on(){
	if (pinState == 100) {
		return;
	}

#ifdef PWM_LED_BRIGHNESS
  analogWrite(pin, PWM_LED_BRIGHNESS);
#else
  digitalWrite(pin, HIGH);
#endif
  pinState = 100;
}

// blah blah off blah
void EZLED::off(){
	if (pinState == 0) {
		return;
	}
#ifdef PWM_LED_BRIGHNESS
  analogWrite(pin, 0);
#else
  digitalWrite(pin, LOW);
#endif
  pinState = 0;
}

// start playing the animation script
void EZLED::begin(unsigned long startTime){
	if (script == NULL) {
		Dbg_println("exception: uninitialized led script");
		return; // real exception handling in arduino could be nice ...
	}
	// measure script duration:
	scriptDuration = 0;
	for (LEDCommand *i = script; i->cmd != end; i++) {
		scriptDuration += i->duration;
	}
	timer = startTime;
	running = true;
}

// halt the animation script
void EZLED::stop(){
	// TODO: how to pause & resume the timer?
	running = false;
}

void EZLED::resume(){
	// TODO: how to pause & resume the timer?
	running = true;
}

// find the current command & make sure the led is updated.
void EZLED::update(){
	if (!running) return;

	unsigned long t = timer; 
	while (timer > scriptDuration) {
		if (! looping) {
			running = false;
			return;
		}
		timer -= scriptDuration;
		// For the very-slightly-possible corner case where the timer advances past scriptDuration right after I measure it:
		t -= scriptDuration;
	}

	LEDCommand *currentCmd = script;
	if (script == NULL) {
		Dbg_println("exception: uninitialized led script");
		return; // real exception handling in arduino could be nice ...
	}

	// traverse the script to find the current command.
	while (currentCmd->duration < t){
		if (currentCmd->cmd == end) {
			Dbg_println("exception: reached end of script before duration");
			return; // yeah that would be nice
		}
		t -= currentCmd->duration;
		currentCmd++;
	}

	Dbg_println(currentCmd->brightness);

	if (currentCmd->brightness == 100) {
		Dbg_println("on");
		on();
	} else if (currentCmd->brightness == 0){
		Dbg_println("off");
		off();
	} else{
		Dbg_println("dim");
		fdim(currentCmd->brightness);
	}
}


#ifdef NONSTOP_HACK
// update the blighted PWM hack.
void EZLED::pwmUpdate(){
	if (! softpwm) return;
	if (pinState == 0) return; // darkness!

	unsigned long nowTime = micros();
	if (pwmState) {                            // is on now
		if (nowTime - pwmClock > NSLEDPWM_ON) {  // on-time expired
			pwmClock += NSLEDPWM_ON;               // adj clock
			pwmState = false;                      // turn off
		}
	} else {                                              // is off now
		if (nowTime - pwmClock > NSLEDPWM_OFF) {  // off-time expired
			pwmClock += NSLEDPWM_OFF;               // adj clock
			pwmState = true;                        // turn on
		}
	}
	set(pwmState);
}
#endif


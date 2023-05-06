#include "EZLED.h"
#include <Arduino.h> // uint8_t, etc.
#include "config.h" // PWM_LED_BRIGHNESS, etc.
	
// constructor
EZLED::EZLED(uint8_t p){
	pin = p;
	state = 0;
#ifdef NONSTOP_HACK
	pwm = false;
#endif
}

// call once before other methods, to init hardware.
void EZLED::init(){
	pinMode(pin, OUTPUT);
}
#ifdef NONSTOP_HACK
void EZLED::initPWM(){
	init();
	pwm = true;
	pwmClock = micros();
}
#endif

// turn either off or full-on
void EZLED::set(bool s){
	if (s == state)
		return;
	if (s)
		on();
	else
		off();
}

// dim to a float percentage
void EZLED::fdim(float pct){
	if (pct == state)
		return;
  float newstate = max(0.0, min(pct,100.0));
	if (newstate == state)
		return;
#ifdef PWM_LED_BRIGHNESS
  analogWrite(pin, (state / 100.0 * PWM_LED_BRIGHNESS));
#else
  // turn it on if it's nonzero.
  if (newstate > 0.0)
    on();
  else
    off();
#endif
	state = newstate;
}

// dim to an int percentage
void EZLED::dim(unsigned int pct){
	if (pct == state) 
		return;
  unsigned int newstate = max(0, min(pct,100));
	if (newstate == state) 
		return;
#ifdef PWM_LED_BRIGHNESS
  analogWrite(pin, (state * PWM_LED_BRIGHNESS / 100));
#else
  // turn it on if it's nonzero.
  if (newstate > 0)
    on();
  else
    off();
#endif
	state = newstate;
}

// turn led on
void EZLED::on(){
	if (state == 100)
		return;
#ifdef PWM_LED_BRIGHNESS
  analogWrite(pin, PWM_LED_BRIGHNESS);
#else
  digitalWrite(pin, HIGH);
#endif
  state = 100;
}

// blah blah off blah
void EZLED::off(){
	if (state == 0)
		return;
#ifdef PWM_LED_BRIGHNESS
  analogWrite(pin, 0);
#else
  digitalWrite(pin, LOW);
#endif
  state = 0;
}

// update animations, timed things ... and the blighted PWM hack.
void EZLED::update(){
	// TODO: animate dimming
#ifdef NONSTOP_HACK
	// for now: handle PWM.
	if (! pwm) return;
	if (state == 0) return;

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
#endif
}


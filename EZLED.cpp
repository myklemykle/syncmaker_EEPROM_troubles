#include "EZLED.h"
#include <Arduino.h> // uint8_t, etc.
#include <elapsedMillis.h>
#include "config.h" // LED_ANALOG_RANGE, etc.
	
// constructor
EZLED::EZLED(uint8_t p, bool isAnalog){
	pin = p;
	pinState = 0;
	analog = isAnalog;
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

  if (analog) {
		analogWrite(pin, (newState / 100.0 * LED_ANALOG_RANGE));
		pinState = newState;
	} else { 
		// turn it on if it's nonzero.
		if (newState > 0.0) {
			digitalWrite(pin, HIGH);
		  pinState = 100;
		} else {
			digitalWrite(pin, LOW);
			pinState = 0;
		}
	}
}

// dim to an int percentage
void EZLED::dim(unsigned int pct){
	if (pct == pinState) 
		return;
  unsigned int newState = max(0, min(pct,100));
	if (newState == pinState) 
		return;

	if (analog) {
		analogWrite(pin, (newState * LED_ANALOG_RANGE / 100));
		pinState = newState;
	} else {
		// turn it on if it's nonzero.
		if (newState > 0) {
			digitalWrite(pin, HIGH);
		  pinState = 100;
		} else {
			digitalWrite(pin, LOW);
			pinState = 0;
		}
	}
}

// turn led on 
void EZLED::on(){
	if (pinState == 100) {
		return;
	}

 	if (analog)
		analogWrite(pin, LED_ANALOG_RANGE);
  else 
		digitalWrite(pin, HIGH);

  pinState = 100;
}

// blah blah off blah
void EZLED::off(){
	if (pinState == 0) {
		return;
	}

	if (analog)
		analogWrite(pin, 0);
	else 
		digitalWrite(pin, LOW);

  pinState = 0;
}

void EZLED::setScript(LEDCommand cmds[], bool loop){
	script = cmds;
	looping = loop;
}

void EZLED::runScript(LEDCommand cmds[], bool loop){
	setScript(cmds, loop);
	begin();
}

void EZLED::rmScript(){
	stop();
	script = NULL;
	looping = false;
	off();
}

// measure script duration:
// If you manipulate any script command durations behind the scenes,
// call this after the change, before calling update().
void EZLED::measureScript(){
	scriptDuration = 0;
	for (LEDCommand *i = script; i->cmd != end; i++) {
		scriptDuration += (i->duration * speed / 100);
	}
}

// start playing the animation script
void EZLED::begin(unsigned long startTime){
	if (script == NULL) {
		Dbg_println("exception: uninitialized led script");
		return; // real exception handling in arduino could be nice ...
	}
	measureScript();
	timer = startTime;
	running = true;
}

// halt the animation script
void EZLED::stop(){
	timerPausedAt = timer;
	running = false;
}

void EZLED::resume(){
	running = true;
	timer = timerPausedAt;
}

// find the current command & make sure the led is updated.
void EZLED::update(){
	if (!running) return;

	while (timer > scriptDuration) {
		if (! looping) {
			running = false;
			return;
		}
		timer -= scriptDuration;
	}

	unsigned long t = timer;  
														
	// corner case:
	if (t > scriptDuration)
		t -= scriptDuration;

	LEDCommand *currentCmd = script;
	if (script == NULL) {
		Dbg_println("exception: uninitialized led script");
		return; // real exception handling in arduino could be nice ...
	}

	unsigned int realDuration;
	// traverse the script to find the current command.
	while ((realDuration = currentCmd->duration * speed / 100) < t) {
		if (currentCmd->cmd == end) {
			Dbg_println("exception: reached end of script before duration");
			return; // yeah that would be nice
		}
		t -= realDuration;
		currentCmd++;
	}

	if (currentCmd->brightness == 100) {
		on();
	} else if (currentCmd->brightness == 0){
		off();
	} else{
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


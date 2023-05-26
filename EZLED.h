#ifndef LEDS_H
#define LEDS_H
#include <Arduino.h> // uint8_t, etc.
#include <elapsedMillis.h>

#include "config.h" // LED_ANALOG_RANGE, etc.
										
// Basic management of our LEDs,
// to encompass both digital and analog (dimmed) use,
// and to encapsulate the ugly bitbanged-pwm NONSTOP_HACK.
//
//
// TODO: 
// -- allow individual leds to be either analog or digital,
// instead of a single global flag for all.
//

// LED scripting commands:
enum ledVerb { dim, fade, end };
typedef struct {
	ledVerb cmd;
	float brightness; // 0-100 inclusive
	unsigned int duration; // ms
} LEDCommand;
// An array of those is an LED script.

class EZLED {
public:
	uint8_t pin;
	float pinState;
#ifdef LED_ANALOG_RANGE
	// default to analog pins if analog is possible
	bool analog = true; 
#else
	bool analog = false;
#endif

	LEDCommand *script;
	bool looping  = false;
	int speed = 100; // (a percentage of specified animation speeds)

#ifdef NONSTOP_HACK
	// for PWM hack:
	bool softpwm; // manual PWM?
	bool pwmState; // currently on or off?
	unsigned long pwmClock;
	void initPWM(); // same as init, but turn on manual PWM (sheesh)
	void pwmUpdate(); // for flickering the PWM
#endif

	void init(); // call once before other methods to set up hardware.
	void set(bool state); // turn on or off by a bool
	// in all cases, the dim value is a percentage between 0-100 inclusive. other values are rounded.
	void fdim(float pct); 
	void dim(unsigned int pct); 
	void on();
	void off();

	// animation/script commands:
	void setScript(LEDCommand cmds[], bool loop = true);
	void runScript(LEDCommand cmds[], bool loop = true);
	void rmScript();
	void measureScript();
	void begin(unsigned long startTime=0);
	void stop();
	void resume();
	void update(); 

	// void fade(pct, interval)
	// void fade(pct, arrivalTime)

	EZLED(uint8_t p);

private:
	unsigned long scriptDuration;
	elapsedMillis timer;
	unsigned long timerPausedAt;
	bool running = false;
};

#ifdef NONSTOP_HACK
// bit-banging PWM, to dim the nonstop LED on EVT4 boards:
#define NSLEDPWM_ON 300    // usec
#define NSLEDPWM_OFF 5000  // usec
#endif


#endif // LEDS_H

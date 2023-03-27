#include "config.h"
#include "sleep.h"
#include "pins.h"
#include "Bounce2.h"
#include <elapsedMillis.h>

#ifdef MCU_RP2040

#include "RP2040Audio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#else // TEENSY
			
#include <Snooze.h>
SnoozeDigital s_digital;
SnoozeTimer s_timer;
SnoozeBlock s_config(s_timer);

#endif


// globlas from syncmaker.ino:
extern unsigned int awakePinState;
extern Bounce btn1, btn2, btn3;
extern bool btn1pressed, btn2pressed, btn3pressed;
#ifdef BUTTON4
extern Bounce btn4;
extern bool btn4pressed;
#endif
extern elapsedMicros awakeTimer;
#ifdef IMU_LSM6DSO32X
#include "lsm6dso32x.h"
extern LSM6DSO32X_IMU imu;  // STMicro IMU used from v6 onward
#elif defined(IMU_ICM42605)
#include "MotionSense.h"
extern MotionSense imu;  // on EVT1, rev2 & rev3 & evt4 the IMU is an ICM42605 MEMS acc/gyro chip
#endif

// enter low-power hibernation, 
// to be woken eithet by any button press or by
// the PO awake pin going high.

void sleep_setup(){
#ifdef MCU_RP2040
	// TODO (Not using Snooze ...)
#else // TEENSY
  ////////////
  // Sleep setup:
  s_digital.pinMode(button2Pin, INPUT_PULLUP, FALLING);  // NO-OP; wake from deepSleep not supported on this pin.
  s_timer.setTimer(SLEEPYTIME);
  /* s_compare.pinMode(PO_wake, HIGH, 1.65); */ // comparison wake from deepSleep not supported on this pin.
#endif
}


unsigned long powerNap(){

#if PI_REV < 6
	// HACK FOR THE MO: do not sleep pls.
	// This is because there's some unsolved wakeup bug, at least on Teensy.
	return 0;
#endif

	// Head towards deep sleep:

	// if (PICO_TIME_DEFAULT_ALARM_POOL_DISABLED) {
	// 	// I've confirmed that arduino-pico core does not disable this.
	// 	// It would prevent sleep on RP2040.
	// 	Dbg_println("derp!");
	// }

	Dbg_println("zzzzz.");

	// turn off LEDs
	digitalWrite(led1Pin, LOW);
	digitalWrite(led2Pin, LOW);
	digitalWrite(boardLedPin, LOW);
#if PI_REV >= 4
	digitalWrite(nonstopLedPin, LOW);
#endif

	// sleep accelerometer
	imu.sleep();

#ifdef MCU_RP2040
	// TODO
	// sleep core 1
	// attach alarm to gpio
#else // TEENSY
	// enable button wakeup
	s_config += s_digital;
	// enable timer wakeup
	s_config += s_timer;
#endif

#if PI_REV >= 9
	// power down analog reference
	digitalWrite(Aref_enable, LOW);
#endif

	unsigned long napDuration = awakeTimer;
	bool anyButtonPressed = false;
	do {
		// sleep N seconds or until right button wakes us
#ifdef MCU_RP2040
		// TODO: reduce clock speed?
		sleep_ms(SLEEPYTIME);
		// TODO: restore clock speed?
#else
		Snooze.deepSleep(s_config);
#endif 

		// good morning!
		awakePinState = analogRead(PO_wake);
		btn1.update();
		btn1pressed = (btn1.read() == LOW);
		btn2.update();
		btn2pressed = (btn2.read() == LOW);
		btn3.update();
		btn3pressed = (btn3.read() == LOW);
		anyButtonPressed = btn1pressed || btn2pressed || btn3pressed;
#ifdef BUTTON4
		btn4.update();
		btn4pressed = (btn4.read() == LOW);
		anyButtonPressed == anyButtonPressed || btn4pressed;

		// TODO: rather than polling buttons at wakeup, configure alarms to wake us if buttons are pressed.
#endif
	} while (awakePinState < awakePinThreshold && (! anyButtonPressed));

	napDuration = awakeTimer - napDuration;

#if PI_REV >= 9
	// power up analog reference
	digitalWrite(Aref_enable, HIGH);
#endif

#ifdef MCU_RP2040
	// TODO: remove alarms from gpio
	// TODO: wake core 1
#else
	// detach from buttons
	s_config -= s_digital;
	s_config -= s_timer;
#endif

	// wake accelerometer
	imu.wake();

	// TODO: animate some LEDs?
	
	Dbg_printf("Good morning! Slept %d us\n", napDuration);

	// LEDs will be restored on next loop.

	awakeTimer = 0;
	return napDuration;
}


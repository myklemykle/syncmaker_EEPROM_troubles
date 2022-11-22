
#include "sleep.h"

// enter low-power hibernation, 
// to be woken eithet by any button press or by
// the PO awake pin going high.

void sleep_setup(){
#ifdef MCU_RP2040
	// TODO
#endif
#ifdef TEENSY
  ////////////
  // Sleep setup:
  s_digital.pinMode(button2Pin, INPUT_PULLUP, FALLING);  // NO-OP; wake from deepSleep not supported on this pin.
  s_timer.setTimer(5000);
  /* s_compare.pinMode(PO_wake, HIGH, 1.65); */ // comparison wake from deepSleep not supported on this pin.
#endif
}


unsigned int powerNap(){
	unsigned int who = 0; 		// Who dares disturb my slumber??

	// HACK FOR THE MO: do not sleep pls.
	return 0;

	// Head towards deep sleep:

#ifdef SDEBUG
	Dbg_println("zzzzz.");
	delay(100);
#endif

	// turn off LEDs
	digitalWrite(led1Pin, LOW);
	digitalWrite(led2Pin, LOW);
	digitalWrite(boardLedPin, LOW);
#ifdef EVT4
	digitalWrite(nonstopLedPin, LOW);
#endif

	// disable interrupts from accelerometer
	/* detachInterrupt(IMU_int); */
	delay(100);
	// sleep accelerometer
	imu.sleep();

	// attach to buttons for button wakeup
#ifdef MCU_RP2040
	// TODO
#else
	s_config += s_digital;
	s_config += s_timer;
#endif

	do {
		// sleep N seconds or until right button wakes us
#ifdef MCU_RP2040
	// TODO
#else
		who = Snooze.deepSleep(s_config);
#endif 

		// good morning!
		awakePinState = analogRead(PO_wake);
		btn2.update();
		btn2pressed = (btn2.read() == LOW);
	} while (awakePinState < awakePinThreshold && (! btn2pressed));

	// detach from buttons
#ifdef MCU_RP2040
	// TODO
#else
	s_config -= s_digital;
	s_config -= s_timer;
#endif

	// wake accelerometer
	imu.wake();

	// reattach interrupts
	/* attachInterrupt(IMU_int, imu_int, FALLING); */

	// reset some counters
	//hc.downbeatTime += (nowTime - thenTime);

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


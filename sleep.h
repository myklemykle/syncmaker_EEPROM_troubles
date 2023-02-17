#ifndef sleep_h
#define sleep_h

// Sleep/hibernation:
//
#ifdef TARGET_RP2040
// rp2040 sleep libs here
#else
// // MK20DX256VLH7 can use Snooze library.
// #include <Snooze.h>
// SnoozeDigital s_digital;
// SnoozeTimer s_timer;
// SnoozeBlock s_config(s_timer);
#endif

// How we detect sleep:
// This pin connects to the PO codec power supply, regulated lower than the main supply.
// On Teensy 3.2, analogRead() shows this as 778 when on usb power & awake,
// then down to 300-ish when Pocket Operator sleeps.
const unsigned int awakePinThreshold = 400;

// call from setup()
void sleep_setup();

// enter low-power hibernation, 
// to be woken eithet by any button press or by
// the PO awake pin going high.
//
unsigned int powerNap();

#endif

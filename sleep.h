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

// how long we sleep the MCU between checks
#define SLEEPYTIME 1000 // ms

// How we detect sleep:
// This pin connects to the PO codec power supply, regulated lower than the main supply.
// The value is relative to a 10-bit read resolution (0-1023), which is the
// default Arduino resolution on both Teensy & RP2040.
//
// On Teensy 3.2, analogRead() shows this as 778 when on usb power & awake,
// then down to 300-ish when Pocket Operator sleeps.
//
// For V9 boards, we are seeing about 850-920 when awake, depending on battery charge, 
// down to 24 when asleep.
//
// For V10 boards: 
	// running the PO without batteries installed, while on USB power:
//
	// at boot time (while PO is waiting for clock to be set) we see 77.
	// at runtime (PO clock is set, PO is stopped): 867
	// At some point after some idle time but before sleep: 904.
	// Then, when PO sleeps: 34
	//
	// running the PO with both batteries and USB:
	// at boot time (while PO is waiting for clock to be set) we see 82.
	// at runtime: 935 (based on some random pair of batteries)
//
// NOTE: the rest of the Pocket Operator will appear to wake from a button press, but 
// its sleeping codec doesn't get powered back on until the very moment you hit PLAY,
// which is when this value goes up.
// So it's tricky to get that first beat just right after sleep, as we have no other signals.
// (Unless we start to wake on motion.)
// It would be helpful maybe to wake up, and stay awake for N seconds, on any button press
const unsigned int awakePinThreshold = 400;

// call from setup()
void sleep_setup();

// enter low-power hibernation, 
// to be woken eithet by any button press or by
// the PO awake pin going high.
//
unsigned long powerNap();

#endif

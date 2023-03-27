#ifndef __PI_MIDI_h
#define __PI_MIDI_h



//
//
// TODO: general midi defs & stuff, including wrappers for all the teensy/rp2040 diffs,
// plus always sending msgs both to USB and both UARTs.
//
#include "config.h"

#ifdef MIDI_RP2040
// Adafruit TinyUSB w/midi:
#include <MIDI.h>
#include <Adafruit_TinyUSB.h>

#else

// usbMIDI is defined automatically by Teensy libs.

#endif

class PI_MIDI {
	public:
		void clockStart();
		void clockStop();
		void clockContinue();
		void clockTick();
		void begin();
		void flushInput();
};

static PI_MIDI myMidi;




#endif // __PI_MIDI_h

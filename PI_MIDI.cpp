#include "config.h"
#include "PI_MIDI.h"

#ifdef MIDI_RP2040
// Adafruit USB MIDI object
Adafruit_USBD_MIDI usb_midi;

// Create a new instance of the (generic) Arduino MIDI Library,
// and attach usb_midi as the transport.
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
// (jeesuz what a hairy-looking macro to do something pretty basic)
// (what is this MIDI now? what type? what symbol? this is just weird.)

// Create another 2 instances for serial/trs MIDI.
// (Not necessarily connected to GPIO yet.)
// Serial1 & Serial2 are provied by the Philtower Arduino-Pico core

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI_OUT1);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI_OUT2);
#else

// usbMIDI is defined automatically by Teensy libs.

#endif


void PI_MIDI::begin(){
#ifdef MIDI_RP2040
  ///////
  // USB MIDI startup for RP2040
  // 
  // WARNING: apparently MIDI.begin() needs to happen before Serial.begin(), or else it fails silently.
  //
  // Initialize MIDI, and listen to all MIDI channels:
  MIDI_USB.begin(MIDI_CHANNEL_OMNI);
#endif
	// nothing to do for Teensy.
}

// discard any incoming MIDI messages.
void PI_MIDI::flushInput(){
#ifdef MIDI_RP2040
	while (MIDI_USB.read(MIDI_CHANNEL_OMNI))
		{ // read & ignore incoming messages
		}
	
#else
	while (usbMIDI.read())
		{ // read & ignore incoming messages
		}
#endif
}

void PI_MIDI::clockStart(){
#ifdef MIDI_RP2040
	MIDI_USB.sendStart();
	MIDI_OUT1.sendStart();
	MIDI_OUT2.sendStart();
#else
	usbMIDI.sendRealTime(usbMIDI.Start);
#endif
}

void PI_MIDI::clockStop(){
#ifdef MIDI_RP2040
	MIDI_USB.sendStop();
	MIDI_OUT1.sendStop();
	MIDI_OUT2.sendStop();
#else
	usbMIDI.sendRealTime(usbMIDI.Stop);
#endif
}

void PI_MIDI::clockContinue(){
#ifdef MIDI_RP2040
	MIDI_USB.sendContinue();
	MIDI_OUT1.sendContinue();
	MIDI_OUT2.sendContinue();
#else
	usbMIDI.sendRealTime(usbMIDI.Continue);
#endif
}

void PI_MIDI::clockTick(){
#ifdef MIDI_RP2040
	MIDI_USB.sendClock();
	MIDI_OUT1.sendClock();
	MIDI_OUT2.sendClock();
#else
	usbMIDI.sendRealTime(usbMIDI.Clock);
#endif
}

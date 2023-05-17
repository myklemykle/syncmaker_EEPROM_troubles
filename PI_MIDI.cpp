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

#ifdef SERIAL_MIDI
// PIO serial ports:
extern SerialPIO SSerialRing1;
extern SerialPIO SSerialTip1;
extern SerialPIO SSerialRing2;
extern SerialPIO SSerialTip2;

//MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI_OUT1);
MIDI_CREATE_INSTANCE(HardwareSerial, SSerialRing1, MIDI_OUT_R1);
MIDI_CREATE_INSTANCE(HardwareSerial, SSerialTip1, MIDI_OUT_T1);

// MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, MIDI_OUT2);
MIDI_CREATE_INSTANCE(HardwareSerial, SSerialRing2, MIDI_OUT_R2);
MIDI_CREATE_INSTANCE(HardwareSerial, SSerialTip2, MIDI_OUT_T2);

// MIDI_CREATE_INSTANCE(HardwareSerial, SSerialRing2, MIDI_PWR2);
//
//
// FWIW the above macro creates two symbols: 
// a MidiInterface instance called "SSerialRing2"
// and SerialMIDI instance called "serialSSerialRing2"
//
#endif // serial midi
#endif // rp2040
// in Teensy land, usbMIDI is defined automatically.


void PI_MIDI::begin(){
#ifdef MIDI_RP2040
  ///////
  // USB MIDI startup for RP2040
  // 
  // WARNING: apparently MIDI.begin() needs to happen before Serial.begin(), or else it fails silently.
  // Initialize MIDI, and listen to all MIDI channels:
  MIDI_USB.begin(MIDI_CHANNEL_OMNI);

#ifdef SERIAL_MIDI
	// For type A connectors we source from ring, sink to tip.
	// To make that work at our voltage, through out filter, we need to tug at both ends.
	//
	// Send inverted logic on ring
	SSerialRing1.setInverted(true);
	MIDI_OUT_R1.begin(MIDI_CHANNEL_OMNI);
	// At the same time, send normal logic on tip
	MIDI_OUT_T1.begin(MIDI_CHANNEL_OMNI);

	SSerialRing2.setInverted(true);
	MIDI_OUT_R2.begin(MIDI_CHANNEL_OMNI);
	MIDI_OUT_T2.begin(MIDI_CHANNEL_OMNI);
#endif

#endif
	// nothing to do for Teensy.
}

// discard any incoming USB MIDI messages.
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
#ifdef SERIAL_MIDI
	MIDI_OUT_R1.sendStart();
	MIDI_OUT_T1.sendStart();
	MIDI_OUT_R2.sendStart();
	MIDI_OUT_T2.sendStart();
#endif
	MIDI_USB.sendStart();
#else
	usbMIDI.sendRealTime(usbMIDI.Start);
#endif
}

void PI_MIDI::clockStop(){
#ifdef MIDI_RP2040
#ifdef SERIAL_MIDI
	MIDI_OUT_R1.sendStop();
	MIDI_OUT_T1.sendStop();
	MIDI_OUT_R2.sendStop();
	MIDI_OUT_T2.sendStop();
#endif
	MIDI_USB.sendStop();
#else
	usbMIDI.sendRealTime(usbMIDI.Stop);
#endif
}

void PI_MIDI::clockContinue(){
#ifdef MIDI_RP2040
#ifdef SERIAL_MIDI
	MIDI_OUT_R1.sendContinue();
	MIDI_OUT_T1.sendContinue();
	MIDI_OUT_R2.sendContinue();
	MIDI_OUT_T2.sendContinue();
#endif
	MIDI_USB.sendContinue();
#else
	usbMIDI.sendRealTime(usbMIDI.Continue);
#endif
}

void PI_MIDI::clockTick(){
#ifdef MIDI_RP2040
#ifdef SERIAL_MIDI
	MIDI_OUT_R1.sendClock();
	MIDI_OUT_T1.sendClock();
	MIDI_OUT_R2.sendClock(); // inverted
	MIDI_OUT_T2.sendClock(); // normal
#endif
	MIDI_USB.sendClock();
#else
	usbMIDI.sendRealTime(usbMIDI.Clock);
#endif
}

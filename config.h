// CONFIG AREA:
//
/////////////////
// 1) the big question: what board are we building for?
//
// is it EVT4: SPI comms, IMU interrupt 1, misc pin changes, added nonstop button and light.
#define EVT4 itsathing
//	
// or is it REV3: I2C comms, IMU interrupt 2
// (rev3 is the default when no other board is defined)
	

///////////////////
// 2) what MIDI signals should we try to send?
//
// send MIDI clock?
#define MIDICLOCK timex
//
// send MIDI timecode?
//#define MIDITIMECODE rolex

	
//////////////////////
// 3) send debug output to usb serial?
// (disable in production)
#define SDEBUG crittersbuggin


//////////////////////
// 4) should the IMU report new data via interrupt?
// #define IMU_INTERRUPTS excuseme!









//////////// ////////////// ////////////// ////////////// //////////////
// END CONFIG AREA.  The rest below is derived from above.
//////////// ////////////// ////////////// ////////////// //////////////



// using SPI for IMU?  (default is i2c)
#ifdef EVT4
#define IMU_SPI cuzitsgroovy!
// I don't yet perceive any improvement over 2khz, for the record
#define IMU_8KHZ fasterpussycat!
//#define IMU_4KHZ gospeedracer
#endif

#ifdef SDEBUG
// Teensy will eventually hang on a clogged output buffer if we Serial.print() without USB plugged in.
// is there some more normal solution for this?  Seems like this would be a common problem.
#define Dbg_print(X) if(Serial) Serial.print(X)
#define Dbg_print2(X, Y) if(Serial) Serial.print(X, Y)
#define Dbg_println(X) if(Serial) Serial.println(X)
#define Dbg_flush(X) if(Serial) Serial.flush()
#else
#define Dbg_print(X) {}
#define Dbg_println(X) {}
#define Dbg_flush(X) {}
#endif

#ifdef MIDITIMECODE
#include "miditimecode.cpp"
#endif

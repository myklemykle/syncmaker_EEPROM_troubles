#ifndef __syncmaster_config_h
#define __syncmaster_config_h

// CONFIG AREA:
//
/////////////////
// 1) the big question: what board are we building for?
//
// is it v6/v7: RP2040, misc pin changes, SPI but a different IMU, added bottom button & JTAG header
//#define PI_V6 pleasebethelastone
//
// is it EVT4: SPI comms, IMU interrupt 1, misc pin changes, added nonstop button and light.
#define EVT4 itsathing
//	
// or is it REV3: I2C comms, IMU interrupt 2
// (rev3 is the default when no other board is defined)
//

////////////////////
// 1.5) are we hardcoding for a particular model of Pocket Operator?
// (in the future I hope we aren't.)
// (one must be defined!)
// These keep the LED steady on when playing (with caveats):
//#define MGRP_A PO12,PO14,PO16
// These keep the LED mostly off, and just flash it every 4 beats:
//#define MGRP_B PO22,24,28,33
// These keep the LED mostly on, but wink it off every 4 beats:
//#define MGRP_C PO32,35
	

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
// 5) should we try to manage IMU calibration data?
//#define IMU_CALIBRATION sweepingthenation




//////////// ////////////// ////////////// ////////////// //////////////
// END CONFIG AREA.  The rest below is derived from above.
//////////// ////////////// ////////////// ////////////// //////////////


#ifdef PI_V6
#define MCU_RP2040 /* mcu is raspPi */
#define IMU_LSM6DSO32X /* imu from TDK */
#define SPIPORT SPI1 /* from RP2040's SPI.h */
#else
#define TEENSY32 /* mcu is a NXP cortex-m4 */
#define IMU_ICM42605 /* imu from STMicro */
#define SPIPORT SPI // from Teensy's SPI.h 
#endif


// using SPI for IMU?  (default is i2c)
#if defined(EVT4) || defined(PI_V6)
#define NONSTOPBUTTON pressmepressme
#define IMU_SPI cuzitsgroovy!
// I don't yet perceive any improvement over 2khz, for the record
//#define IMU_8KHZ fasterpussycat!
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

#include "pins.h"

#endif /* __syncmaster_config_h */

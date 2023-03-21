#ifndef __syncmaster_config_h
#define __syncmaster_config_h

// CONFIG AREA:
//
/////////////////
// 1) the big question: what board are we building for?
// (these are defined in Makefile targets)
//

// is it v6/v7: RP2040, misc pin changes, SPI but a different IMU, added bottom button & JTAG header
#ifdef PI_V6 				// the symbol PI_V* is deprecated;
#define PI_REV 6 		// PI_REV stores an integer instead.

#elif defined(PI_V9)
#define PI_REV 9

// HACK: the inscrutible arduino-cli lets me pass -DPI_V6 unmolested when compiling rp2040,
// but swallows/erases/loses all my build options when compiling Teensy.  
// So I can't just -DEVT4. But I can do this, assuming i support no other Teensy3.2 platform:
#elif defined(ARDUINO_TEENSY32)
// is it EVT4: SPI comms, IMU interrupt 1, misc pin changes, added nonstop button and light.
#define EVT4 itsathing 		// symbol EVT4 is deprecated too.
#define PI_REV 4

//	
// or is it REV3: I2C comms, IMU interrupt 2
// (rev3 is the default when no other board is defined)
// This has not been tested in a while and should be officially dropped ...
//
#else
#define PI_REV 3

#endif



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

#include <SPI.h>

// Is IMU on the SPI bus? (used to be i2c)
#if PI_REV >= 4
#define IMU_SPI cuzitsgroovy!
#endif

#if PI_REV >= 6 		/* RP2040 versions */
#define MCU_RP2040
#define IMU_LSM6DSO32X /* imu from STMicro */
#define SPIPORT SPI1   /* the default SPI device name, from RP2040's SPI.h */
//#define IMU_1_666KHZ 
#define IMU_3_333KHZ  /* imu update rate */
#define AUDIO_RP2040
#define MIDI_RP2040
#define BUTTON4 					/* from v6 on, we have a fourth (reset) button */
#define PWM_LED_BRIGHNESS /* from v6 on, all the LEDs are on analog pins */
#endif


#if PI_REV == 4 		/* Teensy 3.2 version: */
#define MCU_NXP
#define TEENSY32 		/* mcu is a NXP cortex-m4 */
#define IMU_ICM42605 /* imu from TDK */
#define SPIPORT SPI  /* default from Teensy's SPI.h */
// I don't yet perceive any improvement over 2khz, for the record
//#define IMU_8KHZ fasterpussycat!
//#define IMU_4KHZ gospeedracer
#define NONSTOP_HACK // bit-banging PWM on the too-bright LED on that rev:
#define AUDIO_TEENSY
#define MIDI_TEENSY
#endif


#ifdef SDEBUG
// Teensy will eventually hang on a clogged output buffer if we Serial.print() without USB plugged in.
// is there some more normal solution for this?  Seems like this would be a common problem.
// https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html 
#define Dbg_println(...) if(Serial) Serial.println(__VA_ARGS__)
#define Dbg_printf(...) if(Serial) Serial.printf(__VA_ARGS__)
#define Dbg_print(...) if(Serial) Serial.print(__VA_ARGS__)
#define Dbg_flush(X) if(Serial) Serial.flush()

#else
#define Dbg_println(...) {}
#define Dbg_printf(...) {}
#define Dbg_print(...) {}
#define Dbg_flush(X) {}
#endif

#include "pins.h"

#endif /* __syncmaster_config_h */

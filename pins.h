#ifndef __syncmaster_pins_h
#define __syncmaster_pins_h


// Pins:
#ifdef PI_V6
const int button1Pin = 10;   // sw1 = gpio10
const int button2Pin = 24;   // sw2 = gpio24
const int button3Pin = 0;   // sw3/nonstop
const int button4Pin = 5;   // sw4/reset
const int led1Pin =  9;    // led1
const int led2Pin =  23;    // led2
const int led3Pin = 8; // led3/nonstop led
const int nonstopLedPin = led3Pin; 
const int led4Pin = 6;	// builtin led on Teensy 3.2, or led4 of v6
const int boardLedPin = led4Pin;
const int tip1 = 19; 	// tip1 / l_sync_out
const int tip2 = 21; 		// tip2 / r_sync_out
const int ring1 = 18;
const int ring2 = 20;
const int PO_play = 4; 		// goes high on PO play (runs to play LED)
const int PO_wake = 26;			// goes low when PO sleeps I think  gpio26/adc0
const int PO_reset = 22;			
const int PO_SWCLK = 1;			// jtag/stlink pins:
const int PO_SWDIO = 2;
const int PO_SWO   = 3;
const int IMU_int = 11;
const int SPI_clock = 14;
const int SPI_cs = 13;
const int SPI_miso = 12;
const int SPI_mosi = 15;
#elif defined(EVT4)
const int button1Pin = 22;   // sw1
const int button2Pin = 21;   // sw2
const int boardLedPin = 13;	// builtin led on Teensy 3.2
const int led1Pin =  15;    // led1
const int led2Pin =  8;    // led2
const int button3Pin = 0;   // nonstop
const int nonstopLedPin = 1; // nonstop led
const int tip1 = 20; 	// j1 tip / l_sync_out
const int tip2 = 23; 		// j2 tip / r_sync_out
const int PO_play = 16; 		// goes high on PO play (runs to play LED)
const int PO_wake = 14;			// goes low when PO sleeps I think
const int PO_reset = 7;			
const int PO_SWCLK = 6;			// jtag/stlink pins:
const int PO_SWDIO = 5;
const int PO_SWO   = 4;
/* const int IMU_fsync = -1;   // not connected on evt4 */
const int IMU_int = 2;
const int SPI_clock = 13;
const int SPI_cs = 10;
const int SPI_miso = 12;
const int SPI_mosi = 11;
#else
const int button1Pin = 0;   // sw1
const int button2Pin = 9;   // sw2
const int boardLedPin = 13;	// builtin led on Teensy 3.2
const int led1Pin =  21;    // led1
const int led2Pin =  20;    // led2
const int tip1 = 11; 	// j1 tip
const int tip2 = 8; 		// j2 tip
const int PO_play = 16; 		// goes high on PO play (runs to play LED)
const int PO_wake = 17;			// goes low when PO sleeps I think
const int PO_reset = 7;			
const int PO_SWCLK = 6;			// jtag/stlink pins:
const int PO_SWDIO = 5;
const int PO_SWO   = 4;
const int IMU_fsync = 1;   // need to ground this on icm-42605
const int IMU_int = 2;
#endif

#endif /* __syncmaster_pins */

#ifndef __syncmaster_pins_h
#define __syncmaster_pins_h


// Pins:
#if PI_REV == 9
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
const int outPins[4] = { tip1, ring1, tip2, ring2 };
const int PO_play = 4; 		// goes high on PO play (runs to play LED)
const int PO_wake = 26;			// goes low when PO sleeps -- gpio26/adc0
const int PO_reset = 22;			
const int PO_SWCLK = 1;			// jtag/stlink pins:
const int PO_SWDIO = 2;
const int PO_SWO   = 3;
const int IMU_int = 11;
const int SPI_clock = 14;
const int SPI_cs = 13;
const int SPI_miso = 12;
const int SPI_mosi = 15;
// drive Aref_enable high to enable 2.5v audio power supply
const int Aref_enable = 27; 
// Unused pins must be tied high or low, or else the whole CMOS chip can glitch.
const int unusedPins[6] = { 7,16,17,25,28,29 };
// These are normally unused, but used in testing/debug:
// a UART for picoprobe or misc future handiness, goes to JTAG pins
const int PI_swo = 16; // UART0 TX
const int PI_txin = 17; // UART0 RX
// For testing/tuning crystal, RP2040 can send clk_sys direct to this pin:
const int PI_clk_sys = 25;

#elif PI_REV >= 6
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
const int outPins[4] = { tip1, ring1, tip2, ring2 };
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
const int unusedPins[7] = { 7,16,17,25,27,28,29 };

#elif PI_REV == 4
const int button1Pin = 22;   // sw1
const int button2Pin = 21;   // sw2
const int boardLedPin = 13;	// builtin led on Teensy 3.2
const int led4Pin = boardLedPin;	
const int led1Pin =  15;    // led1
const int led2Pin =  8;    // led2
const int button3Pin = 0;   // nonstop
const int nonstopLedPin = 1; // nonstop led
const int led3Pin = nonstopLedPin;
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

#ifndef __syncmaster_config_h
#define __syncmaster_config_h

#include <SPI.h>

#define Dbg_println(...) if(Serial) Serial.println(__VA_ARGS__)
#define Dbg_printf(...) if(Serial) Serial.printf(__VA_ARGS__)
#define Dbg_print(...) if(Serial) Serial.print(__VA_ARGS__)
#define Dbg_flush(X) if(Serial) Serial.flush()

#include "pins.h"

#endif /* __syncmaster_config_h */

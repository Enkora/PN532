#ifndef __DEBUG_H__
#define __DEBUG_H__

//#define DEBUG_X

#include "Arduino.h"

#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define DEBUG_SERIAL SerialUSB
#else
    #define DEBUG_SERIAL Serial
#endif

#ifdef DEBUG_X
#define DMSG(args...)       DEBUG_SERIAL.print(args)
#define DMSG_STR(str)       DEBUG_SERIAL.println(str)
#define DMSG_HEX(num)       DEBUG_SERIAL.print(' '); DEBUG_SERIAL.print((num>>4)&0x0F, HEX); DEBUG_SERIAL.print(num&0x0F, HEX)
#define DMSG_INT(num)       DEBUG_SERIAL.print(' '); DEBUG_SERIAL.print(num)
#else
#define DMSG(args...)
#define DMSG_STR(str)
#define DMSG_HEX(num)
#define DMSG_INT(num)
#endif

#endif

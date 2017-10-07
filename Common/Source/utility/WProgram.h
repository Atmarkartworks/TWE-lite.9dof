#ifndef WProgram_h
#define WProgram_h

//#include <stdlib.h>
//#include <string.h>
//#include <math.h>

/*
  * type define for arduino
  */

typedef bool boolean;
typedef unsigned char byte;

/*
 * /cygdrive/c/Dev/MonoWireless/SDK_blue/TWESDK/Tools/ba-elf-ba2/bin/../lib/gcc/ba-elf/4.1.2/../../../../ba-elf/include/stdint.h
 */

//typedef unsigned short uint8_t;
//typedef unsigned int uint16_t;
//typedef short int8_t;
//typedef int int32_t;

#include "jendefs.h"
#include "AppHardwareApi.h"
#include "string.h"

#if defined __cplusplus
extern "C" {
#endif

#include "utils.h"

#include "sensor_driver.h"
//#include "SHT21.h"
#include "SMBus.h"

#include <stdint.h>
//#include <SMBus.h>

//#include <jendefs.h>
extern uint32 u32TickCount_ms; //!< TOCONETのタイムスタンプ @ingroup DUPCHK


#if defined __cplusplus
}
#endif

#endif

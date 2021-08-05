#ifndef __DEBUG_H
#define __DEBUG_H

// turn debugging on or off
#ifndef DEBUG
//#define DEBUG
#endif

// #########################
// # Debug stuff
// #########################
#if DEBUG == 1
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#endif
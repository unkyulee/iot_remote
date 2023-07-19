#ifndef IR_h
#define IR_h

#include <ArduinoJson.h>

// config control
void IR_setup();
void IR_loop();

/*
 * Specify which protocol(s) should be used for decoding.
 * If no protocol is defined, all protocols (except Bang&Olufsen) are active.
 * This must be done before the #include <IRremote.hpp>
 */
#define DECODE_DENON // Includes Sharp
#define DECODE_JVC
#define DECODE_KASEIKYO
#define DECODE_PANASONIC // alias for DECODE_KASEIKYO
#define DECODE_LG
#define DECODE_NEC // Includes Apple and Onkyo
#define DECODE_SAMSUNG
#define DECODE_SONY
#define DECODE_RC5
#define DECODE_RC6
#define DECODE_BOSEWAVE
#define DECODE_LEGO_PF
#define DECODE_MAGIQUEST
#define DECODE_WHYNTER
#define DECODE_FAST
#define DECODE_DISTANCE_WIDTH // Universal decoder for pulse distance width protocols
#define DECODE_HASH           // special decoder for all protocols
// #define DECODE_BEO          // This protocol must always be enabled manually, i.e. it is NOT enabled if no protocol is defined. It prevents decoding of SONY!
// #define DEBUG               // Activate this for lots of lovely debug output from the decoders.
#define RAW_BUFFER_LENGTH 512 // Default is 112 if DECODE_MAGIQUEST is enabled, otherwise 100.

#define IR_RECEIVE_PIN 23 // D15
#define IR_SEND_PIN 22    // D4
#define TONE_PIN 27       // D27 25 & 26 are DAC0 and 1
#define LED_BUILTIN 21
#define APPLICATION_PIN 16 // RX2 pin
#define SEND_PWM_BY_TIMER  // We do not have pin restrictions for this CPU's, so lets use the hardware PWM for send carrier signal generation
#define FLASHEND 0xFFFF    // Dummy value for platforms where FLASHEND is not defined
#define RAMEND 0xFFFF      // Dummy value for platforms where RAMEND is not defined
#define RAMSIZE 0xFFFF     // Dummy value for platforms where RAMSIZE is not defined

#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

#endif
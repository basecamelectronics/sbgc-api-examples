/******************************************************************************
This library is used by Arduino examples that show 
how to control SimpleBGC-driven gimbal via Serial API.
API specs are available at http://www.basecamelectronics.com/serialapi/
  
Place SBGC libraries to you Arduino IDE 'libraries' location.
  
!!!WARNING!!! Its recommended to increase the serial buffer size to 150 or more in the Arduino IDE, 
(default buffer 64 bytes is not enough to fit incoming SBGC API commands)

Copyright (c) 2014-2015 Aleksey Moskalenko
*******************************************************************************/


#ifndef  __SBGC_Arduino__
#define  __SBGC_Arduino__

#include <inttypes.h>
#include "Arduino.h"
#include <SBGC.h>


#define LED_PIN  13

extern SBGC_Parser sbgc_parser; 

void SBGC_Demo_setup(Stream *serial);
inline void LED_ON() {   digitalWrite(LED_PIN, HIGH); }
inline void LED_OFF() {    digitalWrite(LED_PIN, LOW); }
void blink_led(uint8_t cnt);





#endif
/******************************************************************************
  This library is used by Arduino examples that show 
  how to control SimpleBGC-driven gimbal via Serial API.
  API specs are available at http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksey Moskalenko
*******************************************************************************/
#include "Arduino.h"
#include "SBGC_Arduino.h"


/* Defines serial port routines required for SBGC_parser, here */
class ArduinoComObj : public SBGC_ComObj {
	Stream *serial;
	public:
	inline void init(Stream *s) {
		serial = s;
	}

	virtual uint16_t getBytesAvailable() {
		return serial->available();
	}
	
	virtual uint8_t readByte() {
		return serial->read();
	}
	
	virtual void writeByte(uint8_t b) {
		serial->write(b);
	}
	
	// Arduino com port is not buffered, so empty space is unknown.
	virtual uint16_t getOutEmptySpace() {
		return 0xFFFF;
	}

};


/* Global variables */
SBGC_Parser sbgc_parser;  // SBGC command parser. Define one for each port.
ArduinoComObj com_obj; // COM-port wrapper required for parser



// Prepare hardware, used in examples
void SBGC_Demo_setup(Stream *serial) {
  com_obj.init(serial);
  sbgc_parser.init(&com_obj);
  
  // Use LED to show steps of program run
  pinMode(13, OUTPUT);
  LED_ON();
}



void blink_led(uint8_t cnt) {
	for(uint8_t i=0; i<cnt; i++) {
		LED_OFF();
		delay(150);
		LED_ON();
		delay(200);
	}
}

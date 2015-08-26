/* 
	SimpleBGC Serial API  library - RC (Remote Control) definitions
	More info: http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksei Moskalenko
  All rights reserved.
	
	See license info in the SBGC.h
*/   
#ifndef  __SBGC_rc__
#define  __SBGC_rc__

// RC channels used in the SBGC controller
#define SBGC_RC_NUM_CHANNELS 6 // ROLL, PITCH, YAW, CMD, EXT_ROLL, EXT_PITCH

// Hardware RC inputs, as labeled on the board
#define SBGC_RC_INPUT_NO 0
#define SBGC_RC_INPUT_ROLL 1
#define SBGC_RC_INPUT_PITCH 2
#define SBGC_RC_INPUT_EXT_ROLL 3
#define SBGC_RC_INPUT_EXT_PITCH 4
#define SBGC_RC_INPUT_YAW 5 // not connected in 1.0 board

// Analog inputs (board v.3.0)
#define SBGC_RC_INPUT_ADC1 33
#define SBGC_RC_INPUT_ADC2 34
#define SBGC_RC_INPUT_ADC3 35



// Bit indicates input is in analog mode
#define SBGC_RC_INPUT_ANALOG_BIT (1<<5) // 32
// Bit indicates input is a virtual channel
#define SBGC_RC_INPUT_VIRT_BIT (1<<6) // 64
// Bit indicates input is a API virtual channel
#define SBGC_RC_INPUT_API_VIRT_BIT (1<<7) // 128

// Mask to separate input channel number from input mode
#define SBGC_RC_INPUT_CH_MASK ((1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4)) 
#define SBGC_RC_INPUT_MODE_MASK ((1<<5) | (1<<6) | (1<<7))


// Number of virtual channels for RC serial input (s-bus, spektrum, Sum-PPM)
#define SBGC_VIRT_NUM_CHANNELS 32
// Number of virtual channels for API serial input
#define SBGC_API_VIRT_NUM_CHANNELS 32


// Normal range of RC signal. Real signal may go outside this range
#define SBGC_RC_MIN_VAL -500
#define SBGC_RC_MAX_VAL 500
// Value to encode 'RC no signal'
#define SBGC_RC_UNDEF -10000



#endif // __SBGC_rc__

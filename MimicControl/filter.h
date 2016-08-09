/*
This is a part of examples of usage of SimpleBGC Serial API.
API specs are available at http://www.basecamelectronics.com/serialapi/

Copyright (c) 2016 Aleksei Moskalenko
*/

#ifndef FILTER_H_
#define FILTER_H_

#include <inttypes.h>

typedef struct {
	uint16_t last_time_ms;
	uint8_t  state; // current state
	uint8_t trigger_state; // de-bounced state
} btn_state_t;

uint8_t debounce_button(btn_state_t &btn, uint8_t new_state);


/* Exponential moving average filter (optimized for integers) with factor = 2^n
 * It correctly handles averaging of 16bit angles with overflow
 * */
class avg_var16 {
	int32_t buf; // internal buffer to store non-rounded average value
	uint8_t factor;

public:
	int16_t res; // result (rounded to int)

	void init(uint8_t _factor) {
		factor = _factor;
		buf = 0;
		res = 0;
	}

	inline void average(int16_t data) {
		buf+= (int32_t)data - (int32_t)res;
		res = (int16_t)(buf >> factor);
	}
};



#endif /* FILTER_H_ */
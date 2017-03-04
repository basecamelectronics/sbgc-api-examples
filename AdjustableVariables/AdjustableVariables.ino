/******************************************************************************
  This is example sketch for Arduino, that
  shows how to control SimpleBGC-driven gimbal via Serial API.
  API specs are available at http://www.basecamelectronics.com/serialapi/
  
  Demo: Change gimbal's parameters in real-time using Adjustable Variables
  
  Arduino hardware:  
   - analog potentiometer on the pin A1  (connect GND, +5V to its side outputs)
  
  Gimbal settings:
   - RC control in SPEED mode, RC signal should come from active RC source 
  
  Copyright (c) 2014 Aleksei Moskalenko
*******************************************************************************/
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>



// Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200

// Set serial port where SBGC32 is connected
#define serial Serial


void setup() {
	serial.begin(SERIAL_SPEED);
	SBGC_Demo_setup(&serial);

  // Take a pause to let gimbal controller to initialize
  delay(3000);
}

void loop() {
  SBGC_cmd_set_adj_vars_var_t adj_vars[] = {
	  { ADJ_VAR_RC_SPEED_ROLL, 0 },
	  { ADJ_VAR_RC_SPEED_PITCH, 0 },
	  { ADJ_VAR_RC_SPEED_YAW, 0 }
  };


  blink_led(1);
  ////////////// Demo1: Gradually increase the speed of RC control for all axes
  // You can see the speed changes by applying RC control from you remote or joystick
  for(int i=0; i<100; i++) {
	  adj_vars[0].val = i;
	  adj_vars[1].val = i;
	  adj_vars[2].val = i;

	  SBGC_cmd_set_adj_vars_send(adj_vars, 3, sbgc_parser);
	  delay(100); // 100*100 = 10 seconds to finish demo
  }
  delay(1000);


  blink_led(2);
  ////////////// Demo2: Use the value from analog potentiometer to control speed of RC control for all axes
  // You can see the speed changes by applying RC control from you remote or joystick
  for(int i=0; i<500; i++) {
	  uint8_t speed = (analogRead(1)>>2); // range 0..1023 -> 0..255
	  adj_vars[0].val = speed;
	  adj_vars[1].val = speed;
	  adj_vars[2].val = speed;

	  SBGC_cmd_set_adj_vars_send(adj_vars, 3, sbgc_parser);
	  delay(20); // 20*500 = 10 seconds to finish demo
  }
}

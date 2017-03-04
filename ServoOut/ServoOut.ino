/******************************************************************************
  This is example sketch for Arduino.
  Shows how to control SimpleBGC-driven gimbal via Serial API.
  API specs are available at http://www.basecamelectronics.com/serialapi/
  
  Demo: 
   - Control hobby servo connected to the board.  
   - Triger the state of AUX pins.
   
  Arduino hardware:
  - no
  
  Gimbal settings:
  - Hobby servo connected to FC_ROLL port 
  - J1 jumper is soldered to put +5V power to the central pin of JR connector
  - FC_ROLL input is not occupied in the RC configuration
  - PWM rate is set to value safe for your servo (50..100 Hz)
  - Buzzer connected to buzzer port
  - Tester on the AUX1 pin to check its signal level

  Copyright (c) 2014-2015 Aleksey Moskalenko
*******************************************************************************/
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>


// Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200

// Set serial port where SBGC32 is connected
#define serial Serial


// All servo outputs are disabled (not occupied and floating)
SBGC_cmd_servo_out_t cmd_servo_out = { { SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED } };


void setup() { 
	serial.begin(SERIAL_SPEED);
	SBGC_Demo_setup(&serial);

	// Take a pause to let gimbal controller to initialize
	delay(3000);
	
	// Use servo1 output (labeled as FC_ROLL) and  set it to low level (no PWM generation)
	cmd_servo_out.servo[0] = 0; 
	SBGC_cmd_servo_out_send(cmd_servo_out, sbgc_parser);
	delay(1000);
} 


void loop() { 
	
  blink_led(1);
  /////////// Demo1:  control servo connected to FC_ROLL
  // Move servo1 to neutral position
	cmd_servo_out.servo[0] = 1500;
	SBGC_cmd_servo_out_send(cmd_servo_out, sbgc_parser);
	delay(1000);


  // Slowly move servo1 from neutral position to top
  for(cmd_servo_out.servo[0]=1500; cmd_servo_out.servo[0]<2500; cmd_servo_out.servo[0]+= 5) {
	SBGC_cmd_servo_out_send(cmd_servo_out, sbgc_parser);
  	delay(20);
  }

  // Slowly move servo1 from top to bottom
  for(cmd_servo_out.servo[0]=2500; cmd_servo_out.servo[0]>500; cmd_servo_out.servo[0]-= 5) {
	SBGC_cmd_servo_out_send(cmd_servo_out, sbgc_parser);
  	delay(20);
  }
  delay(1000);
  


  blink_led(2);
  ////////// Demo2:  Control the state of AUX pins and BUZZER port
  SBGC_cmd_trigger_t t = { 0, 0 };
  t.pin = SBGC_PIN_AUX1;
  t.state = 1;
  SBGC_cmd_trigger_send(t, sbgc_parser);
  t.pin = SBGC_PIN_BUZZER;
  t.state = 1;
  SBGC_cmd_trigger_send(t, sbgc_parser);
  delay(500);
  t.pin = SBGC_PIN_AUX1;
  t.state = 0;
  SBGC_cmd_trigger_send(t, sbgc_parser);
  t.pin = SBGC_PIN_BUZZER;
  t.state = 0;
  SBGC_cmd_trigger_send(t, sbgc_parser);
  delay(1000);
} 

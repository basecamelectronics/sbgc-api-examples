/******************************************************************************
	This is example sketch for Arduino.
	Shows how to control SimpleBGC-driven gimbal via Serial API.
	API specs are available at http://www.basecamelectronics.com/serialapi/

	Demo:  control camera angles by Serial API direct control and by
	emulating various RC input methods;

	Arduino hardware:
	- analog joystick on the pins A1, A2  (connect GND, +5V to the side outputs of its potentiometers)

	Gimbal settings:
	- RC control in SPEED mode, RC signal should come from active RC source
	- RC SPEED is set to about 30..100

	Copyright (c) 2014-2015 Aleksey Moskalenko
*******************************************************************************/
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>


// Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200

// delay between commands, ms
#define SBGC_CMD_DELAY 10

/*****************************************************************************/

// Set serial port where SBGC32 is connected
#define serial Serial

void setup() {
	serial.begin(SERIAL_SPEED);
	SBGC_Demo_setup(&serial);

	// Take a pause to let gimbal controller to initialize
	//delay(1000);
}


void loop() {
	SBGC_cmd_control_t c = { 0, 0, 0, 0, 0, 0, 0 };


	// Move camera to initial position (all angles are zero)
	// Set speed 30 degree/sec
	c.mode = SBGC_CONTROL_MODE_ANGLE;
	c.speedROLL = c.speedPITCH = c.speedYAW = 100 * SBGC_SPEED_SCALE;
	SBGC_cmd_control_send(c, sbgc_parser);
	delay(1000);

	
	/*	
	blink_led(1);
	/////////////////// Demo 1. PITCH and YAW gimbal by 40 and 30 degrees both sides and return back.
	// Actual speed depends on PID setting.
	// Whait 5 sec to finish
	c.mode = SBGC_CONTROL_MODE_ANGLE;
	c.anglePITCH = SBGC_DEGREE_TO_ANGLE(60);
	SBGC_cmd_control_send(c, sbgc_parser);
	delay(2000);

	c.anglePITCH = SBGC_DEGREE_TO_ANGLE(-60);
	SBGC_cmd_control_send(c, sbgc_parser);
	delay(3000);

	// .. and back
	c.anglePITCH = 0;
	SBGC_cmd_control_send(c, sbgc_parser);
	delay(1000);


	blink_led(2);
	/////////////////// Demo 2. Pitch gimbal down with constant speed 10 degree/sec
	// by 50 degree (it takes 5 sec)
	// (this is simplified version of speed control. To prevent jerks, you should
	// add acceleration and de-acceleration phase)
	c.mode = SBGC_CONTROL_MODE_SPEED;
	c.speedPITCH = 40 * SBGC_SPEED_SCALE;
	c.speedROLL = 0;
	c.speedYAW = 0;
	SBGC_cmd_control_send(c, sbgc_parser);
	delay(2000);

	// Stop
	c.speedPITCH = 0;
	SBGC_cmd_control_send(c, sbgc_parser);
	delay(100);

	// .. and back
	c.speedPITCH = -40 * SBGC_SPEED_SCALE;
	SBGC_cmd_control_send(c, sbgc_parser);
	delay(4000);

	// Stop
	c.speedPITCH = 0;
	SBGC_cmd_control_send(c, sbgc_parser);
	delay(100);

	*/
	
	blink_led(3);
	//////////// Demo 4. More complicated example: Pitch gimbal by 40 degrees
	// with the full control of speed and angle.
	//  - send control command with the fixed frame rate
	//  - angle is calculated by the integration of the speed
	float speed = 0, angle = 0;
	c.mode = SBGC_CONTROL_MODE_SPEED_ANGLE;
	c.speedROLL = 0;
	c.speedYAW = 0;

	// acceleration phase
	while(angle < 40.0f) {
		speed+= 1.0f;
		c.speedPITCH = speed * SBGC_SPEED_SCALE;
		angle+= speed * SBGC_CMD_DELAY / 1000.0f;  // degree/sec -> degree/ms
		c.anglePITCH = SBGC_DEGREE_TO_ANGLE(angle);
		SBGC_cmd_control_send(c, sbgc_parser);
		delay(SBGC_CMD_DELAY);
	}
	// de-acceleration phase
	while(angle < 80.0f && speed > 0.0f) {
		speed-= 1.0f;
		c.speedPITCH = speed * SBGC_SPEED_SCALE;
		angle+= speed * SBGC_CMD_DELAY / 1000.0f;
		c.anglePITCH = SBGC_DEGREE_TO_ANGLE(angle);
		SBGC_cmd_control_send(c, sbgc_parser);
		delay(SBGC_CMD_DELAY);
	}

	delay(1000);
}

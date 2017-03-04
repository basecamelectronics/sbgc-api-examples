/******************************************************************************
	This is example sketch for Arduino.
	Shows how to control SimpleBGC-driven gimbal via Serial API.
	API specs are available at http://www.basecamelectronics.com/serialapi/

	Demo: gimbal will repeat motion of the remote controller build of 2 potentiometers and 2 push-buttons:
	  "Menu" button for gimbal controller
	  "Rec" button for camera control via PWM->IR converter
	  

	Arduino hardware:
	- 2 potentimeters (or encoders with analog output), connected to the pins A1, A2  
	  (connect GND, +5V to the side outputs of potentiometers)
	- Toggle buttons connected to D11, D12
	- (Optional) PWM-to-IR camera control adapter, connected to FC_PITCH of gimbal controller
	- Serial link over Bluetooth, 3DR radio or wires.

	Gimbal settings:
	- RC SPEED should be set high  enough to track fast movements of a controller's handle (100..150)
	- Acceleration limit is set to a value that gimbal can process without loosing sync in motors

	Copyright (c) 2016 Aleksey Moskalenko
*******************************************************************************/
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>
#include "filter.h"


// Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200

// delay between CMD_CONTROL commands, ms
#define CMD_CONTROL_DELAY 20

#define JOY_X_ANALOG_PIN 1  // joystick X-axis input
#define JOY_Y_ANALOG_PIN 2  // joystick Y-axis input
#define MENU_BTN_PIN		11  // "MENU" button input
#define REC_BTN_PIN		12  // "REC" button input

//#define SET_SPEED 100 // uncomment to override SPEED setting in the GUI

// Extreme angles in degrees, that corresponds to 0..Vcc analog input range
#define PITCH_ANGLE_MIN -60
#define PITCH_ANGLE_MAX 60
#define YAW_ANGLE_MIN -60
#define YAW_ANGLE_MAX 60

// LPF filter applyed to a signal to smooth sharp movements, 0..16, 0 - no filtering, 16 - max filtering
#define LOW_PASS_FACTOR  6 

// PWM output port index
#define PWM_SERVO_OUT_IDX 1 // 0=FC_ROLL, 1=FC_PITCH, 2=RC_PITCH, 3=AUX1

// PWM values for external infra-red camera control adapter
#define PWM_CAM_REC_ON 2000
#define PWM_CAM_REC_OFF 1000

/*****************************************************************************/


#define BTN_STATE_RELEASED  0
#define BTN_STATE_PRESSED  1

// Set serial port where SBGC32 is connected
#define serial Serial

static SBGC_cmd_control_t c = { 0, 0, 0, 0, 0, 0, 0 };
static SBGC_cmd_servo_out_t cmd_servo_out = { { SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED, SBGC_SERVO_OUT_DISABLED } };
static btn_state_t menu_btn, rec_btn;
static uint16_t last_cmd_control_time_ms = 0;
static avg_var16 lpf[2];



void setup() {
	serial.begin(SERIAL_SPEED);
	SBGC_Demo_setup(&serial);

	c.mode = SBGC_CONTROL_MODE_ANGLE;
#ifdef SET_SPEED
	c.speedPITCH = SET_SPEED;
	c.speedYAW = SET_SPEED;
#endif

	lpf[0].init(LOW_PASS_FACTOR);
	lpf[1].init(LOW_PASS_FACTOR);

	pinMode(MENU_BTN_PIN, INPUT);
	pinMode(REC_BTN_PIN, INPUT);
	// set button pins pulled up to VCC, button will short it to GND (i,e, pressed state = LOW)
	digitalWrite(MENU_BTN_PIN, 1); 
	digitalWrite(REC_BTN_PIN, 1);

	// Take a pause to let gimbal controller to initialize
	delay(1000);
	
	// Set servo output to "REC OFF" state
	cmd_servo_out.servo[PWM_SERVO_OUT_IDX] = PWM_CAM_REC_OFF;
	SBGC_cmd_servo_out_send(cmd_servo_out, sbgc_parser);
}




void loop() {
	uint16_t cur_time_ms = millis();
	
	// Read analog input and convert to SBGC angle range
	lpf[0].average( (int16_t)SBGC_DEGREE_TO_ANGLE_INT(PITCH_ANGLE_MIN) + 
		(int16_t)((int32_t)analogRead(JOY_Y_ANALOG_PIN)*SBGC_DEGREE_TO_ANGLE_INT(PITCH_ANGLE_MAX - PITCH_ANGLE_MIN)/1024) );
	lpf[1].average( (int16_t)SBGC_DEGREE_TO_ANGLE_INT(YAW_ANGLE_MIN) + 
		(int16_t)((int32_t)analogRead(JOY_X_ANALOG_PIN)*SBGC_DEGREE_TO_ANGLE_INT(YAW_ANGLE_MAX - YAW_ANGLE_MIN)/1024) );
		
		
	if((cur_time_ms - last_cmd_control_time_ms ) > CMD_CONTROL_DELAY) {
		last_cmd_control_time_ms = cur_time_ms;
		
	  c.anglePITCH = lpf[0].res;
	 	c.angleYAW = lpf[1].res;
		
		SBGC_cmd_control_send(c, sbgc_parser);
	}


	// Process menu button and send it as a "menu" cmd press to the board
	if(debounce_button(menu_btn, digitalRead(MENU_BTN_PIN) == LOW ? BTN_STATE_PRESSED : BTN_STATE_RELEASED)) {
		if(menu_btn.trigger_state == BTN_STATE_PRESSED) {
			SBGC_cmd_execute_menu_send(SBGC_MENU_BUTTON_PRESS, sbgc_parser);
		}
	}

	// Process "REC" button and send it as a "servo out" command
	if(debounce_button(rec_btn, digitalRead(REC_BTN_PIN) == LOW ? BTN_STATE_PRESSED : BTN_STATE_RELEASED)) {
		if(rec_btn.trigger_state == BTN_STATE_PRESSED) {
			cmd_servo_out.servo[PWM_SERVO_OUT_IDX] = PWM_CAM_REC_ON;
		} else {
			cmd_servo_out.servo[PWM_SERVO_OUT_IDX] = PWM_CAM_REC_OFF;
		}
		
		SBGC_cmd_servo_out_send(cmd_servo_out, sbgc_parser);
	}
	
	// Make a constant sampling time by inserting delay 1ms
	while((millis() - cur_time_ms) < 1);
}



/*
* De-bounce button: it should keep its state for a given period of time (30ms)
* Returns 1 if btn.trigger_state is changed.
*/
uint8_t debounce_button(btn_state_t &btn, uint8_t new_state) {
	uint16_t cur_time_ms = millis();
	
	if(new_state != btn.state) {
		btn.state = new_state;
		btn.last_time_ms = cur_time_ms;
	} else if(btn.trigger_state != btn.state && (uint16_t)(cur_time_ms - btn.last_time_ms) > 30) {
		btn.trigger_state = btn.state;
		return 1;
	}
	return 0;
}

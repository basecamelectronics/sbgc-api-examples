/******************************************************************************
This is example sketch for Arduino, that
shows how to control SimpleBGC-driven gimbal via Serial API.
API specs are available at http://www.basecamelectronics.com/serialapi/
  
Demo: This is a skeleton for LCD-enabled remote controller for SBGC32-powered gimbal.
Tested with the Arduino UNO and Arduino 1.0 IDE.

Basic functions:
 - Display system status using multiple pages (Up/Down arrow to scroll):
   - battery voltage, 
   - active profile
   - average error of stabilization in 0.001 degrees
   - communication errors, I2C errors, various debug information

 - Display a customizable set of adjustable variables (Left/Right arrow or encoder Push-button to scroll),
	 and change their values by the rotary encoder;
	- Trim ROLL axis with 0.1 degree precision

 - Use analog joystick to control PITCH and YAW axis (passed to SBGC controller as regular RC channels)
	- joystick push-button acts as "Menu" button
 
 - Navigation "select" button turns motors ON/OFF

Arduino hardware:  
	- Arduino UNO
	- LCD Key Shield from elfreaks, that includes:
		- 1602 LCD display 
		- Rotary encoder with push button 
		- 5 navigation buttons (Left,Right,Up,Down,Select)
		http://www.elecfreaks.com/wiki/index.php?title=LCD_Key_Shield
	- 2-axis joystick with push button
	- Wireless serial connection (optional)
  
Gimbal settings:
	- Firmware version 2.55b7 or above
	- RC control in SPEED or ANGLE mode
	- Assign desired actions to menu button in the "Service" tab

!!!WARNING!!! Its recommended to increase the serial buffer size to the max. size of a command you want to receive.
Default buffer is 64 bytes, that is not enough to fit incoming SBGC API commands.
For this example, 150 bytes works well. 
You can do it in the HardwareSerial.cpp (depends on Arduino IDE)

  
Copyright (c) 2014-2015 Aleksei Moskalenko
*******************************************************************************/
#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>
#include <LiquidCrystal.h>
#include "filter.h"
#include "MemoryFree.h"


/************************************************************************************
* Hardware Configuration
*************************************************************************************/
// Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200  // Default is 115200
//#define SOFTWARE_SERIAL_RX_PIN 11 // define to use software serial instead of hardware serial
//#define SOFTWARE_SERIAL_TX_PIN 12 // (but actually, it does not work for unknown reason)
#define ENCODER_A_PIN  3            // Incremental Encoder singal A is PD3
#define ENCODER_B_PIN  2            // Incremental Encoder singal B is PD2
#define LCD_BACLIGHT_PIN 10	  // pin to toggle back-light ON
#define LCD_ROWS 2
#define LCD_COLS 16
#define NAV_BTN_ANALOG_PIN 0  // 6 navigation buttons are connected to single analog input by dividers
#define LCD_SHIELD_VER  11  // version of LCD shield (from elfreaks)
#define JOY_X_ANALOG_PIN 1  // joystick X-axis input
#define JOY_Y_ANALOG_PIN 2  // joystick Y-axis input
#define JOY_BTN_PIN		11  // joystick button input
#define LCD_RS_PIN    8
#define LCD_ENABLE_PIN 9
#define LCD_D4_PIN 4
#define LCD_D5_PIN 5
#define LCD_D6_PIN 6
#define LCD_D7_PIN 7
/************************************************************************************
* Serial wireless connection. Remove (comment) definitions in this block if  you want only wire connection.
*************************************************************************************/
#define BLUETOOTH_CONNECTION
// Choose  (uncomment) one of the  supported modules, or implement bt_master_connect() for your own module.
//#define BLUETOOTH_RN42 // RN-42. 
//#define BLUETOOTH_HM10 // HM-10, HM-11 (BLE). Implement bt_master_connect() for your own module.
#define BLUETOOTH_HC05 // Be sure to use HC-05 module with KEY pin. This pin shuld be pulled to 3.3V
#define BLUETOOTH_CLIENT_MAC_ADDR "98D3,34,90DB5E" // If defined, use this MAC address to connect (12 hex digits), instead of searching | If HC-05 is used write MAC address in following format: "XXXX,XX,XXXXXX"
//#define BLUETOOTH_CLIENT_NAME_PATTERN "SBGC"  // If defined, will search for a device containing this pattern in its name
#define BLUETOOTH_CLIENT_PIN "1234" // PIN code set for connection
#define BLUETOOTH_BAUD 38400
#define BLUETOOTH_DO_SETUP  // configure BT module as master role and set PIN. May be done only once.
#define BLUETOOTH_BUF_SIZE 60 // size of buffer for answers from module
#define BLUETOOTH_DEBUG false // set to true to display answers from BT module
#if(defined(BLUETOOTH_HM10)) 
	#define BLE_MODE  // BLE requires special mode: 20-byte packets with pause between them. Tx is not yet implemented in this example
#endif
/*************************************************************************************
* Timings Configuration
*************************************************************************************/
#ifdef BLE_MODE
	#define REALTIME_DATA_REQUEST_INTERAL_MS 500 // interval between reatime data requests
	#define JOYSTICK_CMD_SEND_INTERVAL_MS 50     // interval of sending joystick control commands
#else
	#define REALTIME_DATA_REQUEST_INTERAL_MS 100 // interval between reatime data requests
	#define JOYSTICK_CMD_SEND_INTERVAL_MS 50     // interval of sending joystick control commands
#endif
#define MAX_WAIT_TIME_MS 2000  // time to wait for incoming commands to be in CONNECTED state
#define LOW_RATE_TASK_INTERVAL 500  // interval between low-priority tasks (display update, etc)
#define BTN_BOUNCE_THRESHOLD_MS 30  // interval for button de-bouncer threshold
/*************************************************************************************/



#define BTN_STATE_RELEASED  0
#define BTN_STATE_PRESSED  1

/* Navigation buttons */
enum {
	NAV_BTN_RIGHT=1,
	NAV_BTN_UP,
	NAV_BTN_DOWN,
	NAV_BTN_LEFT,
	NAV_BTN_SELECT,
	NAV_BTN_ENCODER_SELECT
};



/* Custom types of adjustable variables. Take any big ID that is not used by the SBGC API */
#define ADJ_VAR_CUSTOM_ID  200
enum {
	ADJ_VAR_ROLL_TRIM = ADJ_VAR_CUSTOM_ID,
};

/* A set of adjustable variables that can be changed by encoder knob.
* You may add any variables listed in the SBGC_adj_vars.h
* Be carefull, this structure is placed in to the RAM, that may be a problem with the low memory for boards like UNO
*/
adjustable_var_t adj_vars[] = {
	{ { ADJ_VAR_ROLL_TRIM, "ROLL_TRIM", -900, 900 }, 1, 0, 0 },
	{ ADJ_VAR_DEF_RC_SPEED_PITCH, 1, 0, 0 },
	{ ADJ_VAR_DEF_RC_SPEED_YAW, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_SPEED_PITCH, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_SPEED_YAW, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_LPF_PITCH, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_LPF_YAW, 1, 0, 0 },
	{ ADJ_VAR_DEF_FOLLOW_DEADBAND, 1, 0, 0 },
};

#define ADJ_VARS_NUM  (sizeof(adj_vars)/sizeof(adjustable_var_t))


// Functions prototypes
inline void process_cmd_realtime_data();
inline void process_in_queue();
void encoder_irq_handler();
inline uint8_t read_nav_buttons_state();
inline void update_display();
void set_connected();
void request_adj_vars_val();
void set_local_adj_var(uint8_t id, int32_t val);
inline void bt_master_connect();

// Global variables
static LiquidCrystal lcd(LCD_RS_PIN, LCD_ENABLE_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);
static uint8_t cur_adj_var_idx = 0;
static SBGC_cmd_realtime_data_t rt_data;
static uint16_t cur_time_ms, low_rate_time_ms, last_cmd_time_ms, rt_req_last_time_ms, joy_last_time_ms; //last_bt_connect_ms;
static uint8_t is_connected = 0;
static avg_var16 target_error_avg;
static btn_state_t nav_btn, joy_btn;
static int8_t cur_page = 0;
static int16_t debug1, debug2, debug3, debug4, free_memory;
static SBGC_cmd_control_ext_t cmd_control = { 
	{ SBGC_CONTROL_MODE_ANGLE, SBGC_CONTROL_MODE_RC, SBGC_CONTROL_MODE_RC },  // control mode for ROLL, PITCH, YAW
	{ { 0, 0 }, { 0, 0 }, { 0, 0 } }  // angle and speed
}; 
static int16_t joy_x_neutral, joy_y_neutral;
static uint8_t need_update_display;



#ifdef SOFTWARE_SERIAL_RX_PIN
	#include <SoftwareSerial.h>
	SoftwareSerial serial(SOFTWARE_SERIAL_RX_PIN, SOFTWARE_SERIAL_TX_PIN);
#else
	// Set serial port where SBGC32 is connected
	#define serial Serial
#endif




void setup() {
	serial.begin(SERIAL_SPEED);
	SBGC_Demo_setup(&serial);

	/////// Setup LCD
	lcd.begin(LCD_COLS, LCD_ROWS);
	// enable back light
	pinMode(LCD_BACLIGHT_PIN, OUTPUT);
	digitalWrite(LCD_BACLIGHT_PIN, 1);

	
	////// Setup encoder
	pinMode(ENCODER_A_PIN, INPUT);
	pinMode(ENCODER_B_PIN, INPUT);
	digitalWrite(ENCODER_A_PIN, 1);
	digitalWrite(ENCODER_B_PIN, 1);
	attachInterrupt(1, encoder_irq_handler, FALLING);        //interrupts: numbers 0 (on digital pin 2) and 1 (on digital pin 3).
	
	/////// Analog joystick
	pinMode(JOY_BTN_PIN, INPUT);
	digitalWrite(JOY_BTN_PIN, 1); // pulled up to VCC, button will short it to GND (i,e, pressed state = LOW)
	joy_x_neutral = analogRead(JOY_X_ANALOG_PIN);
	joy_y_neutral = analogRead(JOY_Y_ANALOG_PIN);
	
	target_error_avg.init(4);
	
	bt_master_connect();
	
	// We are ready
	//blink_led(3);
}


// Main loop
void loop() {
	cur_time_ms = millis();
	
	process_in_queue();
	
	////////// Request realtime data with the fixed rate
	if((cur_time_ms - rt_req_last_time_ms) > REALTIME_DATA_REQUEST_INTERAL_MS) {
		SerialCommand cmd;
		if(is_connected) {
			cmd.init(SBGC_CMD_REALTIME_DATA_4);
		} else { // Set version request to init connection
			cmd.init(SBGC_CMD_BOARD_INFO);
#ifdef BLE_MODE
			cmd.writeWord(10); // pause between packets, ms
#endif
		}
			
		sbgc_parser.send_cmd(cmd, 0);
		
		rt_req_last_time_ms = cur_time_ms;
	}
	
	
	////////// Process navigation
	if(debounce_button(nav_btn, read_nav_buttons_state())) {
		switch(nav_btn.trigger_state) {
		case NAV_BTN_RIGHT: // select next adj. var
		case NAV_BTN_ENCODER_SELECT:
			cur_adj_var_idx = (cur_adj_var_idx+1)%ADJ_VARS_NUM;
			break;
			
		case NAV_BTN_LEFT: // select prev adj. var
			cur_adj_var_idx = (cur_adj_var_idx + (ADJ_VARS_NUM - 1))%ADJ_VARS_NUM;
			break;
			
		case NAV_BTN_UP: // select next page
			cur_page++;
			break;
			
		case NAV_BTN_DOWN: // select prev page
			cur_page--;
			break;
			
		case NAV_BTN_SELECT: // turn motors ON/OFF
			SBGC_cmd_execute_menu_send(SBGC_MENU_MOTOR_TOGGLE, sbgc_parser);
			break;

		}
		// update display immediately to reduce lag
		need_update_display = 1;
	}
	
	

	////////// Send the value of updated adj. variables to the board
	for(uint8_t i=0; i<ADJ_VARS_NUM; i++) {
		adjustable_var_t &v = adj_vars[i]; // make a shortcut
		if(v.need_update) {
			if(v.cfg.id < ADJ_VAR_CUSTOM_ID) {
				SBGC_cmd_set_adj_vars_var_t var;
				var.id = v.cfg.id;
				var.val = v.val;
				SBGC_cmd_set_adj_vars_send(&var, 1, sbgc_parser);
			} else {
				///////// custom variables: need special processing //////////////
				switch(v.cfg.id) {
					case ADJ_VAR_ROLL_TRIM:
						// Just store new trimming angle. This command will be send later for joystick values
						cmd_control.data[ROLL].angle = SBGC_DEGREE_01_TO_ANGLE_INT(v.val);
						break;
				}
			}
			
			v.need_update = 0; // reset flag
			need_update_display = 1;
		}
	}
	
	//////////// Send joystick control as RC signal for PITCH, YAW axes with the fixed rate
	if((cur_time_ms - joy_last_time_ms) > JOYSTICK_CMD_SEND_INTERVAL_MS) {
		cmd_control.data[PITCH].angle = analogRead(JOY_Y_ANALOG_PIN) - joy_y_neutral; 
		cmd_control.data[YAW].angle = analogRead(JOY_X_ANALOG_PIN) - joy_x_neutral;
		SBGC_cmd_control_ext_send(cmd_control, sbgc_parser);
		
		joy_last_time_ms = cur_time_ms;
	}
	
	
	
	////////////  Process joystick button and send it as a "menu" cmd press to the board
	if(debounce_button(joy_btn, digitalRead(JOY_BTN_PIN) == LOW ? BTN_STATE_PRESSED : BTN_STATE_RELEASED)) {
		if(joy_btn.trigger_state == BTN_STATE_PRESSED) {
			SBGC_cmd_execute_menu_send(SBGC_MENU_BUTTON_PRESS, sbgc_parser);
		}
	}
	

	/////////// Low-rate tasks
	if((cur_time_ms - low_rate_time_ms) > LOW_RATE_TASK_INTERVAL || need_update_display) {
		// update LCD to display animation and state
		update_display();
		
		low_rate_time_ms = cur_time_ms;
		need_update_display = 0;
	}
	
	// Try to restore BT connection
	/*
	if(!is_connected && (cur_time_ms - last_bt_connect_ms) > MAX_WAIT_TIME_MS*2) {
		bt_master_connect();
	}
	*/
	

	
	free_memory = freeMemory();
}

// Called once on a connection established
void set_connected() {
	is_connected = 1;
	request_adj_vars_val();
}


// Process incoming commands. Call it as frequently as possible, to prevent overrun of serial input buffer.
void process_in_queue() {
	while(sbgc_parser.read_cmd()) {
		SerialCommand &cmd = sbgc_parser.in_cmd;
		last_cmd_time_ms = cur_time_ms;
		if(!is_connected) set_connected();
		
		uint8_t error = 0;
		
		switch(cmd.id) {
		// Receive realtime data
		case SBGC_CMD_REALTIME_DATA_3:
		case SBGC_CMD_REALTIME_DATA_4:
			error = SBGC_cmd_realtime_data_unpack(rt_data, cmd);
			if(!error) {
				// Extract some usefull data
				// Average stabilization error (0.001 degree)
				uint32_t err = (uint32_t)(abs(rt_data.imu_angle[ROLL] - rt_data.target_angle[ROLL])
					+ abs(rt_data.imu_angle[PITCH] - rt_data.target_angle[PITCH])
					+ abs(rt_data.imu_angle[YAW] - rt_data.target_angle[YAW])) * (uint32_t)(SBGC_ANGLE_DEGREE_SCALE*1000);
					
				target_error_avg.average(min(err, 999));
			} else {
				sbgc_parser.onParseError(error);
			}
			break;
			
		
		// Receive the actual values of adjustable variables
		case SBGC_CMD_SET_ADJ_VARS_VAL:
		{
			SBGC_cmd_set_adj_vars_var_t buf[ADJ_VARS_NUM];	
			uint8_t vars_num = ADJ_VARS_NUM;
			error = SBGC_cmd_set_adj_vars_unpack(buf, vars_num, cmd);
			if(!error) {
				// Assign received values to our local set of variables
				for(uint8_t i=0; i<vars_num; i++) {
					set_local_adj_var(buf[i].id, buf[i].val);
				}
			} else {
				sbgc_parser.onParseError(error);
			}
		}
			break;
		
		}
	}
	
	// If no commands for a long time, set connected state to false
	if(is_connected && (uint16_t)(cur_time_ms - last_cmd_time_ms) > MAX_WAIT_TIME_MS) {
		is_connected = 0;
		//last_bt_connect_ms = cur_time_ms;
	}
}


/*
* Request the actual values of adjustable variables from the board
*/
void request_adj_vars_val() {
	// Filter only variables that board understands
	uint8_t ids[ADJ_VARS_NUM];
	uint8_t vars_num = 0;
	for(uint8_t i=0; i<ADJ_VARS_NUM; i++) {
		if(adj_vars[i].cfg.id < ADJ_VAR_CUSTOM_ID) {
			ids[vars_num++] = adj_vars[i].cfg.id;
		}
	}
		
	// Format and send command
	SerialCommand cmd;
	cmd.init(SBGC_CMD_GET_ADJ_VARS_VAL);
	cmd.writeByte(vars_num);
	cmd.writeBuf(ids, vars_num);
	sbgc_parser.send_cmd(cmd);
}

/* 
* Sets adj. variable in the local set  by id
*/
void set_local_adj_var(uint8_t id, int32_t val) {
	for(uint8_t i=0; i<ADJ_VARS_NUM; i++) {
		if(adj_vars[i].cfg.id == id) {
			adj_vars[i].val = val;
			break;
		}
	}
}

// Handles the encoder step interrupt
void encoder_irq_handler() {
	// Change the value of current adjustable variable
	adjustable_var_t &var = adj_vars[cur_adj_var_idx];
	if(digitalRead(ENCODER_B_PIN)) {
		// Increment
		var.val = min(var.val + var.step, var.cfg.max_val);
	} else {
		// Decrement
		var.val = max(var.val - var.step, var.cfg.min_val);
	}				
	var.need_update = 1;
}

/*
* De-bounce button: it should keep its state for a given period of time, specified in the BTN_BOUNCE_THRESHOLD_MS
* Returns 1 if btn.trigger_state is changed.
*/
uint8_t debounce_button(btn_state_t &btn, uint8_t new_state) {
	if(new_state != btn.state) {
		btn.state = new_state;
		btn.last_time_ms = cur_time_ms;
	} else if(btn.trigger_state != btn.state && (uint16_t)(cur_time_ms - btn.last_time_ms) > BTN_BOUNCE_THRESHOLD_MS) {
		btn.trigger_state = btn.state;
		return 1;
	}
	return 0;
}


// Reads the state of buttons connected to A0
// (this is hardware-specific)
uint8_t read_nav_buttons_state() {
	uint16_t adc_key_in = analogRead(NAV_BTN_ANALOG_PIN);
	
	if (adc_key_in > 1000) return BTN_STATE_RELEASED;
	
	// For V1.1 us this threshold
#if(LCD_SHIELD_VER == 11)
	if (adc_key_in < 50)   return NAV_BTN_LEFT;
	if (adc_key_in < 150)  return NAV_BTN_UP;
	if (adc_key_in < 250)  return NAV_BTN_RIGHT;
	if (adc_key_in < 450)  return NAV_BTN_SELECT;
	if (adc_key_in < 700)  return NAV_BTN_DOWN;
	if (adc_key_in < 850)  return NAV_BTN_ENCODER_SELECT;
#elif(LCD_SHIELD_VER == 10)
	// For V1.0 use the one below:
	if (adc_key_in < 50)   return NAV_BTN_RIGHT;
	if (adc_key_in < 195)  return NAV_BTN_UP;
	if (adc_key_in < 380)  return NAV_BTN_DOWN;
	if (adc_key_in < 555)  return NAV_BTN_LEFT;
	if (adc_key_in < 790)  return NAV_BTN_SELECT;
#else
	#error "Unknown LCD shield version"
#endif
	
	return BTN_STATE_RELEASED;  // when all others fail, return this...
}

// Shows progress animation
inline void lcd_print_progress(uint8_t &cursor_pos) {
	for(uint8_t i=0; i<3; i++) {
		cursor_pos+= lcd.print(((uint8_t)(cur_time_ms/1000)) % 4 > i ? '.' : ' ');
	}
}
// Finish a display raw by space characters
inline void lcd_fill_space(uint8_t &cursor_pos, uint8_t to_pos = LCD_COLS) {
	while(cursor_pos < to_pos) { lcd.print(' '); cursor_pos++; }
}

// Re-paint display
void update_display() {
	///////////////////////////// 1st raw ///////////////////////////////
	lcd.setCursor(0,0);
	uint8_t pos = 0;
	char buf[10];
		
	cur_page = (cur_page+50)%5; // should be in the range of available pages

	/////// PAGE0 ////////
	if(cur_page == 0) {
		if(is_connected) {
			// Battery voltage
			sprintf(buf, "%2d.%02dV", rt_data.battery_voltage/100, rt_data.battery_voltage%100);
			pos+= lcd.print(buf);
		
			// Current profile
			pos+= lcd.print(" P:");
			pos+= lcd.print(rt_data.cur_profile+1);
		
			// Max. error
			pos+= lcd.print(" E:");
			uint16_t err = target_error_avg.res;
			sprintf(buf, "%03d", err);
			pos+= lcd.print(buf);
		} else {
			pos+= lcd.print("CONNECTING");
			lcd_print_progress(pos);
		}
	} 
	/////// PAGE1 ////////
	else if(cur_page == 1) {
		// Serial error counter
		pos+= lcd.print("SE:");
		pos+= lcd.print(sbgc_parser.get_parse_error_count());
		
		// Arduino free memory
		pos+= lcd.print(" FM:");
		pos+= lcd.print(free_memory);
	} 
	/////// PAGE2 ////////
	else if(cur_page == 2) {
		// TODO
		pos+= lcd.print("D1:");
		pos+= lcd.print(debug1);
		pos+= lcd.print(" D2:");
		pos+= lcd.print(debug2);
	}
	/////// PAGE3 ////////
	else if(cur_page == 3) {
		pos+= lcd.print("D3:");
		pos+= lcd.print(debug3);
		pos+= lcd.print(" D4:");
		pos+= lcd.print(debug4);
	}
	/////// PAGE4 ////////
	else if(cur_page == 4) {
		pos+= lcd.print("I2C_ERR:");
		pos+= lcd.print(rt_data.i2c_error_count);
	}
	
	lcd_fill_space(pos);
	
	/////////////////////////// 2nd raw /////////////////////////////
	lcd.setCursor(0,1);
	pos = 0;
	
	// Currently selected adj. variable name and value
	adjustable_var_t &var = adj_vars[cur_adj_var_idx];
	pos+= lcd.print(var.cfg.name);
	if(pos > ADJ_VAR_NAME_MAX_LENGTH) lcd.setCursor(ADJ_VAR_NAME_MAX_LENGTH, 1);
	pos+= lcd.print(":");
	lcd_fill_space(pos, (ADJ_VAR_NAME_MAX_LENGTH+1));
	pos+= lcd.print(var.val);
	
	lcd_fill_space(pos);
}


// Displays a NULL-terminated string
void lcd_debug_msg(char *str, uint8_t raw=1) {
  lcd.setCursor(0,raw);
  uint8_t pos = lcd.print(str);
  lcd_fill_space(pos);
}
	

// Reads answer to buf[] sized 'buf_size' bytes as string.
// Display answer on LCD.
// Returns actualy read size.
uint8_t _bt_read_answer(char *buf, uint16_t timeout_ms=500, bool debug=BLUETOOTH_DEBUG) {
	serial.setTimeout(timeout_ms); // wait 200 ms for each answer to be accomplished  
	uint8_t size;
	
#if(defined(BLUETOOTH_RN42))
  size = serial.readBytesUntil('\r', buf, (BLUETOOTH_BUF_SIZE-1));
#elif(defined(BLUETOOTH_HM10)) 
  size = serial.readBytes(buf, (BLUETOOTH_BUF_SIZE-1));
#elif(defined(BLUETOOTH_HC05))
  size = serial.readBytesUntil('\r', buf, (BLUETOOTH_BUF_SIZE - 1));
#else
  #error "Define serial.read() for bluetooth!"
#endif

	buf[size] = '\0';
	if(size) {
		if(debug) lcd_debug_msg(buf);
	} else {
		if(debug) {
			lcd_debug_msg("NO ANSWER");
			delay(1000);
		}
	}
	
	serial.setTimeout(1000); // return default value
	
	return size;
}


/* 
* Initiates wireless connection. Leave it empty for wire connection.
*/
void bt_master_connect() {
#ifdef BLUETOOTH_CONNECTION

  char buf[BLUETOOTH_BUF_SIZE];
  lcd_debug_msg("BLUETOOTH INIT...", 0);
  // set baud rate as set in the BT module
  serial.begin(BLUETOOTH_BAUD);
  delay(500); // let module to start


#if(defined(BLUETOOTH_RN42))
  serial.print("$$$"); // start command mode
  _bt_read_answer(buf);
	
	
#ifdef BLUETOOTH_DO_SETUP
  serial.println("SM,1");   // set to master mode
  _bt_read_answer(buf);
  serial.print("SP,"); // set PIN-code
  serial.println(BLUETOOTH_CLIENT_PIN);
  _bt_read_answer(buf);
  lcd_debug_msg("REBOOT..");
  serial.println("R,1");   // reboot to apply changes
  _bt_read_answer(buf);
  delay(1000);
  serial.print("$$$"); // command mode again
  _bt_read_answer(buf);
#endif // BLUETOOTH_DO_SETUP
  
  // Connect to specified device and switch to fast data mode
  lcd_debug_msg("START CONNECTION..");
  serial.print("CF");
  serial.println(BLUETOOTH_CLIENT_MAC_ADDR);


#elif(defined(BLUETOOTH_HM10)) 

#ifdef BLUETOOTH_DO_SETUP
  serial.print("AT+VERS?"); 
  _bt_read_answer(buf);
  serial.print("AT+IMME1"); // set AT mode at boot 
  _bt_read_answer(buf);
  serial.print("AT+ROLE1"); // set "master" mode
  _bt_read_answer(buf);
	serial.print("AT+MODE0"); // set "data transmission mode"
  _bt_read_answer(buf);	
  sprintf(buf, "AT+PASS%s", BLUETOOTH_CLIENT_PIN);
  serial.print(buf); // set PIN-code
  _bt_read_answer(buf);	
#endif  // BLUETOOTH_DO_SETUP

#ifdef BLUETOOTH_CLIENT_MAC_ADDR
  // Connect to specified device and switch to fast data mode
  lcd_debug_msg("START CONNECTION..");
  serial.print("AT+CON");
  serial.print(BLUETOOTH_CLIENT_MAC_ADDR);
  _bt_read_answer(buf);
#else
  // Search BLE module by name
  lcd_debug_msg("SEARCHING...", 0);
  serial.print("AT+SHOW1"); // set showing the name os discovered device
  _bt_read_answer(buf);
  serial.print("AT+DISC?");

  uint8_t device_idx = 0;
  int8_t found_idx = -1;
  for(uint8_t i=0; i<100; i++) {  // wait 20 seconds
  	if(uint8_t size = _bt_read_answer(buf, 100, false)) { // wait answer 100 ms, skip debugging
	  	if(strstr(buf, "OK+DISCE") != NULL) break; // finished discovery
	
	  	if(char *name_str = strstr(buf, "OK+NAME:")) {
	  		// We found a device, display its name
	  		lcd_debug_msg(&name_str[8]);
	  		if(BLUETOOTH_DEBUG) delay(500);
	  		
	  		if(strstr(&name_str[8], BLUETOOTH_CLIENT_NAME_PATTERN) != NULL) {
	  			found_idx = device_idx;
		  	}
		  	
		  	device_idx++;
	  	}
	  }
  }
  
  if(found_idx >= 0) {
		lcd_debug_msg("CONNECTING BT..", 0);
	  // connect using array index
	  sprintf(buf, "AT+CONN%d", found_idx);
	  serial.print(buf); 
	  _bt_read_answer(buf);
  } else {
  	lcd_debug_msg("NO DEVICE FOUND", 0);
  }
  	
  
#endif // BLUETOOTH_CLIENT_MAC_ADDR

#elif(defined(BLUETOOTH_HC05))

#ifdef BLUETOOTH_DO_SETUP

  serial.println("AT+RMAAD"); //Clear paired devices
  _bt_read_answer(buf);
  sprintf(buf, "AT+PSWD=%s", BLUETOOTH_CLIENT_PIN); // set PIN (both master and slave should have the same PIN)
  serial.println(buf);
  _bt_read_answer(buf);
  serial.println("AT+ROLE=1"); //set 'master' mode
  _bt_read_answer(buf);
  serial.println("AT+CMODE=0"); //connect only to fixed MAC address
  _bt_read_answer(buf);
  serial.println("AT+INIT"); //Initialize 'SPP'
  _bt_read_answer(buf);
  sprintf(buf, "AT+LINK=%s", BLUETOOTH_CLIENT_MAC_ADDR); // connect to slave HC05, MAC address should be in following format: "XXXX,XX,XXXXXX" ("98D3,34,90DB5E")
  serial.println(buf);
  _bt_read_answer(buf);

#endif  // BLUETOOTH_DO_SETUP
  
#endif // ..BLUETOOTH TYPE..

	delay(3000);

	//last_bt_connect_ms = millis();


#endif // BLUETOOTH_CONNECTION
}

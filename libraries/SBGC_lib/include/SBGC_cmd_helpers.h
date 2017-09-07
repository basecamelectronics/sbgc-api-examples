/*
	SimpleBGC Serial API  library - helpers to pack and parse command data
	More info: http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksei Moskalenko
  All rights reserved.
	
	See license info in the SBGC.h
*/

#ifndef __SBGC_CMD_HELPERS__
#define __SBGC_CMD_HELPERS__


//////////////// Units conversion /////////////////
#define SBGC_ANGLE_FULL_TURN 16384
// Conversion from degree/sec to units that command understand
#define SBGC_SPEED_SCALE  (1.0f/0.1220740379f)
#define SBGC_DEGREE_ANGLE_SCALE ((float)SBGC_ANGLE_FULL_TURN/360.0f)
#define SBGC_ANGLE_DEGREE_SCALE (360.0f/(float)SBGC_ANGLE_FULL_TURN)


// Conversions for angle in degrees to angle in SBGC 14bit representation, and back
#define SBGC_DEGREE_TO_ANGLE(val) ((val)*SBGC_DEGREE_ANGLE_SCALE)
#define SBGC_ANGLE_TO_DEGREE(val) ((val)*SBGC_ANGLE_DEGREE_SCALE)
// The same, optimized for integers
#define SBGC_DEGREE_TO_ANGLE_INT(val) ((int32_t)(val)*SBGC_ANGLE_FULL_TURN/360)
#define SBGC_DEGREE_01_TO_ANGLE_INT(val) ((int32_t)(val)*SBGC_ANGLE_FULL_TURN/3600)
#define SBGC_ANGLE_TO_DEGREE_INT(val) ((int32_t)(val)*360/SBGC_ANGLE_FULL_TURN)
#define SBGC_ANGLE_TO_DEGREE_01_INT(val) ((int32_t)(val)*3600/SBGC_ANGLE_FULL_TURN)



#define ROLL	0
#define PITCH	1
#define YAW		2



// CMD_CONTROL
typedef struct {
  uint8_t mode;
  int16_t speedROLL;
  int16_t angleROLL;
  int16_t speedPITCH;
  int16_t anglePITCH;
  int16_t speedYAW;
  int16_t angleYAW;
} SBGC_cmd_control_t;

void SBGC_cmd_control_pack(SBGC_cmd_control_t &p, SerialCommand &cmd);
inline uint8_t SBGC_cmd_control_send(SBGC_cmd_control_t &p, SBGC_Parser &parser) {
	SerialCommand cmd;
	SBGC_cmd_control_pack(p, cmd);
	return parser.send_cmd(cmd);
}


// CMD_CONTROL (extended version)
typedef struct {
  uint8_t mode[3];
  struct {
  	int16_t angle;
  	int16_t speed;
  } data[3];
} SBGC_cmd_control_ext_t;

void SBGC_cmd_control_ext_pack(SBGC_cmd_control_ext_t &p, SerialCommand &cmd);
inline uint8_t SBGC_cmd_control_ext_send(SBGC_cmd_control_ext_t &p, SBGC_Parser &parser) {
	SerialCommand cmd;
	SBGC_cmd_control_ext_pack(p, cmd);
	return parser.send_cmd(cmd);
}


// CMD_API_VIRT_CH_CONTROL
typedef struct {
	int16_t data[SBGC_API_VIRT_NUM_CHANNELS];
} SBGC_cmd_api_virt_ch_control_t;

void SBGC_cmd_api_virt_ch_control_pack(SBGC_cmd_api_virt_ch_control_t &p, SerialCommand &cmd);
inline uint8_t SBGC_cmd_api_virt_ch_control_send(SBGC_cmd_api_virt_ch_control_t &p, SBGC_Parser &parser) {
	SerialCommand cmd;
	SBGC_cmd_api_virt_ch_control_pack(p, cmd);
	return parser.send_cmd(cmd);
}


// CMD_TRIGGER_PIN
typedef struct {
	uint8_t pin;
	int8_t state;
} SBGC_cmd_trigger_t;

void SBGC_cmd_trigger_pack(SBGC_cmd_trigger_t &p, SerialCommand &cmd);
inline uint8_t SBGC_cmd_trigger_send(SBGC_cmd_trigger_t &p, SBGC_Parser &parser) {
	SerialCommand cmd;
	SBGC_cmd_trigger_pack(p, cmd);
	return parser.send_cmd(cmd);
}


// CMD_SERVO_OUT
typedef struct {
	int16_t servo[8];
} SBGC_cmd_servo_out_t;

void SBGC_cmd_servo_out_pack(SBGC_cmd_servo_out_t &p, SerialCommand &cmd);
inline uint8_t SBGC_cmd_servo_out_send(SBGC_cmd_servo_out_t &p, SBGC_Parser &parser) {
	SerialCommand cmd;
	SBGC_cmd_servo_out_pack(p, cmd);
	return parser.send_cmd(cmd);
}


//CMD_SET_ADJ_VARS_VAL
typedef struct {
	uint8_t id;
	int32_t val;
} SBGC_cmd_set_adj_vars_var_t;

void SBGC_cmd_set_adj_vars_pack(SBGC_cmd_set_adj_vars_var_t vars[], uint8_t vars_num, SerialCommand &cmd);
uint8_t SBGC_cmd_set_adj_vars_unpack(SBGC_cmd_set_adj_vars_var_t vars_buf[], uint8_t &vars_num, SerialCommand &cmd);
inline uint8_t SBGC_cmd_set_adj_vars_send(SBGC_cmd_set_adj_vars_var_t vars[], uint8_t vars_num, SBGC_Parser &parser) {
	SerialCommand cmd;
	SBGC_cmd_set_adj_vars_pack(vars, vars_num, cmd);
	return parser.send_cmd(cmd);
}



// CMD_REALTIME_DATA_3, CMD_REALTIME_DATA_4
typedef struct {
	struct {
		int16_t acc_data;
		int16_t gyro_data;
	} sensor_data[3];  // ACC and Gyro sensor data (with calibration) for current IMU (see cur_imu field)
	int16_t serial_error_cnt; // counter for communication errors
	int16_t system_error; // system error flags, defined in SBGC_SYS_ERR_XX 
	uint8_t reserved1[4];
	int16_t rc_raw_data[SBGC_RC_NUM_CHANNELS]; // RC signal in 1000..2000 range for ROLL, PITCH, YAW, CMD, EXT_ROLL, EXT_PITCH channels
	int16_t imu_angle[3]; // ROLL, PITCH, YAW Euler angles of a camera, 16384/360 degrees
	int16_t frame_imu_angle[3]; // ROLL, PITCH, YAW Euler angles of a frame, if known
	int16_t target_angle[3]; // ROLL, PITCH, YAW target angle
	uint16_t cycle_time_us; // cycle time in us. Normally should be 800us
	uint16_t i2c_error_count; // I2C errors counter
	uint8_t reserved2;
	uint16_t battery_voltage; // units 0.01 V
	uint8_t state_flags1; // bit0: motor ON/OFF state;  bits1..7: reserved
	uint8_t cur_imu; // actually selecteted IMU for monitoring. 1: main IMU, 2: frame IMU
	uint8_t cur_profile; // active profile number starting from 0
	uint8_t motor_power[3]; // actual motor power for ROLL, PITCH, YAW axis, 0..255
	
	// Fields below are filled only for CMD_REALTIME_DATA_4 command
	int16_t rotor_angle[3]; // relative angle of each motor, 16384/360 degrees
	uint8_t reserved3;
	int16_t balance_error[3]; // error in balance. Ranges from -512 to 512,  0 means perfect balance.
	uint16_t current; // Current that gimbal takes, in mA.
	int16_t magnetometer_data[3]; // magnetometer sensor data (with calibration)
	int8_t  imu_temp_celcius;  // temperature measured by the main IMU sensor, in Celsius
	int8_t  frame_imu_temp_celcius;  // temperature measured by the frame IMU sensor, in Celsius
	uint8_t reserved4[38];
} SBGC_cmd_realtime_data_t;

uint8_t SBGC_cmd_realtime_data_unpack(SBGC_cmd_realtime_data_t &p, SerialCommand &cmd);



inline uint8_t SBGC_cmd_execute_menu_send(uint8_t menu_action, SBGC_Parser &parser) {
	SerialCommand cmd;
	cmd.init(SBGC_CMD_EXECUTE_MENU);
	cmd.writeByte(menu_action);
	return parser.send_cmd(cmd);
}	


#endif

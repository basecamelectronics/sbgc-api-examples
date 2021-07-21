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


#define ROLL    0
#define PITCH   1
#define YAW     2


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
    int8_t imu_temp_celcius;  // temperature measured by the main IMU sensor, in Celsius
    int8_t frame_imu_temp_celcius;  // temperature measured by the frame IMU sensor, in Celsius
    uint8_t reserved4[38];
} SBGC_cmd_realtime_data_t;

uint8_t SBGC_cmd_realtime_data_unpack(SBGC_cmd_realtime_data_t &p, SerialCommand &cmd);

// CMD_DATA_STREAM_INTERVAL
typedef struct {
    uint8_t cmd_id;
    uint16_t interval;

    union {
        struct {
            uint32_t flags;
        } cmd_realtime_data_custom;

        struct {
            uint8_t imu_type;
        } cmd_ahrs_helper;

        struct {
            uint8_t event_id;
            uint8_t event_type;
        } cmd_event;

        uint8_t data[8];
    } config;

    uint8_t sync_to_data;
    uint8_t reserved[9];
} SBGC_cmd_data_stream_interval_t;

void SBGC_cmd_data_stream_interval_pack(const SBGC_cmd_data_stream_interval_t &realtime_data, SerialCommand &cmd);

uint8_t SBGC_cmd_data_stream_interval_unpack(SBGC_cmd_data_stream_interval_t &realtime_data, SerialCommand &cmd);

inline uint8_t SBGC_cmd_data_stream_interval_send(const SBGC_cmd_data_stream_interval_t &realtime_data,
                                                  SBGC_Parser &parser) {
    SerialCommand cmd;
    SBGC_cmd_data_stream_interval_pack(realtime_data, cmd);
    return parser.send_cmd(cmd);
}

// CMD_REALTIME_DATA_CUSTOM
typedef struct {
    uint16_t timestamp_mp;
    int16_t imu_angles[3];
    int16_t target_angles[3];
    int16_t target_speed[3];
    int16_t stator_rotor_angle[3];
    int16_t gyro_data[3];
    int16_t rc_data[6];
    float z_vector[3];
    float h_vector[3];
    int16_t rc_channels[18];
    int16_t acc_data[3];
    uint8_t ahrs_debug_info[16];
    uint8_t motor4_control[8];
    uint32_t encoder_raw24[3];
    float imu_angles_rad[3];
} SBGC_cmd_realtime_data_custom_t;

uint8_t SBGC_cmd_realtime_data_custom_unpack(SBGC_cmd_realtime_data_custom_t &p, uint32_t data_ordered_flags,
                                             SerialCommand &cmd);

inline uint8_t SBGC_cmd_execute_menu_send(uint8_t menu_action, SBGC_Parser &parser) {
    SerialCommand cmd;
    cmd.init(SBGC_CMD_EXECUTE_MENU);
    cmd.writeByte(menu_action);
    return parser.send_cmd(cmd);
}

// CMD_MOTORS_ON
inline uint8_t SBGC_cmd_motors_on_send(SBGC_Parser &parser) {
    SerialCommand cmd;
    cmd.init(SBGC_CMD_MOTORS_ON);
    return parser.send_cmd(cmd);
}

// CMD_MOTORS_OFF
inline uint8_t SBGC_cmd_motors_off_send(uint8_t mode, SBGC_Parser &parser) {
    SerialCommand cmd;
    cmd.init(SBGC_CMD_MOTORS_OFF);
    cmd.writeByte(mode);
    return parser.send_cmd(cmd);
}

// CMD_READ_PARAMS_3 / CMD_WRITE_PARAMS_3
typedef struct {
    uint8_t profile_id;
    struct {
        uint8_t P;
        uint8_t I;
        uint8_t D;
        uint8_t power;
        uint8_t invert;
        uint8_t poles;
    } axis_pid[3];
    uint8_t acc_limiter_all;
    int8_t ext_fc_gain[2];
    struct {
        int16_t rc_min_angle;
        int16_t rc_max_angle;
        uint8_t rc_mode;
        uint8_t rc_lpf;
        uint8_t rc_speed;
        uint8_t rc_follow;
    } axis_rc[3];
    uint8_t gyro_trust;
    uint8_t use_model;
    uint8_t pwm_freq;
    uint8_t serial_speed;
    int8_t rc_trim[3];
    uint8_t rc_deadband;
    uint8_t rc_expo_rate;
    uint8_t rc_virt_mode;
    uint8_t rc_map_values[6]; /* RC_MAP_ROLL, RC_MAP_PITCH, RC_MAP_YAW, RC_MAP_CMD, RC_MAP_FC_ROLL, RC_MAP_FC_PITCH */
    uint8_t rc_mix_fc_values[2]; /*RC_MIX_FC_ROLL, RC_MIX_FC_PITCH*/
    uint8_t follow_mode;
    uint8_t follow_deadband;
    uint8_t follow_expo_rate;
    int8_t follow_offset[3];
    int8_t axis_imu[4]; /*AXIS_TOP, AXIS_RIGHT, FRAME_AXIS_TOP, FRAME_AXIS_RIGHT*/
    uint8_t frame_imu_pos;
    uint8_t gyro_deadband;
    uint8_t gyro_sens;
    uint8_t i2c_speed_fast;
    uint8_t skip_gyro_calib;
    uint8_t rc_cmd_actions[9]; // Assign action to various event sources. See CMD_EXECUTE_MENU for available actions
    /*RC_CMD_LOW, RC_CMD_MID, RC_CMD_HIGH, MENU_CMD_1, MENU_CMD_2, MENU_CMD_3, MENU_CMD_4, MENU_CMD_5, MENU_CMD_LONG*/
    uint8_t motor_output[3];
    int16_t bat_threshold_alarm;
    int16_t bat_threshold_motors;
    int16_t bat_comp_ref;
    uint8_t beeper_modes;
    uint8_t follow_roll_mix_start;
    uint8_t follow_roll_mix_range;
    uint8_t booster_power[3];
    uint8_t follow_speed[3];
    uint8_t frame_angle_from_motors;
    int16_t rc_memory[3];
    uint8_t servo[4]; /* SERVO1_OUT, SERVO2_OUT, SERVO3_OUT, SERVO4_OUT*/
    uint8_t servo_rate;
    uint8_t adaptive_pid_enable;
    uint8_t adaptive_pid_threshold;
    uint8_t adaptive_pid_rate;
    uint8_t adaptive_pid_recovery_factor;
    uint8_t follow_lpf[3];
    uint16_t general_flags;
    uint16_t profile_flags;
    uint8_t spektrum_mode;
    uint8_t order_of_axes;
    uint8_t euler_order;
    uint8_t cur_imu;
    uint8_t cur_profile_id;
} SBGC_cmd_read_write_params_3_t;

uint8_t SBGC_cmd_read_params_3_unpack(SBGC_cmd_read_write_params_3_t &p, SerialCommand &cmd);

inline uint8_t SBGC_cmd_read_params_3_send(SBGC_Parser &parser) {
    SerialCommand cmd;
    cmd.init(SBGC_CMD_READ_PARAMS_3);
    return parser.send_cmd(cmd);
}

#endif

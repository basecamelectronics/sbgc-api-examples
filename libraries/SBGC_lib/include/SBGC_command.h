/* 
	SimpleBGC Serial API  library - definition of commands
	More info: http://www.basecamelectronics.com/serialapi/


	Copyright (c) 2014-2015 Aleksei Moskalenko
	All rights reserved.
	
	See license info in the SBGC.h
*/

#ifndef  __SBGC_command__
#define  __SBGC_command__

#include <inttypes.h>
#include "SBGC_rc.h"


// Size of header and checksums
#define SBGC_CMD_NON_PAYLOAD_BYTES 5
// Max. size of a command after packing to bytes
#define SBGC_CMD_MAX_BYTES 255
// Max. size of a payload data
#define SBGC_CMD_DATA_SIZE (SBGC_CMD_MAX_BYTES - SBGC_CMD_NON_PAYLOAD_BYTES)



////////////////////// Command ID definitions ////////////////
#define SBGC_CMD_READ_PARAMS  82
#define SBGC_CMD_WRITE_PARAMS  87
#define SBGC_CMD_REALTIME_DATA  68
#define SBGC_CMD_BOARD_INFO  86
#define SBGC_CMD_CALIB_ACC  65
#define SBGC_CMD_CALIB_GYRO  103
#define SBGC_CMD_CALIB_EXT_GAIN  71
#define SBGC_CMD_USE_DEFAULTS  70
#define SBGC_CMD_CALIB_POLES  80
#define SBGC_CMD_RESET  114
#define SBGC_CMD_HELPER_DATA 72
#define SBGC_CMD_CALIB_OFFSET  79
#define SBGC_CMD_CALIB_BAT  66
#define SBGC_CMD_MOTORS_ON   77
#define SBGC_CMD_MOTORS_OFF  109
#define SBGC_CMD_CONTROL   67
#define SBGC_CMD_TRIGGER_PIN  84
#define SBGC_CMD_EXECUTE_MENU 69
#define SBGC_CMD_GET_ANGLES  73
#define SBGC_CMD_CONFIRM  67


// Starting from board ver.3.0
#define SBGC_CMD_BOARD_INFO_3  20
#define SBGC_CMD_READ_PARAMS_3 21
#define SBGC_CMD_WRITE_PARAMS_3 22
#define SBGC_CMD_REALTIME_DATA_3  23
#define SBGC_CMD_SELECT_IMU_3 24
#define SBGC_CMD_REALTIME_DATA_4  25
#define SBGC_CMD_ENCODERS_CALIB_OFFSET_4  26
#define SBGC_CMD_ENCODERS_CALIB_FLD_OFFSET_4 27
#define SBGC_CMD_READ_PROFILE_NAMES 28
#define SBGC_CMD_WRITE_PROFILE_NAMES 29

#define SBGC_CMD_QUEUE_PARAMS_INFO_3 30
#define SBGC_CMD_SET_ADJ_VARS_VAL 31
#define SBGC_CMD_SAVE_PARAMS_3 32
#define SBGC_CMD_READ_PARAMS_EXT 33
#define SBGC_CMD_WRITE_PARAMS_EXT 34
#define SBGC_CMD_AUTO_PID 35
#define SBGC_CMD_SERVO_OUT 36
#define SBGC_CMD_BODE_TEST_START_STOP 37
#define SBGC_CMD_BODE_TEST_DATA 38
#define SBGC_CMD_I2C_WRITE_REG_BUF 39
#define SBGC_CMD_I2C_READ_REG_BUF 40
#define SBGC_CMD_WRITE_EXTERNAL_DATA 41
#define SBGC_CMD_READ_EXTERNAL_DATA 42
#define SBGC_CMD_READ_ADJ_VARS_CFG 43
#define SBGC_CMD_WRITE_ADJ_VARS_CFG 44
#define SBGC_CMD_API_VIRT_CH_CONTROL 45
#define SBGC_CMD_ADJ_VARS_STATE 46
#define SBGC_CMD_EEPROM_WRITE 47
#define SBGC_CMD_EEPROM_READ 48
#define SBGC_CMD_CALIB_INFO 49
#define SBGC_CMD_SIGN_MESSAGE_3 50
#define SBGC_CMD_BOOT_MODE_3 51
#define SBGC_CMD_SYSTEM_STATE 52
#define SBGC_CMD_READ_FILE 53
#define SBGC_CMD_WRITE_FILE 54
#define SBGC_CMD_FS_CLEAR_ALL 55
#define SBGC_CMD_AHRS_HELPER 56
#define SBGC_CMD_RUN_SCRIPT 57
#define SBGC_CMD_SCRIPT_DEBUG 58
#define SBGC_CMD_CALIB_MAG 59
#define SBGC_CMD_UART_BYPASS 60
#define SBGC_CMD_GET_ANGLES_EXT 61
#define SBGC_CMD_READ_PARAMS_EXT2 62
#define SBGC_CMD_WRITE_PARAMS_EXT2 63
#define SBGC_CMD_GET_ADJ_VARS_VAL 64
#define SBGC_CMD_CALIB_MOTOR_MAG_LINK 74
#define SBGC_CMD_GYRO_CORRECTION 75


#define SBGC_CMD_DEBUG_VARS_INFO_3 253
#define SBGC_CMD_DEBUG_VARS_3  254
#define SBGC_CMD_ERROR  255

#define SBGC_CMD_DATA_STREAM_INTERVAL 85
#define SBGC_CMD_REALTIME_DATA_CUSTOM 88

constexpr uint32_t
cmd_realtime_data_custom_flags_imu_angles = 1 << 0,
        cmd_realtime_data_custom_flags_target_angles = 1 << 1,
        cmd_realtime_data_custom_flags_target_speed = 1 << 2,
        cmd_realtime_data_custom_flags_stator_rotor_angle = 1 << 3,
        cmd_realtime_data_custom_flags_gyro_data = 1 << 4,
        cmd_realtime_data_custom_flags_rc_data = 1 << 5,
        cmd_realtime_data_custom_flags_z_vector_h_vector = 1 << 6,
        cmd_realtime_data_custom_flags_rc_channels = 1 << 8,
        cmd_realtime_data_custom_flags_acc_data = 1 << 9,
        cmd_realtime_data_custom_flags_motor4_control = 1 << 10,
        cmd_realtime_data_custom_flags_ahrs_debug_info = 1 << 11,
        cmd_realtime_data_custom_flags_encoder_raw24 = 1 << 12,
        cmd_realtime_data_custom_flags_imu_angles_rad = 1 << 13;


#endif //__SBGC_command__

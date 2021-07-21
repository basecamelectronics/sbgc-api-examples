/* 
	SimpleBGC Serial API  library - helpers to pack and parse command data
	More info: http://www.basecamelectronics.com/serialapi/

	Copyright (c) 2014-2015 Aleksei Moskalenko
	All rights reserved.
	
	See license info in the SBGC.h
*/

#include <string.h>
#include "SBGC_lib/SBGC.h"

/* Packs command structure to SerialCommand object */
void SBGC_cmd_control_pack(SBGC_cmd_control_t &p, SerialCommand &cmd) {
    cmd.init(SBGC_CMD_CONTROL);
#ifdef SBGC_CMD_STRUCT_ALIGNED
    memcpy(cmd.data, &p, sizeof(p));
    cmd.len = sizeof(p);
#else
    cmd.writeByte(p.mode);
    cmd.writeWord(p.speedROLL);
    cmd.writeWord(p.angleROLL);
    cmd.writeWord(p.speedPITCH);
    cmd.writeWord(p.anglePITCH);
    cmd.writeWord(p.speedYAW);
    cmd.writeWord(p.angleYAW);
#endif
}

/* Packs command structure to SerialCommand object */
void SBGC_cmd_data_stream_interval_pack(const SBGC_cmd_data_stream_interval_t &p, SerialCommand &cmd) {
    cmd.init(SBGC_CMD_DATA_STREAM_INTERVAL);
#ifdef SBGC_CMD_STRUCT_ALIGNED
    memcpy(cmd.data, &p, sizeof(p));
    cmd.len = sizeof(p);
#else
    cmd.writeByte(p.cmd_id);
    cmd.writeWord(p.interval);
    cmd.writeBuf(p.config.data, 8);
    cmd.writeByte(p.sync_to_data);
    cmd.writeBuf(p.reserved, 9);
#endif
}

/*
* Unpacks SerialCommand object to command structure.
* Returns 0 on success, PARSER_ERROR_XX code on fail.
*/
uint8_t SBGC_cmd_data_stream_interval_unpack(SBGC_cmd_data_stream_interval_t &p, SerialCommand &cmd) {
#ifdef SBGC_CMD_STRUCT_ALIGNED
    if(cmd.len <= sizeof(p)) {
        memcpy(&p, cmd.data, cmd.len);
        return 0;
    } else {
        return PARSER_ERROR_WRONG_DATA_SIZE;
    }
#else
    p.cmd_id = cmd.readByte();
    p.interval = cmd.readWord();
    cmd.readBuf(p.config.data, 8);
    p.sync_to_data = cmd.readByte();
    cmd.readBuf(p.reserved, 9);

    if (cmd.checkLimit()) return 0;
    else return PARSER_ERROR_WRONG_DATA_SIZE;
#endif
}

/* Packs command structure to SerialCommand object */
void SBGC_cmd_control_ext_pack(SBGC_cmd_control_ext_t &p, SerialCommand &cmd) {
    cmd.init(SBGC_CMD_CONTROL);
#ifdef SBGC_CMD_STRUCT_ALIGNED
    memcpy(cmd.data, &p, sizeof(p));
    cmd.len = sizeof(p);
#else
    cmd.writeBuf(p.mode, 3);
    for (uint8_t i = 0; i < 3; i++) {
        cmd.writeWord(p.data[i].speed);
        cmd.writeWord(p.data[i].angle);
    }
#endif
}


/* Packs command structure to SerialCommand object */
void SBGC_cmd_api_virt_ch_control_pack(SBGC_cmd_api_virt_ch_control_t &p, SerialCommand &cmd) {
    cmd.init(SBGC_CMD_API_VIRT_CH_CONTROL);
#ifdef SBGC_CMD_STRUCT_ALIGNED
    memcpy(cmd.data, &p, sizeof(p));
    cmd.len = sizeof(p);
#else
    for (uint8_t i = 0; i < SBGC_API_VIRT_NUM_CHANNELS; i++) {
        cmd.writeWord(p.data[i]);
    }
#endif
}

/* Packs command structure to SerialCommand object */
void SBGC_cmd_trigger_pack(SBGC_cmd_trigger_t &p, SerialCommand &cmd) {
    cmd.init(SBGC_CMD_TRIGGER_PIN);
#ifdef SBGC_CMD_STRUCT_ALIGNED
    memcpy(cmd.data, &p, sizeof(p));
    cmd.len = sizeof(p);
#else
    cmd.writeByte(p.pin);
    cmd.writeByte(p.state);
#endif
}

/* Packs command structure to SerialCommand object */
void SBGC_cmd_servo_out_pack(SBGC_cmd_servo_out_t &p, SerialCommand &cmd) {
    cmd.init(SBGC_CMD_SERVO_OUT);
#ifdef SBGC_CMD_STRUCT_ALIGNED
    memcpy(cmd.data, &p, sizeof(p));
    cmd.len = sizeof(p);
#else
    for (uint8_t i = 0; i < 8; i++) {
        cmd.writeWord(p.servo[i]);
    }
#endif
}

/* Packs command structure to SerialCommand object */
void SBGC_cmd_set_adj_vars_pack(SBGC_cmd_set_adj_vars_var_t vars[], uint8_t vars_num, SerialCommand &cmd) {
    cmd.init(SBGC_CMD_SET_ADJ_VARS_VAL);
    cmd.writeByte(vars_num); // number of variables

#ifdef SBGC_CMD_STRUCT_ALIGNED
    cmd.writeBuf(vars, sizeof(SBGC_cmd_set_adj_vars_var_t)*vars_num);
#else
    for (uint8_t i = 0; i < vars_num; i++) {
        cmd.writeByte(vars[i].id);
        cmd.writeLong(vars[i].val);
    }
#endif
}

/*
* Unpacks SerialCommand object to vars_buf[var_num].
* 'var_num' specifies the buffer capacity.
* On return, 'var_num' will be set to actual number of received variables.
* Returns 0 on success, PARSER_ERROR_XX code on fail.
*/
uint8_t SBGC_cmd_set_adj_vars_unpack(SBGC_cmd_set_adj_vars_var_t vars_buf[], uint8_t &vars_num, SerialCommand &cmd) {
    uint8_t num = cmd.readByte(); // actual number of variables
    if (num <= vars_num) {
        vars_num = num;
#ifdef SBGC_CMD_STRUCT_ALIGNED
        cmd.readBuf(vars_buf, sizeof(SBGC_cmd_set_adj_vars_var_t)*vars_num);
#else
        for (uint8_t i = 0; i < num; i++) {
            vars_buf[i].id = cmd.readByte();
            vars_buf[i].val = cmd.readLong();
        }
#endif

        if (cmd.checkLimit()) return 0;
        else return PARSER_ERROR_WRONG_DATA_SIZE;
    } else {
        return PARSER_ERROR_BUFFER_IS_FULL;
    }
}


/*
* Unpacks SerialCommand object to command structure.
* Returns 0 on success, PARSER_ERROR_XX code on fail.
*/
uint8_t SBGC_cmd_realtime_data_unpack(SBGC_cmd_realtime_data_t &p, SerialCommand &cmd) {
#ifdef SBGC_CMD_STRUCT_ALIGNED
    if(cmd.len <= sizeof(p)) {
        memcpy(&p, cmd.data, cmd.len);
        return 0;
    } else {
        return PARSER_ERROR_WRONG_DATA_SIZE;
    }
#else
    for (uint8_t i = 0; i < 3; i++) {
        p.sensor_data[i].acc_data = cmd.readWord();
        p.sensor_data[i].gyro_data = cmd.readWord();
    }
    p.serial_error_cnt = cmd.readWord();
    p.system_error = cmd.readWord();
    cmd.skipBytes(4); // reserved
    cmd.readWordArr(p.rc_raw_data, SBGC_RC_NUM_CHANNELS);
    cmd.readWordArr(p.imu_angle, 3);
    cmd.readWordArr(p.frame_imu_angle, 3);
    cmd.readWordArr(p.target_angle, 3);
    p.cycle_time_us = cmd.readWord();
    p.i2c_error_count = cmd.readWord();
    cmd.readByte(); // reserved
    p.battery_voltage = cmd.readWord();
    p.state_flags1 = cmd.readByte();
    p.cur_imu = cmd.readByte();
    p.cur_profile = cmd.readByte();
    cmd.readBuf(p.motor_power, 3);

    if (cmd.id == SBGC_CMD_REALTIME_DATA_4) {
        cmd.readWordArr(p.rotor_angle, 3);
        cmd.readByte(); // reserved
        cmd.readWordArr(p.balance_error, 3);
        p.current = cmd.readWord();
        cmd.readWordArr(p.magnetometer_data, 3);
        p.imu_temp_celcius = cmd.readByte();
        p.frame_imu_temp_celcius = cmd.readByte();
        cmd.skipBytes(38);
    }


    if (cmd.checkLimit()) return 0;
    else return PARSER_ERROR_WRONG_DATA_SIZE;
#endif
}


/*
* Unpacks SerialCommand object to command structure.
* Returns 0 on success, PARSER_ERROR_XX code on fail.
*/
uint8_t SBGC_cmd_realtime_data_custom_unpack(SBGC_cmd_realtime_data_custom_t &p, const uint32_t data_ordered_flags,
                                             SerialCommand &cmd) {
    p.timestamp_mp = cmd.readWord();

    if (data_ordered_flags & cmd_realtime_data_custom_flags_target_angles)
        cmd.readWordArr(p.target_angles, 3);

    if (data_ordered_flags & cmd_realtime_data_custom_flags_target_speed)
        cmd.readWordArr(p.target_speed, 3);

    if (data_ordered_flags & cmd_realtime_data_custom_flags_stator_rotor_angle)
        cmd.readWordArr(p.stator_rotor_angle, 3);

    if (data_ordered_flags & cmd_realtime_data_custom_flags_z_vector_h_vector)
    {
        for (int i = 0; i < 3; i++)
            p.z_vector[i] = cmd.readFloat();
        for (int i = 0; i < 3; i++)
            p.h_vector[i] = cmd.readFloat();
    }

    if (data_ordered_flags & cmd_realtime_data_custom_flags_encoder_raw24)
        for (int i = 0; i < 3; i++)
            p.encoder_raw24[i] = cmd.readLong();

    if (cmd.checkLimit()) return 0;
    else return PARSER_ERROR_WRONG_DATA_SIZE;
}

/*
* Unpacks Read params object to command structure.
* Returns 0 on success, PARSER_ERROR_XX code on fail.
*/
uint8_t SBGC_cmd_read_params_3_unpack(SBGC_cmd_read_write_params_3_t& p, SerialCommand &cmd)
{
    p.profile_id = cmd.readByte();
    for (int i = 0; i < 3; ++i) {
        p.axis_pid[i].P = cmd.readByte();
        p.axis_pid[i].I = cmd.readByte();
        p.axis_pid[i].D = cmd.readByte();
        p.axis_pid[i].power = cmd.readByte();
        p.axis_pid[i].invert = cmd.readByte();
        p.axis_pid[i].poles = cmd.readByte();
    }
    p.acc_limiter_all = cmd.readByte();
    cmd.readBuf(p.ext_fc_gain, 2);
    for (int i = 0; i < 3; ++i) {
        p.axis_rc[i].rc_min_angle = cmd.readWord();
        p.axis_rc[i].rc_max_angle = cmd.readWord();
        p.axis_rc[i].rc_mode = cmd.readByte();
        p.axis_rc[i].rc_lpf = cmd.readByte();
        p.axis_rc[i].rc_speed = cmd.readByte();
        p.axis_rc[i].rc_follow = cmd.readByte();
    }
    p.gyro_trust = cmd.readByte();
    p.use_model = cmd.readByte();
    p.pwm_freq = cmd.readByte();
    p.serial_speed = cmd.readByte();
    cmd.readBuf(p.rc_trim, 3);
    p.rc_deadband = cmd.readByte();
    p.rc_expo_rate = cmd.readByte();
    p.rc_virt_mode = cmd.readByte();
    cmd.readBuf(p.rc_map_values, 6);
    cmd.readBuf(p.rc_mix_fc_values, 2);
    p.follow_mode = cmd.readByte();
    p.follow_deadband = cmd.readByte();
    p.follow_expo_rate = cmd.readByte();
    cmd.readBuf(p.follow_offset, 3);
    cmd.readBuf(p.axis_imu, 4);
    p.frame_imu_pos = cmd.readByte();
    p.gyro_deadband = cmd.readByte();
    p.gyro_sens = cmd.readByte();
    p.i2c_speed_fast = cmd.readByte();
    p.skip_gyro_calib = cmd.readByte();
    cmd.readBuf(p.rc_cmd_actions, 9);
    cmd.readBuf(p.motor_output, 3);
    p.bat_threshold_alarm = cmd.readWord();
    p.bat_threshold_motors = cmd.readWord();
    p.bat_comp_ref = cmd.readWord();
    p.beeper_modes = cmd.readByte();
    p.follow_roll_mix_start = cmd.readByte();
    p.follow_roll_mix_range = cmd.readByte();
    cmd.readBuf(p.booster_power, 3);
    cmd.readBuf(p.follow_speed, 3);
    p.frame_angle_from_motors = cmd.readByte();
    p.rc_memory[0] = cmd.readWord();
    p.rc_memory[1] = cmd.readWord();
    p.rc_memory[2] = cmd.readWord();
    cmd.readBuf(p.servo, 4);
    p.servo_rate = cmd.readByte();
    p.adaptive_pid_enable = cmd.readByte();
    p.adaptive_pid_threshold = cmd.readByte();
    p.adaptive_pid_rate = cmd.readByte();
    p.adaptive_pid_recovery_factor = cmd.readByte();
    cmd.readBuf(p.follow_lpf, 3);
    p.general_flags = cmd.readWord();
    p.profile_flags = cmd.readWord();
    p.spektrum_mode = cmd.readByte();
    p.order_of_axes = cmd.readByte();
    p.euler_order = cmd.readByte();
    p.cur_imu = cmd.readByte();
    p.cur_profile_id = cmd.readByte();
    if(cmd.checkLimit()) return 0;
    else return PARSER_ERROR_WRONG_DATA_SIZE;
}

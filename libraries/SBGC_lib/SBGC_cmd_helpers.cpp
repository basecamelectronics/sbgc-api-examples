/* 
	SimpleBGC Serial API  library - helpers to pack and parse command data
	More info: http://www.basecamelectronics.com/serialapi/

	Copyright (c) 2014-2015 Aleksei Moskalenko
	All rights reserved.
	
	See license info in the SBGC.h
*/   

#include <string.h>
#include "SBGC.h"

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
void SBGC_cmd_control_ext_pack(SBGC_cmd_control_ext_t &p, SerialCommand &cmd) {
	cmd.init(SBGC_CMD_CONTROL);
	#ifdef SBGC_CMD_STRUCT_ALIGNED
		memcpy(cmd.data, &p, sizeof(p));
		cmd.len = sizeof(p);
	#else
		cmd.writeBuf(p.mode, 3);
		for(uint8_t i=0; i<3; i++) {
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
		for(uint8_t i=0; i<SBGC_API_VIRT_NUM_CHANNELS; i++) {
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
		for(uint8_t i=0; i<8; i++) {
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
		for(uint8_t i=0; i<vars_num; i++) {
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
	if(num <= vars_num) {
		vars_num = num;
		#ifdef SBGC_CMD_STRUCT_ALIGNED
				cmd.readBuf(vars_buf, sizeof(SBGC_cmd_set_adj_vars_var_t)*vars_num);
		#else
			for(uint8_t i=0; i<num; i++) {
				vars_buf[i].id = cmd.readByte();
				vars_buf[i].val = cmd.readLong();
			}
		#endif
		
		if(cmd.checkLimit()) return 0;
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
		for(uint8_t i=0; i<3; i++) {
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
		
		if(cmd.id == SBGC_CMD_REALTIME_DATA_4) {
			cmd.readWordArr(p.rotor_angle, 3);
			cmd.readByte(); // reserved
			cmd.readWordArr(p.balance_error, 3);
			p.current = cmd.readWord();
			cmd.readWordArr(p.magnetometer_data, 3);
			p.imu_temp_celcius = cmd.readByte();
			p.frame_imu_temp_celcius = cmd.readByte();
			cmd.skipBytes(38);
		}
		
		
		if(cmd.checkLimit()) return 0;
		else return PARSER_ERROR_WRONG_DATA_SIZE;
	#endif
}



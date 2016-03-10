/* 
	SimpleBGC Serial API  library
	More info: http://www.basecamelectronics.com/serialapi/

  * compatible with the revision 2.4, 2.5 of Serial API specification.

	Copyright (c) 2014-2015 Aleksei Moskalenko
	All rights reserved.
	
	
  This software is free for use and redistribution under a BSD license:
   
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
	    * Redistributions of source code must retain the above copyright
	      notice, this list of conditions and the following disclaimer.
	    * Redistributions in binary form must reproduce the above copyright
	      notice, this list of conditions and the following disclaimer in the
	      documentation and/or other materials provided with the distribution.
	    * Neither the name of the Basecamelectronics company nor the
	      names of its contributors may be used to endorse or promote products
	      derived from this software without specific prior written permission.
	
	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/   

#ifndef  __SBGC__
#define  __SBGC__

// !!!!Define the endianess of your machine!!!! 
#define SYS_LITTLE_ENDIAN

// !!!!Uncomment only if structures are packed (use it for 8bit machines to save some program space)
//#define SYS_STRUCT_PACKED



// Optimize commapnd packing if it's good aligned in the memory
#if(defined(SYS_LITTLE_ENDIAN) && defined(SYS_STRUCT_PACKED))
	#define SBGC_CMD_STRUCT_ALIGNED
#endif



#include <inttypes.h>
#include "include/SBGC_command.h"
#include "include/SBGC_rc.h"
#include "include/SBGC_adj_vars.h"
#include "include/SBGC_parser.h"
#include "include/SBGC_cmd_helpers.h"



// Error codes in command response
#define SBGC_ERR_CMD_SIZE 1
#define SBGC_ERR_WRONG_PARAMS 2
#define SBGC_ERR_GET_DEVICE_ID  3
#define SBGC_ERR_CRYPTO		4
#define SBGC_ERR_CALIBRATE_BAT 5
#define SBGC_ERR_UNKNOWN_COMMAND 6


// System error flags that controller may set
#define SBGC_SYS_ERR_NO_SENSOR (1<<0)
#define SBGC_SYS_ERR_CALIB_ACC (1<<1)
#define SBGC_SYS_ERR_SET_POWER (1<<2)
#define SBGC_SYS_ERR_CALIB_POLES (1<<3)
#define SBGC_SYS_ERR_PROTECTION (1<<4)
#define SBGC_SYS_ERR_SERIAL (1<<5)
#define SBGC_SYS_ERR_BAT_LOW (1<<6)
#define SBGC_SYS_ERR_BAT_CRITICAL (1<<7)
#define SBGC_SYS_ERR_GUI_VERSION (1<<8)
#define SBGC_SYS_ERR_MISS_STEPS (1<<9)
#define SBGC_SYS_ERR_SYSTEM (1<<10)
#define SBGC_SYS_ERR_EMERGENCY_STOP (1<<11)


// Trigger pins
#define SBGC_PIN_AUX1 16
#define SBGC_PIN_AUX2 17
#define SBGC_PIN_AUX3 18
#define SBGC_PIN_BUZZER 32
#define CMD_PIN_SSAT_POWER 33  // pin that control Spektrum Satellite 3.3V power line (low state enables line)

// Value passed in CMD_SERVO_OUT to disable servo output
#define SBGC_SERVO_OUT_DISABLED -1


// Menu actions (used in the SBGC_MENU_BUTTON_PRESS command, menu button assignment, RC_CMD channel assignment)
#define SBGC_MENU_PROFILE1 1
#define SBGC_MENU_PROFILE2 2
#define SBGC_MENU_PROFILE3 3
#define SBGC_MENU_SWAP_PITCH_ROLL 4
#define SBGC_MENU_SWAP_YAW_ROLL 5
#define SBGC_MENU_CALIB_ACC 6
#define SBGC_MENU_RESET 7
#define SBGC_MENU_SET_ANGLE 8
#define SBGC_MENU_CALIB_GYRO 9
#define SBGC_MENU_MOTOR_TOGGLE 10
#define SBGC_MENU_MOTOR_ON 11
#define SBGC_MENU_MOTOR_OFF 12
#define SBGC_MENU_FRAME_UPSIDE_DOWN 13
#define SBGC_MENU_PROFILE4 14
#define SBGC_MENU_PROFILE5 15
#define SBGC_MENU_AUTO_PID 16
#define SBGC_MENU_LOOK_DOWN 17
#define SBGC_MENU_HOME_POSITION 18
#define SBGC_MENU_RC_BIND 19
#define SBGC_MENU_CALIB_GYRO_TEMP 20
#define SBGC_MENU_CALIB_ACC_TEMP 21
#define SBGC_MENU_BUTTON_PRESS 22
#define SBGC_MENU_RUN_SCRIPT1 23
#define SBGC_MENU_RUN_SCRIPT2 24
#define SBGC_MENU_RUN_SCRIPT3 25
#define SBGC_MENU_RUN_SCRIPT4 26
#define SBGC_MENU_RUN_SCRIPT5 27
#define SBGC_MENU_RUN_SCRIPT6 28
#define SBGC_MENU_RUN_SCRIPT7 29
#define SBGC_MENU_RUN_SCRIPT8 30
#define SBGC_MENU_RUN_SCRIPT9 31
#define SBGC_MENU_RUN_SCRIPT10 32
#define SBGC_MENU_CALIB_MAG 33
#define SBGC_MENU_LEVEL_ROLL_PITCH 34
#define SBGC_MENU_CENTER_YAW 35
#define SBGC_MENU_UNTWIST_CABLES 36
#define SBGC_MENU_SET_ANGLE_NO_SAVE 37
#define SBGC_MENU_HOME_POSITION_SHORTEST 38
#define SBGC_MENU_CENTER_YAW_SHORTEST 39


// Control modes
#define SBGC_CONTROL_MODE_NO          0
#define SBGC_CONTROL_MODE_SPEED       1
#define SBGC_CONTROL_MODE_ANGLE       2
#define SBGC_CONTROL_MODE_SPEED_ANGLE 3
#define SBGC_CONTROL_MODE_RC          4
#define SBGC_CONTROL_MODE_ANGLE_REL_FRAME 5

#define SBGC_CONTROL_MODE_MASK 0x0F // bits0..3 used for mode, other for flags

#define SBGC_CONTROL_MODE_FLAG_UNTWIST (1<<7)


#endif //__SBGC__
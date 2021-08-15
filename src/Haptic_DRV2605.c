/*  Haptic DRV2605 - Haptic Driver for Dialog DRV2605
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Haptic_DRV2605.h"

uint8_t Haptic_I2C_Address;
//! ----------------------------
//!   Default Scripts :
//! 0 DRV2605 Reset/Init
//! 1 DRV2605 LRA Motor Init
//! 2 DRV2605 ERM Motor Init
//! 3 DRV2605 ERM COIN Motor Init
//! 4 DRV2605 LRA Calibrate
//! 5 Complex Effect 1
//! 6 Complex Effect 2
//! 7 Complex Effect 3
//! ----------------------------
//! haptic_reset[] - DRV2605 reset & basic setup :
//! (Actuator independent setup commands)
const struct scr_type haptic_reset[] = {
	{DRV2605_REG_MODE,          0x80}, //! DRV2605 - reset 
	{ACTUATOR_SCRIPT_DELAY,     0x50}, //! DRV2605 - delay 50mSec for Reset? no spec? 
	{ACTUATOR_SCRIPT_END,       0x00}  //! DRV2605 - end of script flag
};
#define SCRIPT_RESET 0

//! haptic_LRA_basic[] - DRV2605 setup for LRA coin types :
//! register load table for basic device/actuator initialization 
//! (Actuator dependent setup commands with auto-calibration)
const struct scr_type haptic_LRA_basic[] = {
	{DRV2605_REG_MODE,          0x00}, //! DRV2605 - out of standby 
	{DRV2605_REG_LIBRARY,       0x02}, //! DRV2605 - Library B
	{DRV2605_REG_RTPIN,         0x00}, //! DRV2605 - no real-time-playback
	{DRV2605_REG_WAVESEQ1,         1}, //! DRV2605 - strong click
	{DRV2605_REG_WAVESEQ2,         0}, //! DRV2605 - strong click
	{DRV2605_REG_OVERDRIVE,        0}, //! DRV2605 - no overdrive
	{DRV2605_REG_SUSTAINPOS,       0}, //! DRV2605 - Sustain Time Offset 
	{DRV2605_REG_SUSTAINNEG,       0}, //! DRV2605 - Sustain Time Offset
	{DRV2605_REG_BRAKE,            0}, //! DRV2605 - Braking Time Offset
	{DRV2605_REG_AUDIOVIBECTRL, 0x00}, //! DRV2605 - audio to vibe control   (0x00) 10ms, 100hz
	{DRV2605_REG_AUDIOMINLVL,   0x00}, //! DRV2605 - audio-to-vibe min level 
	{DRV2605_REG_AUDIOMAXLVL,   0x40}, //! DRV2605 -  audio-to-vibe max level
	{DRV2605_REG_AUDIOMINDRV,   0x20}, //! DRV2605 - audio-to-vibe min drive
	{DRV2605_REG_AUDIOMAXDRV,   0x64}, //! DRV2605 - audio-to-vibe max drive
	{DRV2605_REG_FEEDBACK,      0x36}, //! DRV2605 - feedback tuning
//	{DRV2605_REG_CONTROL3,      0x20}, //! DRV2605 - ERM open loop
	{DRV2605_REG_CONTROL3,      0x04}, //! DRV2605 - ERM open loop
	{ACTUATOR_SCRIPT_END,       0x00}  //! DRV2605 - end of script flag
};
#define SCRIPT_LRA_BASIC 1

//! haptic_ERM_basic[] - DRV2605 setup for ERM types :
//! register load table for basic device/actuator initialization 
//! (Actuator dependent setup commands)
const struct scr_type haptic_ERM_basic[] = {
	{DRV2605_REG_MODE,          0x00}, //! DRV2605 - out of standby 
	{DRV2605_REG_LIBRARY,       0x01}, //! DRV2605 - Library A
	{DRV2605_REG_RTPIN,         0x00}, //! DRV2605 - no real-time-playback
	{DRV2605_REG_WAVESEQ1,         1}, //! DRV2605 - strong click
	{DRV2605_REG_WAVESEQ2,         0}, //! DRV2605 - strong click
	{DRV2605_REG_OVERDRIVE,        0}, //! DRV2605 - no overdrive
	{DRV2605_REG_SUSTAINPOS,       0}, //! DRV2605 - Sustain Time Offset 
	{DRV2605_REG_SUSTAINNEG,       0}, //! DRV2605 - Sustain Time Offset
	{DRV2605_REG_BRAKE,            0}, //! DRV2605 - Braking Time Offset
	{DRV2605_REG_AUDIOVIBECTRL, 0x00}, //! DRV2605 - audio to vibe control   (0x00) 10ms, 100hz
	{DRV2605_REG_AUDIOMINLVL,   0x00}, //! DRV2605 - audio-to-vibe min level 
	{DRV2605_REG_AUDIOMAXLVL,   0x40}, //! DRV2605 -  audio-to-vibe max level
	{DRV2605_REG_AUDIOMINDRV,   0x20}, //! DRV2605 - audio-to-vibe min drive
	{DRV2605_REG_AUDIOMAXDRV,   0x64}, //! DRV2605 - audio-to-vibe max drive
	{DRV2605_REG_FEEDBACK,      0x36}, //! DRV2605 - feedback tuning
	{DRV2605_REG_CONTROL3,      0x20}, //! DRV2605 - ERM open loop
	{ACTUATOR_SCRIPT_END,       0x00}  //! DRV2605 - end of script flag
};
#define SCRIPT_ERM_BASIC 2

//! haptic_ERM_coin[] - DRV2605 setup for ERM Disc/Coin types :
//! register load table for basic device/actuator initialization 
//! (Actuator dependent setup commands)
const struct scr_type haptic_ERM_coin[] = {
	{DRV2605_REG_MODE,          0x00}, //! DRV2605 - out of standby 
	{DRV2605_REG_LIBRARY,       0x07}, //! DRV2605 - Library F
	{DRV2605_REG_RTPIN,         0x00}, //! DRV2605 - no real-time-playback
	{DRV2605_REG_WAVESEQ1,         1}, //! DRV2605 - strong click
	{DRV2605_REG_WAVESEQ2,         0}, //! DRV2605 - strong click
	{DRV2605_REG_OVERDRIVE,        0}, //! DRV2605 - no overdrive
	{DRV2605_REG_SUSTAINPOS,       0}, //! DRV2605 - Sustain Time Offset 
	{DRV2605_REG_SUSTAINNEG,       0}, //! DRV2605 - Sustain Time Offset
	{DRV2605_REG_BRAKE,            0}, //! DRV2605 - Braking Time Offset
	{DRV2605_REG_AUDIOVIBECTRL, 0x00}, //! DRV2605 - audio to vibe control   (0x00) 10ms, 100hz
	{DRV2605_REG_AUDIOMINLVL,   0x00}, //! DRV2605 - audio-to-vibe min level 
	{DRV2605_REG_AUDIOMAXLVL,   0x40}, //! DRV2605 -  audio-to-vibe max level
	{DRV2605_REG_AUDIOMINDRV,   0x20}, //! DRV2605 - audio-to-vibe min drive
	{DRV2605_REG_AUDIOMAXDRV,   0x64}, //! DRV2605 - audio-to-vibe max drive
	{DRV2605_REG_FEEDBACK,      0x36}, //! DRV2605 - feedback tuning
	{DRV2605_REG_CONTROL3,      0x20}, //! DRV2605 - ERM open loop
	{ACTUATOR_SCRIPT_END,       0x00}  //! DRV2605 - end of script flag
};
#define SCRIPT_ERM_COIN 3

//! haptic_LRA_calibrate[] - DRV2605 setup for LRA coin types :
//! register load table for basic device/actuator initialization 
//! (Actuator dependent setup commands with auto-calibration)
const struct scr_type haptic_LRA_calibrate[] = {
	{DRV2605_REG_MODE,          0x07}, //! DRV2605 - Calibrate Mode! 
	{DRV2605_REG_FEEDBACK,      0xAA}, //! DRV2605 - LRA mode
	{DRV2605_REG_CONTROL3,      0x20}, //! DRV2605 - LRA closed loop
	{DRV2605_REG_RATEDV,        0x82}, //! DRV2605 - rated voltage 3V
	{DRV2605_REG_CLAMPV,        0x96}, //! DRV2605 - clamp voltage = 3.3V
	{DRV2605_REG_CONTROL4,      0x30}, //! DRV2605 - Autocal time = 3 (1.2 seconds!)
	{DRV2605_REG_CONTROL1,      0x83}, //! DRV2605 - drive_time set for 128Hz = 3.9mSec
	{DRV2605_REG_CONTROL2,      0xF5}, //! DRV2605 - sample_time = 3, balnk =1, idiss = 1
	{DRV2605_REG_LIBRARY,       0x06}, //! DRV2605 - Library 6 is LRA
	{DRV2605_REG_WAVESEQ1,         1}, //! DRV2605 - strong click
	{DRV2605_REG_WAVESEQ2,         0}, //! DRV2605 - strong click
	{DRV2605_REG_OVERDRIVE,        0}, //! DRV2605 - no overdrive
	{DRV2605_REG_SUSTAINPOS,       0}, //! DRV2605 - Sustain Time Offset 
	{DRV2605_REG_SUSTAINNEG,       0}, //! DRV2605 - Sustain Time Offset
	{DRV2605_REG_BRAKE,            0}, //! DRV2605 - Braking Time Offset
	{DRV2605_REG_RTPIN,         0x00}, //! DRV2605 - real-time-playback zero
	{DRV2605_REG_AUDIOVIBECTRL, 0x00}, //! DRV2605 - audio to vibe control   (0x00) 10ms, 100hz
	{DRV2605_REG_AUDIOMINLVL,   0x00}, //! DRV2605 - audio-to-vibe min level 
	{DRV2605_REG_AUDIOMAXLVL,   0x40}, //! DRV2605 -  audio-to-vibe max level
	{DRV2605_REG_AUDIOMINDRV,   0x20}, //! DRV2605 - audio-to-vibe min drive
	{DRV2605_REG_AUDIOMAXDRV,   0x64}, //! DRV2605 - audio-to-vibe max drive
	{DRV2605_REG_GO,            0x01}, //! DRV2605 - trigger a calibrate cycle
	{ACTUATOR_SCRIPT_DELAY,     0xFF}, //! DRV2605 - delay 0.25 sec 
	{ACTUATOR_SCRIPT_DELAY,     0xFF}, //! DRV2605 - delay 0.25 sec 
	{ACTUATOR_SCRIPT_DELAY,     0xFF}, //! DRV2605 - delay 0.25 sec 
	{ACTUATOR_SCRIPT_DELAY,     0xFF}, //! DRV2605 - delay 0.25 sec 
	{ACTUATOR_SCRIPT_DELAY,     0xFF}, //! DRV2605 - delay 0.25 sec (1.25 sec max)
	{DRV2605_REG_MODE,          0x00}, //! DRV2605 - calibrate 
	{ACTUATOR_SCRIPT_END,       0x00}  //! DRV2605 - end of script flag
};
#define SCRIPT_LRA_CALIBRATE 4

//! haptic_complex1[] - complex script 1
const struct scr_type haptic_complex1[] = {
	{DRV2605_REG_WAVESEQ1,         1}, //! DRV2605 - strong click
	{DRV2605_REG_WAVESEQ2,         2}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ3,         3}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ4,         4}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ5,         5}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ6,         6}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ7,         7}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ8,         8}, //! DRV2605 - 
	{ACTUATOR_SCRIPT_GOWAIT,    0x00}, //! DRV2605 - go and wait
	{ACTUATOR_SCRIPT_END,       0x00}  //! DRV2605 - end of script flag
};
#define SCRIPT_COMPLEX1 5

//! haptic_complex2[] - complex script 2
const struct scr_type haptic_complex2[] = {
	{DRV2605_REG_WAVESEQ1,         1},  //! DRV2605 - strong click
	{DRV2605_REG_WAVESEQ2,         2},  //! DRV2605 - 
	{DRV2605_REG_WAVESEQ3,         3},  //! DRV2605 - 
	{DRV2605_REG_WAVESEQ4,         4},  //! DRV2605 - 
	{DRV2605_REG_WAVESEQ5,         5},  //! DRV2605 - 
	{DRV2605_REG_WAVESEQ6,         6},  //! DRV2605 - 
	{DRV2605_REG_WAVESEQ7,         7},  //! DRV2605 - 
	{DRV2605_REG_WAVESEQ8,         8},  //! DRV2605 - 
	{ACTUATOR_SCRIPT_GOWAIT,    0x00},  //! DRV2605 - go and wait
	{DRV2605_REG_WAVESEQ1,         9},  //! DRV2605 - 
	{DRV2605_REG_WAVESEQ2,         10}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ3,         11}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ4,         12}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ5,         13}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ6,         14}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ7,         15}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ8,         16}, //! DRV2605 - 
	{ACTUATOR_SCRIPT_GOWAIT,    0x00},  //! DRV2605 - go and wait
	{ACTUATOR_SCRIPT_END,       0x00}   //! DRV2605 - end of script flag
};
#define SCRIPT_COMPLEX2 6

//! haptic_complex3[] - complex script 3
const struct scr_type haptic_complex3[] = {
	{DRV2605_REG_WAVESEQ1,        17}, //! DRV2605 - strong click
	{DRV2605_REG_WAVESEQ2,        18}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ3,        19}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ4,        20}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ5,        21}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ6,        22}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ7,        23}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ8,        24}, //! DRV2605 - 
	{ACTUATOR_SCRIPT_GOWAIT,    0x00}, //! DRV2605 - go and wait
	{DRV2605_REG_WAVESEQ1,        44}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ2,        45}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ3,        46}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ4,        47}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ5,        48}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ6,        49}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ7,        50}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ8,        51}, //! DRV2605 - 
	{ACTUATOR_SCRIPT_GOWAIT,    0x00}, //! DRV2605 - go and wait
	{DRV2605_REG_WAVESEQ1,       110}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ2,       111}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ3,       112}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ4,       113}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ5,       114}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ6,       115}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ7,       116}, //! DRV2605 - 
	{DRV2605_REG_WAVESEQ8,       117}, //! DRV2605 - 
	{ACTUATOR_SCRIPT_GOWAIT,    0x00}, //! DRV2605 - go and wait
	{ACTUATOR_SCRIPT_END,       0x00}  //! DRV2605 - end of script flag
};
#define SCRIPT_COMPLEX3 7

#define SCRIPT_LAST 7


//! array of pointers to arrays of FX scripts...
const scr_type* scriptList[ACTUATOR_SCRIPT_MAX];

//! Define a structure to hold the actuator instance data for the haptic driver
static struct haptic_driver actuator =
{
	0,
	8,         // fixed for DRV2605L
	0,
	128,  // fixed for DRV2605L
	0,
	0,
	0,
	ACTUATOR_HAPTIC_DEV,
	ACTUATOR_OP_MODE,
	ACTUATOR_BEMF_SENS_EN,
	ACTUATOR_FREQ_TRACK_EN,
	ACTUATOR_ACC_EN,
	ACTUATOR_RAPID_STOP_EN,
	ACTUATOR_AMP_PID_EN
};

/**************************************************************************/
/*! 
    @brief  Instantiate a new DRV2605 class
	        Use -1 for any unused pin assignments
			i.e. only GPIO0 for PWM on Pin #5, no Interrupts
			Haptic_DRV2605 actuator( -1, 5, -1, -1);
*/
//Haptic_DRV2605::Haptic_DRV2605() {
//  // I2C Only
//  _gp0_pin = -1;
//}

//Haptic_DRV2605::Haptic_DRV2605(int8_t gp0_pin) {
//  // I2C with Multi-Input GPI0 Pin
//  _gp0_pin = gp0_pin;
//}

void Haptic_DRV2605_Haptic_DRV2605(){	
Haptic_I2C_Address = DRV2605_I2C_ADDR;
}
/********************************************************************/

int Haptic_DRV2605_readReg(uint8_t reg) {
  uint8_t data;
	IIC_Start();
	IIC_Send_Byte(Haptic_I2C_Address);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(Haptic_I2C_Address|0x01);
    IIC_Wait_Ack();
    data=IIC_Read_Byte(0);
    IIC_Stop();
  return((int)data);  //! a career in dada processing!
}

int Haptic_DRV2605_writeReg(uint8_t reg, uint8_t data) {
	IIC_Start();
    IIC_Send_Byte(Haptic_I2C_Address);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
	IIC_Stop();
	return(HAPTIC_SUCCESS);
}

int Haptic_DRV2605_writeRegBits(uint8_t reg, uint8_t mask, uint8_t bits) {
	uint8_t val = Haptic_DRV2605_readReg(reg);
	if (val < 0) return(HAPTIC_FAIL);
	val = val & ~mask; val |= bits;
	return(Haptic_DRV2605_writeReg(reg, val));
}

int Haptic_DRV2605_writeRegBulk(uint8_t reg, uint8_t *dada, uint8_t size) {
	for (int i = 0; i < size; i++) {
		int ret = Haptic_DRV2605_writeReg(reg + i, *dada++);
		if (ret != 0) return(HAPTIC_FAIL);
	}
	return(HAPTIC_SUCCESS);
}

int Haptic_DRV2605_writeRegScript(const struct scr_type script[])
{
	for (int i = 0; script[i].reg != ACTUATOR_SCRIPT_END; i++) {
		if (script[i].reg == ACTUATOR_SCRIPT_DELAY) {
			delay_ms(script[i].val);
		} else if (script[i].reg == ACTUATOR_SCRIPT_GOWAIT) {
          Haptic_DRV2605_goWait();
		} else if (Haptic_DRV2605_writeReg(script[i].reg, script[i].val)) {
//			printf("ERROR DRV2605: writeRegScript failure\n");
			return(HAPTIC_FAIL);
		}
	}
	return(HAPTIC_SUCCESS);
}

int Haptic_DRV2605_getDeviceID(void) {
  int id = Haptic_DRV2605_readReg(DRV2605_REG_STATUS);
  if (id == 0xFF) {
	  // unconnected I2C bus...
	  return(HAPTIC_FAIL);
  } else {
      return((id & 0xE0 ) >> 5); // read chip ID register
  }
}

/**************************************************************************/
/*! 
    @brief  Initialize the DRV2605 Controller
*/
int Haptic_DRV2605_begin()
{
	//trig en pin init
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //使能GPIOB时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   //普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   //初始化

	GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1); //置高使能
	
	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //使能GPIOB时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   //普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);			   //初始化

	GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9); //置高使能
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //使能GPIOB时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   //普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);			   //初始化

	GPIO_SetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7); //置高使能
	

	//! initialize an array of pointers to the scripts
	scriptList[SCRIPT_RESET] = haptic_reset;		 //! 'reset' script always at index zero
	scriptList[SCRIPT_LRA_BASIC] = haptic_LRA_basic; //! LRA coin basic setup
	scriptList[SCRIPT_ERM_BASIC] = haptic_ERM_basic; //! ERM (bar) basic setup
	scriptList[SCRIPT_ERM_COIN] = haptic_ERM_coin;	 //! ERM (coin) basic setup
	scriptList[SCRIPT_LRA_CALIBRATE] = haptic_LRA_calibrate;
	scriptList[SCRIPT_COMPLEX1] = haptic_complex1;
	scriptList[SCRIPT_COMPLEX2] = haptic_complex2;
	scriptList[SCRIPT_COMPLEX3] = haptic_complex3;
	actuator.dev_scripts_max = SCRIPT_LAST + 1;

	Haptic_DRV2605_Haptic_DRV2605();
	IIC_Init();											//! start the I2C interface
	Haptic_DRV2605_playScript(SCRIPT_RESET);			//! play the reset script
	int haptic_DeviceID = Haptic_DRV2605_getDeviceID(); //! verify that it is a DRV2605L
	Haptic_DRV2605_setActuatorType(LRA);				//初始化线性马达
//	Haptic_DRV2605_writeReg(DRV2605_REG_RATEDV, 0x62);	//设置线性马达Vrms参数
//	Haptic_DRV2605_writeReg(DRV2605_REG_CLAMPV, 0x69);	//设置线性马达Voverride参数
	Haptic_DRV2605_writeReg(DRV2605_REG_RATEDV, 0x8b);	//设置线性马达Vrms参数
	Haptic_DRV2605_writeReg(DRV2605_REG_CLAMPV, 0x96);	//设置线性马达Voverride参数
	Haptic_DRV2605_playScript(SCRIPT_LRA_CALIBRATE);	//运行马达自动校准
	
	
	Haptic_DRV2605_setMode(REGISTER_MODE);				//内部寄存器触发模式
	//Haptic_DRV2605_setMode(GPIO_MODE); //外部触发模式
	//Haptic_DRV2605_setMode(INACTIVE_MODE); //PWM模式

	if (haptic_DeviceID != HAPTIC_CHIP_ID)
	{
		return (HAPTIC_FAIL);
	}
	else
	{
		return (HAPTIC_SUCCESS);
	}
}

/**************************************************************************/
/*!
    Settings and control API's
*/
int Haptic_DRV2605_setActuatorType(enum haptic_dev_t type) {
	if (type >= HAPTIC_DEV_MAX) {
		return (HAPTIC_FAIL);
	}
    actuator.dev_type = type;
	switch (actuator.dev_type)
	{
	  case LRA :
		// setup for LRA Type
	    Haptic_DRV2605_playScript(SCRIPT_LRA_BASIC); //! the LRA "setup"/init script
		break;
	  case ERM :
		// setup for ERM type
	    Haptic_DRV2605_playScript(SCRIPT_ERM_BASIC); //! the ERM "setup"/init script
		break;
	  case ERM_COIN :
		// setup for ERM coin type
	    Haptic_DRV2605_playScript(SCRIPT_ERM_COIN);  //! the ERM Disc "setup"/init script
	    break;
	  case ERM_DMA :
        // setup for ERM Dual Mode type
	    Haptic_DRV2605_playScript(SCRIPT_ERM_BASIC); //! the ERM "setup"/init script
	    break;
	  case LRA_DMA :
		// setup for LRA Dual Mode type
	    Haptic_DRV2605_playScript(SCRIPT_LRA_BASIC); //! the LRA "setup"/init script
	    break;
	  default:
		break;
	}
    return(HAPTIC_SUCCESS);
}

int Haptic_DRV2605_writeWaveformBulk(uint8_t reg, uint8_t *wave, uint8_t size) {
//		printf("ERROR DRV2605: Waveforms are FIXED in ROM, not changeable \n");
		return(HAPTIC_FAIL);
}

int Haptic_DRV2605_readWaveformBulk(uint8_t reg, uint8_t *wave, uint8_t size) {
//		printf("ERROR DRV2605: Waveforms are FIXED in ROM, not readable \n");
		return(HAPTIC_FAIL);
}

int Haptic_DRV2605_setWaveform(uint8_t slot, uint8_t wave) {
	if (slot > 8) {
//		printf("ERROR DRV2605: Invalid Parameter - Slot out of range \n");
		return(HAPTIC_ILL_ARG);
	}
	if (wave >= actuator.dev_waveforms_max) {
//		printf("ERROR DRV2605: Invalid Parameter - Waveform out of range");
		return(HAPTIC_ILL_ARG);
	}
    Haptic_DRV2605_writeReg(DRV2605_REG_WAVESEQ1+slot, wave);
  return(HAPTIC_SUCCESS);
}

int Haptic_DRV2605_setWaveformLib(uint8_t lib){
	if (lib >= actuator.dev_libs_max) {
//		printf("ERROR DRV2605: Invalid Parameter - ROM Library out of range \n");
		return(HAPTIC_ILL_ARG);
	}
	actuator.dev_lib = lib;
	Haptic_DRV2605_writeReg(DRV2605_REG_LIBRARY, lib);
	return(HAPTIC_SUCCESS);
}

int Haptic_DRV2605_getWaveforms(void) {
	return(actuator.dev_waveforms_max);
}

int Haptic_DRV2605_setScript(int num) {
	if (num < 0 || num >= actuator.dev_scripts_max) {
		return(HAPTIC_ILL_ARG);
	}
	actuator.dev_script = num;
  return(HAPTIC_SUCCESS);
}

int Haptic_DRV2605_getScripts(void) {
  return(actuator.dev_scripts_max);
}

int Haptic_DRV2605_playScript(int num) {
	if (num < 0 || num >= actuator.dev_scripts_max) {
		return(HAPTIC_ILL_ARG);
	}
	actuator.dev_script = num;
    return(Haptic_DRV2605_writeRegScript(scriptList[num]));
}

int Haptic_DRV2605_addScript(int num, const struct scr_type script[]) {
	if (num < 0 || num >= ACTUATOR_SCRIPT_MAX || ((actuator.dev_scripts_max+1) >= ACTUATOR_SCRIPT_MAX)) {
		return(HAPTIC_ILL_ARG);
	}
	actuator.dev_script = num;
	actuator.dev_scripts_max++;
    scriptList[num] = script;
    return(HAPTIC_SUCCESS);
}

/*! @brief go - trigger sequence play and return immediately
*/
int Haptic_DRV2605_go(void) {
  Haptic_DRV2605_writeReg(DRV2605_REG_GO, 1);
  return(HAPTIC_SUCCESS);
}

/*! @brief goWait - trigger sequence play and wait for completion
*/
int Haptic_DRV2605_goWait(void) {
  Haptic_DRV2605_writeReg(DRV2605_REG_GO, 1);
  while (Haptic_DRV2605_readReg(DRV2605_REG_GO) & 0x01) {
	 // yield(); // needed for RTOS based ESP8266/ESP32, etc.
  }
  return(HAPTIC_SUCCESS);
}

/*! @brief stop - stop sequence play immediately
*/
int Haptic_DRV2605_stop(void) {
  Haptic_DRV2605_writeReg(DRV2605_REG_GO, 0);
  return(HAPTIC_SUCCESS);
}

/*!
    @brief setMode(enum op_mode mode) - set operating mode
*/
int Haptic_DRV2605_setMode(enum op_mode mode)
{
	if (mode < 0 || mode >= HAPTIC_MODE_MAX) {
		return(HAPTIC_FAIL);
	}
	actuator.op_mode = mode;

	switch (actuator.op_mode)
	{
	  case INACTIVE_MODE :
		// setup for inactive/software standby mode
	    Haptic_DRV2605_writeReg(DRV2605_REG_MODE, DRV2605_MODE_STANDBY);
		break;
	  case STREAM_MODE :
		// setup for Direct Write/Realtime modes
	    Haptic_DRV2605_writeReg(DRV2605_REG_MODE, DRV2605_MODE_REALTIME);
		break;
	  case PWM_MODE :
		// PWM input (GPIO-0) controls vibration
	    Haptic_DRV2605_writeReg(DRV2605_REG_MODE, DRV2605_MODE_PWMANALOG);
	    break;
	  case REGISTER_MODE :
		// register triggered playback
	    Haptic_DRV2605_writeReg(DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);
	    break;
	  case GPIO_MODE :
		// GPIO controlled playback
	    // TODO: add edge/level and polarity setting/logic
	    Haptic_DRV2605_writeReg(DRV2605_REG_MODE, DRV2605_MODE_EXTTRIGLVL); 
	    break;
      case AUDIO_MODE :
        Haptic_DRV2605_writeReg(DRV2605_REG_MODE, DRV2605_MODE_AUDIOVIBE);
        break;
	  // "modes" above DRV2605_OPERATION_MODE_MASK are "soft" modes
	  case SLEEP_MODE :
		// setup for low power/sleep mode
	    Haptic_DRV2605_writeReg(DRV2605_REG_MODE, DRV2605_MODE_STANDBY);
	    break;
	  default:
		break;
	}
	return(HAPTIC_SUCCESS);
}

//! Real-Time play value
int Haptic_DRV2605_setRealtimeValue(uint8_t rtp) {
  return(Haptic_DRV2605_writeReg(DRV2605_REG_RTPIN, rtp));
}

//! audio play mode
int Haptic_DRV2605_playAudio(uint8_t vibectrl, uint8_t minlevel, uint8_t maxlevel, uint8_t mindrv, uint8_t maxdrv) {
    Haptic_DRV2605_setMode(AUDIO_MODE);
    Haptic_DRV2605_writeReg(DRV2605_REG_CONTROL1, 0xB3);           //! set to ac coupled input, with a 0.9V bias (was 0x20)
    Haptic_DRV2605_writeReg(DRV2605_REG_CONTROL3, 0xA3);           //! set to 4%, supply compensation disabled, analog input (was 0xA3)
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOVIBECTRL, vibectrl);  //! audio to vibe control   (0x00) 10ms, 100hz
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOMINLVL, minlevel);    //! audio-to-vibe min level (0x19)
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOMAXLVL, maxlevel);    //! audio-to-vibe max level (0xFF) (0x64?)
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOMINDRV, mindrv);      //! audio-to-vibe min drive (0x19)
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOMAXDRV, maxdrv);      //! audio-to-vibe max drive (0xFF)
    Haptic_DRV2605_go();
    return(HAPTIC_SUCCESS);
}

//! stop audio play mode
int Haptic_DRV2605_stopAudio(void) {
	Haptic_DRV2605_stop();
    Haptic_DRV2605_setMode(REGISTER_MODE);
    // Restore power up defaults 
    Haptic_DRV2605_writeReg(DRV2605_REG_CONTROL1,      0x93);      //! set to ac coupled input, with a 0.9V bias
    Haptic_DRV2605_writeReg(DRV2605_REG_CONTROL3,      0xA0);      //! set to 4%, supply compensation disabled, analog input
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOVIBECTRL, 0x05);      //! audio to vibe control   (0x03) 10ms, 100hz
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOMINLVL,   0x19);      //! audio-to-vibe min level (0x19)
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOMAXLVL,   0xFF);      //! audio-to-vibe max level (0xFF) (0x64?)
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOMINDRV,   0x19);      //! audio-to-vibe min drive (0x19)
    Haptic_DRV2605_writeReg(DRV2605_REG_AUDIOMAXDRV,   0xFF);      //! audio-to-vibe max drive (0xFF)
	return(HAPTIC_SUCCESS);
}

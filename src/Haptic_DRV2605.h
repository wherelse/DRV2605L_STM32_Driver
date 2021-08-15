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
#ifndef _HAPTIC_DRV2605_H
#define _HAPTIC_DRV2605_H
#include "Haptic_DRV2605_registers.h"

//! ----------------------------------------------------------
//! Actuator Meta-Data : actuator and library default configuration
//! "hard" actuator defines
#define ACTUATOR_HAPTIC_DEV			LRA		         //! Actuator Type
//#define ACTUATOR_HAPTIC_DEV			ERM		         //! Actuator Type
#define ACTUATOR_OP_MODE			REGISTER_MODE
#define ACTUATOR_BEMF_SENS_EN		1
#define ACTUATOR_FREQ_TRACK_EN		1
#define ACTUATOR_ACC_EN				1
#define ACTUATOR_RAPID_STOP_EN		1
#define ACTUATOR_AMP_PID_EN			0
#define ACTUATOR_NOM_MV				300				//! Voltage setting / Unit: mili Volt 
#define ACTUATOR_OVERDRIVE_mV		3300			//! Voltage setting / Unit: mili Volt 
#define ACTUATOR_ABS_MAX_mV			5000
#define ACTUATOR_RESONANT_FREQ_Hz	180
#define ACTUATOR_IMAX_mA			137
#define ACTUATOR_IMPD_mOhm			10500			//! in milliOhm units 
#define ACTUATOR_RISE_TIME_mS       50              //! rise time in milliseconds
#define ACTUATOR_BRAKE_TIME_mS      50              //! braking time in milliseconds
#define ACTUATOR_GPI_0_MOD			DRV2605_SINGLE_PTN
#define ACTUATOR_GPI_0_POL			DRV2605_BOTH_EDGE

//! "soft" actuator defines
#define ACTUATOR_OVERIDE_VAL		0x59
#define ACTUATOR_SEQ_ID				7
#define ACTUATOR_SEQ_LOOP			3
#define ACTUATOR_SEQ_ID_MAX			15		    //! SEQ_ID should not be larger than 15 (0 <= X <= 15) 
#define ACTUATOR_SEQ_LOOP_MAX		15
#define ACTUATOR_GPI_0_SEQ_ID		7
#define ACTUATOR_SCRIPT_GOWAIT		0xFD
#define ACTUATOR_SCRIPT_DELAY		0xFE
#define ACTUATOR_SCRIPT_END			0xFF
#define ACTUATOR_SCRIPT_MAX			32
#define ACTUATOR_SCRIPT_COMPLEX		 5          //! start of complex scripts
//! ----------------------------------------------------------
#define HAPTIC_CHIP_ID				0x07

#define HAPTIC_SUCCESS			 0	/*!< success = no errors             */
#define HAPTIC_FAIL             -1  /*!< general failure - cause unknown */
#define HAPTIC_TIMEOUT			-2  /*!< timeout occurred                */
#define HAPTIC_BUSY				-3  /*!< timeout occurred                */
#define HAPTIC_ILL_ARG			-4  /*!< illegal command                 */
#define HAPTIC_ILL_STATE		-5  /*!< illegal state                   */
#define HAPTIC_ILL_ADDR			-6  /*!< illegal address                 */
#define HAPTIC_ILL_IO			-7  /*!< illegal I/O operation           */
#define HAPTIC_NO_RESOURCE		-8  /*!< insufficient resource           */
#define HAPTIC_NOT_SUPPORTED	-9  /*!< not supported                   */
#define HAPTIC_ERR_PLATFORM		-10 /*!< platform or porting layer error */
#define HAPTIC_ERR_IO			-11 /*!< general I/O system error        */

//! script type - a table of register address/value pairs
typedef struct scr_type {
	uint8_t 	reg;
	uint8_t 	val;
}scr_type;

//! script masks
struct scr_mask_type {
	uint8_t 	reg;
	uint8_t 	mask;
	uint8_t 	val;
};

//! device type enumeration
enum haptic_dev_t {
	LRA            = 0,
	ERM            = 1,
	ERM_COIN       = 2,
    ERM_DMA        = 3,
	LRA_DMA        = 4,
	HAPTIC_DEV_MAX,
};

/*!  DRV2605 Operating Modes
----------------------------------------------------------------------------
# Mode  Description
----------------------------------------------------------------------------
INACTIVE_MODE  = 0 Inactive- System waits in IDLE or STANDBY state based on STANDBY_EN setting
STREAM_MODE	   = 1 Direct register override - Playback streaming; input written to OVERRIDE_VAL
PWM_MODE	   = 2 Pulse width modulated - Playback streaming from PWM data input source on pin GPI_0/PWM
REGISTER_MODE  = 3 Register triggered waveform memory - Playback from Waveform Memory triggered only by write to SEQ_START
GPIO_MODE	   = 4 Edge triggered waveform memory - Playback from Waveform Memory triggered by GPIs or via write to SEQ_START
AUDIO_MODE     = 5 Audio Input to Vibration Mode 
DIAG_MODE      = 6 Daagnostic Mode
CALIBRATE_MODE = 7 Auto Calibrate Mode
------------------
SLEEP_MODE     = 8 SLEEP Prepare for lowest-power mode (Inactive+)
----------------------------------------------------------------------------
*/
//! operating modes enumeration
enum op_mode {
	INACTIVE_MODE  = 0,
	STREAM_MODE	   = 1,
	PWM_MODE	   = 2,
	REGISTER_MODE  = 3,
	GPIO_MODE	   = 4,
	AUDIO_MODE     = 5,
	DIAG_MODE      = 6,
	CALIBRATE_MODE = 7,
	SLEEP_MODE     = 8,
	HAPTIC_MODE_MAX
};

//! gpio modes enumeration
enum gpi_modes {
	SINGLE_PTN	= 0,
	HAPTIC_GPI_MODE_MAX
};

//! gpio polarity enumeration
enum gpi_polarity {
	RISING_EDGE		= 0,
	FALLING_EDGE	= 1,
	BOTH_EDGE		= 2,
	LEVEL_HIGH      = 3,
	LEVEL_LOW       = 4,
	HAPTIC_GPI_POLARITY_MAX
};

//! gpio control structure
struct gpi_ctl {
	uint8_t seq_id;
	uint8_t mode;
	uint8_t polarity;
};

//! actuator description structure
//struct haptic_driver {
//	int     dev_lib = 0;
//	int		dev_libs_max = 8;         // fixed for DRV2605L
//	int     dev_waveform = 0;
//	int		dev_waveforms_max = 128;  // fixed for DRV2605L
//	int     dev_script = 0;
//	int		dev_scripts_max = 0;
//	uint8_t dev_state = 0;
//	uint8_t dev_type = ACTUATOR_HAPTIC_DEV;
//	uint8_t op_mode = ACTUATOR_OP_MODE;
//	uint8_t bemf_sense_en = ACTUATOR_BEMF_SENS_EN;
//	uint8_t freq_track_en = ACTUATOR_FREQ_TRACK_EN;
//	uint8_t acc_en = ACTUATOR_ACC_EN;
//	uint8_t rapid_stop_en = ACTUATOR_RAPID_STOP_EN;
//	uint8_t amp_pid_en = ACTUATOR_AMP_PID_EN;
//};

struct haptic_driver {
	int     dev_lib;
	int		dev_libs_max;         // fixed for DRV2605L
	int     dev_waveform;
	int		dev_waveforms_max;  // fixed for DRV2605L
	int     dev_script;
	int		dev_scripts_max;
	uint8_t dev_state;
	uint8_t dev_type;
	uint8_t op_mode;
	uint8_t bemf_sense_en;
	uint8_t freq_track_en;
	uint8_t acc_en;
	uint8_t rapid_stop_en;
	uint8_t amp_pid_en;
};


//  Haptic_DRV2605(void);
//  Haptic_DRV2605(int8_t gp0_pin);
  int Haptic_DRV2605_begin(void);  
  int Haptic_DRV2605_readReg(uint8_t reg);
  int Haptic_DRV2605_writeReg(uint8_t reg, uint8_t val);
  int Haptic_DRV2605_writeRegBits(uint8_t reg, uint8_t mask, uint8_t bits);
  int Haptic_DRV2605_writeRegBulk(uint8_t reg, uint8_t *dada, uint8_t size);
  int Haptic_DRV2605_writeRegScript(const struct scr_type script[]);
  int Haptic_DRV2605_writeWaveformBulk(uint8_t reg, uint8_t *wave, uint8_t size);
  int Haptic_DRV2605_readWaveformBulk(uint8_t reg, uint8_t *wave, uint8_t size);
  int Haptic_DRV2605_setWaveform(uint8_t slot, uint8_t wave);
  int Haptic_DRV2605_setWaveformLib(uint8_t lib);
  int Haptic_DRV2605_getWaveforms(void);
  int Haptic_DRV2605_setScript(int num);
  int Haptic_DRV2605_playScript(int num);
  int Haptic_DRV2605_getScripts(void);
  int Haptic_DRV2605_addScript(int num, const struct scr_type script[]);
  int Haptic_DRV2605_getDeviceID(void);
  int Haptic_DRV2605_setMode(enum op_mode mode);
  int Haptic_DRV2605_setRealtimeValue(uint8_t rtp);
  int Haptic_DRV2605_setActuatorType(enum haptic_dev_t type);
  int Haptic_DRV2605_go(void);
  int Haptic_DRV2605_goWait(void);
  int Haptic_DRV2605_stop(void);
  int Haptic_DRV2605_playAudio(uint8_t vibectrl, uint8_t minlevel, uint8_t maxlevel, uint8_t mindrv, uint8_t maxdrv);
  void Haptic_DRV2605_Haptic_DRV2605(void);
  int Haptic_DRV2605_stopAudio(void);

#endif

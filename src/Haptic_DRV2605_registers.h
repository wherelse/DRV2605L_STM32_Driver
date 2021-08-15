/*
 * Haptic_DRV2605_registers.h - Haptics REGISTERS for DRV2605
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __HAPTIC_DRV2605_REGISTERS_H
#define __HAPTIC_DRV2605_REGISTERS_H

#define DRV2605_I2C_ADDR          0x5A<<1

#define DRV2605_REG_STATUS        0x00
#define DRV2605_REG_MODE          0x01

#define DRV2605_MODE_INTTRIG      0x00
#define DRV2605_MODE_EXTTRIGEDGE  0x01
#define DRV2605_MODE_EXTTRIGLVL   0x02
#define DRV2605_MODE_PWMANALOG    0x03
#define DRV2605_MODE_AUDIOVIBE    0x04
#define DRV2605_MODE_REALTIME     0x05
#define DRV2605_MODE_DIAGNOS      0x06
#define DRV2605_MODE_AUTOCAL      0x07
#define DRV2605_MODE_STANDBY      0x40

#define DRV2605_REG_RTPIN         0x02
#define DRV2605_REG_LIBRARY       0x03
#define DRV2605_REG_WAVESEQ1      0x04
#define DRV2605_REG_WAVESEQ2      0x05
#define DRV2605_REG_WAVESEQ3      0x06
#define DRV2605_REG_WAVESEQ4      0x07
#define DRV2605_REG_WAVESEQ5      0x08
#define DRV2605_REG_WAVESEQ6      0x09
#define DRV2605_REG_WAVESEQ7      0x0A
#define DRV2605_REG_WAVESEQ8      0x0B

#define DRV2605_REG_GO            0x0C
#define DRV2605_REG_OVERDRIVE     0x0D
#define DRV2605_REG_SUSTAINPOS    0x0E
#define DRV2605_REG_SUSTAINNEG    0x0F
#define DRV2605_REG_BRAKE         0x10
#define DRV2605_REG_AUDIOVIBECTRL 0x11
#define DRV2605_REG_AUDIOMINLVL   0x12
#define DRV2605_REG_AUDIOMAXLVL   0x13
#define DRV2605_REG_AUDIOMINDRV   0x14
#define DRV2605_REG_AUDIOMAXDRV   0x15
#define DRV2605_REG_RATEDV        0x16
#define DRV2605_REG_CLAMPV        0x17
#define DRV2605_REG_AUTOCALCOMP   0x18
#define DRV2605_REG_AUTOCALEMP    0x19
#define DRV2605_REG_FEEDBACK      0x1A
#define DRV2605_REG_CONTROL1      0x1B
#define DRV2605_REG_CONTROL2      0x1C
#define DRV2605_REG_CONTROL3      0x1D
#define DRV2605_REG_CONTROL4      0x1E
#define DRV2605_REG_RFU1          0x1F
#define DRV2605_REG_RFU2          0x20
#define DRV2605_REG_VBAT          0x21
#define DRV2605_REG_LRARESON      0x22

#endif /* __HAPTIC_DRV2605_REGISTERS_H */

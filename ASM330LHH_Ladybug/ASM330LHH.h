/* 06/19/2019 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Ladybug default), respectively, and it uses the Ladybug STM32L432KC Breakout Board.
  The ASM330LHH is a AEC-Q100 qualified ST motion sensor with embedded accel and gyro, similar to the LSM6DSM.

  Library may be used freely and without limit with attribution.
  
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Tlera Corp, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.

*/

#ifndef ASM330LHH_h
#define ASM330LHH_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

/* ASM330LHH registers
https://hr.mouser.com/pdfDocs/STM_AN5296.pdf
*/
#define ASM330LHH_PIN_CTRL                  0x02
#define ASM330LHH_FIFO_CTRL1                0x07
#define ASM330LHH_FIFO_CTRL2                0x08
#define ASM330LHH_FIFO_CTRL3                0x09
#define ASM330LHH_FIFO_CTRL4                0x0A
#define ASM330LHH_COUNTER_BDR_REG1          0x0B
#define ASM330LHH_COUNTER_BDR_REG2          0x0C
#define ASM330LHH_INT1_CTRL                 0x0D
#define ASM330LHH_INT2_CTRL                 0x0E
#define ASM330LHH_WHO_AM_I                  0x0F  // should be 0x6B
#define ASM330LHH_CTRL1_XL                  0x10
#define ASM330LHH_CTRL2_G                   0x11
#define ASM330LHH_CTRL3_C                   0x12
#define ASM330LHH_CTRL4_C                   0x13
#define ASM330LHH_CTRL5_C                   0x14
#define ASM330LHH_CTRL6_C                   0x15
#define ASM330LHH_CTRL7_G                   0x16
#define ASM330LHH_CTRL8_XL                  0x17
#define ASM330LHH_CTRL9_XL                  0x18
#define ASM330LHH_CTRL10_C                  0x19
#define ASM330LHH_ALL_INT_SRC               0x1A
#define ASM330LHH_WAKE_UP_SRC               0x1B
#define ASM330LHH_D6D_SRC                   0x1D
#define ASM330LHH_STATUS_REG                0x1E
#define ASM330LHH_OUT_TEMP_L                0x20
#define ASM330LHH_OUT_TEMP_H                0x21
#define ASM330LHH_OUTX_L_G                  0x22
#define ASM330LHH_OUTX_H_G                  0x23
#define ASM330LHH_OUTY_L_G                  0x24
#define ASM330LHH_OUTY_H_G                  0x25
#define ASM330LHH_OUTZ_L_G                  0x26
#define ASM330LHH_OUTZ_H_G                  0x27
#define ASM330LHH_OUTX_L_XL                 0x28
#define ASM330LHH_OUTX_H_XL                 0x29
#define ASM330LHH_OUTY_L_XL                 0x2A
#define ASM330LHH_OUTY_H_XL                 0x2B
#define ASM330LHH_OUTZ_L_XL                 0x2C
#define ASM330LHH_OUTZ_H_XL                 0x2D
#define ASM330LHH_FIFO_STATUS1              0x3A
#define ASM330LHH_FIFO_STATUS2              0x3B
#define ASM330LHH_TIMESTAMP0_REG            0x40
#define ASM330LHH_TIMESTAMP1_REG            0x41
#define ASM330LHH_TIMESTAMP2_REG            0x42
#define ASM330LHH_TIMESTAMP3_REG            0x43
#define ASM330LHH_INT_CFG0                  0x56
#define ASM330LHH_INT_CFG1                  0x58
#define ASM330LHH_THS_6D                    0x59
#define ASM330LHH_INT_DUR2                  0x5A
#define ASM330LHH_WAKE_UP_THS               0x5B
#define ASM330LHH_WAKE_UP_DUR               0x5C
#define ASM330LHH_FREE_FALL                 0x5D
#define ASM330LHH_MD1_CFG                   0x5E
#define ASM330LHH_MD2_CFG                   0x5F
#define ASM330LHH_INTERNAL_FREQ_FINE        0x63
#define ASM330LHH_X_OFS_USR                 0x73
#define ASM330LHH_Y_OFS_USR                 0x74
#define ASM330LHH_Z_OFS_USR                 0x75
#define ASM330LHH_FIFO_DATA_OUT_TAG         0x78
#define ASM330LHH_FIFO_DATA_OUT_X_L         0x79
#define ASM330LHH_FIFO_DATA_OUT_X_H         0x7A
#define ASM330LHH_FIFO_DATA_OUT_Y_L         0x7B
#define ASM330LHH_FIFO_DATA_OUT_Y_H         0x7C
#define ASM330LHH_FIFO_DATA_OUT_Z_L         0x7D
#define ASM330LHH_FIFO_DATA_OUT_Z_H         0x7E

#define ASM330LHH_ADDRESS           0x6B   // Address of ASM330LHH accel/gyro when ADO = 0


#define AFS_2G  0x00
#define AFS_4G  0x02
#define AFS_8G  0x03
#define AFS_16G 0x01

#define GFS_250DPS  0x00
#define GFS_500DPS  0x01
#define GFS_1000DPS 0x02
#define GFS_2000DPS 0x03

#define AODR_12_5Hz  0x01  // same for accel and gyro in normal mode
#define AODR_26Hz    0x02
#define AODR_52Hz    0x03
#define AODR_104Hz   0x04
#define AODR_208Hz   0x05
#define AODR_417Hz   0x06
#define AODR_833Hz   0x07
#define AODR_1667Hz  0x08
#define AODR_3333Hz  0x09
#define AODR_6667Hz  0x0A

#define GODR_12_5Hz  0x01   
#define GODR_26Hz    0x02
#define GODR_52Hz    0x03
#define GODR_104Hz   0x04
#define GODR_208Hz   0x05
#define GODR_417Hz   0x06
#define GODR_833Hz   0x07
#define GODR_1667Hz  0x08
#define GODR_3333Hz  0x09
#define GODR_6667Hz  0x0A


class ASM330LHH
{
  public:
  ASM330LHH(I2Cdev* i2c_bus);
  float getAres(uint8_t Ascale);
  float getGres(uint8_t Gscale);
  uint8_t getChipID();
  void init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR);
  void offsetBias(float * dest1, float * dest2);
  void reset();
  void selfTest();
  void readData(int16_t * destination);
  void readXLData(int16_t * destination);
  void readGData(int16_t * destination);

  private:
  float _aRes, _gRes;
  I2Cdev* _i2c_bus;
};

#endif

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

#include "ASM330LHH.h"
#include "I2Cdev.h"


ASM330LHH::ASM330LHH(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t ASM330LHH::getChipID()
{
  uint8_t c = _i2c_bus->readByte(ASM330LHH_ADDRESS, ASM330LHH_WHO_AM_I);
  return c;
}

float ASM330LHH::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (10), 8 Gs (11), and 16 Gs  (01). 
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
         _aRes = 2.0f/32768.0f;
         return _aRes;
         break;
    case AFS_4G:
         _aRes = 4.0f/32768.0f;
         return _aRes;
         break;
    case AFS_8G:
         _aRes = 8.0f/32768.0f;
         return _aRes;
         break;
    case AFS_16G:
         _aRes = 16.0f/32768.0f;
         return _aRes;
         break;
  }
}

float ASM330LHH::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    case GFS_250DPS:
          _gRes = 250.0f/32768.0f;
          return _gRes;
          break;
    case GFS_500DPS:
          _gRes = 500.0f/32768.0f;
          return _gRes;
          break;
    case GFS_1000DPS:
         _gRes = 1000.0f/32768.0f;
         return _gRes;
         break;
    case GFS_2000DPS:
          _gRes = 2000.0f/32768.0f;
         return _gRes;
         break;
  }
}


void ASM330LHH::reset()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL3_C);
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL3_C, temp | 0x01); // Set bit 0 to 1 to reset ASM330LHH
  delay(100); // Wait for all registers to reset 
}


void ASM330LHH::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{

  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_COUNTER_BDR_REG1, 0x80); // set data ready pulse mode
  // enable low pass filter (bit 1)
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL1_XL, AODR << 4 | Ascale << 2 | 0x02);
  
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL2_G, GODR << 4 | Gscale << 2);
 
  uint8_t temp = _i2c_bus->readByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL3_C);
  // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL3_C, temp | 0x40 | 0x04); 
  // by default, interrupts active HIGH, push pull, little endian data 
  // (can be changed by writing to bits 5, 4, and 1, resp to above register)

  // mask data ready until filter settling ends
   _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL4_C, 0x08); 

   // set LP2 to ODR/10 (bits 5 - 7 = 001) 
   _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL8_XL,  0x01 << 5 );

   // interrupt handling
    _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_INT1_CTRL, 0x01);      // enable XL data ready interrupts on INT1
    _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_INT2_CTRL, 0x02);      // enable G data ready interrupts on INT2  
}


void ASM330LHH::selfTest()
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};
  uint8_t status = false;

// accel test
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL1_XL, 0x38);  
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL3_C,  0x44); 
  delay(100);

  status = false;
  while(status == false) {
    status = (_i2c_bus->readByte(ASM330LHH_ADDRESS, ASM330LHH_STATUS_REG)) & 0x01;
  }
  readData(temp); // read and discard data

  for (uint8_t i = 0; i < 5; i++){
  readData(temp);
  accelNom[0] += temp[4]; // read data five times
  accelNom[1] += temp[5];
  accelNom[2] += temp[6];
  }
  accelNom[0] /= 5.0f; // average data
  accelNom[1] /= 5.0f;
  accelNom[2] /= 5.0f;
  
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL5_C, 0x01); // positive accel self test
  delay(100); // let accel respond

  status = false;
  while(status == false) {
    status = (_i2c_bus->readByte(ASM330LHH_ADDRESS, ASM330LHH_STATUS_REG)) & 0x01;
  }
  readData(temp); // read and discard data

  for (uint8_t i = 0; i < 5; i++){
  readData(temp);
  accelPTest[0] += temp[4];
  accelPTest[1] += temp[5];
  accelPTest[2] += temp[6];
  }
  accelPTest[0] /= 5.0f;
  accelPTest[1] /= 5.0f;
  accelPTest[2] /= 5.0f;

  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL5_C, 0x02); // negative accel self test
  delay(100); // let accel respond

  status = false;
  while(status == false) {
    status = (_i2c_bus->readByte(ASM330LHH_ADDRESS, ASM330LHH_STATUS_REG)) & 0x01;
  }
  readData(temp); // read and discard data

  for (uint8_t i = 0; i < 5; i++){
  readData(temp);
  accelNTest[0] += temp[4];
  accelNTest[1] += temp[5];
  accelNTest[2] += temp[6];
  }
  accelNTest[0] /= 5.0f;
  accelNTest[1] /= 5.0f;
  accelNTest[2] /= 5.0f;


// gyro self test
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL1_XL, 0x00);  
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL2_G, 0x5C);
  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL3_C, 0x44); 
  delay(100);

  status = false;
  while(status == false) {
    status = (_i2c_bus->readByte(ASM330LHH_ADDRESS, ASM330LHH_STATUS_REG)) & 0x02;
  }
  readData(temp); // read and discard data

  for (uint8_t i = 0; i < 5; i++){
  readData(temp);
  gyroNom[0] += temp[1]; // read data five times
  gyroNom[1] += temp[2];
  gyroNom[2] += temp[3];
  }
  gyroNom[0] /= 5.0f; // average data
  gyroNom[1] /= 5.0f;
  gyroNom[2] /= 5.0f;

  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL5_C, 0x04); // positive gyro self test
  delay(100); // let gyro respond
  
  status = false;
  while(status == false) {
    status = (_i2c_bus->readByte(ASM330LHH_ADDRESS, ASM330LHH_STATUS_REG)) & 0x02;
  }
  readData(temp); // read and discard data

  for (uint8_t i = 0; i < 5; i++){
  readData(temp);
  gyroPTest[0] += temp[1];
  gyroPTest[1] += temp[2];
  gyroPTest[2] += temp[3];
  }
  gyroPTest[0] /= 5.0f;
  gyroPTest[1] /= 5.0f;
  gyroPTest[2] /= 5.0f;

  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL5_C, 0x0C); // negative gyro self test
  delay(100); // let gyro respond
  
  status = false;
  while(status == false) {
    status = (_i2c_bus->readByte(ASM330LHH_ADDRESS, ASM330LHH_STATUS_REG)) & 0x02;
  }
  readData(temp); // read and discard data

  for (uint8_t i = 0; i < 5; i++){
  readData(temp);
  gyroNTest[0] += temp[1];
  gyroNTest[1] += temp[2];
  gyroNTest[2] += temp[3];
  }
  gyroNTest[0] /= 5.0f;
  gyroNTest[1] /= 5.0f;
  gyroNTest[2] /= 5.0f;

  _i2c_bus->writeByte(ASM330LHH_ADDRESS, ASM330LHH_CTRL5_C, 0x00); // normal mode
  delay(100); // let accel and gyro respond

  Serial.println("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");

  Serial.println("Gyro Self Test:");
  Serial.print("+Gx results:"); Serial.print((gyroPTest[0] - gyroNom[0]) * _gRes); Serial.println(" dps");
  Serial.print("-Gx results:"); Serial.println((gyroNTest[0] - gyroNom[0]) * _gRes);
  Serial.print("+Gy results:"); Serial.println((gyroPTest[1] - gyroNom[1]) * _gRes);
  Serial.print("-Gy results:"); Serial.println((gyroNTest[1] - gyroNom[1]) * _gRes);
  Serial.print("+Gz results:"); Serial.println((gyroPTest[2] - gyroNom[2]) * _gRes);
  Serial.print("-Gz results:"); Serial.println((gyroNTest[2] - gyroNom[2]) * _gRes);
  Serial.println("Should be between 20 and 80 dps");
  delay(2000);
}


void ASM330LHH::offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};
    
  Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  delay(4000);

  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1]*_gRes/128.0f;
  dest1[1] = sum[2]*_gRes/128.0f;
  dest1[2] = sum[3]*_gRes/128.0f;
  dest2[0] = sum[4]*_aRes/128.0f;
  dest2[1] = sum[5]*_aRes/128.0f;
  dest2[2] = sum[6]*_aRes/128.0f;

  if(dest2[0] > 0.8f)  {dest2[0] -= 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest2[0] < -0.8f) {dest2[0] += 1.0f;}  // Remove gravity from the x-axis accelerometer bias calculation
  if(dest2[1] > 0.8f)  {dest2[1] -= 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest2[1] < -0.8f) {dest2[1] += 1.0f;}  // Remove gravity from the y-axis accelerometer bias calculation
  if(dest2[2] > 0.8f)  {dest2[2] -= 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation
  if(dest2[2] < -0.8f) {dest2[2] += 1.0f;}  // Remove gravity from the z-axis accelerometer bias calculation

}


void ASM330LHH::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(ASM330LHH_ADDRESS, ASM330LHH_OUT_TEMP_L, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;   
  destination[4] = ((int16_t)rawData[9] << 8) | rawData[8] ;  
  destination[5] = ((int16_t)rawData[11] << 8) | rawData[10] ;  
  destination[6] = ((int16_t)rawData[13] << 8) | rawData[12] ; 
}


void ASM330LHH::readGData(int16_t * destination)
{
  uint8_t rawData[8];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(ASM330LHH_ADDRESS, ASM330LHH_OUT_TEMP_L, 8, &rawData[0]);  // Read the 8 raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;    
}


void ASM330LHH::readXLData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(ASM330LHH_ADDRESS, ASM330LHH_OUTX_L_XL, 6, &rawData[0]);  // Read the 6 raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

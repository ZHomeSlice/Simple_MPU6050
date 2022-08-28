/* ============================================
  Simple_MPU6050 device library code is placed under the MIT license
  Copyright (c) 2021 Homer Creutz

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

#ifndef MPU_ReadMacros_h
#define MPU_ReadMacros_h
// Names are taken straight from the register map for the gyroscope and accelerometer Specifically MPU9250 which should match the MPU6050
//        Register Name | ReadWrite | Bits Name    (regAddr size length bit Data Read/Write)
//9250 Self Test Gyro
//#define SELF_TEST_X_GYRO_READ_xg_st_data(Data)						ReadByte(0x00, (uint8_t *)Data)    //   self test output generated during manufacturing tests
//#define SELF_TEST_Y_GYRO_READ_yg_st_data(Data)						ReadByte(0x01, (uint8_t *)Data)    //   self test output generated during manufacturing tests
//#define SELF_TEST_Z_GYRO_READ_zg_st_data(Data)						ReadByte(0x02, (uint8_t *)Data)    //   self test output generated during manufacturing tests
//6050 Accelerometer Fine Gain Offsets (Need Documentation)
#define RA_X_FINE_GAIN_READ(Data)         									ReadByte(0x03, (uint8_t *)Data)		 //[7:0] X_FINE_GAIN
#define RA_Y_FINE_GAIN_READ(Data)         									ReadByte(0x04, (uint8_t *)Data)		 //[7:0] Y_FINE_GAIN
#define RA_Z_FINE_GAIN_READ(Data)         									ReadByte(0x05, (uint8_t *)Data)		 //[7:0] Z_FINE_GAIN
//6050 Accellerator Offsets
#define A_OFFSET_H_READ_A_OFFS(Data)												ReadUInts(0x06, 3, (uint16_t *)Data)//   X accelerometer offset cancellation
#define XA_OFFSET_H_READ_XA_OFFS(Data)											ReadUInt(0x06, (uint16_t *)Data)    //   X accelerometer offset cancellation
#define YA_OFFSET_H_READ_YA_OFFS(Data)											ReadUInt(0x08, (uint16_t *)Data)    //   Y accelerometer offset cancellation
#define ZA_OFFSET_H_READ_ZA_OFFS(Data)											ReadUInt(0x0A, (uint16_t *)Data)    //   Z accelerometer offset cancellation
//9250 Self Test Accel
//#define SELF_TEST_X_ACCEL_READ_XA_ST_DATA(Data)						ReadByte(0x0D, (uint8_t *)Data)    //   self test output generated during manufacturing tests
//#define SELF_TEST_Y_ACCEL_READ_YA_ST_DATA(Data)						ReadByte(0x0E, (uint8_t *)Data)    //   self test output generated during manufacturing tests
//#define SELF_TEST_Z_ACCEL_READ_ZA_ST_DATA(Data)						ReadByte(0x0F, (uint8_t *)Data)    //   self test output generated during manufacturing tests
//6050 Self Test Gyro and Accel
#define SELF_TEST_X_READ_XA_TEST(Data)											ReadBit(0x0D, 3, 7, (uint8_t *)Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_X_READ_XG_TEST(Data)											ReadBit(0x0D, 5, 4, (uint8_t *)Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_Y_READ_YA_TEST(Data)											ReadBit(0x0E, 3, 7, (uint8_t *)Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_Y_READ_YG_TEST(Data)											ReadBit(0x0E, 5, 4, (uint8_t *)Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_Z_READ_ZA_TEST(Data)											ReadBit(0x0F, 3, 7, (uint8_t *)Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_Z_READ_ZG_TEST(Data)											ReadBit(0x0F, 5, 4, (uint8_t *)Data)  //   self test output generated during manufacturing tests
//Both
#define XG_OFFSET_H_READ_OFFS_USR(Data)											ReadUInts(0x13, 3, (uint16_t *)Data)//   Remove DC bias from the gyro sensor Step 0.0305 dps
#define XG_OFFSET_H_READ_X_OFFS_USR(Data)										ReadUInt(0x13, (uint16_t *)Data)    //   Remove DC bias from the gyro sensor Step 0.0305 dps
#define YG_OFFSET_H_READ_Y_OFFS_USR(Data)										ReadUInt(0x15, (uint16_t *)Data)    //   Remove DC bias from the gyro sensor Step 0.0305 dps
#define ZG_OFFSET_H_READ_Z_OFFS_USR(Data)										ReadUInt(0x17, (uint16_t *)Data)    //   Remove DC bias from the gyro sensor Step 0.0305 dps

#define SMPLRT_DIV_READ_SMPLRT_DIV(Data)										ReadByte(0x19, (uint8_t *)Data)    //   Divides the internal sample rate  controls sensor data output rate, FIFO sample rate.   SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)

#define CONFIG_READ_FIFO_MODE(Data)													ReReadBitad(0x1A, 1, 6, (uint8_t *)Data)  //   When set to  1 , when the fifo is full, additional writes will not be written to fifo.  When set to  0 , when the fifo is full, additional writes will be written to the fifo, replacing the oldest data.
#define CONFIG_READ_EXT_SYNC_SET(Data)											ReadBit(0x1A, 3, 5, (uint8_t *)Data)  //   Enables the FSYNC pin data to be sampled.
#define CONFIG_READ_DLPF_CFG(Data)													ReadBit(0x1A, 3, 2, (uint8_t *)Data)  //   DLPF Configuration


#define GYRO_CONFIG_READ_XGYRO_Ct_en(Data)									ReadBit(0x1B, 1, 7, (uint8_t *)Data)  //   X Gyro self-test
#define GYRO_CONFIG_READ_YGYRO_Ct_en(Data)									ReadBit(0x1B, 1, 6, (uint8_t *)Data)  //   Y Gyro self-test
#define GYRO_CONFIG_READ_ZGYRO_Ct_en(Data)									ReadBit(0x1B, 1, 5, (uint8_t *)Data)  //   Z Gyro self-test
#define GYRO_CONFIG_READ_GYRO_FS_SEL(Data)									ReadBit(0x1B, 2, 4, (uint8_t *)Data)  //   Gyro Full Scale Select: 00B = +250 01B = +500 10B = +1000 11B = 2000

#define GYRO_CONFIG_READ_FCHOICE_B(Data)										ReadBit(0x1B, 2, 2, (uint8_t *)Data)  //   Used to bypass DLPF
#define ACCEL_CONFIG_READ_ax_st_en(Data)										ReadBit(0x1C, 1, 7, (uint8_t *)Data)  //   X Accel self-test
#define ACCEL_CONFIG_READ_ay_st_en(Data)										ReadBit(0x1C, 1, 6, (uint8_t *)Data)  //   Y Accel self-test
#define ACCEL_CONFIG_READ_az_st_en(Data)										ReadBit(0x1C, 1, 5, (uint8_t *)Data)  //   Z Accel self-test
#define ACCEL_CONFIG_READ_ACCEL_FS_SEL(Data)								ReadBit(0x1C, 2, 3, (uint8_t *)Data)  //   Accel Full Scale Select:  2g (00),  4g (01),  8g (10),  16g (11)


#define ACCEL_CONFIG_2_READ_ACCEL_FCHOICE_B(Data)						ReadBit(0x1D, 2, 2, (uint8_t *)Data)  //   Used to bypass DLPF
#define ACCEL_CONFIG_2_READ_A_DLPF_CFG(Data)								ReadBit(0x1D, 2, 1, (uint8_t *)Data)  //   Accelerometer low pass filter setting
#define LP_ACCEL_ODR_READ_Lposc_clksel(Data)								ReadBit(0x1E, 4, 3, (uint8_t *)Data)  //   Frequency of waking up the chip to take a sample of accel data
#define WOM_THR_READ_WOM_Threshold(Data)										ReadByte(0x1F, (uint8_t *)Data)    //   Threshold Value for the Wake on Motion Interrupt


#define FIFO_EN_READ(Data)                                  ReadByte(0x23,(uint8_t *)Data)     //  Read FIFO_EN_READ 
#define FIFO_EN_READ_FIFO_EN(Data)					   			   			ReadByte(0x23, (uint8_t *)Data)    //   Adds Temp data to FIFO
#define FIFO_EN_READ_TEMP_FIFO_EN(Data)											ReadBit(0x23, 1, 7, (uint8_t *)Data)  //   Adds Temp data to FIFO
#define FIFO_EN_READ_GYRO_XOUT(Data)				 			     			ReadBit(0x23, 1, 6, (uint8_t *)Data)  //   Adds GYRO X  data to FIFO
#define FIFO_EN_READ_GYRO_YOUT(Data)			   			   				ReadBit(0x23, 1, 5, (uint8_t *)Data)  //   Adds GYRO Y data to FIFO
#define FIFO_EN_READ_GYRO_ZOUT(Data)			  			    		  ReadBit(0x23, 1, 4, (uint8_t *)Data)  //   Adds GYRO Z data to FIFO
#define FIFO_EN_READ_ACCEL(Data)					 						     	ReadBit(0x23, 1, 3, (uint8_t *)Data)  //   Adds ACCEL data to FIFO
#define FIFO_EN_READ_SLV2(Data)														  ReadBit(0x23, 1, 2, (uint8_t *)Data)  //   Adds SLV2 data to FIFO
#define FIFO_EN_READ_SLV1(Data)														  ReadBit(0x23, 1, 1, (uint8_t *)Data)  //   Adds SLV1 data to FIFO
#define FIFO_EN_READ_SLV0(Data)														  ReadBit(0x23, 1, 0, (uint8_t *)Data)  //   Adds SLV0 data to FIFO

#define I2C_MST_CTRL_READ_ALL(Data)													ReadByte(0x24, (uint8_t *)Data)	   //   I2C STUFF
#define I2C_MST_CTRL_READ_MULT_MST_EN(Data)								  ReadBit(0x24, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_MST_CTRL_READ_WAIT_FOR_ES(Data)								  ReadBit(0x24, 1, 6, (uint8_t *)Data)  //   I2C STUFF
#define I2C_MST_CTRL_READ_SLV_3_FIFO_EN(Data)						    ReadBit(0x24, 1, 5, (uint8_t *)Data)  //   I2C STUFF
#define I2C_MST_CTRL_READ_I2C_MST_P_NSR(Data)						    ReadBit(0x24, 1, 4, (uint8_t *)Data)  //   I2C STUFF
#define I2C_MST_CTRL_READ_I2C_MST_CLK(Data)							    ReadBit(0x24, 4, 3, (uint8_t *)Data)  //   I2C STUFF

#define I2C_SLV0_ADDR_READ_ALL(Data)												ReadByte(0x25, (uint8_t *)Data)	   //   I2C STUFF
#define I2C_SLV0_ADDR_READ_I2C_SLV0_RNW(Data)		      	    ReadBit(0x25, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV0_ADDR_READ_I2C_ID_0(Data)				  			    ReadBit(0x25, 7, 6, (uint8_t *)Data)  //   I2C STUFF

#define I2C_SLV0_REG_READ_I2C_SLV0_REG(Data)                ReadByte(0x26, (uint8_t *)Data)    //   I2C STUFF

#define I2C_SLV0_CTRL_READ_ALL(Data)												ReadByte(0x27, (uint8_t *)Data)    //   I2C STUFF
#define I2C_SLV0_CTRL_READ_I2C_SLV0_EN(Data)                ReadBit(0x27, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV0_CTRL_READ_I2C_SLV0_BYTE_SW(Data)           ReadBit(0x27, 1, 6, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV0_CTRL_READ_I2C_SLV0_REG_DIS(Data)           ReadBit(0x27, 1, 5, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV0_CTRL_READ_I2C_SLV0_GRP(Data)               ReadBit(0x27, 1, 4, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV0_CTRL_READ_I2C_SLV0_LENG(Data)              ReadBit(0x27, 4, 3, (uint8_t *)Data)  //   I2C STUFF

#define I2C_SLV1_ADDR_READ_ALL(Data)												ReadByte(0x28, (uint8_t *)Data)    //   I2C STUFF
#define I2C_SLV1_ADDR_READ_I2C_SLV1_RNW(Data)               ReadBit(0x28, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV1_ADDR_READ_I2C_ID_1(Data)                   ReadBit(0x28, 7, 6, (uint8_t *)Data)  //   I2C STUFF

#define I2C_SLV1_REG_READ_I2C_SLV1_REG(Data)                ReadByte(0x29, (uint8_t *)Data)    //   I2C STUFF

#define I2C_SLV1_CTRL_READ_ALL(Data)												ReadByte(0x2A, (uint8_t *)Data)    //   I2C STUFF
#define I2C_SLV1_CTRL_READ_I2C_SLV1_EN(Data)                ReadBit(0x2A, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV1_CTRL_READ_I2C_SLV1_BYTE_SW(Data)           ReadBit(0x2A, 1, 6, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV1_CTRL_READ_I2C_SLV1_REG_DIS(Data)           ReadBit(0x2A, 1, 5, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV1_CTRL_READ_I2C_SLV1_GRP(Data)               ReadBit(0x2A, 1, 4, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV1_CTRL_READ_I2C_SLV1_LENG(Data)              ReadBit(0x2A, 4, 3, (uint8_t *)Data)  //   I2C STUFF

#define I2C_SLV2_ADDR_READ_I2C_SLV2_RNW(Data)               ReadBit(0x2B, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV2_ADDR_READ_I2C_ID_2(Data)                   ReadBit(0x2B, 7, 6, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV2_REG_READ_I2C_SLV2_REG(Data)                ReadByte(0x2C, (uint8_t *)Data)    //   I2C STUFF
#define I2C_SLV2_CTRL_READ_I2C_SLV2_EN(Data)                ReadBit(0x2D, 1, 6, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV2_CTRL_READ_I2C_SLV2_BYTE_SW(Data)           ReadBit(0x2D, 1, 5, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV2_CTRL_READ_I2C_SLV2_REG_DIS(Data)           ReadBit(0x2D, 1, 4, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV2_CTRL_READ_I2C_SLV2_GRP(Data)               ReadBit(0x2D, 1, 3, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV2_CTRL_READ_I2C_SLV2_LENG(Data)              ReadBit(0x2D, 4, 2, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV3_ADDR_READ_I2C_SLV3_RNW(Data)               ReadBit(0x2E, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV3_ADDR_READ_I2C_ID_3(Data)                   ReadBit(0x2E, 7, 6, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV3_REG_READ_I2C_SLV3_REG(Data)                ReadByte(0x2F, (uint8_t *)Data)    //   I2C STUFF
#define I2C_SLV3_CTRL_READ_I2C_SLV3_EN(Data)                ReadBit(0x30, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV3_CTRL_READ_I2C_SLV3_BYTE_SW(Data)           ReadBit(0x30, 1, 6, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV3_CTRL_READ_I2C_SLV3_REG_DIS(Data)           ReadBit(0x30, 1, 5, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV3_CTRL_READ_I2C_SLV3_GRP(Data)               ReadBit(0x30, 1, 4, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV3_CTRL_READ_I2C_SLV3_LENG(Data)              ReadBit(0x30, 5, 3, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV4_ADDR_READ_I2C_SLV4_RNW(Data)               ReadBit(0x31, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV4_ADDR_READ_I2C_ID_4(Data)                   ReadBit(0x31, 7, 6, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV4_REG_READ_I2C_SLV4_REG(Data)                ReadByte(0x32, (uint8_t *)Data)    //   I2C STUFF
#define I2C_SLV4_DO_READ_I2C_SLV4_DO(Data)                  ReadByte(0x33, (uint8_t *)Data)    //   I2C STUFF

#define I2C_SLV4_CTRL_READ_ALL(Data)              			    ReadByte(0x34, (uint8_t *)Data)    //   I2C STUFF

#define I2C_SLV4_CTRL_READ_I2C_SLV4_EN(Data)                ReadBit(0x34, 1, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV4_CTRL_READ_SLV4_DONE_INT_EN(Data)           ReadBit(0x34, 1, 6, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV4_CTRL_READ_I2C_SLV4_REG_DIS(Data)           ReadBit(0x34, 1, 5, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV4_CTRL_READ_I2C_MST_DLY(Data)                ReadBit(0x34, 5, 4, (uint8_t *)Data)  //   I2C STUFF
#define I2C_SLV4_DI_READ_I2C_SLV4_DI(Data)                  ReadBit(0x35, 7, 7, (uint8_t *)Data)  //   I2C STUFF
#define I2C_MST_STATUS_READ_PASS_THROUGH(Data)              ReadBit(0x36, 1, 7, (uint8_t *)Data)  //   Status of FSYNC interrupt   used as a way to pass an external interrupt through this chip to the host.
#define I2C_MST_STATUS_READ_I2C_SLV4_DONE(Data)             ReadBit(0x36, 1, 6, (uint8_t *)Data)  //   Asserted when I2C slave 4 s transfer is complete,
#define I2C_MST_STATUS_READ_I2C_LOST_ARB(Data)              ReadBit(0x36, 1, 5, (uint8_t *)Data)  //   Asserted when I2C slave looses arbitration of the I2C bus,
#define I2C_MST_STATUS_READ_I2C_SLV4_NACK(Data)             ReadBit(0x36, 1, 4, (uint8_t *)Data)  //   Asserted when slave 4 receives a nack,
#define I2C_MST_STATUS_READ_I2C_SLV3_NACK(Data)             ReadBit(0x36, 1, 3, (uint8_t *)Data)  //   Asserted when slave 3 receives a nack,
#define I2C_MST_STATUS_READ_I2C_SLV2_NACK(Data)             ReadBit(0x36, 1, 2, (uint8_t *)Data)  //   Asserted when slave 2 receives a nack,
#define I2C_MST_STATUS_READ_I2C_SLV1_NACK(Data)             ReadBit(0x36, 1, 1, (uint8_t *)Data)  //   Asserted when slave 1 receives a nack,
#define I2C_MST_STATUS_READ_I2C_SLV0_NACK(Data)             ReadBit(0x36, 1, 0, (uint8_t *)Data)  //   Asserted when slave 0 receives a nack,


#define INT_PIN_CFG_READ(Data)                              ReadBytes(0x37, 1, (uint8_t *)Data)//  Read INT_PIN_CFG 
#define INT_PIN_CFG_READ_ACTL(Data)                         ReadBit(0x37, 1, 7, (uint8_t *)Data)  //   1   The logic level for int16_t pin is active low. 0   The logic level for int16_t pin is active high.
#define INT_PIN_CFG_READ_OPEN(Data)                         ReadBit(0x37, 1, 6, (uint8_t *)Data)  //   1   int16_t pin is configured as open drain. 0   int16_t pin is configured as push-pull.
#define INT_PIN_CFG_READ_LATCH_INT_EN(Data)                 ReadBit(0x37, 1, 5, (uint8_t *)Data)  //   1   int16_t pin level held until interrupt status is cleared. 0   int16_t pin indicates interrupt pulse s is width 50us.
#define INT_PIN_CFG_READ_INT_ANYRD_2CLEAR(Data)             ReadBit(0x37, 1, 4, (uint8_t *)Data)  //   1   Interrupt status is cleared if any read operation is performed. 0   Interrupt status is cleared only by reading INT_STATUS register
#define INT_PIN_CFG_READ_ACTL_FSYNC(Data)                   ReadBit(0x37, 1, 3, (uint8_t *)Data)  //   1   The logic level for the FSYNC pin as an interrupt is active low. 0   The logic level for the FSYNC pin as an interrupt is active high.
#define INT_PIN_CFG_READ_FSYNC_INT_MODE_EN(Data)            ReadBit(0x37, 1, 2, (uint8_t *)Data)  //   1   This enables the FSYNC pin to be used as an interrupt.
#define INT_PIN_CFG_READ_BYPASS_EN(Data)                    ReadBit(0x37, 1, 1, (uint8_t *)Data)  //   When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into  bypass mode  when the i2c master interface is disabled.


#define INT_ENABLE_READ(Data)                               ReadByte(0x38,Data)  //  Read INT_ENABLE_READ 
#define INT_ENABLE_READ_FF_EN(Data)                         ReadBit(0x38, 1, 7, (uint8_t *)Data)  //   1   Enable interrupt for
#define INT_ENABLE_READ_WOM_EN(Data)                        ReadBit(0x38, 1, 6, (uint8_t *)Data)  //   1   Enable interrupt for wake on motion to propagate to interrupt pin. 0   function is disabled.
#define INT_ENABLE_READ_ZMOT_OFLOW_EN(Data)                 ReadBit(0x38, 1, 5, (uint8_t *)Data)  //   1   Enable interrupt for
#define INT_ENABLE_READ_FIFO_OFLOW_EN(Data)                 ReadBit(0x38, 1, 4, (uint8_t *)Data)  //   1   Enable interrupt for fifo overflow to propagate to interrupt pin. 0   function is disabled.
#define INT_ENABLE_READ_FSYNC_INT_EN(Data)                  ReadBit(0x38, 1, 3, (uint8_t *)Data)  //   1   Enable (I2C_MST_INT_BIT) Fsync interrupt to propagate to interrupt pin. 0   function is disabled.
#define INT_ENABLE_READ_RAW_PLL_RDY_INT_EN(Data)            ReadBit(0x38, 1, 2, (uint8_t *)Data)  //   1   Enable
#define INT_ENABLE_READ_RAW_DMP_INT_EN(Data)                ReadBit(0x38, 1, 1, (uint8_t *)Data)  //   1   Enable
#define INT_ENABLE_READ_RAW_RDY_EN(Data)                    ReadBit(0x38, 1, 0, (uint8_t *)Data)  //   1   Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin.  0   function is disabled.

//#define INT_STATUS_ALL_READ(Data)                           ReadUInt(0x39,  (uint16_t *)Data) //  Read INT_STATUS_DMP_READ 
// Above first byte is considered a read. so the second byte is cleared... fail
// Create macros for all read to detect the states of each byte

#define INT_STATUS_DMP_READ(Data)                           ReadByte(0x39, (uint8_t *)Data)    //  Read INT_STATUS_DMP_READ 

// The following fail when more than the one value is needed we need alternate macros after grabbing the full byte or int.
#define INT_STATUS_DMP_READ_FIFO_OVERFLOW(Data)             ReadBit(0x39,1,4,(uint8_t *)Data)     //   1   dmp Fifo Overflow interrupt occurred.

#define INT_STATUS_READ(Data)                               ReadByte(0x3A, (uint8_t *)Data)    //  Read INT_STATUS_READ
// The following fail when more than the one value is needed we need alternate macros after grabbing the full byte or int.
#define INT_STATUS_READ_WOM_INT(Data)                       ReadBit(0x3A, 1, 6, (uint8_t *)Data)  //   1   Wake on motion interrupt occurred.
#define INT_STATUS_READ_FIFO_OFLOW_INT(Data)                ReadBit(0x3A, 1, 4, (uint8_t *)Data)  //   1   Fifo Overflow interrupt occurred.
#define INT_STATUS_READ_FSYNC_INT(Data)                     ReadBit(0x3A, 1, 3, (uint8_t *)Data)  //   1   Fsync interrupt occurred.
#define INT_STATUS_READ_RAW_DATA_RDY_INT(Data)              ReadBit(0x3A, 1, 0, (uint8_t *)Data)  //   1   Sensor Register Raw Data sensors are updated and Ready to be read.


#define ACCEL_XOUT_H_READ_ACCEL_TMP_GYTO(Data)              ReadUInts(0x3B,7, (uint16_t *)Data) //   accelerometer x-axis data.

#define ACCEL_XOUT_H_READ_ACCEL(Data)                       ReadUInts(0x3B, 3, (uint16_t *)Data)//   accelerometer x-axis data.

#define ACCEL_XOUT_H_READ_ACCEL_XOUT(Data)                  ReadUInt(0x3B, (uint16_t *)Data)    //   accelerometer x-axis data.
#define ACCEL_YOUT_H_READ_ACCEL_YOUT(Data)                  ReadUInt(0x3D, (uint16_t *)Data)    //   accelerometer y-axis data.
#define ACCEL_ZOUT_H_READ_ACCEL_ZOUT(Data)                  ReadUInt(0x3F, (uint16_t *)Data)    //   accelerometer z-axis data.

#define TEMP_OUT_H_READ_TEMP_OUT(Data)                      ReadUInt(0x41, (uint16_t *)Data)    //   temperature sensor output   TEMP_degC = ((TEMP_OUT   RoomTemp_Offset)/Temp_Sensitivity) + 21degC

#define GYRO_XOUT_H_READ_GYRO(Data)                         ReadUInts(0x43, 3, (uint16_t *)Data)//   X-Axis gyroscope output

#define GYRO_XOUT_H_READ_GYRO_XOUT(Data)                    ReadUInt(0x43, (uint16_t *)Data)    //   X-Axis gyroscope output
#define GYRO_YOUT_H_READ_GYRO_YOUT(Data)                    ReadUInt(0x45, (uint16_t *)Data)    //   Y-Axis gyroscope output
#define GYRO_ZOUT_H_READ_GYRO_ZOUT(Data)                    ReadUInt(0x47, (uint16_t *)Data)    //   Z-Axis gyroscope output


#define EXT_SENS_DATA_READ_LENGTH(Length,Data)							ReadBytes(0x49,Length, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_READ_EXT_SENS_DATA_ALL(Data)					ReadBytes(0x49,(0x66-0x49), (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_00_READ_EXT_SENS_DATA_00(Data)        ReadByte(0x49, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_01_READ_EXT_SENS_DATA_01(Data)        ReadByte(0x4A, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_02_READ_EXT_SENS_DATA_02(Data)        ReadByte(0x4B, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_03_READ_EXT_SENS_DATA_03(Data)        ReadByte(0x4C, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_04_READ_EXT_SENS_DATA_04(Data)        ReadByte(0x4D, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_05_READ_EXT_SENS_DATA_05(Data)        ReadByte(0x4E, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_06_READ_EXT_SENS_DATA_06(Data)        ReadByte(0x4F, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_07_READ_EXT_SENS_DATA_07(Data)        ReadByte(0x50, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_08_READ_EXT_SENS_DATA_08(Data)        ReadByte(0x51, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_09_READ_EXT_SENS_DATA_09(Data)        ReadByte(0x52, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_10_READ_EXT_SENS_DATA_10(Data)        ReadByte(0x53, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_11_READ_EXT_SENS_DATA_11(Data)        ReadByte(0x54, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_12_READ_EXT_SENS_DATA_12(Data)        ReadByte(0x55, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_13_READ_EXT_SENS_DATA_13(Data)        ReadByte(0x56, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_14_READ_EXT_SENS_DATA_14(Data)        ReadByte(0x57, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_15_READ_EXT_SENS_DATA_15(Data)        ReadByte(0x58, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_16_READ_EXT_SENS_DATA_16(Data)        ReadByte(0x59, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_17_READ_EXT_SENS_DATA_17(Data)        ReadByte(0x5A, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_18_READ_EXT_SENS_DATA_18(Data)        ReadByte(0x5B, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_19_READ_EXT_SENS_DATA_19(Data)        ReadByte(0x5C, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_20_READ_EXT_SENS_DATA_20(Data)        ReadByte(0x5D, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_21_READ_EXT_SENS_DATA_21(Data)        ReadByte(0x5E, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_22_READ_EXT_SENS_DATA_22(Data)        ReadByte(0x5F, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define EXT_SENS_DATA_23_READ_EXT_SENS_DATA_23(Data)        ReadByte(0x60, (uint8_t *)Data)  //   Sensor data read from external I2C devices
#define I2C_SLV0_DO_READ_I2C_SLV0_DO(Data)				  			  ReadByte(0x63, (uint8_t *)Data)  //   Data out when slave 0 is set to write
#define I2C_SLV1_DO_READ_I2C_SLV1_DO(Data)                  ReadByte(0x64, (uint8_t *)Data)  //   Data out when slave 1 is set to write
#define I2C_SLV2_DO_READ_I2C_SLV2_DO(Data)                  ReadByte(0x65, (uint8_t *)Data)  //   Data out when slave 2 is set to write
#define I2C_SLV3_DO_READ_I2C_SLV3_DO(Data)                  ReadByte(0x66, (uint8_t *)Data)  //   Data out when slave 3 is set to write

#define I2C_MST_DELAY_CTRL_READ_ALL(Data)										ReadByte(0x67, (uint8_t *)Data)  //   Delays shadowing of external sensor data until     ALL data is received
#define I2C_MST_DELAY_CTRL_READ_DELAY_ES_SHADOW(Data)       ReadBit(0x67, 1, 7, (uint8_t *)Data)  //   Delays shadowing of external sensor data until     ALL data is received
#define I2C_MST_DELAY_CTRL_READ_I2C_SLV4_DLY_EN(Data)       ReadBit(0x67, 1, 4, (uint8_t *)Data)  //   When enabled, slave 4 will only be accessed
#define I2C_MST_DELAY_CTRL_READ_I2C_SLV3_DLY_EN(Data)       ReadBit(0x67, 1, 3, (uint8_t *)Data)  //   When enabled, slave 3 will only be accessed
#define I2C_MST_DELAY_CTRL_READ_I2C_SLV2_DLY_EN(Data)       ReadBit(0x67, 1, 2, (uint8_t *)Data)  //   N When enabled, slave 2 will only be accessed
#define I2C_MST_DELAY_CTRL_READ_I2C_SLV1_DLY_EN(Data)       ReadBit(0x67, 1, 1, (uint8_t *)Data)  //   When enabled, slave 1 will only be accessed
#define I2C_MST_DELAY_CTRL_READ_I2C_SLV0_DLY_EN(Data)       ReadBit(0x67, 1, 0, (uint8_t *)Data)  //   When enabled, slave 0 will only be accessed


#define SIGNAL_PATH_RESET_READ(Data)                        ReadByte(0x68, (uint8_t *)Data)  //  Read SIGNAL_PATH_RESET 
#define SIGNAL_PATH_RESET_READ_GYRO_RST(Data)               ReadBit(0x68, 1, 2, (uint8_t *)Data)  //   Reset gyro digital signal path  Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers.
#define SIGNAL_PATH_RESET_READ_ACCEL_RST(Data)              ReadBit(0x68, 1, 1, (uint8_t *)Data)  //   Reset accel digital signal path.  Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers.
#define SIGNAL_PATH_RESET_READ_TEMP_RST(Data)               ReadBit(0x68, 1, 0, (uint8_t *)Data)  //   Reset temp digital signal path. Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers.
#define MOT_DETECT_CTRL_READ_ACCEL_INTEL_EN(Data)           ReadBit(0x69, 1, 7, (uint8_t *)Data)  //   This bit enables the Wake-on-Motion detection logic
#define MOT_DETECT_CTRL_READ_ACCEL_INTEL_MODE(Data)         ReadBit(0x69, 1, 6, (uint8_t *)Data)  //   1 = Compare the current sample with the previous sample. 0 = Not used.


#define USER_CTRL_READ(Data)                                ReadByte(0x6A, (uint8_t *)Data)  //  Read USER_CTRL_READ 
#define USER_CTRL_READ_DMP_EN(Data)                         ReadBit(0x6A, 1, 7, (uint8_t *)Data)  //   1   Enable DMP operation mode.
#define USER_CTRL_READ_FIFO_EN(Data)                        ReadBit(0x6A, 1, 6, (uint8_t *)Data)  //   1   Enable FIFO operation mode. 0   Disable FIFO access from serial interface.  To disable FIFO writes by dma, use FIFO_EN register. To disable possible FIFO writes from DMP, disable the DMP.
#define USER_CTRL_READ_I2C_MST_EN(Data)                     ReadBit(0x6A, 1, 5, (uint8_t *)Data)  //   1   Enable the I2C Master
#define USER_CTRL_READ_I2C_IF_DIS(Data)                     ReadBit(0x6A, 1, 4, (uint8_t *)Data)  //   1   Disable I2C Slave module and put the serial interface in SPI mode only.
#define USER_CTRL_READ_DMP_RST(Data)                        ReadBit(0x6A, 1, 3, (uint8_t *)Data)  //   1   Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
#define USER_CTRL_READ_FIFO_RST(Data)                       ReadBit(0x6A, 1, 2, (uint8_t *)Data)  //   1   Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
#define USER_CTRL_READ_I2C_MST_RESET_BIT(Data)              ReadBit(0x6A, 1, 1, (uint8_t *)Data)  //   1   Reset  gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.
#define USER_CTRL_READ_SIG_COND_RST(Data)                   ReadBit(0x6A, 1, 0, (uint8_t *)Data)  //   1   Reset  gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.


#define USER_CTRL_READ_SIG_COND_RST(Data)                   ReadBit(0x6A, 1, 0, (uint8_t *)Data)  //   1   Reset  gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.

#define PWR_MGMT_1_READ(Data)                               ReadByte(0x6B, (uint8_t *)Data)  //  Read PWR_MGMT_1 
#define PWR_MGMT_1_READ_H_RESET(Data)                       ReadBit(0x6B, 1, 7, (uint8_t *)Data)  //   1   Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear.
#define PWR_MGMT_1_READ_SLEEP(Data)                         ReadBit(0x6B, 1, 6, (uint8_t *)Data)  //   When set, the chip is set to sleep mode (After OTP loads, the PU_SLEEP_MODE bit will be written here)
#define PWR_MGMT_1_READ_CYCLE(Data)                         ReadBit(0x6B, 1, 5, (uint8_t *)Data)  //   When set, and SLEEP and STANDBY are not set, the chip will cycle between sleep and taking a single sample at a rate determined by LP_ACCEL_ODR register
#define PWR_MGMT_1_READ_GYRO_STANDBY(Data)                  ReadBit(0x6B, 1, 4, (uint8_t *)Data)  //   When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled. This is a low power mode that allows quick enabling of the gyros.
#define PWR_MGMT_1_READ_PD_PTAT(Data)                       ReadBit(0x6B, 1, 3, (uint8_t *)Data)  //   Power down internal PTAT voltage generator and PTAT ADC
#define PWR_MGMT_1_READ_CLKSEL(Data)                        ReadBit(0x6B, 3, 2, (uint8_t *)Data)  //   Clock Source Select

#define PWR_MGMT_2_READ(Data)                               ReadByte(0x6C, (uint8_t *)Data)  //  Read PWR_MGMT_2 
#define PWR_MGMT_2_READ_LP_WAKE_CTRL(Data)                  ReadBit(0x6C, 2, 7, (uint8_t *)Data)  //   1   X accelerometer is disabled 0   X accelerometer is on
#define PWR_MGMT_2_READ_DIS_XA(Data)                        ReadBit(0x6C, 1, 6, (uint8_t *)Data)  //   1   X accelerometer is disabled 0   X accelerometer is on
#define PWR_MGMT_2_READ_DIS_YA(Data)                        ReadBit(0x6C, 1, 5, (uint8_t *)Data)  //   1   Y accelerometer is disabled 0   Y accelerometer is on
#define PWR_MGMT_2_READ_DIS_ZA(Data)                        ReadBit(0x6C, 1, 4, (uint8_t *)Data)  //   1   Z accelerometer is disabled 0   Z accelerometer is on
#define PWR_MGMT_2_READ_DIS_XG(Data)												ReadBit(0x6C, 1, 3, (uint8_t *)Data)  //   1   X gyro is disabled 0   X gyro is on
#define PWR_MGMT_2_READ_DIS_YG(Data)			  			          ReadBit(0x6C, 1, 2, (uint8_t *)Data)  //   1   Y gyro is disabled 0   Y gyro is on
#define PWR_MGMT_2_READ_DIS_ZG(Data)			   						    ReadBit(0x6C, 1, 1, (uint8_t *)Data)  //   1   Z gyro is disabled 0   Z gyro is on

#define BANK_SEL_READ(Data)																	ReadUInt(0x6D, (uint16_t *)Data)  //   DMP Bank Select for Loading Image
#define DMP_MEM_START_ADDR_READ(Data)			     			        ReadUInt(0x6E, (uint16_t *)Data)  //   Not Used
#define DMP_MEM_READ(Length,Data)			        			        ReadBytes(0x6F,Length, (uint8_t *)Data)  // DMP Image Loading Location
#define PRGM_START_H_READ(Data)													  	ReadUInt(0x70, (uint16_t *)Data)  // Set program start address.

#define FIFO_COUNTH_READ_FIFO_CNT(Data)											ReadUInt(0x72, (uint16_t *)Data)  //   High Bits, count indicates the number of written bytes in the FIFO.
#define FIFO_READ(PacketLength, Data)												ReadBytes(0x74, PacketLength, (uint8_t *)Data)  //   Read/Write command provides Read or Write operation for the FIFO.
#define WHO_AM_I_READ_WHOAMI(Data)                          ReadBit(0x75,6,6, (uint8_t *)Data)  //   Register to indicate to user which device is being accessed.
//6500 and 9250
#define XA_OFFSET_H_READ_0x77_XA_OFFS(Data)									ReadUInt(0x77, (uint16_t *)Data)  //   X accelerometer offset cancellation
#define YA_OFFSET_H_READ_0x77_YA_OFFS(Data)					  			ReadUInt(0x7A, (uint16_t *)Data)  //   Y accelerometer offset cancellation
#define ZA_OFFSET_H_READ_0x77_ZA_OFFS(Data)								  ReadUInt(0x7D, (uint16_t *)Data)  //   Z accelerometer offset cancellation


// Scan i2c buss for addresses Start Address,End Limit:
// example;
// uint8_t Address = 0;
// FindAddress(Address,128) // checks and increments Address until we find something then stops
// Address now contains the address of the discovered device.
#define FindAddressx(Address,Limit)	{  uint8_t error = 4; while(1){ Wire.beginTransmission(Address);if((Wire.endTransmission() == 0)||!((Address++)%Limit)) break;}}
#endif

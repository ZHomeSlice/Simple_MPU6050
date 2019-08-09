#ifndef MPU_WriteMacros_h
#define MPU_WriteMacros_h
// Names are taken straight from the register map for the gyroscope and accelerometer Specifically MPU9250 which should match the MPU6050
//        Register Name | ReadWrite | Bits Name    (regAddr size length bit Data Read/Write)
//9250 Self Test Gyro
#define SELF_TEST_X_GYRO_WRITE_xg_st_data(Data)         MPUi2cWriteByte(0x00, (uint8_t)Data)			//   self test output generated during manufacturing tests
#define SELF_TEST_Y_GYRO_WRITE_yg_st_data(Data)         MPUi2cWriteByte(0x01, (uint8_t)Data)			//   self test output generated during manufacturing tests
#define SELF_TEST_Y_GYRO_WRITE_I2C_MST_VDDIO(Data)      MPUi2cWrite(0x01, 1,7, (uint8_t)Data)		//   self test output generated during manufacturing tests
#define SELF_TEST_Z_GYRO_WRITE_zg_st_data(Data)         MPUi2cWriteByte(0x02, (uint8_t)Data)			//   self test output generated during manufacturing tests
//6050 Accelerometer Offsets
#define A_OFFSET_WRITE_A_OFFS(Shift, Data)              MPUi2cWriteInt(0x06 + (Shift * 2), (uint16_t)Data)//
#define A_OFFSET_H_WRITE_A_OFFS(Data)                   MPUi2cWriteInts(0x06, 6, (uint16_t *)Data)		//   X accelerometer offset cancellation
#define XA_OFFSET_H_WRITE_XA_OFFS(Data)                 MPUi2cWriteInt(0x06, (uint16_t)Data)			//   X accelerometer offset cancellation
#define YA_OFFSET_H_WRITE_YA_OFFS(Data)                 MPUi2cWriteInt(0x08, (uint16_t)Data)			//   Y accelerometer offset cancellation
#define ZA_OFFSET_H_WRITE_ZA_OFFS(Data)                 MPUi2cWriteInt(0x0A,  (uint16_t)Data)			//   Z accelerometer offset cancellation
//9250 Self Test Accel
//#define SELF_TEST_X_ACCEL_WRITE_XA_ST_DATA(Data)      MPUi2cWriteByte(0x0D, (uint8_t)Data)			//   self test output generated during manufacturing tests
//#define SELF_TEST_Y_ACCEL_WRITE_YA_ST_DATA(Data)      MPUi2cWriteByte(0x0E, (uint8_t)Data)			//   self test output generated during manufacturing tests
//#define SELF_TEST_Z_ACCEL_WRITE_ZA_ST_DATA(Data)      MPUi2cWriteByte(0x0F, (uint8_t)Data)			//   self test output generated during manufacturing tests
//6050 Self Test Gyro and Accel
#define SELF_TEST_X_WRITE_XA_TEST(Data)                 MPUi2cWrite(0x0D,3, 7, (uint8_t)Data)		//   self test output generated during manufacturing tests
#define SELF_TEST_X_WRITE_XG_TEST(Data)                 MPUi2cWrite(0x0D,5, 4, (uint8_t)Data)		//   self test output generated during manufacturing tests
#define SELF_TEST_Y_WRITE_YA_TEST(Data)                 MPUi2cWrite(0x0E, 3, 7, (uint8_t)Data)		//   self test output generated during manufacturing tests
#define SELF_TEST_Y_WRITE_YG_TEST(Data)                 MPUi2cWrite(0x0E, 5, 4, (uint8_t)Data)		//   self test output generated during manufacturing tests
#define SELF_TEST_Z_WRITE_ZA_TEST(Data)                 MPUi2cWrite(0x0F, 3, 7, (uint8_t)Data)		//   self test output generated during manufacturing tests
#define SELF_TEST_Z_WRITE_ZG_TEST(Data)                 MPUi2cWrite(0x0F, 5, 4, (uint8_t)Data)		//   self test output generated during manufacturing tests
//Both
#define XG_OFFSET_WRITE_OFFS_USR(Shift, Data)           MPUi2cWriteInts(0x13 + (Shift * 2), (uint16_t *)Data)		//   Remove DC bias from the gyro sensor Step 0.0305 dps
#define XG_OFFSET_H_WRITE_OFFS_USR(Data)                MPUi2cWriteInts(0x13, 6, (uint16_t *)Data)		//   Remove DC bias from the gyro sensor Step 0.0305 dps
#define XG_OFFSET_H_WRITE_X_OFFS_USR(Data)              MPUi2cWriteInt(0x13,  (uint16_t)Data)			//   Remove DC bias from the gyro sensor Step 0.0305 dps
#define YG_OFFSET_H_WRITE_Y_OFFS_USR(Data)              MPUi2cWriteInt(0x15,  (uint16_t)Data)			//   Remove DC bias from the gyro sensor Step 0.0305 dps
#define ZG_OFFSET_H_WRITE_Z_OFFS_USR(Data)              MPUi2cWriteInt(0x17,  (uint16_t)Data)			//   Remove DC bias from the gyro sensor Step 0.0305 dps

#define SMPLRT_DIV_WRITE_SMPLRT_DIV(Data)               MPUi2cWriteByte(0x19, (uint8_t)Data)			//   Divides the internal sample rate  controls sensor data output rate, FIFO sample rate.   SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)

#define CONFIG_WRITE_FIFO_MODE(Data)                    MPUi2cWrite(0x1A, 1, 6, (uint8_t)Data)		//   When set to 1, when the fifo is full, additional writes will not be written to fifo.  When set to 0, when the fifo is full, additional writes will be written to the fifo, replacing the oldest data.
#define CONFIG_WRITE_EXT_SYNC_SET(Data)                 MPUi2cWrite(0x1A, 3, 5, (uint8_t)Data)		//   Enables the FSYNC pin data to be sampled.
#define CONFIG_WRITE_DLPF_CFG(Data)                     MPUi2cWrite(0x1A, 3, 2, (uint8_t)Data)		//   DLPF Configuration

#define CONFIG_WRITE_DLPF_256HZ_NOLPF2(...)             MPUi2cWrite(0x1A, 3, 2, (uint8_t)0)			//   DLPF Configuration
#define CONFIG_WRITE_DLPF_188HZ(...)                    MPUi2cWrite(0x1A, 3, 2, (uint8_t)1)			//   DLPF Configuration
#define CONFIG_WRITE_DLPF_98HZ(...)                     MPUi2cWrite(0x1A, 3, 2, (uint8_t)2)			//   DLPF Configuration
#define CONFIG_WRITE_DLPF_42HZ(...)                     MPUi2cWrite(0x1A, 3, 2, (uint8_t)3)			//   DLPF Configuration
#define CONFIG_WRITE_DLPF_20HZ(...)                     MPUi2cWrite(0x1A, 3, 2, (uint8_t)4)			//   DLPF Configuration
#define CONFIG_WRITE_DLPF_10HZ(...)                     MPUi2cWrite(0x1A, 3, 2, (uint8_t)5)			//   DLPF Configuration
#define CONFIG_WRITE_DLPF_5HZ(...)                      MPUi2cWrite(0x1A, 3, 2, (uint8_t)6)			//   DLPF Configuration
#define CONFIG_WRITE_DLPF_2100HZ_NOLPF(...)             MPUi2cWrite(0x1A, 3, 2, (uint8_t)7)			//   DLPF Configuration

#define GYRO_CONFIG_WRITE_XGYRO_Ct_en(Data)             MPUi2cWrite(0x1B, 1, 7, (uint8_t)Data)		//   X Gyro self-test
#define GYRO_CONFIG_WRITE_YGYRO_Ct_en(Data)             MPUi2cWrite(0x1B, 1, 6, (uint8_t)Data)		//   Y Gyro self-test
#define GYRO_CONFIG_WRITE_ZGYRO_Ct_en(Data)             MPUi2cWrite(0x1B, 1, 5, (uint8_t)Data)		//   Z Gyro self-test
#define GYRO_CONFIG_WRITE_GYRO_FS_SEL(Data)             MPUi2cWrite(0x1B, 2, 4, (uint8_t)Data)		//   Gyro Full Scale Select: 00B = +250 01B = +500 10B = +1000 11B = 2000

#define GYRO_CONFIG_WRITE_GYRO_FS_SEL_250(...)          MPUi2cWrite(0x1B, 3, 4, (uint8_t)0)			//   Gyro Full Scale Select: +250 Deg/sec
#define GYRO_CONFIG_WRITE_GYRO_FS_SEL_500(...)          MPUi2cWrite(0x1B, 3, 4, (uint8_t)1)			//   Gyro Full Scale Select: +500 Deg/sec
#define GYRO_CONFIG_WRITE_GYRO_FS_SEL_1000(...)         MPUi2cWrite(0x1B, 3, 4, (uint8_t)2)			//   Gyro Full Scale Select: +1000 Deg/sec
#define GYRO_CONFIG_WRITE_GYRO_FS_SEL_2000(...)         MPUi2cWrite(0x1B, 3, 4, (uint8_t)3)			//   Gyro Full Scale Select: +2000 Deg/sec
#define GYRO_CONFIG_WRITE_FCHOICE_B(Data)               MPUi2cWrite(0x1B, 2, 2, (uint8_t)Data)		//   Used to bypass DLPF

#define ACCEL_CONFIG_WRITE_ax_st_en(Data)               MPUi2cWrite(0x1C, 1, 7, (uint8_t)Data)		//   X Accel self-test
#define ACCEL_CONFIG_WRITE_ay_st_en(Data)               MPUi2cWrite(0x1C, 1, 6, (uint8_t)Data)		//   Y Accel self-test
#define ACCEL_CONFIG_WRITE_az_st_en(Data)               MPUi2cWrite(0x1C, 1, 5, (uint8_t)Data)		//   Z Accel self-test
#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL(Data)           MPUi2cWrite(0x1C, 2, 3, (uint8_t)Data)		//   Accel Full Scale Select: 2g (00), 4g (01), 8g (10), 16g (11)

#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL_2g(...)         MPUi2cWrite(0x1C, 2, 3, (uint8_t)0)			//   Accel Full Scale Select: 2g
#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL_4g(...)         MPUi2cWrite(0x1C, 2, 3, (uint8_t)1)			//   Accel Full Scale Select: 4g
#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL_8g(...)         MPUi2cWrite(0x1C, 2, 3, (uint8_t)2)			//   Accel Full Scale Select: 8g
#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL_16g(...)        MPUi2cWrite(0x1C, 2, 3, (uint8_t)3)			//   Accel Full Scale Select: 16g


#define ACCEL_CONFIG_2_WRITE_ACCEL_FCHOICE_B(Data)      MPUi2cWrite(0x1D, 2, 2, (uint8_t)Data)		//   Used to bypass DLPF
#define ACCEL_CONFIG_2_WRITE_A_DLPF_CFG(Data)           MPUi2cWrite(0x1D, 2, 1, (uint8_t)Data)		//   Accelerometer low pass filter setting

#define LP_ACCEL_ODR_WRITE_LPOSC_CLKSEL(Data)           MPUi2cWrite(0x1E, 4, 3, (uint8_t)Data)		//   Frequency of waking up the chip to take a sample of accel data
#define WOM_THR_WRITE_WOM_THRESHOLD(Data)               MPUi2cWriteByte(0x1F, (uint8_t)Data)			//   Threshold Value for the Wake on Motion Interrupt

#define FIFO_EN_WRITE_FIFO_EN(Data)                     MPUi2cWriteByte(0x23, (uint8_t)Data)			//   Adds Temp data to FIFO
#define FIFO_EN_WRITE_TEMP_FIFO_EN(Data)                MPUi2cWrite(0x23, 1, 7, (uint8_t)Data)		//   Adds Temp data to FIFO
#define FIFO_EN_WRITE_GYRO_XOUT(Data)                   MPUi2cWrite(0x23, 1, 6, (uint8_t)Data)		//   Adds GYRO X  data to FIFO
#define FIFO_EN_WRITE_GYRO_YOUT(Data)                   MPUi2cWrite(0x23, 1, 5, (uint8_t)Data)		//   Adds GYRO Y data to FIFO
#define FIFO_EN_WRITE_GYRO_ZOUT(Data)                   MPUi2cWrite(0x23, 1, 4, (uint8_t)Data)		//   Adds GYRO Z data to FIFO
#define FIFO_EN_WRITE_ACCEL(Data)                       MPUi2cWrite(0x23, 1, 3, (uint8_t)Data)		//   Adds ACCEL data to FIFO
#define FIFO_EN_WRITE_SLV2(Data)                        MPUi2cWrite(0x23, 1, 2, (uint8_t)Data)		//   Adds SLV2 data to FIFO
#define FIFO_EN_WRITE_SLV1(Data)                        MPUi2cWrite(0x23, 1, 1, (uint8_t)Data)		//   Adds SLV1 data to FIFO
#define FIFO_EN_WRITE_SLV0(Data)                        MPUi2cWrite(0x23, 1, 0, (uint8_t)Data)		//   Adds SLV0 data to FIFO


// Controlls the Secondayr bus to the Magnetometer
#define I2C_MST_CTRL_WRITE_MULT_MST_EN(Data)            MPUi2cWrite(0x24, 1, 7, (uint8_t)Data)		//   *Enables multi-master capability. When disabled, clocking to the I2C_MST_IF
																							//   can be disabled when not in use and the logic to detect lost arbitration is disabled.
#define I2C_MST_CTRL_WRITE_WAIT_FOR_ES(Data)            MPUi2cWrite(0x24, 1, 6, (uint8_t)Data)		//   Delays the data ready interrupt until external sensor data is loaded. If
																							//   I2C_MST_IF is disabled, the interrupt will still occur.
#define I2C_MST_CTRL_WRITE_SLV_3_FIFO_EN(Data)          MPUi2cWrite(0x24, 1, 5, (uint8_t)Data)		//   1 – write EXT_SENS_DATA registers associated to SLV_3 (as determined by
																							//   I2C_SLV0_CTRL and I2C_SLV1_CTRL and I2C_SLV2_CTRL) to the FIFO at the sample rate;
																							//   0 – function is disabled
#define I2C_MST_CTRL_WRITE_I2C_MST_P_NSR(Data)          MPUi2cWrite(0x24, 1, 4, (uint8_t)Data)		//   This bit controls the I2C Master’s transition from one slave read to the next
																							//   slave read. If 0, there is a restart between reads. If 1, there is a stop between reads.
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK(Data)            MPUi2cWrite(0x24, 4, 3, (uint8_t)Data)		//   I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the MPU9250 internal 8MHz clock. 																							//   It sets the I2C master clock speed according to the following table:#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_348(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)0)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_333(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)1)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_320(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)2)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_308(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)3)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_296(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)4)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_286(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)5)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_276(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)6)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_267(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)7)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_258(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)8)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_500(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)9)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_471(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)10)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_444(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)11)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_421(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)12)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_400(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)13)			//   * 13 400 kHz 
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_381(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)14)
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK_364(...)         MPUi2cWrite(0x24, 4, 3, (uint8_t)15) 

// I2C Slave 0
#define I2C_SLV0_ADDR_WRITE_I2C_SLV0_RNW(Data)          MPUi2cWrite(0x25, 1, 7, (uint8_t)Data)		//   1 – Transfer is a read, 0 – Transfer is a write
#define I2C_SLV0_ADDR_WRITE_I2C_ID_0(Data)              MPUi2cWrite(0x25, 7, 6, (uint8_t)Data)		//   Physical address of I2C slave 0
#define I2C_SLV0_REG_WRITE_I2C_SLV0_REG(Data)           MPUi2cWriteByte(0x26, (uint8_t)Data)			//   I2C slave 0 register address from where to begin data transfer

#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_EN(Data)           MPUi2cWrite(0x27, 1, 7, (uint8_t)Data)		//   1 – Enable reading data from this slave at the sample rate and storing data at the first available EXT_SENS_DATA register,
																							//	which is always EXT_SENS_DATA_00 for I2C slave 0. 0 – function is disabled for this slave
#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_BYTE_SW(Data)      MPUi2cWrite(0x27, 1, 6, (uint8_t)Data)		//   1 – Swap bytes when reading both the low and high byte of a word. Note there is nothing to swap after reading the first byte
																							//	if I2C_SLV0_REG[0] = 1, or if the last byte read has a register address lsb = 0. 0 – no swapping occurs, bytes are written in order read.
#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_REG_DIS(Data)      MPUi2cWrite(0x27, 1, 5, (uint8_t)Data)		//   When set, the transaction does not write a register value, it will only read data, or write data
#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_GRP(Data)          MPUi2cWrite(0x27, 1, 4, (uint8_t)Data)		//   External sensor data typically comes in as groups of two bytes. This bit is used to determine if the groups are from the slaves
																							//	register address 0 and 1, 2 and 3, etc.., or if the groups are address 1 and 2, 3 and 4, etc..
																							//   0 indicates slave register addresses 0 and 1 are grouped together (odd numbered register ends the group).
																							//   1 indicates slave register addresses 1 and 2 are grouped together (even numbered register ends the group).
																							//   This allows byte swapping of registers that are grouped starting at any address.
#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_LENG(Data)         MPUi2cWrite(0x27, 4, 3, (uint8_t)Data)		//   Number of bytes to be read from I2C slave 0

// I2C Slave 1
#define I2C_SLV1_ADDR_WRITE_I2C_SLV1_RNW(Data)          MPUi2cWrite(0x28, 1, 7, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV1_ADDR_WRITE_I2C_ID_1(Data)              MPUi2cWrite(0x28, 7, 6, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV1_REG_WRITE_I2C_SLV1_REG(Data)           MPUi2cWriteByte(0x29, (uint8_t)Data)			//   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_EN(Data)           MPUi2cWrite(0x2A, 1, 7, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_BYTE_SW(Data)      MPUi2cWrite(0x2A, 1, 6, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_REG_DIS(Data)      MPUi2cWrite(0x2A, 1, 5, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_GRP(Data)          MPUi2cWrite(0x2A, 1, 4, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_LENG(Data)         MPUi2cWrite(0x2A, 4, 3, (uint8_t)Data)		//   I2C STUFF

// I2C Slave 2
#define I2C_SLV2_ADDR_WRITE_I2C_SLV2_RNW(Data)          MPUi2cWrite(0x2B, 1, 7, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV2_ADDR_WRITE_I2C_ID_2(Data)              MPUi2cWrite(0x2B, 7, 6, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV2_REG_WRITE_I2C_SLV2_REG(Data)           MPUi2cWriteByte(0x2C, (uint8_t)Data)			//   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_EN(Data)           MPUi2cWrite(0x2D, 1, 6, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_BYTE_SW(Data)      MPUi2cWrite(0x2D, 1, 5, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_REG_DIS(Data)      MPUi2cWrite(0x2D, 1, 4, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_GRP(Data)          MPUi2cWrite(0x2D, 1, 3, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_LENG(Data)         MPUi2cWrite(0x2D, 4, 2, (uint8_t)Data)		//   I2C STUFF

// I2C Slave 3
#define I2C_SLV3_ADDR_WRITE_I2C_SLV3_RNW(Data)          MPUi2cWrite(0x2E, 1, 7, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV3_ADDR_WRITE_I2C_ID_3(Data)              MPUi2cWrite(0x2E, 7, 6, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV3_REG_WRITE_I2C_SLV3_REG(Data)           MPUi2cWriteByte(0x2F, (uint8_t)Data)			//   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_EN(Data)           MPUi2cWrite(0x30, 1, 7, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_BYTE_SW(Data)      MPUi2cWrite(0x30, 1, 6, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_REG_DIS(Data)      MPUi2cWrite(0x30, 1, 5, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_GRP(Data)          MPUi2cWrite(0x30, 1, 4, (uint8_t)Data)		//   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_LENG(Data)         MPUi2cWrite(0x30, 5, 3, (uint8_t)Data)		//   I2C STUFF

// I2C Slave 4
#define I2C_SLV4_ADDR_WRITE_I2C_SLV4_RNW(Data)          MPUi2cWrite(0x31, 1, 7, (uint8_t)Data)		//   1 – Transfer is a read, 0 – Transfer is a write
#define I2C_SLV4_ADDR_WRITE_I2C_ID_4(Data)              MPUi2cWrite(0x31, 7, 6, (uint8_t)Data)		//   Physical address of I2C slave 4
#define I2C_SLV4_REG_WRITE_I2C_SLV4_REG(Data)           MPUi2cWriteByte(0x32, (uint8_t)Data)			//   I2C slave 0 register address from where to begin data transfer
#define I2C_SLV4_DO_WRITE_I2C_SLV4_DO(Data)             MPUi2cWriteByte(0x33, (uint8_t)Data)			//   Data to be written to I2C Slave 4 if enabled.
#define I2C_SLV4_CTRL_WRITE_I2C_SLV4_EN(Data)           MPUi2cWrite(0x34, 1, 7, (uint8_t)Data)		//   1 – Enable data transfer with this slave at the sample rate. If read
																							//	 command, store data in I2C_SLV4_DI register, if write command, write data
																							//	 stored in I2C_SLV4_DO register. Bit is cleared when a single transfer is
																							//	 complete. Be sure to write I2C_SLV4_DO first
																							//	 0 – function is disabled for this slave
#define I2C_SLV4_CTRL_WRITE_SLV4_DONE_INT_EN(Data)      MPUi2cWrite(0x34, 1, 6, (uint8_t)Data)		//   1 – Enables the completion of the I2C slave 4 data transfer to cause an interrupt.
																							//	 0 – Completion of the I2C slave 4 data transfer will not cause an interrupt.
#define I2C_SLV4_CTRL_WRITE_I2C_SLV4_REG_DIS(Data)      MPUi2cWrite(0x34, 1, 5, (uint8_t)Data)		//   When set, the transaction does not write a register value, it will only read data, or write data
#define I2C_SLV4_CTRL_WRITE_I2C_MST_DLY(Data)           MPUi2cWrite(0x34, 5, 4, (uint8_t)Data)		//   When enabled via the I2C_MST_DELAY_CTRL, those slaves will only be enabled every (1+I2C_MST_DLY) samples (as determined by the SMPLRT_DIV and DLPF_CFG registers.
																							//   Delay mag data retrieval to once every other accel/gyro data sample
#define I2C_SLV4_DI_WRITE_I2C_SLV4_DI(Data)             MPUi2cWriteByte(0x35, (uint8_t)Data)			//   Data read from I2C Slave 4.






#define INT_PIN_CFG_WRITE_ACTL(Data)                    MPUi2cWrite(0x37, 1, 7, (uint8_t)Data)		//   1  The logic level for int16_t pin is active low. 0  The logic level for int16_t pin is active high.
#define INT_PIN_CFG_WRITE_OPEN(Data)                    MPUi2cWrite(0x37, 1, 6, (uint8_t)Data)		//   1  int16_t pin is configured as open drain. 0  int16_t pin is configured as push-pull.
#define INT_PIN_CFG_WRITE_LATCH_INT_EN(Data)            MPUi2cWrite(0x37, 1, 5, (uint8_t)Data)		//   1  int16_t pin level held until interrupt status is cleared. 0  int16_t pin indicates interrupt pulses is width 50us.
#define INT_PIN_CFG_WRITE_INT_ANYRD_2CLEAR(Data)        MPUi2cWrite(0x37, 1, 4, (uint8_t)Data)		//   1  Interrupt status is cleared if any read operation is performed. 0  Interrupt status is cleared only by reading INT_STATUS register
#define INT_PIN_CFG_WRITE_ACTL_FSYNC(Data)              MPUi2cWrite(0x37, 1, 3, (uint8_t)Data)		//   1  The logic level for the FSYNC pin as an interrupt is active low. 0  The logic level for the FSYNC pin as an interrupt is active high.
#define INT_PIN_CFG_WRITE_FSYNC_INT_MODE_EN(Data)       MPUi2cWrite(0x37, 1, 2, (uint8_t)Data)		//   1  This enables the FSYNC pin to be used as an interrupt.
#define INT_PIN_CFG_WRITE_BYPASS_EN(Data)               MPUi2cWrite(0x37, 1, 1, (uint8_t)Data)		//   When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into bypass mode when the i2c master interface is disabled.


#define INT_ENABLE_WRITE_FF_EN(Data)                    MPUi2cWrite(0x38, 1, 7, (uint8_t)Data)		//   1  Enable interrupt for
#define INT_ENABLE_WRITE_WOM_EN(Data)                   MPUi2cWrite(0x38, 1, 6, (uint8_t)Data)		//   1  Enable interrupt for wake on motion to propagate to interrupt pin. 0  function is disabled.
#define INT_ENABLE_WRITE_ZMOT_OFLOW_EN(Data)            MPUi2cWrite(0x38, 1, 5, (uint8_t)Data)		//   1  Enable interrupt for
#define INT_ENABLE_WRITE_FIFO_OFLOW_EN(Data)            MPUi2cWrite(0x38, 1, 4, (uint8_t)Data)		//   1  Enable interrupt for fifo overflow to propagate to interrupt pin. 0  function is disabled.
#define INT_ENABLE_WRITE_FSYNC_INT_EN(Data)             MPUi2cWrite(0x38, 1, 3, (uint8_t)Data)		//   1  Enable (I2C_MST_INT_BIT) Fsync interrupt to propagate to interrupt pin. 0  function is disabled.
#define INT_ENABLE_WRITE_RAW_PLL_RDY_INT_EN(Data)       MPUi2cWrite(0x38, 1, 2, (uint8_t)Data)		//   1  Enable
#define INT_ENABLE_WRITE_RAW_DMP_INT_EN(Data)           MPUi2cWrite(0x38, 1, 1, (uint8_t)Data)		//   1  Enable DMP interrupt
#define INT_ENABLE_WRITE_RAW_RDY_EN(Data)               MPUi2cWrite(0x38, 1, 0, (uint8_t)Data)		//   1  Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin.  0  function is disabled.



#define I2C_SLV0_DO_WRITE_I2C_SLV0_DO(Data)             MPUi2cWriteByte(0x63, (uint8_t)Data)			//   Data out when slave 0 is set to write
#define I2C_SLV1_DO_WRITE_I2C_SLV1_DO(Data)             MPUi2cWriteByte(0x64, (uint8_t)Data)			//   Data out when slave 1 is set to write
#define I2C_SLV2_DO_WRITE_I2C_SLV2_DO(Data)             MPUi2cWriteByte(0x65, (uint8_t)Data)			//   Data out when slave 2 is set to write
#define I2C_SLV3_DO_WRITE_I2C_SLV3_DO(Data)             MPUi2cWriteByte(0x66, (uint8_t)Data)			//   Data out when slave 3 is set to write

#define I2C_MST_DELAY_CTRL_WRITE_DELAY_ES_SHADOW(Data)  MPUi2cWrite(0x67, 1, 7, (uint8_t)Data)		//   Delays shadowing of external sensor data until     ALL data is received
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV4_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 4, (uint8_t)Data)		//   When enabled, slave 4 will only be accessed
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV3_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 3, (uint8_t)Data)		//   When enabled, slave 3 will only be accessed
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV2_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 2, (uint8_t)Data)		//   N When enabled, slave 2 will only be accessed
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV1_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 1, (uint8_t)Data)		//   When enabled, slave 1 will only be accessed
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV0_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 0, (uint8_t)Data)		//   When enabled, slave 0 will only be accessed

#define SIGNAL_PATH_RESET_WRITE_GYRO_RST(...)           MPUi2cWrite(0x68, 1, 2, (uint8_t)1)			//  Reset gyro digital signal path  Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers.
#define SIGNAL_PATH_RESET_WRITE_ACCEL_RST(...)          MPUi2cWrite(0x68, 1, 1, (uint8_t)1)			//  Reset accel digital signal path.  Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers.
#define SIGNAL_PATH_RESET_WRITE_TEMP_RST(...)           MPUi2cWrite(0x68, 1, 0, (uint8_t)1)			//  Reset temp digital signal path. Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers.


#define MOT_DETECT_CTRL_WRITE_ACCEL_INTEL_EN(Data)      MPUi2cWrite(0x69, 1, 7, (uint8_t)Data)		//   This bit enables the Wake-on-Motion detection logic
#define MOT_DETECT_CTRL_WRITE_ACCEL_INTEL_MODE(Data)    MPUi2cWrite(0x69, 1, 6, (uint8_t)Data)		//   1 = Compare the current sample with the previous sample. 0 = Not used.


#define SIGNAL_PATH_FULL_RESET_WRITE_RESET(...)			MPUi2cWrite(0x6A, 3, 2, (uint8_t)0b1111)		//  Reset DMP, FIFO, gyro, accel and temp signal paths
#define SIGNAL_PATH_RESET_WRITE_RESET(...)			    MPUi2cWrite(0x6A, 3, 2, (uint8_t)0b0111)		//  Reset gyro,accel, temp signal paths
#define USER_CTRL_WRITE_RESET_FIFO_DMP(...)			    MPUi2cWrite(0x6A, 3, 2, (uint8_t)0b1100)		//  Reset DMP and FIFO
#define USER_CTRL_WRITE_DMP_EN(Data)                    MPUi2cWrite(0x6A, 1, 7, (uint8_t)Data)		//   1  Enable DMP operation mode.
#define USER_CTRL_WRITE_FIFO_EN(Data)                   MPUi2cWrite(0x6A, 1, 6, (uint8_t)Data)		//   1  Enable FIFO operation mode. 0  Disable FIFO access from serial interface.  To disable FIFO writes by dma, use FIFO_EN register. To disable possible FIFO writes from DMP, disable the DMP.
#define USER_CTRL_WRITE_I2C_MST_EN(Data)                MPUi2cWrite(0x6A, 1, 5, (uint8_t)Data)		//   1  Enable the I2C Master (BIT_AUX_IF_EN)
#define USER_CTRL_WRITE_I2C_IF_DIS(Data)                MPUi2cWrite(0x6A, 1, 4, (uint8_t)Data)		//   1  Disable I2C Slave module and put the serial interface in SPI mode only.
#define USER_CTRL_WRITE_DMP_RST(...)                    MPUi2cWrite(0x6A, 1, 3, (uint8_t)1)			//   Reset DMP module. Reset is asynchronous. This bit auto clears after one clock cycle.
#define USER_CTRL_WRITE_FIFO_RST(...)                   MPUi2cWrite(0x6A, 1, 2, (uint8_t)1)			//   Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
#define USER_CTRL_WRITE_I2C_MST_RESET_BIT(...)          MPUi2cWrite(0x6A, 1, 1, (uint8_t)1)			//   Reset  gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.
#define USER_CTRL_WRITE_SIG_COND_RST(...)               MPUi2cWrite(0x6A, 1, 0, (uint8_t)1)			//   Reset  gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.


#define USER_CTRL_WRITE_RESET(...)                      MPUi2cWrite(0x6A, 4, 3, (uint8_t)0b1111)		//   1  Reset DMP,  gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.

																							// per MPU-6000/MPU-6050 Register Map and Descriptions page 41 The proper reset sequence is Reset Device wait 100ms Reset gyro,accel, temp signal paths wait 100ms
#define PWR_MGMT_1_WRITE_DEVICE_RESET(...)              MPUi2cWrite(0x6B, 1, 7, (uint8_t)1);delay(100);MPUi2cWrite(0x6A, 3, 2, (uint8_t)0b111);delay(100);  //   1  Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear.
#define PWR_MGMT_1_WRITE_SLEEP(Data)                    MPUi2cWrite(0x6B, 1, 6, (uint8_t)Data)		//   When set, the chip is set to sleep mode (After OTP loads, the PU_SLEEP_MODE bit will be written here)
#define PWR_MGMT_1_WRITE_CYCLE(Data)                    MPUi2cWrite(0x6B, 1, 5, (uint8_t)Data)		//   When set, and SLEEP and STANDBY are not set, the chip will cycle between sleep and taking a single sample at a rate determined by LP_ACCEL_ODR register
#define PWR_MGMT_1_WRITE_GYRO_STANDBY(Data)             MPUi2cWrite(0x6B, 1, 4, (uint8_t)Data)		//   When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled. This is a low power mode that allows quick enabling of the gyros.
#define PWR_MGMT_1_WRITE_PD_PTAT(Data)                  MPUi2cWrite(0x6B, 1, 3, (uint8_t)Data)		//   Power down internal PTAT voltage generator and PTAT ADC
#define PWR_MGMT_1_WRITE_CLKSEL(Data)                   MPUi2cWrite(0x6B, 3, 2, (uint8_t)Data)		//   Clock Source Select

#define PWR_MGMT_1_WRITE_CLKSEL_Internal_20MHz_osc(...) MPUi2cWrite(0x6B, 3, 2, (uint8_t)0)			//   Clock Source Select
#define PWR_MGMT_1_WRITE_CLKSEL_PLL_X_gyro(...)         MPUi2cWrite(0x6B, 3, 2, (uint8_t)1)			//   Clock Source Select

#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL_1_25Hz(...)       MPUi2cWrite(0x6C, 2, 7, (uint8_t)0)			//   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL_5Hz(...)          MPUi2cWrite(0x6C, 2, 7, (uint8_t)1)			//   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL_20Hz(...)         MPUi2cWrite(0x6C, 2, 7, (uint8_t)2)			//   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL_40Hz(...)         MPUi2cWrite(0x6C, 2, 7, (uint8_t)3)			//   1  X accelerometer is disabled 0  X accelerometer is on


#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL(Data)             MPUi2cWrite(0x6C, 2, 7, (uint8_t)Data)		//   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_DIS_XA(Data)                   MPUi2cWrite(0x6C, 1, 6, (uint8_t)Data)		//   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_DIS_YA(Data)                   MPUi2cWrite(0x6C, 1, 5, (uint8_t)Data)		//   1  Y accelerometer is disabled 0  Y accelerometer is on
#define PWR_MGMT_2_WRITE_DIS_ZA(Data)                   MPUi2cWrite(0x6C, 1, 4, (uint8_t)Data)		//   1  Z accelerometer is disabled 0  Z accelerometer is on
#define PWR_MGMT_2_WRITE_DIS_XG(Data)                   MPUi2cWrite(0x6C, 1, 3, (uint8_t)Data)		//   1  X gyro is disabled 0  X gyro is on
#define PWR_MGMT_2_WRITE_DIS_YG(Data)                   MPUi2cWrite(0x6C, 1, 2, (uint8_t)Data)		//   1  Y gyro is disabled 0  Y gyro is on
#define PWR_MGMT_2_WRITE_DIS_ZG(Data)                   MPUi2cWrite(0x6C, 1, 1, (uint8_t)Data)		//   1  Z gyro is disabled 0  Z gyro is on

#define BANK_SEL_WRITE(Data)                            MPUi2cWriteInt(0x6D,  (uint16_t)Data)			//   DMP Bank Select for Loading Image
#define DMP_MEM_START_ADDR_WRITE(Data)                  MPUi2cWriteInt(0x6E,  (uint16_t)Data)			//   Not Used
#define DMP_MEM_WRITE(Length,Data)                      MPUi2cWriteBytes(0x6F,Length, (uint8_t *)Data) // DMP Image Loading Location
#define PRGM_START_WRITE(Data)                          MPUi2cWriteInt(0x70,  (uint16_t)Data)			// Set program start address.




#define FIFO_COUNTH_WRITE_FIFO_CNT(Data)                MPUi2cWriteBytes(0x72, 2, (uint8_t *)Data)		//   High Bits, count indicates the number of written bytes in the FIFO.
#define FIFO_WRITE(Data,PacketLength)                   MPUi2cWriteBytes(0x74, PacketLength, (uint8_t *)Data)  //   Read/Write command provides Read or Write operation for the FIFO.
//6500 and 9250
// Warning Offsets have a extra byte between them 
#define X_OFFSET_WRITE_0x77_X_OFFS(Shift, Data)         MPUi2cWriteInt(0x77 + (Shift * 3),  (uint16_t)Data)			//   accelerometer offset cancellation
#define XA_OFFSET_H_WRITE_0x77_XA_OFFS(Data)            MPUi2cWriteInt(0x77,  (uint16_t)Data)			//   X accelerometer offset cancellation
#define YA_OFFSET_H_WRITE_0x77_YA_OFFS(Data)            MPUi2cWriteInt(0x7A,  (uint16_t)Data)			//   Y accelerometer offset cancellation
#define ZA_OFFSET_H_WRITE_0x77_ZA_OFFS(Data)            MPUi2cWriteInt(0x7D,  (uint16_t)Data)			//   Z accelerometer offset cancellation



// Magnetometer AK#### Macros:




// Address Definitions
#define AKM_S0_ADDR				0x25
#define AKM_S0_REG				0x26
#define AKM_S0_CTRL				0x27
#define AKM_S1_ADDR				0x28
#define AKM_S1_REG				0x29
#define AKM_S1_CTR				0x2A
#define AKM_S4_CTRL				0x34
#define AKM_S0_DO				0x63
#define AKM_S1_DO				0x64
#define AKM_I2C_DELAY_CTRL		0x67

// setup_compass Creates:
#define EXT_I2C_ST1_READ_BYTE(Data)						MPUi2cReadByte(0x49, (uint8_t *)Data)
#define EXT_I2C_ST1_READ_DATA_READY(Data)				MPUi2cRead(0x49, 1,0, (uint8_t *)Data)
#define EXT_I2C_ST1_READ_OVERRUN(Data)					MPUi2cRead(0x49, 1,1, (uint8_t *)Data)
#define EXT_I2C_READ_RAW_COMPASS(Data)					MPUi2cReadInts(0x4A, 3, (uint16_t *)Data)
#define EXT_I2C_ST1_READ_BYTE(Data)						MPUi2cReadByte(0x4D, (uint8_t *)Data)
#define EXT_I2C_ST1_READ_DATA_ERROR(Data)				MPUi2cRead(0x4D, 1,2 (uint8_t *)Data)
#define EXT_I2C_ST1_READ_SENSOR_OVERFLOW(Data)			MPUi2cRead(0x4D, 1,3 (uint8_t *)Data)
#define EXT_I2C_ST1_READ_BITM(Data)						MPUi2cRead(0x4D, 1,4 (uint8_t *)Data)
#define AKM_YG_OFFS_TC			0x01
#define AKM_COMPASS_FSR
#define AKM_COMPASS_SAMPLE_RATE
#define AKM_COMPASS_ADDRESS
#define AKM_MAG_SENSE_ADJ short mag_sens_adj[3];

//AK8975_SECONDARY
#define SUPPORTS_AK8975_HIGH_SENS   (0x00)
#define AK8975_FSR                  (9830)
//AK8963_SECONDARY
#define SUPPORTS_AK8963_HIGH_SENS   (0x10)
#define AK8963_FSR                  (4915)

#define SUPPORTS_AK89xx_HIGH_SENS   SUPPORTS_AK8975_HIGH_SENS
#define AK89xx_FSR                  AK8975_FSR

//Read Only
#define AKM_WHOAMI_READ(compass_addr, Data)					MPUi2cReadByte(compass_addr,0x00,(uint8_t *)Data)	//Device ID of AKM. It is described in one byte and fixed value.  0x48: fixed
#define AKM_INFO_READ(compass_addr, Data)					MPUi2cReadByte(compass_addr,0x01,(uint8_t *)Data)	//INFO[7:0]: Device information for AKM



#define AKM_ST1_READ_BYTE(compass_addr, Data)				MPUi2cReadByte(compass_addr,0x02,(uint8_t *)Data)	//BIT 0 and Bit 1
#define AKM_ST1_READ_DATA_READY(compass_addr, Data)			MPUi2cRead(compass_addr,0x02,1,0,(uint8_t *)Data)	//BIT 0 DRDY bit turns to “1” when data is ready in single measurement mode or self-test mode. It returns to “0” when any one of ST2 register or measurement data register (HXL to HZH) is read.
#define AKM_ST1_READ_DATA_OVERRUN(compass_addr, Data)		MPUi2cRead(compass_addr,0x02,1,1,(uint8_t *)Data)	//BIT 1 mpu9250 ONLY DOR bit turns to “1” when data has been skipped in continuous measurement mode or external trigger measurement mode. It returns to “0” when any one of ST2 register or measurement data register (HXL~HZH) is read.

#define AKM_DATA_READ_RAW_COMPASS_DATA(compass_addr, Data)	MPUi2cReadBytes(compass_addr,0x03,6,(uint8_t *)Data) // Read the six raw data and ST2 registers sequentially into data array

#define SwapBytes(Data)										{uint8_t L = (uint8_t)((uint16_t)Data>>8); Data = (int16_t)((uint16_t)Data<<8) | (uint16_t)L;}  // Swaps the Byte order of the integer


#define AKM_DATA_READ_RAW_COMPASS_SWAP(compass_addr, Data)	MPUi2cReadInts(compass_addr,0x03,3,(uint16_t *)Data);SwapBytes(Data[0]);SwapBytes(Data[1]);SwapBytes(Data[2]) //Read 8Bytes of Data
#define AKM_DATA_READ_RAW_COMPASS(compass_addr, Data)		MPUi2cReadInts(compass_addr,0x03,3,(uint16_t *)Data) //Read 8Bytes of Data
#define AKM_HX_READ_HX(compass_addr, Data)					MPUi2cReadInt(compass_addr,0x03,(uint16_t *)Data);SwapBytes(Data[0])
#define AKM_HY_READ_HY(compass_addr, Data)					MPUi2cReadInt(compass_addr,0x05,(uint16_t *)Data);SwapBytes(Data[0])
#define AKM_HZ_READ_HZ(compass_addr, Data)					MPUi2cReadInt(compass_addr,0x07,(uint16_t *)Data);SwapBytes(Data[0])

#define AKM_ST2_READ_BYTE(compass_addr, Data)				MPUi2cReadByte(compass_addr,0x09,(uint8_t *)Data)	// BIT 2, BIT 3, BIT 4
#define AKM_ST2_READ_SENSOR_OVERFLOW(compass_addr, Data)	MPUi2cRead(compass_addr,0x09,1,3,(uint8_t *)Data)	// BIT 3 HOFL In single measurement mode and self-test mode, magnetic sensor may overflow even though measurement data register is not saturated. In this case, measurement data is not correct and HOFL bit turns to “1”. When next measurement stars, it returns to “0”.
#define AKM_ST2_READ_DATA_ERROR(compass_addr, Data)			MPUi2cRead(compass_addr,0x09,1,2,(uint8_t *)Data)	// BIT 2 DERR  When data reading is started out of data readable period, the read data are not correct. In this case, data read error occurs and DERR bit turns to “1”. When ST2 register is read, it returns to “0”.
#define AKM_ST2_READ_BITM(compass_addr, Data)				MPUi2cRead(compass_addr,0x09,1,4,(uint8_t *)Data)	// BIT 4 Output bit setting Mirror of CNTL1 BIT 4  0 = 14-BIT 1 = 16-BIT

//AKM_CNTL	Options		(0x0A)
// HIGH_SENSE 0 = 14-bit output *** If available MPU9250 *** 1 = 16-bit output
#define HS_14bit 0
#define HS_16bit 1
#define AKM_POWER_DOWN(HIGHS_SENS)											(0b0000|(HIGHS_SENS & 1)<<4)
#define AKM_CNTL_WRITE_POWER_DOWN(compass_addr, HIGHS_SENS)					MPUi2cWriteByte(compass_addr,0x0A,(uint8_t)(0b0000|((HIGHS_SENS & 1)<<4))) // Power to almost all internal circuits is turned off. All registers are accessible in power-down mode. However, fuse ROM data cannot be read correctly. Data stored in read/write registers are remained. They can be reset by soft reset.
#define AKM_SINGLE_MEAS_MODE(HIGHS_SENS)									(0b0001|(HIGHS_SENS & 1)<<4)
#define AKM_CNTL_WRITE_SINGLE_MEAS_MODE(compass_addr, HIGHS_SENS)			MPUi2cWriteByte(compass_addr,0x0A,(uint8_t)(0b0001|((HIGHS_SENS & 1)<<4))) // Gets Data and then sets DATA_READY = True, after Get Data > DATA_READY = false  afterwords AKM set to POWER_DOWN
#define AKM_SELFTEST_MODE(HIGHS_SENS)										(0b1000|(HIGHS_SENS & 1)<<4)
#define AKM_CNTL_WRITE_SELFTEST_MODE(compass_addr, HIGHS_SENS)				MPUi2cWriteByte(compass_addr,0x0A,(uint8_t)(0b1000|((HIGHS_SENS & 1)<<4))) // Self-test mode is used to check if the sensor is working normally
#define AKM_FUSE_ROM_ACCESS(HIGHS_SENS)										(0b1111|(HIGHS_SENS & 1)<<4)
#define AKM_CNTL_WRITE_FUSE_ROM_ACCESS(compass_addr, HIGHS_SENS)			MPUi2cWriteByte(compass_addr,0x0A,(uint8_t)(0b1111|((HIGHS_SENS & 1)<<4))) // Fuse ROM access mode is used to read Fuse ROM data. Sensitivity adjustment data for each axis is stored in fuse ROM.

//MPU9250 ONLY
#define CONT_MEAS_MODE1(HIGHS_SENS)											(0b0010|(HIGHS_SENS & 1)<<4)
#define AKM_CNTL_WRITE_CONT_MEAS_MODE1(compass_addr, HIGHS_SENS)			MPUi2cWriteByte(compass_addr,0x0A,(uint8_t)(0b0010|((HIGHS_SENS & 1)<<4))) // 8Hz When sensor measurement and signal processing is finished, measurement data is stored and DATA_READY = True
#define CONT_MEAS_MODE2(HIGHS_SENS)											(0b0110|(HIGHS_SENS & 1)<<4)
#define AKM_CNTL_WRITE_CONT_MEAS_MODE2(compass_addr, HIGHS_SENS)			MPUi2cWriteByte(compass_addr,0x0A,(uint8_t)(0b0110|((HIGHS_SENS & 1)<<4))) // 100Hz When sensor measurement and signal processing is finished, measurement data is stored and DATA_READY = True
#define EXT_TRIG_MEAS_MODE(HIGHS_SENS)										(0b0100|(HIGHS_SENS & 1)<<4)
#define AKM_CNTL_WRITE_EXT_TRIG_MEAS_MODE(compass_addr, HIGHS_SENS)			MPUi2cWriteByte(compass_addr,0x0A,(uint8_t)(0b0100|((HIGHS_SENS & 1)<<4))) // waits for trigger input. When a pulse is input from TRG pin, sensor measurement is started on the rising edge of TRG pin Gets Data and then sets DATA_READY = True

#define AKM_CNTL_READ_ALL(compass_addr,Data)				MPUi2cReadByte(compass_addr,0x0A,(uint8_t *)Data)

#define AKM_SOFT_RESET(compass_addr)						MPUi2cWriteByte(compass_addr,0x0B,(uint8_t)1)


#define AKM_ASTC_WRITE_GEN_MAG_FIELD(compass_addr)			MPUi2cWrite(compass_addr,0x0C,1,6,(uint8_t)1) //Generate magnetic field for self-test
#define AKM_ASTC_WRITE_NORMAL(compass_addr)					MPUi2cWrite(compass_addr,0x0C,1,6,(uint8_t)0) //Normal

//#define AKM_ASTC_WRITE_All(compass_addr, Data)			MPUi2cWrite(compass_addr,0x0F,1,0,(uint8_t)1) // Once I2CDIS is turned to “1” and I2C bus interface is disabled, re-setting I2CDIST to “0” is prohibited. To enable I2C bus interface, reset AK8975C by turning VDD to OFF (0V) once.

//ASAX, ASAY, ASAZ: Sensitivity Adjustment values
#define AKM_ASAXYZ_WRITE_SENS_ADJ_XYZ(compass_addr, Data)	MPUi2cWriteBytes(compass_addr,0x10,3,(uint8_t *)Data)
#define AKM_ASAX_WRITE_SENS_ADJ_X(compass_addr, Data)		MPUi2cWriteByte(compass_addr,0x10,(uint8_t)Data)
#define AKM_ASAY_WRITE_SENS_ADJ_Y(compass_addr, Data)		MPUi2cWriteByte(compass_addr,0x11,(uint8_t)Data)
#define AKM_ASAZ_WRITE_SENS_ADJ_Z(compass_addr, Data)		MPUi2cWriteByte(compass_addr,0x12,(uint8_t)Data)

#define AKM_ASAXYZ_READ_SENS_ADJ_XYZ(compass_addr, Data)	MPUi2cReadBytes(compass_addr,0x10,3,(uint8_t *)Data)
#define AKM_ASAX_READ_SENS_ADJ_X(compass_addr, Data)		MPUi2cReadByte(compass_addr,0x10,(uint8_t *)Data)
#define AKM_ASAY_READ_SENS_ADJ_Y(compass_addr, Data)		MPUi2cReadByte(compass_addr,0x11,(uint8_t *)Data)
#define AKM_ASAZ_READ_SENS_ADJ_Z(compass_addr, Data)		MPUi2cReadByte(compass_addr,0x12,(uint8_t *)Data)

#define AKM_WHOAMI_VALUE		(0x48)
#define AKM_REG_ST1				(0x02) //ST1 Start reading this byte
#define AKM_XOUT_L				(0x03) // Starting data register
#define MAX_COMPASS_SAMPLE_RATE (100)
#define AKM_MAX_SAMPLE_RATE		(100)

#define AKM_REG_CNTL			(0x0A)
#define AKM_REG_ASTC			(0x0C)
#define AKM_REG_ASAX			(0x10)
#define AKM_REG_ASAY			(0x11)
#define AKM_REG_ASAZ			(0x12)

#define AKM_POWER_DOWN          (0x00)
#define AKM_SINGLE_MEASUREMENT  (0x01)
#define AKM_FUSE_ROM_ACCESS     (0x0F)
#define AKM_MODE_SELF_TEST      (0x08)




#endif

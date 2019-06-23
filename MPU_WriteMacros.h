#ifndef MPU_WriteMacros_h
#define MPU_WriteMacros_h
// Names are taken straight from the register map for the gyroscope and accelerometer Specifically MPU9250 which should match the MPU6050
//        Register Name | ReadWrite | Bits Name    (regAddr size length bit Data Read/Write)
//9250 Self Test Gyro
//#define SELF_TEST_X_GYRO_WRITE_xg_st_data(Data)       MPUi2cWriteBytes(0x00, 1, Data)  //   self test output generated during manufacturing tests
//#define SELF_TEST_Y_GYRO_WRITE_yg_st_data(Data)       MPUi2cWriteBytes(0x01, 1, Data)  //   self test output generated during manufacturing tests
//#define SELF_TEST_Z_GYRO_WRITE_zg_st_data(Data)       MPUi2cWriteBytes(0x02, 1, Data)  //   self test output generated during manufacturing tests
//6050 Accellerator Offsets
#define A_OFFSET_H_WRITE_A_OFFS(Data)                   MPUi2cWriteInts(0x06, 6, Data)  //   X accelerometer offset cancellation
#define XA_OFFSET_H_WRITE_XA_OFFS(Data)                 MPUi2cWriteInt(0x06, Data)  //   X accelerometer offset cancellation
#define YA_OFFSET_H_WRITE_YA_OFFS(Data)                 MPUi2cWriteInt(0x08, Data)  //   Y accelerometer offset cancellation
#define ZA_OFFSET_H_WRITE_ZA_OFFS(Data)                 MPUi2cWriteInt(0x0A,  Data)  //   Z accelerometer offset cancellation
//9250 Self Test Accel
//#define SELF_TEST_X_ACCEL_WRITE_XA_ST_DATA(Data)      MPUi2cWriteBytes(0x0D, 1, Data)  //   self test output generated during manufacturing tests
//#define SELF_TEST_Y_ACCEL_WRITE_YA_ST_DATA(Data)      MPUi2cWriteBytes(0x0E, 1, Data)  //   self test output generated during manufacturing tests
//#define SELF_TEST_Z_ACCEL_WRITE_ZA_ST_DATA(Data)      MPUi2cWriteBytes(0x0F, 1, Data)  //   self test output generated during manufacturing tests
//6050 Self Test Gyro and Accel
#define SELF_TEST_X_WRITE_XA_TEST(Data)                 MPUi2cWrite(0x0D,3, 7, Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_X_WRITE_XG_TEST(Data)                 MPUi2cWrite(0x0D,5, 4, Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_Y_WRITE_YA_TEST(Data)                 MPUi2cWrite(0x0E, 3, 7, Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_Y_WRITE_YG_TEST(Data)                 MPUi2cWrite(0x0E, 5, 4, Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_Z_WRITE_ZA_TEST(Data)                 MPUi2cWrite(0x0F, 3, 7, Data)  //   self test output generated during manufacturing tests
#define SELF_TEST_Z_WRITE_ZG_TEST(Data)                 MPUi2cWrite(0x0F, 5, 4, Data)  //   self test output generated during manufacturing tests
//Both
#define XG_OFFSET_H_WRITE_OFFS_USR(Data)                MPUi2cWriteInts(0x13, 6, Data)  //   Remove DC bias from the gyro sensor Step 0.0305 dps
#define XG_OFFSET_H_WRITE_X_OFFS_USR(Data)              MPUi2cWriteInt(0x13,  Data)  //   Remove DC bias from the gyro sensor Step 0.0305 dps
#define YG_OFFSET_H_WRITE_Y_OFFS_USR(Data)              MPUi2cWriteInt(0x15,  Data)  //   Remove DC bias from the gyro sensor Step 0.0305 dps
#define ZG_OFFSET_H_WRITE_Z_OFFS_USR(Data)              MPUi2cWriteInt(0x17,  Data)  //   Remove DC bias from the gyro sensor Step 0.0305 dps

#define SMPLRT_DIV_WRITE_SMPLRT_DIV(Data)               MPUi2cWriteBytes(0x19, 1, Data)  //   Divides the internal sample rate  controls sensor data output rate, FIFO sample rate.   SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)

#define CONFIG_WRITE_FIFO_MODE(Data)                    MPUi2cWrite(0x1A, 1, 6, Data)  //   When set to 1, when the fifo is full, additional writes will not be written to fifo.  When set to 0, when the fifo is full, additional writes will be written to the fifo, replacing the oldest data.
#define CONFIG_WRITE_EXT_SYNC_SET(Data)                 MPUi2cWrite(0x1A, 3, 5, Data)  //   Enables the FSYNC pin data to be sampled.
#define CONFIG_WRITE_DLPF_CFG(Data)                     MPUi2cWrite(0x1A, 3, 2, Data)  //   DLPF Configuration

#define CONFIG_WRITE_DLPF_256HZ_NOLPF2(...)             MPUi2cWrite(0x1A, 3, 2, 0)  //   DLPF Configuration
#define CONFIG_WRITE_DLPF_188HZ(...)                    MPUi2cWrite(0x1A, 3, 2, 1)  //   DLPF Configuration
#define CONFIG_WRITE_DLPF_98HZ(...)                     MPUi2cWrite(0x1A, 3, 2, 2)  //   DLPF Configuration
#define CONFIG_WRITE_DLPF_42HZ(...)                     MPUi2cWrite(0x1A, 3, 2, 3)  //   DLPF Configuration
#define CONFIG_WRITE_DLPF_20HZ(...)                     MPUi2cWrite(0x1A, 3, 2, 4)  //   DLPF Configuration
#define CONFIG_WRITE_DLPF_10HZ(...)                     MPUi2cWrite(0x1A, 3, 2, 5)  //   DLPF Configuration
#define CONFIG_WRITE_DLPF_5HZ(...)                      MPUi2cWrite(0x1A, 3, 2, 6)  //   DLPF Configuration
#define CONFIG_WRITE_DLPF_2100HZ_NOLPF(...)             MPUi2cWrite(0x1A, 3, 2, 7)  //   DLPF Configuration

#define GYRO_CONFIG_WRITE_XGYRO_Ct_en(Data)             MPUi2cWrite(0x1B, 1, 7, Data)  //   X Gyro self-test
#define GYRO_CONFIG_WRITE_YGYRO_Ct_en(Data)             MPUi2cWrite(0x1B, 1, 6, Data)  //   Y Gyro self-test
#define GYRO_CONFIG_WRITE_ZGYRO_Ct_en(Data)             MPUi2cWrite(0x1B, 1, 5, Data)  //   Z Gyro self-test
#define GYRO_CONFIG_WRITE_GYRO_FS_SEL(Data)             MPUi2cWrite(0x1B, 2, 4, Data)  //   Gyro Full Scale Select: 00B = +250 01B = +500 10B = +1000 11B = 2000

#define GYRO_CONFIG_WRITE_GYRO_FS_SEL_250(...)          MPUi2cWrite(0x1B, 3, 4, 0)  //   Gyro Full Scale Select: +250 Deg/sec
#define GYRO_CONFIG_WRITE_GYRO_FS_SEL_500(...)          MPUi2cWrite(0x1B, 3, 4, 1)  //   Gyro Full Scale Select: +500 Deg/sec
#define GYRO_CONFIG_WRITE_GYRO_FS_SEL_1000(...)         MPUi2cWrite(0x1B, 3, 4, 2)  //   Gyro Full Scale Select: +1000 Deg/sec
#define GYRO_CONFIG_WRITE_GYRO_FS_SEL_2000(...)         MPUi2cWrite(0x1B, 3, 4, 3)  //   Gyro Full Scale Select: +2000 Deg/sec

#define GYRO_CONFIG_WRITE_FCHOICE_B(Data)               MPUi2cWrite(0x1B, 2, 2, Data)  //   Used to bypass DLPF
#define ACCEL_CONFIG_WRITE_ax_st_en(Data)               MPUi2cWrite(0x1C, 1, 7, Data)  //   X Accel self-test
#define ACCEL_CONFIG_WRITE_ay_st_en(Data)               MPUi2cWrite(0x1C, 1, 6, Data)  //   Y Accel self-test
#define ACCEL_CONFIG_WRITE_az_st_en(Data)               MPUi2cWrite(0x1C, 1, 5, Data)  //   Z Accel self-test
#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL(Data)           MPUi2cWrite(0x1C, 2, 3, Data)  //   Accel Full Scale Select: 2g (00), 4g (01), 8g (10), 16g (11)

#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL_2g(...)         MPUi2cWrite(0x1C, 2, 3, 0)  //   Accel Full Scale Select: 2g
#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL_4g(...)         MPUi2cWrite(0x1C, 2, 3, 1)  //   Accel Full Scale Select: 4g
#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL_8g(...)         MPUi2cWrite(0x1C, 2, 3, 2)  //   Accel Full Scale Select: 8g
#define ACCEL_CONFIG_WRITE_ACCEL_FS_SEL_16g(...)        MPUi2cWrite(0x1C, 2, 3, 3)  //   Accel Full Scale Select: 16g


#define ACCEL_CONFIG_2_WRITE_ACCEL_FCHOICE_B(Data)      MPUi2cWrite(0x1D, 2, 2, Data)  //   Used to bypass DLPF
#define ACCEL_CONFIG_2_WRITE_A_DLPF_CFG(Data)           MPUi2cWrite(0x1D, 2, 1, Data)  //   Accelerometer low pass filter setting
#define LP_ACCEL_ODR_WRITE_Lposc_clksel(Data)           MPUi2cWrite(0x1E, 4, 3, Data)  //   Frequency of waking up the chip to take a sample of accel data
#define WOM_THR_WRITE_WOM_Threshold(Data)               MPUi2cWriteBytes(0x1F, 1, Data)  //   Threshold Value for the Wake on Motion Interrupt

#define FIFO_EN_WRITE_FIFO_EN(Data)                     MPUi2cWriteBytes(0x23, 1, Data) //   Adds Temp data to FIFO
#define FIFO_EN_WRITE_TEMP_FIFO_EN(Data)                MPUi2cWrite(0x23, 1, 7, Data)  //   Adds Temp data to FIFO
#define FIFO_EN_WRITE_GYRO_XOUT(Data)                   MPUi2cWrite(0x23, 1, 6, Data)  //   Adds GYRO X  data to FIFO
#define FIFO_EN_WRITE_GYRO_YOUT(Data)                   MPUi2cWrite(0x23, 1, 5, Data)  //   Adds GYRO Y data to FIFO
#define FIFO_EN_WRITE_GYRO_ZOUT(Data)                   MPUi2cWrite(0x23, 1, 4, Data)  //   Adds GYRO Z data to FIFO
#define FIFO_EN_WRITE_ACCEL(Data)                       MPUi2cWrite(0x23, 1, 3, Data)  //   Adds ACCEL data to FIFO
#define FIFO_EN_WRITE_SLV2(Data)                        MPUi2cWrite(0x23, 1, 2, Data)  //   Adds SLV2 data to FIFO
#define FIFO_EN_WRITE_SLV1(Data)                        MPUi2cWrite(0x23, 1, 1, Data)  //   Adds SLV1 data to FIFO
#define FIFO_EN_WRITE_SLV0(Data)                        MPUi2cWrite(0x23, 1, 0, Data)  //   Adds SLV0 data to FIFO

#define I2C_MST_CTRL_WRITE_MULT_MST_EN(Data)            MPUi2cWrite(0x24, 1, 7, Data)  //   I2C STUFF
#define I2C_MST_CTRL_WRITE_WAIT_FOR_ES(Data)            MPUi2cWrite(0x24, 1, 6, Data)  //   I2C STUFF
#define I2C_MST_CTRL_WRITE_SLV_3_FIFO_EN(Data)          MPUi2cWrite(0x24, 1, 5, Data)  //   I2C STUFF
#define I2C_MST_CTRL_WRITE_I2C_MST_P_NSR(Data)          MPUi2cWrite(0x24, 1, 4, Data)  //   I2C STUFF
#define I2C_MST_CTRL_WRITE_I2C_MST_CLK(Data)            MPUi2cWrite(0x24, 4, 3, Data)  //   I2C STUFF
#define I2C_SLV0_ADDR_WRITE_I2C_SLV0_RNW(Data)          MPUi2cWrite(0x25, 1, 7, Data)  //   I2C STUFF
#define I2C_SLV0_ADDR_WRITE_I2C_ID_0(Data)              MPUi2cWrite(0x25, 7, 6, Data)  //   I2C STUFF
#define I2C_SLV0_REG_WRITE_I2C_SLV0_REG(Data)           MPUi2cWriteBytes(0x26, 1, Data)  //   I2C STUFF
#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_EN(Data)           MPUi2cWrite(0x27, 1, 7, Data)  //   I2C STUFF
#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_BYTE_SW(Data)      MPUi2cWrite(0x27, 1, 6, Data)  //   I2C STUFF
#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_REG_DIS(Data)      MPUi2cWrite(0x27, 1, 5, Data)  //   I2C STUFF
#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_GRP(Data)          MPUi2cWrite(0x27, 1, 4, Data)  //   I2C STUFF
#define I2C_SLV0_CTRL_WRITE_I2C_SLV0_LENG(Data)         MPUi2cWrite(0x27, 4, 3, Data)  //   I2C STUFF
#define I2C_SLV1_ADDR_WRITE_I2C_SLV1_RNW(Data)          MPUi2cWrite(0x28, 1, 7, Data)  //   I2C STUFF
#define I2C_SLV1_ADDR_WRITE_I2C_ID_1(Data)              MPUi2cWrite(0x28, 7, 6, Data)  //   I2C STUFF
#define I2C_SLV1_REG_WRITE_I2C_SLV1_REG(Data)           MPUi2cWriteBytes(0x29, 1, Data)  //   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_EN(Data)           MPUi2cWrite(0x2A, 1, 7, Data)  //   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_BYTE_SW(Data)      MPUi2cWrite(0x2A, 1, 6, Data)  //   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_REG_DIS(Data)      MPUi2cWrite(0x2A, 1, 5, Data)  //   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_GRP(Data)          MPUi2cWrite(0x2A, 1, 4, Data)  //   I2C STUFF
#define I2C_SLV1_CTRL_WRITE_I2C_SLV1_LENG(Data)         MPUi2cWrite(0x2A, 4, 3, Data)  //   I2C STUFF
#define I2C_SLV2_ADDR_WRITE_I2C_SLV2_RNW(Data)          MPUi2cWrite(0x2B, 1, 7, Data)  //   I2C STUFF
#define I2C_SLV2_ADDR_WRITE_I2C_ID_2(Data)              MPUi2cWrite(0x2B, 7, 6, Data)  //   I2C STUFF
#define I2C_SLV2_REG_WRITE_I2C_SLV2_REG(Data)           MPUi2cWriteBytes(0x2C, 1, Data)  //   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_EN(Data)           MPUi2cWrite(0x2D, 1, 6, Data)  //   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_BYTE_SW(Data)      MPUi2cWrite(0x2D, 1, 5, Data)  //   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_REG_DIS(Data)      MPUi2cWrite(0x2D, 1, 4, Data)  //   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_GRP(Data)          MPUi2cWrite(0x2D, 1, 3, Data)  //   I2C STUFF
#define I2C_SLV2_CTRL_WRITE_I2C_SLV2_LENG(Data)         MPUi2cWrite(0x2D, 4, 2, Data)  //   I2C STUFF
#define I2C_SLV3_ADDR_WRITE_I2C_SLV3_RNW(Data)          MPUi2cWrite(0x2E, 1, 7, Data)  //   I2C STUFF
#define I2C_SLV3_ADDR_WRITE_I2C_ID_3(Data)              MPUi2cWrite(0x2E, 7, 6, Data)  //   I2C STUFF
#define I2C_SLV3_REG_WRITE_I2C_SLV3_REG(Data)           MPUi2cWriteBytes(0x2F, 1, Data)  //   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_EN(Data)           MPUi2cWrite(0x30, 1, 7, Data)  //   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_BYTE_SW(Data)      MPUi2cWrite(0x30, 1, 6, Data)  //   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_REG_DIS(Data)      MPUi2cWrite(0x30, 1, 5, Data)  //   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_GRP(Data)          MPUi2cWrite(0x30, 1, 4, Data)  //   I2C STUFF
#define I2C_SLV3_CTRL_WRITE_I2C_SLV3_LENG(Data)         MPUi2cWrite(0x30, 5, 3, Data)  //   I2C STUFF
#define I2C_SLV4_ADDR_WRITE_I2C_SLV4_RNW(Data)          MPUi2cWrite(0x31, 1, 7, Data)  //   I2C STUFF
#define I2C_SLV4_ADDR_WRITE_I2C_ID_4(Data)              MPUi2cWrite(0x31, 7, 6, Data)  //   I2C STUFF
#define I2C_SLV4_REG_WRITE_I2C_SLV4_REG(Data)           MPUi2cWriteBytes(0x32, 1, Data)  //   I2C STUFF
#define I2C_SLV4_DO_WRITE_I2C_SLV4_DO(Data)             MPUi2cWriteBytes(0x33, 1, Data)  //   I2C STUFF
#define I2C_SLV4_CTRL_WRITE_I2C_SLV4_EN(Data)           MPUi2cWrite(0x34, 1, 7, Data)  //   I2C STUFF
#define I2C_SLV4_CTRL_WRITE_SLV4_DONE_INT_EN(Data)      MPUi2cWrite(0x34, 1, 6, Data)  //   I2C STUFF
#define I2C_SLV4_CTRL_WRITE_I2C_SLV4_REG_DIS(Data)      MPUi2cWrite(0x34, 1, 5, Data)  //   I2C STUFF
#define I2C_SLV4_CTRL_WRITE_I2C_MST_DLY(Data)           MPUi2cWrite(0x34, 5, 4, Data)  //   I2C STUFF

#define INT_PIN_CFG_WRITE_ACTL(Data)                    MPUi2cWrite(0x37, 1, 7, Data)  //   1  The logic level for int16_t pin is active low. 0  The logic level for int16_t pin is active high.
#define INT_PIN_CFG_WRITE_OPEN(Data)                    MPUi2cWrite(0x37, 1, 6, Data)  //   1  int16_t pin is configured as open drain. 0  int16_t pin is configured as push-pull.
#define INT_PIN_CFG_WRITE_LATCH_INT_EN(Data)            MPUi2cWrite(0x37, 1, 5, Data)  //   1  int16_t pin level held until interrupt status is cleared. 0  int16_t pin indicates interrupt pulses is width 50us.
#define INT_PIN_CFG_WRITE_INT_ANYRD_2CLEAR(Data)        MPUi2cWrite(0x37, 1, 4, Data)  //   1  Interrupt status is cleared if any read operation is performed. 0  Interrupt status is cleared only by reading INT_STATUS register
#define INT_PIN_CFG_WRITE_ACTL_FSYNC(Data)              MPUi2cWrite(0x37, 1, 3, Data)  //   1  The logic level for the FSYNC pin as an interrupt is active low. 0  The logic level for the FSYNC pin as an interrupt is active high.
#define INT_PIN_CFG_WRITE_FSYNC_INT_MODE_EN(Data)       MPUi2cWrite(0x37, 1, 2, Data)  //   1  This enables the FSYNC pin to be used as an interrupt.
#define INT_PIN_CFG_WRITE_BYPASS_EN(Data)               MPUi2cWrite(0x37, 1, 1, Data)  //   When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into bypass mode when the i2c master interface is disabled.


#define INT_ENABLE_WRITE_FF_EN(Data)                    MPUi2cWrite(0x38, 1, 7, Data)  //   1  Enable interrupt for
#define INT_ENABLE_WRITE_WOM_EN(Data)                   MPUi2cWrite(0x38, 1, 6, Data)  //   1  Enable interrupt for wake on motion to propagate to interrupt pin. 0  function is disabled.
#define INT_ENABLE_WRITE_ZMOT_OFLOW_EN(Data)            MPUi2cWrite(0x38, 1, 5, Data)  //   1  Enable interrupt for
#define INT_ENABLE_WRITE_FIFO_OFLOW_EN(Data)            MPUi2cWrite(0x38, 1, 4, Data)  //   1  Enable interrupt for fifo overflow to propagate to interrupt pin. 0  function is disabled.
#define INT_ENABLE_WRITE_FSYNC_INT_EN(Data)             MPUi2cWrite(0x38, 1, 3, Data)  //   1  Enable (I2C_MST_INT_BIT) Fsync interrupt to propagate to interrupt pin. 0  function is disabled.
#define INT_ENABLE_WRITE_RAW_PLL_RDY_INT_EN(Data)       MPUi2cWrite(0x38, 1, 2, Data)  //   1  Enable
#define INT_ENABLE_WRITE_RAW_DMP_INT_EN(Data)           MPUi2cWrite(0x38, 1, 1, Data)  //   1  Enable DMP interrupt
#define INT_ENABLE_WRITE_RAW_RDY_EN(Data)               MPUi2cWrite(0x38, 1, 0, Data)  //   1  Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin.  0  function is disabled.



#define I2C_SLV0_DO_WRITE_I2C_SLV0_DO(Data)             MPUi2cWriteBytes(0x63, 1, Data)  //   Data out when slave 0 is set to write
#define I2C_SLV1_DO_WRITE_I2C_SLV1_DO(Data)             MPUi2cWriteBytes(0x64, 1, Data)  //   Data out when slave 1 is set to write
#define I2C_SLV2_DO_WRITE_I2C_SLV2_DO(Data)             MPUi2cWriteBytes(0x65, 1, Data)  //   Data out when slave 2 is set to write
#define I2C_SLV3_DO_WRITE_I2C_SLV3_DO(Data)             MPUi2cWriteBytes(0x66, 1, Data)  //   Data out when slave 3 is set to write
#define I2C_MST_DELAY_CTRL_WRITE_DELAY_ES_SHADOW(Data)  MPUi2cWrite(0x67, 1, 7, Data)  //   Delays shadowing of external sensor data until     ALL data is received
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV4_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 4, Data)  //   When enabled, slave 4 will only be accessed
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV3_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 3, Data)  //   When enabled, slave 3 will only be accessed
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV2_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 2, Data)  //   N When enabled, slave 2 will only be accessed
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV1_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 1, Data)  //   When enabled, slave 1 will only be accessed
#define I2C_MST_DELAY_CTRL_WRITE_I2C_SLV0_DLY_EN(Data)  MPUi2cWrite(0x67, 1, 0, Data)  //   When enabled, slave 0 will only be accessed

#define SIGNAL_PATH_RESET_WRITE_RESET(...)            MPUi2cWrite(0x6A, 3, 2, 0b111)  //  Reset gyro,accel, temp signal paths
#define SIGNAL_PATH_RESET_WRITE_GYRO_RST(...)          MPUi2cWrite(0x68, 1, 2, 1)  //   Reset gyro digital signal path  Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers.
#define SIGNAL_PATH_RESET_WRITE_ACCEL_RST(...)         MPUi2cWrite(0x68, 1, 1, 1)  //   Reset accel digital signal path.  Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers.
#define SIGNAL_PATH_RESET_WRITE_TEMP_RST(...)          MPUi2cWrite(0x68, 1, 0, 1)  //   Reset temp digital signal path. Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers.


#define MOT_DETECT_CTRL_WRITE_ACCEL_INTEL_EN(Data)      MPUi2cWrite(0x69, 1, 7, Data)  //   This bit enables the Wake-on-Motion detection logic
#define MOT_DETECT_CTRL_WRITE_ACCEL_INTEL_MODE(Data)    MPUi2cWrite(0x69, 1, 6, Data)  //   1 = Compare the current sample with the previous sample. 0 = Not used.



#define USER_CTRL_WRITE_DMP_EN(Data)                    MPUi2cWrite(0x6A, 1, 7, Data)  //   1  Enable DMP operation mode.
#define USER_CTRL_WRITE_FIFO_EN(Data)                   MPUi2cWrite(0x6A, 1, 6, Data)  //   1  Enable FIFO operation mode. 0  Disable FIFO access from serial interface.  To disable FIFO writes by dma, use FIFO_EN register. To disable possible FIFO writes from DMP, disable the DMP.
#define USER_CTRL_WRITE_I2C_MST_EN(Data)                MPUi2cWrite(0x6A, 1, 5, Data)  //   1  Enable the I2C Master
#define USER_CTRL_WRITE_I2C_IF_DIS(Data)                MPUi2cWrite(0x6A, 1, 4, Data)  //   1  Disable I2C Slave module and put the serial interface in SPI mode only.
#define USER_CTRL_WRITE_DMP_RST(...)                    MPUi2cWrite(0x6A, 1, 3, 1)  //   Reset DMP module. Reset is asynchronous. This bit auto clears after one clock cycle.
#define USER_CTRL_WRITE_FIFO_RST(...)                   MPUi2cWrite(0x6A, 1, 2, 1)  //   Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
#define USER_CTRL_WRITE_I2C_MST_RESET_BIT(...)          MPUi2cWrite(0x6A, 1, 1, 1)  //   Reset  gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.
#define USER_CTRL_WRITE_SIG_COND_RST(...)               MPUi2cWrite(0x6A, 1, 0, 1)  //   Reset  gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.


#define USER_CTRL_WRITE_RESET(...)                     MPUi2cWrite(0x6A, 4, 3, 0b1111)  //   1  Reset DMP,  gyro digital signal path, accel digital signal path, and temp digital signal path. This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.

// per MPU-6000/MPU-6050 Register Map and Descriptions page 41 The proper reset sequence is Reset Device wait 100ms Reset gyro,accel, temp signal paths wait 100ms
#define PWR_MGMT_1_WRITE_DEVICE_RESET(...)              MPUi2cWrite(0x6B, 1, 7, 1);delay(100);MPUi2cWrite(0x6A, 3, 2, 0b111);delay(100);  //   1  Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear.
#define PWR_MGMT_1_WRITE_SLEEP(Data)                    MPUi2cWrite(0x6B, 1, 6, Data)  //   When set, the chip is set to sleep mode (After OTP loads, the PU_SLEEP_MODE bit will be written here)
#define PWR_MGMT_1_WRITE_CYCLE(Data)                    MPUi2cWrite(0x6B, 1, 5, Data)  //   When set, and SLEEP and STANDBY are not set, the chip will cycle between sleep and taking a single sample at a rate determined by LP_ACCEL_ODR register
#define PWR_MGMT_1_WRITE_GYRO_STANDBY(Data)             MPUi2cWrite(0x6B, 1, 4, Data)  //   When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled. This is a low power mode that allows quick enabling of the gyros.
#define PWR_MGMT_1_WRITE_PD_PTAT(Data)                  MPUi2cWrite(0x6B, 1, 3, Data)  //   Power down internal PTAT voltage generator and PTAT ADC
#define PWR_MGMT_1_WRITE_CLKSEL(Data)                   MPUi2cWrite(0x6B, 3, 2, Data)  //   Clock Source Select

#define PWR_MGMT_1_WRITE_CLKSEL_Internal_20MHz_osc(...) MPUi2cWrite(0x6B, 3, 2, 0)  //   Clock Source Select
#define PWR_MGMT_1_WRITE_CLKSEL_PLL_X_gyro(...)         MPUi2cWrite(0x6B, 3, 2, 1)  //   Clock Source Select

#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL_1_25Hz(...)       MPUi2cWrite(0x6C, 2, 7, 0)  //   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL_5Hz(...)          MPUi2cWrite(0x6C, 2, 7, 1)  //   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL_20Hz(...)         MPUi2cWrite(0x6C, 2, 7, 2)  //   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL_40Hz(...)         MPUi2cWrite(0x6C, 2, 7, 3)  //   1  X accelerometer is disabled 0  X accelerometer is on


#define PWR_MGMT_2_WRITE_LP_WAKE_CTRL(Data)             MPUi2cWrite(0x6C, 2, 7, Data)  //   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_DIS_XA(Data)                   MPUi2cWrite(0x6C, 1, 6, Data)  //   1  X accelerometer is disabled 0  X accelerometer is on
#define PWR_MGMT_2_WRITE_DIS_YA(Data)                   MPUi2cWrite(0x6C, 1, 5, Data)  //   1  Y accelerometer is disabled 0  Y accelerometer is on
#define PWR_MGMT_2_WRITE_DIS_ZA(Data)                   MPUi2cWrite(0x6C, 1, 4, Data)  //   1  Z accelerometer is disabled 0  Z accelerometer is on
#define PWR_MGMT_2_WRITE_DIS_XG(Data)                   MPUi2cWrite(0x6C, 1, 3, Data)  //   1  X gyro is disabled 0  X gyro is on
#define PWR_MGMT_2_WRITE_DIS_YG(Data)                   MPUi2cWrite(0x6C, 1, 2, Data)  //   1  Y gyro is disabled 0  Y gyro is on
#define PWR_MGMT_2_WRITE_DIS_ZG(Data)                   MPUi2cWrite(0x6C, 1, 1, Data)  //   1  Z gyro is disabled 0  Z gyro is on

#define BANK_SEL_WRITE(Data)                            MPUi2cWriteInt(0x6D,  Data)  //   DMP Bank Select for Loading Image
#define DMP_MEM_START_ADDR_WRITE(Data)                  MPUi2cWriteInt(0x6E,  Data)  //   Not Used
#define DMP_MEM_WRITE(Length,Data)                      MPUi2cWriteBytes(0x6F,Length, Data)  // DMP Image Loading Location
#define PRGM_START_WRITE(Data)                          MPUi2cWriteInt(0x70,  Data)  // Set program start address.




#define FIFO_COUNTH_WRITE_FIFO_CNT(Data)                MPUi2cWriteBytes(0x72, 2, Data)  //   High Bits, count indicates the number of written bytes in the FIFO.
#define FIFO_WRITE(Data,PacketLength)                   MPUi2cWriteBytes(0x74, PacketLength, Data  //   Read/Write command provides Read or Write operation for the FIFO.
//9250
//#define A_OFFSET_H_WRITE_A_OFFS(Data)                 MPUi2cWriteInts(0x77, 6, Data)  //   X accelerometer offset cancellation
//#define XA_OFFSET_H_WRITE_XA_OFFS(Data)               MPUi2cWriteInt(0x77,  Data)  //   X accelerometer offset cancellation
//#define YA_OFFSET_H_WRITE_YA_OFFS(Data)               MPUi2cWriteInt(0x7A,  Data)  //   Y accelerometer offset cancellation
//#define ZA_OFFSET_H_WRITE_ZA_OFFS(Data)               MPUi2cWriteInt(0x7D,  Data)  //   Z accelerometer offset cancellation

#endif

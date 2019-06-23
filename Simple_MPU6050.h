#ifndef Simple_MPU6050_h
#define Simple_MPU6050_h
#include <Wire.h>
#include <I2Cdev.h>
#include "DMP_Image.h"

#define ENABLE_MPU_OVERFLOW_PROTECTION(...) void yield(void){mpu.OverflowProtection();} // yield is called from within the delay() function 
#ifndef OFFSETS
#define OFFSETS      0,       0,       0,       0,       0,       0
#define CalibrationLoops 8
#endif


class Simple_MPU6050 : public I2Cdev {
    static void nothing(void) {};
    static void nothing(int16_t *, int16_t *, int32_t *, uint32_t *) {};
    typedef void (*_ON_FIFO_CB_PTR)(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp); // Create a type to point to a function.
    _ON_FIFO_CB_PTR on_FIFO_cb = nothing;
  public:
    typedef struct SensorList_s {
      int16_t ax ;
      int16_t ay ;
      int16_t az ;
      int16_t Temp;
      int16_t gx ;
      int16_t gy ;
      int16_t gz ;
    };


    typedef union AccelGyro_u {
      SensorList_s V;
      int16_t intData[sizeof(SensorList_s) / 2];
    };

    AccelGyro_u S;
    uint8_t buffer[14];
    uint8_t devAddr;
    uint8_t dmp_on;/* 1 if DMP is enabled. */
    uint8_t data[16];
    uint8_t packet_length;
    uint16_t dmp_features;
    uint16_t  sensor_timestamp;
    int16_t  gyro[3], accel[3];
    int32_t  quat[4];

    //Startup Functins
    Simple_MPU6050(); // Constructor
    Simple_MPU6050 & SetAddress(uint8_t address);
    uint8_t CheckAddress();
    uint8_t TestConnection(int Stop = 1);
    void OverflowProtection(void);
    Simple_MPU6050 & load_DMP_Image();
    Simple_MPU6050 & load_DMP_Image(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_);
    Simple_MPU6050 & setOffset(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_);
    Simple_MPU6050 & on_FIFO(void (*CB)(int16_t *, int16_t *, int32_t *, uint32_t *));
    Simple_MPU6050 & reset_fifo();
    Simple_MPU6050 & full_reset_fifo(void); //Clears fifo and sensor paths.
    Simple_MPU6050 & DMP_InterruptEnable(uint8_t Data);

    // usage functions
    uint8_t CheckForInterrupt(void);
    Simple_MPU6050 & dmp_read_fifo(); //Overloaded Callback trigger
    uint8_t dmp_read_fifo(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp);// Basic Send and recieve
    /* register management functions and Helper Macros:
        See MPU_ReadMacros.h and MPU_WriteMacros.h for a complete set of Register management Macros
       Class functions Macros to access Just about every needed register bit, bits, Bytes and ints ever
       needed to minipulate the registers of the mpu.
       Macros are in all CAPS for with underscores for spacing
       for example: Lets change FIFO overflow interrupt bit to ture
       mpu.INT_ENABLE_WRITE_FIFO_OFLOW_EN(1);
       I can look in the pdf document under INT_ENABLE and FIFO_OFLOW_EN bit know exactly what im doing
       the Macro preconfigures the MPUi2cWrite function with all the address and bit location and number of bits
       (0x38, 1, 4, Data) correctly handled
    */
    // using the above mentioned Helper Macros, Every register as needed down to bit level is represented in the Simple_MPU6050 class
    // register management functions
    Simple_MPU6050 & MPUi2cRead(uint8_t regAddr,  uint8_t length, uint8_t bitNum, uint8_t *data);
    Simple_MPU6050 & MPUi2cReadBytes(uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cReadInt(uint8_t regAddr, uint16_t *data);
    Simple_MPU6050 & MPUi2cReadInts(uint8_t regAddr, uint16_t size, uint16_t *Data);
    Simple_MPU6050 & MPUi2cWrite(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val);
    Simple_MPU6050 & MPUi2cWriteBytes(uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cWriteInt(uint8_t regAddr,  uint16_t Val);
    Simple_MPU6050 & MPUi2cWriteInts(uint8_t regAddr, uint16_t size,  uint16_t *data);

    // helper math funcitons

    Simple_MPU6050 & SetAccel(VectorInt16 *v, int16_t *accel);
    Simple_MPU6050 & GetQuaternion(Quaternion *q, const int32_t* qI);
    Simple_MPU6050 & GetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
    Simple_MPU6050 & GetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
    Simple_MPU6050 & GetGravity(VectorFloat *v, Quaternion *q);
    Simple_MPU6050 & GetEuler(float *data, Quaternion *q);
    Simple_MPU6050 & GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
    Simple_MPU6050 & ConvertToDegrees(float*ypr, float*xyz);
    Simple_MPU6050 & ConvertToRadians( float*xyz, float*ypr);

    // firmware management functions
    Simple_MPU6050 & load_firmware(uint16_t  length, const uint8_t *firmware);
    Simple_MPU6050 & read_mem(uint16_t mem_addr, uint16_t length, uint8_t *data);
    Simple_MPU6050 & write_mem(uint16_t mem_addr, uint16_t length, uint8_t *data);

    // Default data gathering functions for program revisions
    void view_Vital_MPU_Registers();
    bool view_DMP_firmware_Instance(uint16_t  length);
    Simple_MPU6050 & PrintActiveOffsets();
    Simple_MPU6050 & CalibrateGyro(int Loops = 6);
    Simple_MPU6050 & CalibrateAccel(int Loops = 6);
    Simple_MPU6050 & PID(uint8_t ReadAddress, uint8_t SaveAddress, float kP, float kI, uint8_t Loops);
};



#endif

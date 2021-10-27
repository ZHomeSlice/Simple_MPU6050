/* ============================================
  I2Cdev device library code is placed under the MIT license
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
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/
#ifndef Simple_MPU6050_h
#define Simple_MPU6050_h

#include <Wire.h>
#include "I2Cdev.h"
#include "DMP_Image.h"
#include "MPU_WriteMacros.h"
#include "MPU_ReadMacros.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#ifndef interruptPin
#define interruptPin 2
#endif
#define Interupt_Attach_Function  if(interruptPin > 0)	attachInterrupt(digitalPinToInterrupt(interruptPin), [] {mpuInterrupt = true;}, RISING); //NOTE: "[]{mpuInterrupt = true;}" Is a complete funciton without a name. It is handed to the callback of attachInterrupts Google: "Lambda anonymous functions"
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt
#elif defined(ESP32)
    #include <pgmspace.h>
    #include <stdlib_noniso.h>
    #ifndef interruptPin
    #define interruptPin 15
    #endif
    #define  Interupt_Attach_Function  if(interruptPin > 0) attachInterrupt(digitalPinToInterrupt(interruptPin),  [] {mpuInterrupt = true;}, RISING); //NOTE: "[]{mpuInterrupt = true;}" Is a complete funciton without a name. It is handed to the callback of attachInterrupts Google: "Lambda anonymous functions"
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); Serial.print(F(" ")); Serial.print(Variable,Precision);Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt
#else
#define Interupt_Attach_Function
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); Serial.print(F(" ")); Serial.print(Variable,Precision);Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt
#endif

#define DPRINTBINL(Num) for (uint16_t i=0;i<(sizeof((uint16_t)Num)*8);i++) Serial.write(((Num >> i) & 1) == 1 ? '1' : '0'); // Prints a binary number with following Placeholder Zeros  (Automatic Handling)
#define DPRINTBINLX(S,Num,nl) Serial.print(F(S)); for (uint16_t i=0;i<(sizeof((uint16_t)Num)*8);i++) Serial.write(((Num >> i) & 1) == 1 ? '1' : '0'); if(nl)Serial.println(); // Prints a binary number with following Placeholder Zeros  (Automatic Handling)
#define DPRINTBIN(Num) for (uint32_t t = (1UL<< ((sizeof(Num)*8)-1)); t; t >>= 1) Serial.write(Num  & t ? '1' : '0'); // Prints a binary number with leading zeros (Automatic Handling)
#define DPRINTHEX(Num) Serial.print(Num>>4,HEX);Serial.print(Num&0X0F,HEX);
#define ShowByte(Addr) {uint8_t val; I2Cdev::readBytes(0x68, Addr, 1, &val);  Serial.print("0x"); DPRINTHEX(Addr); Serial.print(" = 0x"); DPRINTHEX(val); Serial.print(" = 0B"); DPRINTBIN(val); Serial.println();}
#define ShowValue(Name, FunctionD) FunctionD; Serial.print(Name); Serial.print(" = 0x"); DPRINTHEX(D); Serial.print(" = 0B"); DPRINTBIN(D); Serial.println();

class Simple_MPU6050 : public I2Cdev {
    static void nothing(void) {};
    static void nothing(int16_t *, int16_t *, int32_t *, uint32_t *) {};
    typedef void (*_ON_FIFO_CB_PTR)(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp); // Create a type to point to a function.
    _ON_FIFO_CB_PTR on_FIFO_cb = nothing;
  public:

    struct SensorList_s {
      int16_t ax ;
      int16_t ay ;
      int16_t az ;
      int16_t Temp;
      int16_t gx ;
      int16_t gy ;
      int16_t gz ;
    };

	float mx, my, mz; // variables to hold latest magnetometer data values

    union AccelGyro_u {
      SensorList_s V;
      int16_t intData[sizeof(SensorList_s) / 2];
    };

	const float radians_to_degrees = 180.0 / M_PI;
	uint8_t HIGH_SENS  = 1; // 0 = 14-BIT, 1 = 16-BIT 
    AccelGyro_u S;
    uint8_t buffer[14];
	uint16_t IntBuffer[7];
    uint8_t devAddr;
	uint8_t akm_addr = 0;
	uint8_t akm_WhoAmI;
	uint8_t WhoAmI;
    uint8_t dmp_on;/* 1 if DMP is enabled. */
    uint8_t data[16];
	uint8_t TVal; // TVal For any read
    uint8_t packet_length;
    uint16_t dmp_features;
    bool DMP_Loaded = false;
    uint32_t sensor_timestamp;
    int16_t  gyro[3], accel[3];
	float mag[3];
    int32_t quat[4];
    uint8_t compass_addr;
    int16_t mag_sens_adj[3];
    float mag_sens_adj_F[3];
	int32_t mag_sens_adj_L[3];
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
	float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
	float mRes;
	float mag_bias[3] = {0, 0, 0}; // max+min /2
	float mag_scale[3] = {0, 0, 0};// max-min /2
	float dest1[3]; // hard iron correction mag biases in G  magBias * mRes * mag_sens_adj
	float dest2[3]; // soft iron correction estimate  = ( "average" magBias(x+y+z) /3) / mag_bias
    uint8_t DMP_Output_Rate[2]; 
    int8_t I2CReadCount; //items Read 
    bool I2CWriteStatus; //  True False
	int16_t sax_,say_,saz_,sgx_,sgy_,sgz_;


    //Startup Functins MPU
    Simple_MPU6050(); // Constructor
    Simple_MPU6050 & SetAddress(uint8_t address);
    uint8_t CheckAddress();
    uint8_t TestConnection(int Stop = 1);
    Simple_MPU6050 & Set_DMP_Output_Rate(uint16_t value = 0x01); // 100Hz Default
    Simple_MPU6050 & Set_DMP_Output_Rate_Hz(float rate = 100); // 100Hz Default
    Simple_MPU6050 & Set_DMP_Output_Rate_Seconds(float rate = 1); // 1Hz Default
    Simple_MPU6050 & Set_DMP_Output_Rate_Minutes(float rate = 1); // 1 minute Default
	Simple_MPU6050 & CalibrateMPU(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_);
	Simple_MPU6050 & CalibrateMPU(uint8_t Loops = 30);
    Simple_MPU6050 & Enable_Reload_of_DMP();
    Simple_MPU6050 & load_DMP_Image(uint8_t CalibrateMode = 0);
	Simple_MPU6050 & load_DMP_Image(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_,int8_t Calibrate = 1);
	Simple_MPU6050 & resetOffset();
    Simple_MPU6050 & setOffset(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_);
    Simple_MPU6050 & on_FIFO(void (*CB)(int16_t *, int16_t *, int32_t *, uint32_t *));
    Simple_MPU6050 & reset_fifo();
    Simple_MPU6050 & resetFIFO(){reset_fifo();return *this;};
    Simple_MPU6050 & resetDMP(){USER_CTRL_WRITE_DMP_RST();return *this;};
    Simple_MPU6050 & full_reset_fifo(void); //Clears fifo and sensor paths.
    Simple_MPU6050 & DMP_InterruptEnable(uint8_t Data);

    //Startup Functins AKM
	Simple_MPU6050 & AKM_Init();
	Simple_MPU6050 & mpu_set_bypass(unsigned char bypass_on);

    // usage functions
    uint8_t CheckForInterrupt(void);
    int16_t getFIFOCount();
	int8_t GetCurrentFIFOPacket(uint8_t *data, uint8_t length);
    uint8_t dmp_read_fifo(uint8_t CheckInterrupt = 1); // 0 = No interrupt needed to try to get data 
    uint8_t dmp_read_fifo(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp);// Basic receive packet

    /* register management functions and Helper Macros:
       See MPU_ReadMacros.h and MPU_WriteMacros.h for a complete set of Register management Macros
       Class functions Macros to access Just about every needed register bit, bits, Bytes and ints ever
       needed to manipulate the registers of the MPU.
       Macros are in all CAPS for with underscores for spacing
       for example: Lets change FIFO overflow interrupt bit to true
       mpu.INT_ENABLE_WRITE_FIFO_OFLOW_EN(1);
       I can look in the .pdf document under INT_ENABLE and FIFO_OFLOW_EN bit know exactly what i'm doing
       the Macro reconfigures the MPUi2cWrite function with all the address and bit location and number of bits
       (0x38, 1, 4, Data) correctly handled
    */
    // using the above mentioned Helper Macros, Every register as needed down to bit level is represented in the Simple_MPU6050 class
    // register management functions

	// Wrappered I2Cdev read functions
    Simple_MPU6050 & MPUi2cRead(uint8_t regAddr,  uint8_t length, uint8_t bitNum, uint8_t *data);
    Simple_MPU6050 & MPUi2cRead(uint8_t AltAddress,uint8_t regAddr,  uint8_t length, uint8_t bitNum, uint8_t *data);
    Simple_MPU6050 & MPUi2cReadByte(uint8_t regAddr,  uint8_t *Data);
    Simple_MPU6050 & MPUi2cReadByte(uint8_t AltAddress,uint8_t regAddr, uint8_t *Data);
    Simple_MPU6050 & MPUi2cReadBytes(uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cReadBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cReadInt(uint8_t regAddr, uint16_t *data);
    Simple_MPU6050 & MPUi2cReadInt(uint8_t AltAddress,uint8_t regAddr, uint16_t *data);
	Simple_MPU6050 & MPUi2cReadInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size, uint16_t *Data);
    Simple_MPU6050 & MPUi2cReadInts(uint8_t regAddr, uint16_t size, uint16_t *Data);
	// Wrappered I2Cdev write functions
    Simple_MPU6050 & MPUi2cWrite(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val);
    Simple_MPU6050 & MPUi2cWrite(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val);
    Simple_MPU6050 & MPUi2cWriteByte(uint8_t regAddr,  uint8_t Val);
    Simple_MPU6050 & MPUi2cWriteByte(uint8_t AltAddress,uint8_t regAddr,  uint8_t Val);
    Simple_MPU6050 & MPUi2cWriteBytes(uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cWriteBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cWriteInt(uint8_t regAddr,  uint16_t Val);
    Simple_MPU6050 & MPUi2cWriteInt(uint8_t AltAddress,uint8_t regAddr,  uint16_t Val);
    Simple_MPU6050 & MPUi2cWriteInts(uint8_t regAddr, uint16_t size,  uint16_t *data);
    Simple_MPU6050 & MPUi2cWriteInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size,  uint16_t *data);
	int8_t ReadCnt(){return I2CReadCount;};
	int8_t ReadStatus(){return I2CReadCount>0;};
	bool WriteStatus(){return I2CWriteStatus;};


    // helper math functions
    Simple_MPU6050 & SetAccel(VectorInt16 *v, int16_t *accel);
    Simple_MPU6050 & GetQuaternion(Quaternion *q, const int32_t* qI);
    Simple_MPU6050 & GetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
    Simple_MPU6050 & GetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
    Simple_MPU6050 & GetGravity(VectorFloat *v, Quaternion *q);
    Simple_MPU6050 & GetEuler(float *data, Quaternion *q);
    Simple_MPU6050 & GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
	Simple_MPU6050 & GetYawPitchRoll(float *data, Quaternion *q);
    Simple_MPU6050 & ConvertToDegrees(float*ypr, float*xyz);
    Simple_MPU6050 & ConvertToRadians( float*xyz, float*ypr);
	Simple_MPU6050 & MagneticNorth(float*data, VectorInt16 *v, Quaternion*q );

    // firmware management functions
    Simple_MPU6050 & load_firmware(uint16_t  length, const uint8_t *firmware);
    Simple_MPU6050 & read_mem(uint16_t mem_addr, uint16_t length, uint8_t *data);
    Simple_MPU6050 & write_mem(uint16_t mem_addr, uint16_t length, uint8_t *data);

    // Default data gathering functions for program revisions
    void view_Vital_MPU_Registers();
    bool view_DMP_firmware_Instance(uint16_t  length);
	Simple_MPU6050 &  GetActiveOffsets(int16_t *Data);
	Simple_MPU6050 & PrintActiveOffsets(); // See the results of the Calibration
	Simple_MPU6050 & PrintActiveOffsets(uint8_t MPU6500andMPU9250); // See the results of the Calibration
	Simple_MPU6050 & PrintActiveOffsets_MPU6500();
	Simple_MPU6050 & PrintActiveOffsets_MPU9250();

	
	// Calibration Routines
	Simple_MPU6050 & CalibrateGyro(uint8_t Loops = 30); // Fine tune after setting offsets with less Loops.
	Simple_MPU6050 & CalibrateAccel(uint8_t Loops = 30); // Fine tune after setting offsets with less Loops.
	Simple_MPU6050 & PID(uint8_t ReadAddress, float kP, float kI, uint8_t Loops);  // Does the math

	//Compass functions:
	Simple_MPU6050 & I2CScanner();
	uint8_t FindAddress(uint8_t Address,uint8_t Limit);
	Simple_MPU6050 & setup_compass(byte Is_MPU9150 = 0);
	Simple_MPU6050 & mpu_get_compass_reg_bypass(short *Data);
	Simple_MPU6050 & mpu_get_compass_reg_External(int16_t *Data);

	bool readMagData();
    bool readMagData(float *magData);
	Simple_MPU6050 & readMagDataThroughMPU();

	Simple_MPU6050 & magcalMPU();
	Simple_MPU6050 & setMagOffsets(float xMagB,float yMagB,float zMagB, float xMagS,float yMagS,float zMagS);
	Simple_MPU6050 & PrintMagOffsets();

	Simple_MPU6050 & viewMagRegisters();

};



#endif

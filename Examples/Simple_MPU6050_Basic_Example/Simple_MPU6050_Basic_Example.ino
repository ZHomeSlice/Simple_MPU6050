/*
 *  Use with any MPU: MPU6050, MPU6500, MPU9150, MPU9155, MPU9250 
 *  Attach the MPU to the I2C buss
 *  Power MPU According to specs of the breakout board. Generic Breakout Version Powers with 5V and has a onboard Voltage regulator.
 *  run the Sketch
 */

#include "Simple_MPU6050.h"
#define MPU6050_DEFAULT_ADDRESS     0x68 // address pin low (GND), default for InvenSense evaluation board

Simple_MPU6050 mpu;

//***************************************************************************************
//******************                Print Funcitons                **********************
//***************************************************************************************
//Gyro, Accel and Quaternion
int PrintAllValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);
  Serial.print(F("Yaw "));   Serial.print(xyz[0]);   Serial.print(F(",   "));
  Serial.print(F("Pitch ")); Serial.print(xyz[1]);   Serial.print(F(",   "));
  Serial.print(F("Roll "));  Serial.print(xyz[2]);   Serial.print(F(",   "));
  Serial.print(F("ax "));    Serial.print(accel[0]); Serial.print(F(",   "));
  Serial.print(F("ay "));    Serial.print(accel[1]); Serial.print(F(",   "));
  Serial.print(F("az "));    Serial.print(accel[2]); Serial.print(F(",   "));
  Serial.print(F("gx "));    Serial.print(gyro[0]);  Serial.print(F(",   "));
  Serial.print(F("gy "));    Serial.print(gyro[1]);  Serial.print(F(",   "));
  Serial.print(F("gz "));    Serial.print(gyro[2]);  Serial.print(F("\n"));
  Serial.println();
}

//***************************************************************************************
//******************              Callback Funciton                **********************
//***************************************************************************************

// See mpu.on_FIFO(print_Values); in the Setup Loop
void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
  uint8_t Spam_Delay = 100; // Built in Blink without delay timer preventing Serial.print SPAM
  PrintAllValues(gyro, accel, quat, Spam_Delay);
}

//***************************************************************************************
//******************                Setup and Loop                 **********************
//***************************************************************************************

void setup() {
  uint8_t val;
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(3000, true); //timeout value in uSec
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Start:"));

  // Setup the MPU
  mpu.Set_DMP_Output_Rate(DMP_10Hz);     // Set the DMP output rate from 200Hz to 5 Minutes.
/*
   Set the DMP output rate from 200Hz to 5 Minutes!!!
   These are the DMP output rates pre calculated
   #define DMP_200Hz  0x00, 0x00
   #define DMP_100Hz  0x00, 0x01
   #define DMP_50Hz   0x00, 0x03
   #define DMP_40Hz   0x00, 0x04
   #define DMP_25Hz   0x00, 0x07
   #define DMP_20Hz   0x00, 0x09
   #define DMP_10Hz   0x00, 0x13
   #define DMP_5Hz    0x00, 0x27
   #define DMP_1Hz    0x00, 0xC7
   #define DMP_1Sec   0x00, 0xC7
   #define DMP_10Sec  0x07, 0xCF
   #define DMP_60Sec  0x2E, 0xDF
   #define DMP_1Min   0x2E, 0xDF
   #define DMP_5Min   0xEA, 0x5F

   How to Calculate your own
   div = DMP_SAMPLE_RATE / rate_in_Hz - 1;
   div = 200 / rate - 1;
   #define DMP_xxx High Bit,  Low Bit
*/
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS); //Sets the address of the MPU.
  mpu.CalibrateMPU();                      // Calibrates the MPU.
  mpu.load_DMP_Image();                    // Loads the DMP image into the MPU and finish configuration.
  mpu.on_FIFO(print_Values);               // Set callback function that is triggered when FIFO Data is retrieved
  // Setup is complete!
  
}

void loop() {
  mpu.dmp_read_fifo(false); // false = no interrupt pin attachment required.
  // Tests for Data in the FIFO Buffer
  // when it finds data it runs the mpu.on_FIFO(print_Values)
  // functin which we set run the print_Values Function
  // The print_Values function MUST have the following variables available to attach data
  // void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp)
  // Variables:
  // int16_t *gyro for the gyro values to be passed to it (The * tells the function it will be a pointer to the value)
  // int16_t *accel for the accel values to be passed to it
  // int32_t *quat for the quaternion values to be passed to it
  // uint32_t *timestamp which will be the micros()value at the time we retrieved the Newest value from the FIFO Buffer.
}

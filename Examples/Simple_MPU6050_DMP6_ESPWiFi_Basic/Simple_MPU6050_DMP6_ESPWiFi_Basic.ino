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
#include "Simple_MPU6050.h"

#define MPU6050_DEFAULT_ADDRESS     0x68 // address pin low (GND), default for InvenSense evaluation board
Simple_MPU6050 mpu;



//================================================================
//===                    Callback Function                     ===
//================================================================
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt

// See mpu.on_FIFO(print_Values); in the Setup Loop
void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);
    Serial.printfloatx(F("Yaw")  , xyz[0],   9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("Pitch"), xyz[1],   9, 4, F(",   "));
    Serial.printfloatx(F("Roll") , xyz[2],   9, 4, F(",   "));
    Serial.printfloatx(F("ax")   , accel[0], 5, 0, F(",   "));
    Serial.printfloatx(F("ay")   , accel[1], 5, 0, F(",   "));
    Serial.printfloatx(F("az")   , accel[2], 5, 0, F(",   "));
    Serial.printfloatx(F("gx")   , gyro[0],  5, 0, F(",   "));
    Serial.printfloatx(F("gy")   , gyro[1],  5, 0, F(",   "));
    Serial.printfloatx(F("gz")   , gyro[2],  5, 0, F("\n"));
  

}

//================================================================
//===                        MPU Setup                         ===
//================================================================
void mpu_setup()
{
  // Setup the MPU
  int sdaPin = 0, 
  int sclPin = 1
  mpu.begin(sdaPin, sclPin);
  mpu.Set_DMP_Output_Rate_Hz(10);          // Set the DMP output rate from 200Hz to 5 Minutes.
  //mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  //mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minutes
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS); //Sets the address of the MPU.
  mpu.CalibrateMPU();                      // Calibrates the MPU.
  mpu.load_DMP_Image();                    // Loads the DMP image into the MPU and finish configuration.
  mpu.on_FIFO(print_Values);               // Set callback function that is triggered when FIFO Data is retrieved
  // Note that these functions return pointers to themselves so you can write them in one line
  // mpu.Set_DMP_Output_Rate_Hz(4).SetAddress(MPU6050_DEFAULT_ADDRESS).CalibrateMPU().load_DMP_Image().on_FIFO(print_Values);
  // Setup is complete!


}
//================================================================
//===                          Setup                           ===
//================================================================
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("\nOrientation Sensor OSC output")); Serial.println();

  mpu_setup();
}
//================================================================
//===                        Main Loop                         ===
//================================================================

void loop(void)
{
  mpu.dmp_read_fifo(false); // false = no interrupt pin attachment required.
  // Tests for Data in the FIFO Buffer
  // when it finds data it runs the mpu.on_FIFO(print_Values)
  // function which we set run the print_Values Function
  // The print_Values function MUST have the following variables available to attach data
  // void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp)
  // Variables:
  // int16_t *gyro for the gyro values to be passed to it (The * tells the function it will be a pointer to the value)
  // int16_t *accel for the accel values to be passed to it
  // int32_t *quat for the quaternion values to be passed to it
  // uint32_t *timestamp which will be the micros()value at the time we retrieved the Newest value from the FIFO Buffer.
}

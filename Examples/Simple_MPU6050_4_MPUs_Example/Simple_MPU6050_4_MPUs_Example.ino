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

/* 2 MPUXXXX Example
    These can be any MPU MPU6050, MPU6500, MPU9150, MPU9155, MPU9250 ETC...
    Attach multiple MPU's to the I2C buss
    Power both MPU's According to specs. Generic Breakout Version Powers with 5V and has a onboard Voltage regulator.
    attach a 2K ohm resister between Pin 6 and AD0 on mpu#1
    attach a 2K ohm resister between Pin 7 and AD0 on mpu#2
    attach a 2K ohm resister between Pin 8 and AD0 on mpu#3
    attach a 2K ohm resister between Pin 9 and AD0 on mpu#4
    Sketch will load both MPU's with DMP Firmware
    Interrupt pin is not connected.
*/
int MPUNumber;  //

//#include "MPU_ReadMacros.h"
//#inc;ude "MPU_WriteMacros.h"
#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW
#define Three_Axis_Quaternions 3
#define Six_Axis_Quaternions 6  // Default
Simple_MPU6050 mpu(Six_Axis_Quaternions);
/*             _________________________________________________________*/
//               X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
//#define OFFSETS  -5260,    6596,    7866,     -45,       5,      -9  // My Last offsets.
//       You will want to use your own as these are only for my specific MPU6050.
/*             _________________________________________________________*/

//***************************************************************************************
//******************                Print Macors                   **********************
//***************************************************************************************

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!

/* printfloatx() is a helper Macro used with the Serial class to simplify my code and provide enhanced viewing of Float and intergers values:
   usage: printfloatx(Name,Variable,Spaces,Precision,EndTxt);
   Name and EndTxt are just char arrays
   Variable is any numerical value byte, int, long and float
   Spaces is the number of spaces the floating point number could possibly take up including +- and decimal point.
   Precision is the number of digits after the decimal point set to zero for intergers
*/

#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//printfloatx(Name,Variable,Spaces,Precision,EndTxt)

//***************************************************************************************
//******************              Callback Funciton                **********************
//***************************************************************************************
void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(100) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    Serial.print(F("MPU #"));
    Serial.print(MPUNumber);
    Serial.print(" ");
    Serial.printfloatx(F("Yaw")  , xyz[0], 9, 4, F(", ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("Pitch"), xyz[1], 9, 4, F(", "));
    Serial.printfloatx(F("Roll") , xyz[2], 9, 4, F("\n"));
  }
}


//***************************************************************************************
//******************                Setup and Loop                 **********************
//***************************************************************************************

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Start:"));
  mpu.begin();
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS);
  mpu.Set_DMP_Output_Rate_Hz(5);           // Set the DMP output rate from 200Hz to 5 Minutes.
  //mpu.Set_DMP_Output_Rate_Seconds(10);   // Set the DMP output rate in Seconds
  //mpu.Set_DMP_Output_Rate_Minutes(5);    // Set the DMP output rate in Minute
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  for (int i = 6; i < 10; i++) {
    digitalWrite(6, !(i == 6)); // when i == 6 set pin LOW
    digitalWrite(7, !(i == 7)); // when i == 7 set pin LOW
    digitalWrite(8, !(i == 8)); // when i == 8 set pin LOW
    digitalWrite(9, !(i == 9)); // when i == 9 set pin LOW
    mpu.CalibrateMPU().Enable_Reload_of_DMP(Six_Axis_Quaternions).load_DMP_Image();// Does it all for you with Calibration
  }
  mpu.on_FIFO(print_Values);
}

void loop() {
  for (MPUNumber = 1; MPUNumber < 5; MPUNumber++) {
    digitalWrite(6, !(MPUNumber == 1)); // when MPUNumber == 1 set pin 6 LOW
    digitalWrite(7, !(MPUNumber == 2)); // when MPUNumber == 2 set pin 7 LOW
    digitalWrite(8, !(MPUNumber == 3)); // when MPUNumber == 3 set pin 8 LOW
    digitalWrite(9, !(MPUNumber == 4)); // when MPUNumber == 4 set pin 9 LOW
    mpu.dmp_read_fifo(0);// Must be in loop
  }
}
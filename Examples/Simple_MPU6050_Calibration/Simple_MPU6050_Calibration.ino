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
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

/*
    Use with any MPU: MPU6050, MPU6500, MPU9150, MPU9155, MPU9250
    Attach the MPU to the I2C buss
    Power MPU According to specs of the breakout board. Generic Breakout Version Powers with 5V and has a onboard Voltage regulator.
    run the Sketch
*/

#include "Simple_MPU6050.h"
#define MPU6050_DEFAULT_ADDRESS     0x68 // address pin low (GND), default for InvenSense evaluation board

Simple_MPU6050 mpu;

//***************************************************************************************
//******************                Setup and Loop                 **********************
//***************************************************************************************

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Start:"));

  // Setup the MPU
  mpu.begin();
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS); //Sets the address of the MPU.
  Serial.println(F("\n\n The MPU6050 comes with no calibration values set.\n"
                   " This is what the default calibratin values are:\n"
                   " "));
  mpu.PrintActiveOffsets();
   
  Serial.println(F("\n\n We are going to calibrate this specific MPU6050,\n"
                   " Start by having the MPU6050 placed stationary on a flat surface to get a proper accellerometer calibration\n"
                   " \t\t\t[Press Any Key]"));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  mpu.CalibrateMPU();  // Calibrates the Gyro.
  delay(1000);
  Serial.println();
  for (int i = 5; i<=30;i+=5){
    Serial.println("Stabalization Delay");
    delay (100);
    mpu.CalibrateAccel(i); // Calibrates the Accellerometer.
    mpu.CalibrateGyro(i);  // Calibrates the Gyro.
  }
  mpu.PrintActiveOffsets();
  // Setup is complete!

}

void loop() {
  delay(100);
  Serial.println(F("\n\nThe above values are your calibration offsets.\n"
                   " You can recheck the calibration using the last calibration as a starting poing for this specific MPU6050,\n"
                   " This can further inprove the accuracy and may refine the calibration values\n"
                   " Placed the MPU6050 stationary on a flat surface to get a proper accellerometer calibration\n"
                   " \t\t\t[Press Any Key]"));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  mpu.CalibrateMPU();  // Calibrates the MPU.
  mpu.PrintActiveOffsets();

}

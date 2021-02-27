//#include "MPU_ReadMacros.h"
//#inc;ude "MPU_WriteMacros.h"
#include "Simple_MPU6050.h"
#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

Simple_MPU6050 mpu;
ENABLE_MPU_OVERFLOW_PROTECTION();
/*             _________________________________________________________*/
//               X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
//#define OFFSETS  -5260,    6596,    7866,     -45,       5,      -9  // My Last offsets. 
//       You will want to use your own as these are only for my specific MPU6050.
/*             _________________________________________________________*/

//***************************************************************************************
//******************                Print Funcitons                **********************
//***************************************************************************************

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!

/* printfloatx() is a helper Macro used with the Serial class to simplify my code and provide enhanced viewing of Float and interger values:
   usage: printfloatx(Name,Variable,Spaces,Precision,EndTxt);
   Name and EndTxt are just char arrays
   Variable is any numerical value byte, int, long and float
   Spaces is the number of spaces the floating point number could possibly take up including +- and decimal point.
   Percision is the number of digits after the decimal point set to zero for intergers
*/
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt

int PrintValues(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    Serial.printfloatx(F("Yaw")  , ypr[0], 9, 4, F(", ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("Pitch"), ypr[1], 9, 4, F(", "));
    Serial.printfloatx(F("Roll") , ypr[2], 9, 4, F("\n"));
  }
}

int ChartValues(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    Serial.printfloatx("", ypr[0], 9, 4, F(",")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx("", ypr[1], 9, 4, F(","));
    Serial.printfloatx("", ypr[2], 9, 4, F("\n"));
  }
}

//Gyro, Accel and Quaternion
int PrintAllValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    Serial.printfloatx(F("Yaw")  , xyz[0], 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("Pitch"), xyz[1], 9, 4, F(",   "));
    Serial.printfloatx(F("Roll") , xyz[2], 9, 4, F(",   "));
    /*
      Serial.printfloatx(F("ax")   , accel[0], 5, 0, F(",   "));
      Serial.printfloatx(F("ay")   , accel[1], 5, 0, F(",   "));
      Serial.printfloatx(F("az")   , accel[2], 5, 0, F(",   "));
      Serial.printfloatx(F("gx")   , gyro[0],  5, 0, F(",   "));
      Serial.printfloatx(F("gy")   , gyro[1],  5, 0, F(",   "));
      Serial.printfloatx(F("gz")   , gyro[2],  5, 0, F("\n"));
    */
    Serial.println();
  }
}

int ChartAllValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);
    Serial.printfloatx("", ypr[0], 9, 4, F(",")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx("", ypr[1], 9, 4, F(","));
    Serial.printfloatx("", ypr[2], 9, 4, F(", "));
    Serial.printfloatx("", accel[0], 5, 0, F(","));
    Serial.printfloatx("", accel[1], 5, 0, F(","));
    Serial.printfloatx("", accel[2], 5, 0, F(","));
    Serial.printfloatx("", gyro[0],  5, 0, F(","));
    Serial.printfloatx("", gyro[1],  5, 0, F(","));
    Serial.printfloatx("", gyro[2],  5, 0, F("\n"));
  }
}

int PrintQuaternion(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    Serial.printfloatx(F("quat w")  , q.w, 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("x")       , q.x, 9, 4, F(",   "));
    Serial.printfloatx(F("y")       , q.y, 9, 4, F(",   "));
    Serial.printfloatx(F("z")       , q.z, 9, 4, F("\n"));
  }
}

int PrintEuler(int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  float euler[3];         // [psi, theta, phi]    Euler angle container
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetEuler(euler, &q);
    Serial.printfloatx(F("euler  ")  , euler[0] * 180 / M_PI, 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("")       , euler[1] * 180 / M_PI, 9, 4, F(",   "));
    Serial.printfloatx(F("")       , euler[2] * 180 / M_PI, 9, 4, F("\n"));
  }
}

int PrintRealAccel(int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  VectorInt16 aa, aaReal;
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.SetAccel(&aa, accel);
    mpu.GetLinearAccel(&aaReal, &aa, &gravity);
    Serial.printfloatx(F("aReal x")  , aaReal.x , 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("y")        , aaReal.y , 9, 4, F(",   "));
    Serial.printfloatx(F("z")        , aaReal.z, 9, 4, F("\n"));
  }
}


int PrintWorldAccel(int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  Quaternion q;
  VectorFloat gravity;
  VectorInt16 aa, aaReal, aaWorld;
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.SetAccel(&aa, accel);
    mpu.GetLinearAccel(&aaReal, &aa, &gravity);
    mpu.GetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.printfloatx(F("aWorld x")  , aaWorld.x , 9, 4, F(",   ")); //printfloatx is a Helper Macro that works with Serial.print that I created (See #define above)
    Serial.printfloatx(F("y")        , aaWorld.y, 9, 4, F(",   "));
    Serial.printfloatx(F("z")        , aaWorld.z, 9, 4, F("\n"));
  }
}
//***************************************************************************************
//******************              Callback Funciton                **********************
//***************************************************************************************


void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
  uint8_t Spam_Delay = 100; // Built in Blink without delay timer preventing Serial.print SPAM

  // PrintValues(quat, Spam_Delay);
  // ChartValues(quat, Spam_Delay);
  PrintAllValues(gyro, accel, quat, Spam_Delay);
  // ChartAllValues(gyro, accel, quat, Spam_Delay);
  // PrintQuaternion(quat, Spam_Delay);
  // PrintEuler(quat, Spam_Delay);
  // PrintRealAccel(accel, quat,  Spam_Delay);
  // PrintWorldAccel(accel, quat, Spam_Delay);
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
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Serial.println(F("Start:"));
#ifdef OFFSETS
  Serial.println(F("Using Offsets"));
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).load_DMP_Image(OFFSETS); // Does it all for you

#else
  Serial.println(F(" Since no offsets are defined we aregoing to calibrate this specific MPU6050,\n"
                   " Start by having the MPU6050 placed stationary on a flat surface to get a proper accellerometer calibration\n"
                   " Place the new offsets on the #define OFFSETS... line at top of program for super quick startup\n\n"
                   " \t\t\t[Press Any Key]"));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
#endif
  mpu.on_FIFO(print_Values);
}

void loop() {
  mpu.dmp_read_fifo();// Must be in loop
}

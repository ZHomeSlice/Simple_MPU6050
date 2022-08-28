# Simple_MPU6050

Requires Simple_Wire Library also available here: https://github.com/ZHomeSlice/Simple_Wire

Features:
  <br>--Works with <b>MPU6050, MPU6500, MPU9050, MPU9250</b> (Multiple MPU's on the same i2c buss can be mixed or matched. Auto detects)
  <br>--The MPU9050, MPU9250 gives access to the Magnetometer values.
  <br>--Allows for easy access to multiple MPU's on the same i2c buss using the ado pin to isolate the desired MPU for reading.
  <br>--Allows for eDMP firmware to be set to 3 Axis  gyro only or 6 Axis Gyro with Accellerometer Quaternions (Multiple MPU's can be uniqually set).
  <br>--Interrupts are now optional to trigger i2c read of the FIFO Buffer.
  <br>--Overflow of the FIFO buffer has been resolved.
  <br>--Uses the Latest InvenSense release DMP Firmware: Embedded MotionDriver 6.12 Preconfigured to do DMP Quaternion, Accel and Gyro using the FIFO buffer.

The dmp Driver and manufacturer InvenSense provided sourch code can be found here:https://www.invensense.com/developers/software-downloads/

## Embedded MotionDriver 6.12

Embedded MotionDriver 6.12 is our first ever 9-axis solution not locked to a specific MCU.Version 6.12 is an update that includes bug fixes and new libraries. This release is supported across all ARM Mx core architectures and supports the InvenSense MPU-6000, 6050, 6500, 9150, and 9250. The release includes optimized libraries and example projects for M3 and M4 cores as well the generic ARM library for any Mx core and an additional library and project for the TI MSP430. eMD 6.1 also includes a Python client for both visualizing the sensor performance and commands for printing data. This solution will allow you to easily leverage and configure numerous features of the DMP and also benefit from dynamic features in the MPL software library. Libraries specific to IAR, Keil, and GCC are included.

Download: https://www.invensense.com/developers/software-downloads/#sla_content_45

While I reviewed their code for a full understanding of its function, this is a complete rewrite from scratch! <strike>I did use jrowberg/i2cdevlib: I2C device library collection found here https://github.com/jrowberg/i2cdevlib as a base to impliment my library. The i2cdev library is required as my library directly extends his.</strike> with the latest update I have moved away from Jeffs i2cdev library and still wish to give Jeff all the thanks he deserves. Thank you Jeff Rowberg!!!

Update 8-24-22 I added my own twowire helper library that extends the twowire livrary called Simple_Wire included in the bundle. the Simple_Wire library extends the twowire library adding easy to use functions to pass data across the i2c bus. Now the Simple_MPU6050 extends Simple_Wire which extends the twowire library.

I have fixed the extra interrupt that occured without cause.

Instead of creating a ton of functions that handle every possible configuration option. I created a key set of functions to access memory registers is specific ways then created a series of Macros that allow for easy identification of the property to be modified tied directly to the register map by the documents naming scheme.
https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

In addition to creating the maros I've preconfigured the firmware that is loaded into the MPU6050 to provide DMP features at 10ms intervals (Recently addes a feature to adjust this from 5ms to 5minutes! Set your desired rate and it will calculate the closest interval it can handle.). This Fireware is in the DMP_Image.h file ready to be inserted into the MPU6050 without any additional modifications.
just Load and go.

I've included a quick and accurate way to calibrate and even tuneup the MPU6050 and provided this withing this the Simple_MPU6050 class. The offsets are easily viewed for even faster calibration after the first couple of uses.

This compiles with the minimal implimentation at just 27% or 8824 bytes of space used on my atmega328 (UNO). This is with full DMP quaternion accel and gyro in the FIFO buffer every 10ms.

Startup is fast Setup completed in 288 Miliseconds and full calibration took about 2.5 seconds for accurate offsets to be implemented
After storing the offsets for future uses, a quick tuneup of the gyro took only 200ms

What is Quaternion See: 
http://www.cprogramming.com/tutorial/3d/quaternions.html
http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation

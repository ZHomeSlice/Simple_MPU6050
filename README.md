# Simple_MPU6050
Using the Latest  InvenSense release DMP Firmware: Embedded MotionDriver 6.12 Preconfigured to do DMP Quaternion, Accel and Gyro usint the FIFO buffer

The dmp Driver and manufacturer InvedSense provided sourch code can be found here:https://www.invensense.com/developers/software-downloads/
Embedded MotionDriver 6.12
Embedded MotionDriver 6.12 is our first ever 9-axis solution not locked to a specific MCU.Version 6.12 is an update that includes bug fixes and new libraries. This release is supported across all ARM Mx core architectures and supports the InvenSense MPU-6000, 6050, 6500, 9150, and 9250. The release includes optimized libraries and example projects for M3 and M4 cores as well the generic ARM library for any Mx core and an additional library and project for the TI MSP430. eMD 6.1 also includes a Python client for both visualizing the sensor performance and commands for printing data. This solution will allow you to easily leverage and configure numerous features of the DMP and also benefit from dynamic features in the MPL software library. Libraries specific to IAR, Keil, and GCC are included.

Download: https://www.invensense.com/developers/software-downloads/#sla_content_45

While I reviewed their code for a full understanding of its function, this is a complete rewrite from scratch! I did use jrowberg/i2cdevlib: I2C device library collection found here https://github.com/jrowberg/i2cdevlib as a base to impliment my library. The i2cdev library is required as my library directly extends his. Thank you JRowberg!!!

Instead of creating a ton of functions that handle every possible configuration option. I created a key set of functions to access memory registers is specific ways then created a series of Macros that allow for easy identification of the property to be modified tied directly to the register map by the documents naming scheme.
https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

In addition to creating the maros I've preconfigured the firmware that is loaded into the MPU6050 to provide DMP features at 10ms intervals. This Fireware is in the DMP_Image.h file ready to be inserted into the MPU6050 without any additional modifications.
just Load and go.

Additionaly if added a overflow protection feature that taps into the dreded blocking function delay(). this uses the yield() function call and prevents the fifo buffer from ever overflowing. additionally if you need to block the code for any reason a using while() or other loop you can insert the OverflowProtection() function within it to tidy up the fifo buffer as needed. So go ahead and use delay() the next time you need the latest quaternion reading it will be ther for you immediatly. no more waiting after resetting the fifo buffer for the next reading to appear. 

I've discovered a quick and accurate way to calibrate and even tuneup the MPU6050 and provided this withing this the Simple_MPU6050 class. The offsets are easily viewed for even faster calibration after the first couple of uses.

This compiles with the minimal implimentation at just 27% or 8824 bytes of space used on my atmega328 (UNO). This is with full DMP quaternion accel and gyro in the FIFO buffer every 10ms.

Startup is fast Setup completed in 288 Miliseconds and full calibration took about 2.5 seconds for accurate offsets to be implemented
After storing the offsets for future uses, a quick tuneup of the gyro took only 200ms

What is Quaternion See: 
http://www.cprogramming.com/tutorial/3d/quaternions.html
http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation



#include <Wire.h>
#include <I2Cdev.h>
#include "DMP_Image.h"
#include "Simple_MPU6050.h"
#include "MPU_ReadMacros.h"
#include "MPU_WriteMacros.h"

//#define USE_OLD_GETYAWPITCHROLL // Calculation returns different values but possibly relevant for your project Try both out
// OLD Yaw +- 180, Pitch and Roll +- 90 (Peaks at 90 deg then fall back to zero, shows Negative when pointing down pitch and left roll)
// NEW Yaw +- 180, Pitch and Roll +- 180 (Continues to 180 deg then -180 back to zero, shows Negative when pointing down pitch and left roll)

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has changed
volatile uint8_t _maxPackets;

/**
@brief      Initialization functions
*/
Simple_MPU6050::Simple_MPU6050() {

	SetAddress(MPU6050_DEFAULT_ADDRESS);
	packet_length = 28;
	/*
	packet_length += 6;//DMP_FEATURE_SEND_RAW_ACCEL
	packet_length += 6;//DMP_FEATURE_SEND_RAW_GYRO
	packet_length += 16;//DMP_FEATURE_6X_LP_QUAT
	*/
	_maxPackets = floor(512 / packet_length); // MPU 9250 can only handle 512 bytes of data in the FIFO
}

Simple_MPU6050 &  Simple_MPU6050::SetAddress(uint8_t address) {
	devAddr = address;

	return *this;
}

uint8_t  Simple_MPU6050::CheckAddress() {
	return devAddr;
}

/**
@brief      Set FIFO Callback
*/
Simple_MPU6050 & Simple_MPU6050::on_FIFO(void (*CB)(int16_t *, int16_t *, int32_t *, uint32_t *)) {
	on_FIFO_cb = CB;
	return *this; // return Pointer to this class
}

/**
@brief      Reset funnctions
*/
Simple_MPU6050 & Simple_MPU6050::reset_fifo() {
	USER_CTRL_WRITE_FIFO_RST(); //   Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::full_reset_fifo(void) { // Official way to reset fifo
	USER_CTRL_WRITE_RESET_FIFO_DMP();
	return *this;
}


/**
@brief      Start and Stop DMP int pin triggering  //   1  Enable, 0 = Disable
*/
Simple_MPU6050 & Simple_MPU6050::DMP_InterruptEnable(uint8_t Data) {
	INT_ENABLE_WRITE_RAW_DMP_INT_EN(Data);
	dmp_on = Data;
	return *this;
};


//***************************************************************************************
//**********************      Overflow Protection functions        **********************
//***************************************************************************************


/**
@brief      manages properly testing interrupt trigger from interrupt pin
*/
uint8_t Simple_MPU6050::CheckForInterrupt(void) {
	uint8_t InteruptTriggered;
	noInterrupts ();
	InteruptTriggered = mpuInterrupt;
	mpuInterrupt = false;
	interrupts ();
	return InteruptTriggered;
}

/**
@brief      During the Dreded delay() using yield() limit the fifo packets to 10 less than max packets
*/
void Simple_MPU6050::OverflowProtection(void) {
	uint32_t Timestamp ;
	Timestamp = millis();
	static uint32_t spamtimer;
	if ((Timestamp - spamtimer) >= 30) {
		spamtimer = Timestamp;
		uint16_t fifo_count;
		int8_t Packets;
		FIFO_COUNTH_READ_FIFO_CNT(&fifo_count);
		Packets = (fifo_count / packet_length) - (_maxPackets - 10); // Leaves room for 2 more readings
		if (Packets <= 0)return;
		uint8_t trash[packet_length + 1] ;
		while (0 < Packets--) {
			FIFO_READ(packet_length, trash);
		}
	}
}


//***************************************************************************************
//**********************              FIFO functions               **********************
//***************************************************************************************
/**
@brief      Reads Newest packet from fifo then on success triggers Callback routine
*/
Simple_MPU6050 & Simple_MPU6050::dmp_read_fifo(uint8_t CheckInterrupt = 1) {
	if (CheckInterrupt && !CheckForInterrupt()) return *this;
	if (!dmp_read_fifo(gyro, accel, quat, sensor_timestamp)) {
		return *this;
	}
	if (on_FIFO_cb) on_FIFO_cb(gyro, accel, quat, sensor_timestamp);
	return *this;
}
 

 int16_t Simple_MPU6050::getFIFOCount(){
	int16_t fifo_count;
	FIFO_COUNTH_READ_FIFO_CNT(&fifo_count);
	return (fifo_count);
 }


/** Get latest byte from FIFO buffer no matter how much time has passed.
 * ===                  GetCurrentFIFOPacket                    ===
 * ================================================================
 * Returns 1) when nothing special was done
 *         0) when no valid data is available
 * ================================================================ */
 int8_t Simple_MPU6050::GetCurrentFIFOPacket(uint8_t *data, uint8_t length) { // overflow proof
     int16_t fifoC;
     // This section of code is for when we allowed more than 1 packet to be acquired
     uint32_t BreakTimer = micros();
     do {
         if ((fifoC = getFIFOCount())  > length) {

             if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
                 USER_CTRL_WRITE_FIFO_RST(); // Fixes any overflow corruption
                 fifoC = 0;
                 while (!(fifoC = getFIFOCount()) && ((micros() - BreakTimer) <= (11000))); // Get Next New Packet
             } else { //We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
                 uint8_t Trash[BUFFER_LENGTH];
                 while ((fifoC = getFIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
                     fifoC = fifoC - length; // Save the last packet
                     uint16_t  RemoveBytes;
                     while (fifoC) { // fifo count will reach zero so this is safe
                         RemoveBytes = min((int)fifoC, BUFFER_LENGTH); // Buffer Length is different than the packet length this will efficiently clear the buffer
//                        getFIFOBytes(Trash, (uint8_t)RemoveBytes);
						 FIFO_READ((uint8_t)RemoveBytes, Trash);
                         fifoC -= RemoveBytes;
                     }
                 }
             }
         }
         if (!fifoC) return 0; // Called too early no data or we timed out after FIFO Reset
         // We have 1 packet
         if ((micros() - BreakTimer) > (11000)) return 0;
     } while (fifoC != length);
	 FIFO_READ((uint8_t)length, data);  //Get 1 packet
//     getFIFOBytes(data, length); //Get 1 packet
     return 1;
}

/**
@brief      Get the Newest packet from the FIFO. FIFO Buffer will be empty awaiting for next packet
*/
uint8_t Simple_MPU6050::dmp_read_fifo(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
	/* Get a packet. */
	uint8_t fifo_data[MAX_PACKET_LENGTH];
	if(GetCurrentFIFOPacket(fifo_data, packet_length)){

	//	FIFO_READ(packet_length, fifo_data);
		timestamp = micros();
	//	fifo_count -= packet_length;
		/* Parse DMP packet. */
		uint8_t ii = 0;
		quat[0] = ((int32_t)fifo_data[0] << 24) | ((int32_t)fifo_data[1] << 16) | ((int32_t)fifo_data[2] << 8) | fifo_data[3];
		quat[1] = ((int32_t)fifo_data[4] << 24) | ((int32_t)fifo_data[5] << 16) | ((int32_t)fifo_data[6] << 8) | fifo_data[7];
		quat[2] = ((int32_t)fifo_data[8] << 24) | ((int32_t)fifo_data[9] << 16) | ((int32_t)fifo_data[10] << 8) | fifo_data[11];
		quat[3] = ((int32_t)fifo_data[12] << 24) | ((int32_t)fifo_data[13] << 16) | ((int32_t)fifo_data[14] << 8) | fifo_data[15];
		ii += 16;
		accel[0] = ((int16_t)fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
		accel[1] = ((int16_t)fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
		accel[2] = ((int16_t)fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
		ii += 6;
		gyro[0] = ((int16_t)fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
		gyro[1] = ((int16_t)fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
		gyro[2] = ((int16_t)fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
		return 1;
	}
	return 0;
}

//***************************************************************************************
//**********************         i2cdev wrapper functions          **********************
//***************************************************************************************

// I did this to simplify managing all the macros found in MPU_ReadMacros.h and MPU_WriteMacros.h


// Wrappered I2Cdev read functions
Simple_MPU6050 & Simple_MPU6050::MPUi2cRead(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t *Data) {
	return MPUi2cRead( devAddr,  regAddr,  length,  bitNum,  Data);
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cRead(uint8_t AltAddress, uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t *Data) {
	if(length == 1) I2CReadCount = readBit(AltAddress,  regAddr, bitNum, Data);
	else I2CReadCount = readBits(AltAddress,  regAddr, bitNum, length,  Data);
	return *this;
}
// MPUi2cReadBytes


Simple_MPU6050 & Simple_MPU6050::MPUi2cReadByte(uint8_t regAddr,  uint8_t *Data) {
	I2CReadCount = readBytes(devAddr, regAddr,  1, Data);
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadByte(uint8_t AltAddress,uint8_t regAddr,  uint8_t *Data) {
	I2CReadCount = readBytes(AltAddress, regAddr,  1, Data);
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::MPUi2cReadBytes(uint8_t regAddr, uint8_t length, uint8_t *Data) {
	I2CReadCount = readBytes(devAddr, regAddr,  length, Data);
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data) {
	I2CReadCount = readBytes(AltAddress, regAddr,  length, Data);
	return *this;
}

// MPUi2cReadInt or Word
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadInt(uint8_t regAddr, uint16_t *Data) {
	I2CReadCount = readWords(devAddr, regAddr, 1, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadInt(uint8_t AltAddress,uint8_t regAddr, uint16_t *Data) {
	I2CReadCount = readWords(AltAddress, regAddr, 1, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}

// MPUi2cReadInts or Words
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadInts(uint8_t regAddr, uint16_t size, uint16_t *Data) {
	I2CReadCount = readWords(devAddr, regAddr, size, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size, uint16_t *Data) {
	I2CReadCount = readWords(AltAddress, regAddr, size, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}



// Wrappered I2Cdev write functions
// MPUi2cWrite
Simple_MPU6050 & Simple_MPU6050::MPUi2cWrite(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val) {
	return MPUi2cWrite(devAddr, regAddr,  length,  bitNum,  Val);
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWrite(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val) {
	if (length == 1) {
		I2CWriteStatus = writeBit(AltAddress, regAddr, bitNum, &Val); // Alters 1 bit by reading the byte making a change and storing the byte (faster than writeBits)
	}
	else if (bitNum != 255) {
		I2CWriteStatus = writeBits(AltAddress, regAddr, bitNum, length, &Val); // Alters several bits by reading the byte making a change and storing the byte
	}
	return *this;
}

// MPUi2cWriteByte
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteByte(uint8_t regAddr,  uint8_t Val) {
	I2CWriteStatus = writeBytes(devAddr, regAddr,  1, &Val); //Writes 1 or more 8 bit Bytes
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteByte(uint8_t AltAddress,uint8_t regAddr,  uint8_t Val) {
	I2CWriteStatus = writeBytes(AltAddress, regAddr,  1, &Val); //Writes 1 or more 8 bit Bytes
	return *this;
}

// MPUi2cWriteBytes
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteBytes(uint8_t regAddr, uint8_t length, uint8_t *Data) {
	I2CWriteStatus = writeBytes(devAddr, regAddr,  length, Data); //Writes 1 or more 8 bit Bytes
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data) {
	I2CWriteStatus = writeBytes(AltAddress, regAddr,  length, Data); //Writes 1 or more 8 bit Bytes
	return *this;
}

// MPUi2cWriteInt
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteInt(uint8_t regAddr,  uint16_t Val) {
	I2CWriteStatus = writeWords(devAddr, regAddr, 1,  &Val);// Writes 1 or more 16 bit integers (Word)
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteInt(uint8_t AltAddress,uint8_t regAddr,  uint16_t Val) {
	I2CWriteStatus = writeWords(AltAddress, regAddr, 1,  &Val);// Writes 1 or more 16 bit integers (Word)
	return *this;
}

// MPUi2cWriteInts
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteInts(uint8_t regAddr, uint16_t size, uint16_t *Data) {
	I2CWriteStatus = writeWords(devAddr, regAddr, size / 2,  Data);
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size, uint16_t *Data) {
	I2CWriteStatus = writeWords(AltAddress, regAddr, size / 2,  Data);
	return *this;
}




//***************************************************************************************
//**********************      Firmwaer Read Write Functions        **********************
//***************************************************************************************

/**
@brief      Read and Write to the DMP FIRMWARE memory. using these functions alters the firmware instance
*/

Simple_MPU6050 & Simple_MPU6050::read_mem(uint16_t mem_addr, uint16_t length, uint8_t *Data) {

	BANK_SEL_WRITE(mem_addr);
	DMP_MEM_READ(length, Data);
	return *this;
	} Simple_MPU6050 & Simple_MPU6050::write_mem(uint16_t  mem_addr, uint16_t  length, uint8_t *Data) {
	BANK_SEL_WRITE(mem_addr);
	DMP_MEM_WRITE(length, Data);
	return *this;
}


//***************************************************************************************
//**********************              Setup Functions              **********************
//***************************************************************************************
/**
@brief      ***EVERYTHING!*** needed to get DMP up and running!
*/
Simple_MPU6050 & Simple_MPU6050::load_DMP_Image(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_,int8_t Calibrate) {
	sax_ = ax_;
	say_ = ay_;
	saz_ = az_;
	sgx_ = gx_;
	sgy_ = gy_;
	sgz_ = gz_;
	
	load_DMP_Image();
	return *this;
}
#define CompassCheck(Cnt)   {uint8_t D; Serial.print(F("\n")); Serial.print(Cnt); Serial.print(F(" Read AKM Who am I: ")); Serial.print(I2Cdev::readBytes(0x0C,0,1,&D));Serial.print(F(" Value = 0x"));Serial.println(D);}
//#define PWR_MGMT_1_WRITE_DEVICE_RESET(...) MPUi2cWrite(0x6B, 1, 7, 1);delay(100);MPUi2cWrite(0x6A, 4, 3, 0b1111);delay(100);  //   1  Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear.
Simple_MPU6050 & Simple_MPU6050::load_DMP_Image(uint8_t CalibrateMode) {
	uint8_t val;
	TestConnection(1);
	Serial.println();
	PWR_MGMT_1_WRITE_DEVICE_RESET();			//PWR_MGMT_1:(0x6B Bit7 true) reset with 100ms delay and full SIGNAL_PATH_RESET:(0x6A Bits 3,2,1,0 True) with another 100ms delay
/* instruction suggest this sequence
	MPUi2cWriteByte(0x6B, 0x00);				
	MPUi2cWriteByte(0x6C, 0x00);				
	MPUi2cWriteByte(0x1A, 0x03);
	MPUi2cWriteByte(0x1B, 0x18);
	MPUi2cWriteByte(0x1C, 0x00);
	MPUi2cWriteByte(0x23, 0x00);
	MPUi2cWriteByte(0x38, 0x00);
	MPUi2cWriteByte(0x6A, 0x04);
	MPUi2cWriteByte(0x19, 0x04);
	if(!CalibrateMode){
		load_firmware(DMP_CODE_SIZE, dmp_memory);	// Loads the DMP image into the MPU6050 Memory
		MPUi2cWriteInt(0x70,  0x0400);				// DMP Program Start Address
	}
	MPUi2cWriteByte(0x6A, 0x40);
	MPUi2cWriteByte(0x6A, 0x04);
	MPUi2cWriteByte(0x6A, 0x80);
	MPUi2cWriteByte(0x6A, 0x08);
	MPUi2cWriteByte(0x38, 0x02);
*/



	MPUi2cWriteByte(0x6B, 0x00);
	MPUi2cWriteByte(0x6C, 0x00);
	MPUi2cWriteByte(0x1A, 0x03);
	MPUi2cWriteByte(0x1B, 0x18);
	MPUi2cWriteByte(0x1C, 0x00);
	MPUi2cWriteByte(0x23, 0x00);
	MPUi2cWriteByte(0x38, 0x00);
	MPUi2cWriteByte(0x6A, 0x04);
	MPUi2cWriteByte(0x19, 0x04);
	if(!CalibrateMode){
		load_firmware(DMP_CODE_SIZE, dmp_memory);	// Loads the DMP image into the MPU6050 Memory
		MPUi2cWriteInt(0x70,  0x0400);				// DMP Program Start Address
	}
	resetOffset();	// Load Calibration offset values into MPU
	if(CalibrateMode)return;
	PrintActiveOffsets();
	AKM_Init();
	MPUi2cWriteByte(0x6A, 0xC0);				// 1100 1100 USER_CTRL: Enable FIFO and Reset FIFO
	MPUi2cWriteByte(0x38, 0x02);				// 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on





/*
	MPUi2cWriteByte(0x6B, 0x01);				// 0000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
	MPUi2cWriteByte(0x38, 0x00);				// 0000 0000 INT_ENABLE: no Interrupt
	MPUi2cWriteByte(0x23, 0x00);				// 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
	MPUi2cWriteByte(0x1C, 0x00);				// 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
//	MPUi2cWriteByte(0x37, 0x22);				// 0010 0010 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
	MPUi2cWriteByte(0x37, 0x32);				// 0010 0010 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
	MPUi2cWriteByte(0x6B, 0x01);				// 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
	MPUi2cWriteByte(0x19, 0x04);				// 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz ( Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
	MPUi2cWriteByte(0x1A, 0x01);				// 0000 0001 CONFIG: Digital Low Pass Filter (DLPF) Configuration 188HZ  //Im betting this will be the beat
	if(!CalibrateMode){
		load_firmware(DMP_CODE_SIZE, dmp_memory);	// Loads the DMP image into the MPU6050 Memory
		MPUi2cWriteInt(0x70,  0x0400);				// DMP Program Start Address
	}
	MPUi2cWriteByte(0x1B, 0x18);				// 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
	resetOffset();	// Load Calibration offset values into MPU
	if(CalibrateMode)return;
	PrintActiveOffsets();
	AKM_Init();
	MPUi2cWriteByte(0x6A, 0xC0);				// 1100 1100 USER_CTRL: Enable FIFO and Reset FIFO
	MPUi2cWriteByte(0x38, 0x02);				// 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
	MPUi2cWrite(0x6A, 1, 2, 1);					// Reset FIFO one last time just for kicks. (MPUi2cWrite reads 0x6A first and only alters 1 byte and then saves the byte)
*/
	dmp_on = 1;
#ifdef interruptPin
	attachInterrupt(digitalPinToInterrupt(interruptPin), [] {mpuInterrupt = true;}, RISING); //NOTE: "[]{mpuInterrupt = true;}" Is a complete funciton without a name. It is handed to the callback of attachInterrupts Google: "Lambda anonymous functions"
#endif
	//These are the features the above code initialized for you by default (ToDo Allow removal of one or more Features)
	dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO |  DMP_FEATURE_SEND_CAL_GYRO; // These are Fixed into the DMP_Image and Can't be change easily at this time.
	return *this;
}
/*
Simple_MPU6050 & Simple_MPU6050::load_DMP_Image(uint8_t CalibrateMode) {
	uint8_t val;
	TestConnection(1);
	Serial.println();
	PWR_MGMT_1_WRITE_DEVICE_RESET();			//PWR_MGMT_1:(0x6B Bit7 true) reset with 100ms delay and full SIGNAL_PATH_RESET:(0x6A Bits 3,2,1,0 True) with another 100ms delay
	MPUi2cWriteByte(0x6B, 0x01);				// 0000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
	MPUi2cWriteByte(0x38, 0x00);				// 0000 0000 INT_ENABLE: no Interrupt
	MPUi2cWriteByte(0x23, 0x00);				// 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
	MPUi2cWriteByte(0x1C, 0x00);				// 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
	//	MPUi2cWriteByte(0x37, 0x22);				// 0010 0010 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
	MPUi2cWriteByte(0x37, 0x32);				// 0010 0010 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
	MPUi2cWriteByte(0x6B, 0x01);				// 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
	MPUi2cWriteByte(0x19, 0x04);				// 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz ( Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
	MPUi2cWriteByte(0x1A, 0x01);				// 0000 0001 CONFIG: Digital Low Pass Filter (DLPF) Configuration 188HZ  //Im betting this will be the beat
	if(!CalibrateMode){
		load_firmware(DMP_CODE_SIZE, dmp_memory);	// Loads the DMP image into the MPU6050 Memory
		MPUi2cWriteInt(0x70,  0x0400);				// DMP Program Start Address
	}
	MPUi2cWriteByte(0x1B, 0x18);				// 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
	resetOffset();	// Load Calibration offset values into MPU
	if(CalibrateMode)return;
	PrintActiveOffsets();
	AKM_Init();
	MPUi2cWriteByte(0x6A, 0xC0);				// 1100 1100 USER_CTRL: Enable FIFO and Reset FIFO
	MPUi2cWriteByte(0x38, 0x02);				// 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
	MPUi2cWrite(0x6A, 1, 2, 1);					// Reset FIFO one last time just for kicks. (MPUi2cWrite reads 0x6A first and only alters 1 byte and then saves the byte)
	dmp_on = 1;
	attachInterrupt(0, [] {mpuInterrupt = true;}, RISING); //NOTE: "[]{mpuInterrupt = true;}" Is a complete funciton without a name. It is handed to the callback of attachInterrupts Google: "Lambda anonymous functions"
	//These are the features the above code initialized for you by default (ToDo Allow removal of one or more Features)
	dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO |  DMP_FEATURE_SEND_CAL_GYRO; // These are Fixed into the DMP_Image and Can't be change easily at this time.
	return *this;
}
*/

/**
@brief      ***EVERYTHING!*** needed to get DMP up and running! With Calibration!!!
*/

Simple_MPU6050 & Simple_MPU6050::CalibrateMPU(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_) {
	sax_ = ax_;
	say_ = ay_;
	saz_ = az_;
	sgx_ = gx_;
	sgy_ = gy_;
	sgz_ = gz_;
	CalibrateMPU(10);
}

Simple_MPU6050 & Simple_MPU6050::CalibrateMPU(uint8_t Loops) {
	load_DMP_Image(true);
	CalibrateAccel(Loops);
	CalibrateGyro(Loops);
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	if(WhoAmI < 0x38){
		Serial.println(F("Found MPU6050 or MPU9150"));
		XA_OFFSET_H_READ_XA_OFFS(&sax_);
		YA_OFFSET_H_READ_YA_OFFS(&say_);
		ZA_OFFSET_H_READ_ZA_OFFS(&saz_);
		}else {
		Serial.println(F("Found MPU6500 or MPU9250"));
		XA_OFFSET_H_READ_0x77_XA_OFFS(&sax_);
		YA_OFFSET_H_READ_0x77_YA_OFFS(&say_);
		ZA_OFFSET_H_READ_0x77_ZA_OFFS(&saz_);
	}
	XG_OFFSET_H_READ_X_OFFS_USR(&sgx_);
	YG_OFFSET_H_READ_Y_OFFS_USR(&sgy_);
	ZG_OFFSET_H_READ_Z_OFFS_USR(&sgz_);
	return *this;
}

/**
@brief      Loads the DMP firmware.
*/
Simple_MPU6050 & Simple_MPU6050::load_firmware(uint16_t  length, const uint8_t *firmware) {
	static bool loaded = false;
	uint16_t  ii;
	uint16_t  this_write;

	uint16_t bankNum = 0;
	/* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
	#define LOAD_CHUNK  (16)
	uint8_t cur[LOAD_CHUNK], tmp[2];
	uint8_t firmware_chunk[LOAD_CHUNK];
	if (loaded)return *this; /* DMP should only be loaded once. */
	if (!firmware) *this;

	for (ii = 0; ii < length; ii += this_write) {
		this_write = min(LOAD_CHUNK, length - ii);
		int16_t x;
		uint8_t *pFirmware = (uint8_t *)&firmware[ii];
		for ( x = 0; x < this_write; x++ ) firmware_chunk[x] = pgm_read_byte_near(pFirmware + x);
		write_mem(ii, this_write, firmware_chunk);
		#ifdef DEBUG
		// this displays the firmware by retrieving it from the MPU6050 after it was written to the serial port
		read_mem(ii, this_write, cur);
		if ((ii % (16 * 16)) == 0) {
			Serial.print(F("/* bank # "));
			Serial.print(bankNum++);
			Serial.println(F(" */"));
		}
		for (uint16_t c = 0; c < this_write; c++) {
			Serial.print(F(" 0x"));
			Serial.print(Num >> 4, HEX); //Prints 0 insted of nothing when byte is less than 8
			Serial.print(Num & 0X0F, HEX); // Prints the remainder of the hex number
			Serial.print(F(","));
		}
		Serial.println();
		#endif
	}
	#ifdef DEBUG
	Serial.println();
	#endif
	loaded = true;
	return *this;
}

/**
@brief      Test to be sure we have communication to the MPU
returns 1 on success
stops or returns 0 on fail
*/
uint8_t Simple_MPU6050::TestConnection(int Stop = 1) {
	byte x;
	Wire.beginTransmission(CheckAddress());
	if(Wire.endTransmission() != 0){
		if(Stop == 2){
			Serial.print(F("\nNothing at Address: 0x"));
			Serial.println(CheckAddress(), HEX);
			return 2;
		}
	}
	Serial.print("Found MPU at: 0x");
	Serial.println(CheckAddress(), HEX);
	WHO_AM_I_READ_WHOAMI(&WhoAmI);
	Serial.print(F("WhoAmI= 0x"));
	Serial.println(WhoAmI, HEX);
	uint16_t Device = (WhoAmI < 0x38 )? 6050:6500;
	switch(Device){
		case 6500:
		if(Stop<0){
			Serial.println("MPU6500 or MPU9250");
			Serial.print(F("The connection to MPU was successful on: 0x"));
			Serial.println(WhoAmI, HEX); // Bit 0 mirrors AD0 LOW HIGH state
		}
		break;
		case 6050:
		if(Stop<0){
			Serial.println("MPU6050 or MPU9150");
			Serial.print(F("The connection to MPU was successful on: 0x"));
			Serial.println(WhoAmI, HEX); // Bit 0 mirrors AD0 LOW HIGH state
		}
		break;
		default:
		if (Stop > 0) {
			Serial.println(F("\nFailed to Connect /n check connections and reset."));
			while (1) {}
		}
		return 1;
	}

	return 0;
}


//***************************************************************************************
//********************** Gather Configuration from working MPU6050 **********************
//***************************************************************************************

// usage after configuration of the MPU6050 to your liking Get these registers to simplify MPU6050 startup
void Simple_MPU6050::view_Vital_MPU_Registers() {
	uint8_t val;
	// Reset code for your convenience:
	Serial.println(F("uint8_t val;"
	"PWR_MGMT_1_WRITE_DEVICE_RESET();" //PWR_MGMT_1: reset with 100ms delay and full SIGNAL_PATH_RESET: with another 100ms delay
	"MPUi2cWriteByte(0x6B, 0x01);/n"    // 0000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
	"MPUi2cWriteByte(0x38, 0x00);/n"    // 0000 0000 INT_ENABLE: no Interrupt
	"MPUi2cWriteByte(0x23, 0x00);/n")); // 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
	Serial.print(F("writeByte(devAddr,0x1C, 0x")); readBytes(0x68, 0x1C, 1, &val); Serial.print((val >> 4), HEX); Serial.print((val & 0x0F), HEX); Serial.println(F(");/n")); // ACCEL_CONFIG:
	Serial.print(F("writeByte(devAddr,0x37, 0x")); readBytes(0x68, 0x37, 1, &val); Serial.print((val >> 4), HEX); Serial.print((val & 0x0F), HEX); Serial.println(F(");/n")); // INT_PIN_CFG:
	Serial.print(F("writeByte(devAddr,0x6B, 0x")); readBytes(0x68, 0x6B, 1, &val); Serial.print((val >> 4), HEX); Serial.print((val & 0x0F), HEX); Serial.println(F(");/n")); // PWR_MGMT_1:
	Serial.print(F("writeByte(devAddr,0x19, 0x")); readBytes(0x68, 0x19, 1, &val); Serial.print((val >> 4), HEX); Serial.print((val & 0x0F), HEX); Serial.println(F(");/n")); // SMPLRT_DIV:
	Serial.print(F("writeByte(devAddr,0x1A, 0x")); readBytes(0x68, 0x1A, 1, &val); Serial.print((val >> 4), HEX); Serial.print((val & 0x0F), HEX); Serial.println(F(");/n")); // CONFIG:
	Serial.println(F("delay(100);/n"
	"load_firmware(DMP_CODE_SIZE, dmp_memory);/n"  // Loads the DMP image into the MPU6050 Memory
	"MPUi2cWriteInt(0x70,  0x0400);/n"              //DMP Program Start Address"
	"setOffset(OFFSETS);/n" ));                       // Load Calibration offset values into MPU
	Serial.print(F("writeByte(devAddr,0x1B, 0x")); readBytes(0x68, 0x19, 1, &val); Serial.print((val >> 4), HEX); Serial.print((val & 0x0F), HEX); Serial.println(F(");/n")); // GYRO_CONFIG:
	Serial.println(F("MPUi2cWriteByte(0x6A, 0xC0));/n"  // 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
	"MPUi2cWriteBytes(0x38, 0x02));/n"  // 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
	"attachInterrupt(0, [] { mpuInterrupt = true;}, FALLING);/n"
	"reset_fifo();/n"));
}

#define DPRINTBIN(Num) for (uint32_t t = (1UL<< (sizeof(Num)*8)-1); t; t >>= 1) Serial.write(Num  & t ? '1' : '0'); // Prints a binary number with leading zeros (Automatic Handling)
#define DPRINTHEX(Num) Serial.print(Num>>4,HEX);Serial.print(Num&0X0F,HEX);
#define ShowByte(Addr) {uint8_t val; I2Cdev::readBytes(0x68, Addr, 1, &val);  Serial.print("0x"); DPRINTHEX(Addr); Serial.print(" = 0x"); DPRINTHEX(val); Serial.print(" = 0B"); DPRINTBIN(val); Serial.println();}

void view_MPU_Startup_Registers() {
	uint8_t val;
	// Reset code for your convenience:
	ShowByte(0x23);
	ShowByte(0x1C);
	ShowByte(0x37);
	ShowByte(0x6B);
	ShowByte(0x19);
	ShowByte(0x1A);
	ShowByte(0x70);
	ShowByte(0x1B);
	ShowByte(0x6A);
	ShowByte(0x38);
}

#define A_OFFSET_H_READ_A_OFFS(Data)    MPUi2cReadInts(0x06, 3, Data)  //   X accelerometer offset cancellation
#define XG_OFFSET_H_READ_OFFS_USR(Data) MPUi2cReadInts(0x13, 3, Data)  //   Remove DC bias from the gyro sensor Step 0.0305 dps
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) Serial.print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt



Simple_MPU6050 & Simple_MPU6050::PrintActiveOffsets( ) {
	int16_t Data[3];
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	Serial.print(F("\n//              X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro\n#define OFFSETS "));
	if(WhoAmI < 0x38)	A_OFFSET_H_READ_A_OFFS(Data);
	else {
		XA_OFFSET_H_READ_0x77_XA_OFFS(Data);
		YA_OFFSET_H_READ_0x77_YA_OFFS(Data+1);
		ZA_OFFSET_H_READ_0x77_ZA_OFFS(Data+2);
	}
	printfloatx("", Data[0], 5, 0, ",  ");
	printfloatx("", Data[1], 5, 0, ",  ");
	printfloatx("", Data[2], 5, 0, ",  ");

	XG_OFFSET_H_READ_OFFS_USR(Data);
	printfloatx("", Data[0], 5, 0, ",  ");
	printfloatx("", Data[1], 5, 0, ",  ");
	printfloatx("", Data[2], 5, 0, "");

	Serial.println();
	return *this;
}

// I used the following function to retrieve a working configured DMP firmware instance for use in this program
// copy and modify this function to work elseware
/**
@brief      View the DMP firmware.
*/
bool Simple_MPU6050::view_DMP_firmware_Instance(uint16_t  length) {
	uint16_t  ii;
	uint16_t  this_read;
	uint16_t bankNum = 0;
	#define LOAD_CHUNK  (16)
	uint8_t cur[LOAD_CHUNK];
	Serial.print(F("const unsigned char dmp_memory[DMP_CODE_SIZE] PROGMEM = {\n"));
		for (ii = 0; ii < length; ii += this_read) {
			this_read = min(LOAD_CHUNK, length - ii);
			writeWords(devAddr, 0x6D, 1,  ii);
			readBytes(devAddr, 0x6F,  this_read, cur);
			if ((ii % (16 * 16)) == 0) {
				Serial.print(F("/* bank # "));
				Serial.print(bankNum++);
				Serial.println(F(" */"));
			}
			for (uint16_t c = 0; c < this_read; c++) {
				Serial.print(F(" 0x"));
				Serial.print(cur[c] >> 4, HEX); //Prints 0 insted of nothing when byte is less than 8
				Serial.print(cur[c] & 0X0F, HEX); // Prints the remainder of the hex number
				Serial.print(F(","));
			}
			Serial.println();
		}
	Serial.println(F("};"));
	return true;
}

//***************************************************************************************
//**********************           Calibration Routines            **********************
//***************************************************************************************
/**
@brief      Fully calibrate Gyro from ZERO in about 6-7 Loops 600-700 readings
*/
Simple_MPU6050 & Simple_MPU6050::CalibrateGyro(uint8_t Loops ) {
	double kP = 0.3;
	double kI = 90;
	float x;
	x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
	kP *= x;
	kI *= x;
	PID( 0x43,  kP, kI,  Loops);
	//Serial.println();
	return *this;
}

/**
@brief      Fully calibrate Accel from ZERO in about 6-7 Loops 600-700 readings
*/

Simple_MPU6050 & Simple_MPU6050::CalibrateAccel(uint8_t Loops ) {
	float kP = 0.3;
	float kI = 90;
	float x;
	x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
	kP *= x;
	kI *= x;
	PID( 0x3B, kP, kI,  Loops);
	//Serial.println();
	return *this;
}

#define SPrint(Data) Serial.print(Data);Serial.print(", ");
Simple_MPU6050 & Simple_MPU6050::PID(uint8_t ReadAddress, float kP,float kI, uint8_t Loops) {
	uint8_t SaveAddress = (ReadAddress == 0x3B)?((WhoAmI < 0x38 )? 0x06:0x77):0x13;

	int16_t  Data;
	float Reading;
	int16_t BitZero[3];
	uint8_t shift =(SaveAddress == 0x77)?3:2;
	float Error, PTerm, ITerm[3];
	int16_t eSample;
	uint32_t eSum ;
	Serial.write('>');
	for (int i = 0; i < 3; i++) {
		I2Cdev::readWords(devAddr, SaveAddress + (i * shift), 1, (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
		Reading = Data;
		if(SaveAddress != 0x13){
			BitZero[i] = Data & 1;										 // Capture Bit Zero to properly handle Accelerometer calibration
			ITerm[i] = ((float)Reading) * 8;
			} else {
			ITerm[i] = Reading * 4;
		}
	}
	for (int L = 0; L < Loops; L++) {
		eSample = 0;
		for (int c = 0; c < 100; c++) {// 100 PI Calculations
			eSum = 0;
			for (int i = 0; i < 3; i++) {
				I2Cdev::readWords(devAddr, ReadAddress + (i * 2), 1, (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
				Reading = Data;
				if ((ReadAddress == 0x3B)&&(i == 2)) Reading -= 16384;	//remove Gravity
				Error = -Reading;
				eSum += abs(Reading);
				PTerm = kP * Error;
				ITerm[i] += (Error * 0.001) * kI;				// Integral term 1000 Calculations a second = 0.001
				if(SaveAddress != 0x13){
					Data = round((PTerm + ITerm[i] ) / 8);		//Compute PID Output
					Data = ((Data)&0xFFFE) |BitZero[i];			// Insert Bit0 Saved at beginning
				} else Data = round((PTerm + ITerm[i] ) / 4);	//Compute PID Output
				I2Cdev::writeWords(devAddr, SaveAddress + (i * shift), 1, (uint16_t *)&Data);
			}
			if((c == 99) && eSum > 1000){						// Error is still to great to continue
				c = 0;
				Serial.write('*');
			}
			if((eSum * ((ReadAddress == 0x3B)?.05: 1)) < 5) eSample++;	// Successfully found offsets prepare to  advance
			if((eSum < 100) && (c > 10) && (eSample >= 10)) break;		// Advance to next Loop
			delay(1);
		}
		Serial.write('.');
		kP *= .75;
		kI *= .75;
		for (int i = 0; i < 3; i++){
			if(SaveAddress != 0x13) {
				Data = round((ITerm[i] ) / 8);		//Compute PID Output
				Data = ((Data)&0xFFFE) |BitZero[i];	// Insert Bit0 Saved at beginning
			} else Data = round((ITerm[i]) / 4);
			I2Cdev::writeWords(devAddr, SaveAddress + (i * shift), 1, (uint16_t *)&Data);
		}
	}
	SIGNAL_PATH_FULL_RESET_WRITE_RESET();
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::resetOffset() {
	Serial.println("Reset Offsets");
	setOffset( sax_,  say_,  saz_,  sgx_,  sgy_,  sgz_);
	return *this; // return Pointer to this class
}

Simple_MPU6050 & Simple_MPU6050::setOffset(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_) {
	Serial.println("set Offsets");
	sax_ = ax_;
	say_ = ay_;
	saz_ = az_;
	sgx_ = gx_;
	sgy_ = gy_;
	sgz_ = gz_;
	
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	if(WhoAmI < 0x38){
		XA_OFFSET_H_WRITE_XA_OFFS(ax_);
		YA_OFFSET_H_WRITE_YA_OFFS(ay_);
		ZA_OFFSET_H_WRITE_ZA_OFFS(az_);
		} else {
		XA_OFFSET_H_WRITE_0x77_XA_OFFS(ax_);
		YA_OFFSET_H_WRITE_0x77_YA_OFFS(ay_);
		ZA_OFFSET_H_WRITE_0x77_ZA_OFFS(az_);
	}

	XG_OFFSET_H_WRITE_X_OFFS_USR(gx_);
	YG_OFFSET_H_WRITE_Y_OFFS_USR(gy_);
	ZG_OFFSET_H_WRITE_Z_OFFS_USR(gz_);
	return *this;
}
//***************************************************************************************
//**********************          Helper Math Functions            **********************
//***************************************************************************************

Simple_MPU6050 &  Simple_MPU6050::SetAccel(VectorInt16 *v, int16_t *accel) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	v -> x = accel[0];
	v -> y = accel[1];
	v -> z = accel[2];
	return *this;
}

Simple_MPU6050 &  Simple_MPU6050::GetQuaternion(Quaternion *q, const int32_t* qI) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	q -> w = (float)(qI[0] >> 16) / 16384.0f;
	q -> x = (float)(qI[1] >> 16) / 16384.0f;
	q -> y = (float)(qI[2] >> 16) / 16384.0f;
	q -> z = (float)(qI[3] >> 16) / 16384.0f;
	return *this;
}

Simple_MPU6050 &  Simple_MPU6050::GetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
	// get rid of the gravity component (+1g = +16384 in standard DMP FIFO packet, sensitivity is +-2g)
	v -> x = vRaw -> x - gravity -> x * 16384;
	v -> y = vRaw -> y - gravity -> y * 16384;
	v -> z = vRaw -> z - gravity -> z * 16384;
	return *this;
}

Simple_MPU6050 &  Simple_MPU6050::GetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
	// rotate measured 3D acceleration vector into original state
	// frame of reference based on orientation quaternion
	memcpy(v, vReal, sizeof(VectorInt16));
	v -> rotate(q);
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::GetGravity(VectorFloat *v, Quaternion *q) {
	v -> x = 2 * (q -> x * q -> z - q -> w * q -> y);
	v -> y = 2 * (q -> w * q -> x + q -> y * q -> z);
	v -> z = q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z;
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::GetEuler(float *data, Quaternion *q) {
	data[0] = atan2(2 * q -> x * q -> y - 2 * q -> w * q -> z, 2 * q -> w * q -> w + 2 * q -> x * q -> x - 1); // psi
	data[1] = -asin(2 * q -> x * q -> z + 2 * q -> w * q -> y);                      // theta
	data[2] = atan2(2 * q -> y * q -> z - 2 * q -> w * q -> x, 2 * q -> w * q -> w + 2 * q -> z * q -> z - 1); // phi
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
	#ifdef USE_OLD_GETYAWPITCHROLL
	// yaw: (about Z axis)
	data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
	#else 
	// yaw: (about Z axis)
	data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan2(gravity -> y , gravity -> z);
	if (gravity -> z < 0) {
		if(data[1] > 0) {
			data[1] = PI - data[1];
			} else {
			data[1] = -PI - data[1];
		}
	}
	#endif
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::GetYawPitchRoll(float *data, Quaternion *q) {
	data[0] = atan2(2.0f * (q -> x*q -> y + q -> w * q -> z), q -> w * q -> w + q -> x * q -> x - q -> y * q -> y - q -> z * q -> z);
	data[1] = -asin(2.0f * (q -> x * q -> z - q -> w * q -> y));
	data[2]  = atan2(2.0f * (q -> w * q -> x + q -> y * q -> z), q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z);
	return *this;
}




Simple_MPU6050 & Simple_MPU6050::ConvertToDegrees(float*ypr, float*xyz) {
	//const float radians_to_degrees = 180.0 / M_PI;
	for (int i = 0; i < 3; i++) {
		xyz[i] = ypr[i] * radians_to_degrees;

	}
	if ( xyz[0] < -180 ) xyz[0] += 360;
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::ConvertToRadians( float*xyz, float*ypr) {
	const float degrees_to_radians = M_PI / 180.0;
	for (int i = 0; i < 3; i++) ypr[i] = xyz[i] * degrees_to_radians;
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::MagneticNorth(float*data, VectorInt16 *v, Quaternion*q ) {
	float ax = v->x, ay = v->y, az = v->z;
	float q1 = q->w, q2 = q->x, q3 = q->y, q4 = q->z;   // short name local variable for readability
	float mx = mag[0], my = mag[1], mz = mag[2];
	float hx, hy, bx, bz,vx,vy,vz,wx,wy,wz,ex,ey,ez;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;


	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	data[0] = wx;
	data[1] = wy;
	data[2] = wz;
	data[3] = ex;
	data[4] = ey;
	data[5] = ez;
	return *this;
}



//***************************************************************************************
//**********************      Helper Magnetometer Functtions       **********************
//***************************************************************************************
#define PRINTHEX(Num) print(Num>>4,HEX);Serial.print(Num&0X0F,HEX);
Simple_MPU6050 & Simple_MPU6050::I2CScanner(){
	Serial.println(F("Scanning for Addresses on the i2c Buss:"));
	for (int x = 1;x < 128;x++){
		Serial.print("0x");
		if(!(x = FindAddress(x,128))) break;
		Serial.PRINTHEX(x);
		Serial.println();
	}
	return *this;
}




uint8_t Simple_MPU6050::FindAddress(uint8_t Address,uint8_t Limit){
	do {
		Wire.beginTransmission(Address);
		if (Wire.endTransmission() == 0)
		return Address;
	} while (Limit != Address++);// using rollover ate 255 to allow for any number on Limit
	return 0;
}

Simple_MPU6050 & Simple_MPU6050::AKM_Init(){
	
	INT_PIN_CFG_WRITE_BYPASS_EN(1);
	akm_addr = FindAddress(0x0C,0x0F);
	AKM_WHOAMI_READ(akm_addr,&akm_WhoAmI);
	//viewMagRegisters();
	if(!ReadStatus()){
		Serial.print(F("Failed to Find Magnetometer"));
		INT_PIN_CFG_WRITE_BYPASS_EN(0);
		akm_addr = 0;
		return *this;
	}
	#define SPRINTHEX(Num) print(Num>>4,HEX);Serial.print(Num&0X0F,HEX);  // Prints 2 Digits even when Value < 16
	Serial.print(F("Found Magnetometer at Address: 0x"));
	Serial.SPRINTHEX(akm_addr);
	Serial.print(" With WhoAmI = 0x");
	Serial.SPRINTHEX(akm_WhoAmI);
	Serial.println();
	AKM_SOFT_RESET(akm_addr);
	delay(100);
	if(!HIGH_SENS) mRes = 10.*4912./8190.; // Proper scale to return milliGauss MFS_14BITS
	else mRes = 10.*4912./32760.0; // Proper scale to return milliGauss MFS_16BITS
	uint8_t AKMData[3];



	// Directly access the Magnetometer:
	AKM_CNTL_WRITE_POWER_DOWN(akm_addr,0);
	delay(10);
	AKM_CNTL_WRITE_FUSE_ROM_ACCESS(akm_addr,0);
	delay(10);
	AKM_ASAXYZ_READ_SENS_ADJ_XYZ(akm_addr,AKMData);
	mag_sens_adj_F[0] =  (float)(AKMData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
	mag_sens_adj_F[1] =  (float)(AKMData[1] - 128) / 256. + 1.;
	mag_sens_adj_F[2] =  (float)(AKMData[2] - 128) / 256. + 1.;


	AKM_CNTL_WRITE_POWER_DOWN(akm_addr,0);

	Serial.println("mag_sens_adj 2:");
	Serial.print(mag_sens_adj_F[0]);
	Serial.print(", ");
	Serial.print(mag_sens_adj_F[1]);
	Serial.print(", ");
	Serial.println(mag_sens_adj_F[2]);
	
	delay(10);
	uint8_t DirectAccessToMag = 0;
	if(!DirectAccessToMag){

		#define DPRINTBINL(Num) for (int i=0;i<(sizeof(Num)*8);i++) Serial.write(((Num >> i) & 1) == 1 ? '1' : '0'); // Prints a binary number with following Placeholder Zeros  (Automatic Handling)
		#define DPRINTBINLX(S,Num,nl) Serial.print(F(S)); for (int i=0;i<(sizeof(Num)*8);i++) Serial.write(((Num >> i) & 1) == 1 ? '1' : '0'); if(nl)Serial.println(); // Prints a binary number with following Placeholder Zeros  (Automatic Handling)
		int8_t Num;
		// Configure the magnetometer for continuous read and highest resolution
		// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
		// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
		//writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // 0x0A // Set magnetometer data resolution and sample ODR
		//I2Cdev::writeByte(0x0C,0x0A ,  (HIGH_SENS & 1) << 4 | 0x01 );// 16bit single measurement mode
		AKM_CNTL_WRITE_SINGLE_MEAS_MODE(akm_addr,HIGH_SENS);
		INT_PIN_CFG_WRITE_BYPASS_EN(0);

		// Configure MPU I2C Secondary Bus as a Master Bus at 400khz
		//writeByte(MPU9250_ADDRESS, I2C_MST_CTRL		  0x24, 0x1D	0B 0001 1101);       // I2C configuration STOP after each transaction, master I2C bus at 400 KHz
		I2C_MST_CTRL_WRITE_MULT_MST_EN(1);
		I2C_MST_CTRL_WRITE_I2C_MST_P_NSR(1);
		I2C_MST_CTRL_WRITE_I2C_MST_CLK_400();
		I2C_MST_CTRL_READ_ALL(&Num);
		DPRINTBINLX("I2C_MST_CTRL_ = 0B", Num,1);
		//writeByte(MPU9250_ADDRESS, I2C_MST_DELAY_CTRL 0x67, 0x81	0B 1000 0001) ; // Use blocking data retreival and enable delay for mag sample rate mismatch
		I2C_MST_DELAY_CTRL_WRITE_DELAY_ES_SHADOW(1);
		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV1_DLY_EN(1);
		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV0_DLY_EN(1);
		I2C_MST_DELAY_CTRL_READ_ALL(&Num);
		DPRINTBINLX("I2C_MST_DELAY_CTRL_ = 0B", Num,1);

		//writeByte(MPU9250_ADDRESS, I2C_SLV4_CTRL	  0x34, 0x01	0B 000 0001);      // Delay mag data retrieval to once every other accel/gyro data sample
		I2C_SLV4_CTRL_WRITE_I2C_MST_DLY(1); // (1+I2C_MST_DLY) // Delay mag data retrieval to once every other accel/gyro data sample
		I2C_SLV4_CTRL_READ_ALL(&Num);
		DPRINTBINLX("I2C_SLV4_CTRL_ = 0B", Num,1);

		// Slave 0 Retrieves the data
		I2C_SLV0_ADDR_WRITE_I2C_SLV0_RNW(1);			//Slave 0 reads from AKM data registers.
		I2C_SLV0_ADDR_WRITE_I2C_ID_0(akm_addr);			//compass address

		I2C_SLV0_ADDR_READ_ALL(&Num);
		DPRINTBINLX("I2C_SLV0_ADDR_ = 0B", Num,1);


		I2C_SLV0_REG_WRITE_I2C_SLV0_REG(AKM_XOUT_L);	//0x02 Compass reads start at this register.

		I2C_SLV0_REG_READ_I2C_SLV0_REG(&Num);
		DPRINTBINLX("I2C_SLV0_REG_READ_I2C_SLV0_REG = 0B", Num,1);


		I2C_SLV0_CTRL_WRITE_I2C_SLV0_EN(1);				// Enable slave 0,
		I2C_SLV0_CTRL_WRITE_I2C_SLV0_LENG(7);			// 8-byte reads.

		I2C_SLV0_CTRL_READ_ALL(&Num);
		DPRINTBINLX("I2C_SLV0_CTRL_ = 0B", Num,1);


		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV0_DLY_EN(1); //Trigger slave 0 and slave 1 actions at each sample.

		I2C_MST_DELAY_CTRL_READ_ALL(&Num);
		DPRINTBINLX("I2C_MST_DELAY_CTRL_ = 0B", Num,1);


		// Slave 1 Asks for more data to be retrieved
		I2C_SLV1_ADDR_WRITE_I2C_SLV1_RNW(0);			//Slave 1 reads from AKM data registers.
		I2C_SLV1_ADDR_WRITE_I2C_ID_1(akm_addr);			//compass address

		I2C_SLV1_ADDR_READ_ALL(&Num);
		DPRINTBINLX("I2C_SLV1_ADDR_ = 0B", Num,1);


		I2C_SLV1_REG_WRITE_I2C_SLV1_REG(AKM_REG_CNTL);	//0x0A AKM measurement mode register

		I2C_SLV1_REG_READ_I2C_SLV1_REG(&Num);
		DPRINTBINLX("I2C_SLV1_REG_READ_I2C_SLV1_REG = 0B", Num,1);


		I2C_SLV1_CTRL_WRITE_I2C_SLV1_EN(1);				//Enable slave 1
		I2C_SLV1_CTRL_WRITE_I2C_SLV1_LENG(1);			//1-byte writes.

		I2C_SLV1_CTRL_READ_ALL(&Num);
		DPRINTBINLX("I2C_SLV1_CTRL_ = 0B", Num,1);


		I2C_SLV1_DO_WRITE_I2C_SLV1_DO(AKM_SINGLE_MEASUREMENT|(HIGH_SENS<<4)); //Set slave 1 data.

		I2C_SLV1_DO_READ_I2C_SLV1_DO(&Num);
		DPRINTBINLX("I2C_SLV1_DO_ = 0B", Num,1);


		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV1_DLY_EN(1);

		I2C_MST_DELAY_CTRL_READ_ALL(&Num);
		DPRINTBINLX("I2C_MST_DELAY_CTRL_READ_ALL = 0B", Num,1);




		//	AKM_SOFT_RESET(akm_addr);
		delay(100);
		//AKM_CNTL_WRITE_CONT_MEAS_MODE2(akm_addr,1);
		/*
		I2Cdev::writeByte(0x0C,0x0A ,  1 << 4 | 0x01 );// 16bit single measurement mode
		while(1){viewMagRegisters();}
		while(1){
		static unsigned long _ETimer;
		if ( millis() - _ETimer >= (100)) {
		_ETimer += (100);
		//viewMagRegisters();
		AKM_CNTL_WRITE_SINGLE_MEAS_MODE(akm_addr,1);
		}
		}
		delay(1);
		*/
		//INT_PIN_CFG_WRITE_BYPASS_EN(0);

		/*
		I2C_MST_CTRL_WRITE_WAIT_FOR_ES(1);			// Set up master mode, master clock, and ES bit.

		// Slave 0 Configuration
		// Slave 0 reads registers  0x02 ~ 0x0A
		// 0x02			Status 1
		// 0x03 ~ 0x08  Measurement Data
		// 0x09			Status 2
		// This information is available in MPU Registers:
		I2C_SLV0_ADDR_WRITE_I2C_SLV0_RNW(1);			//Slave 0 reads from AKM data registers.
		I2C_SLV0_ADDR_WRITE_I2C_ID_0(akm_addr);			//compass address
		I2C_SLV0_REG_WRITE_I2C_SLV0_REG(AKM_REG_ST1);	//0x02 Compass reads start at this register.
		I2C_SLV0_CTRL_WRITE_I2C_SLV0_EN(1);				// Enable slave 0,
		I2C_SLV0_CTRL_WRITE_I2C_SLV0_LENG(8);			// 8-byte reads.

		// Slave 1 Configuration
		// This sets the trigger to tell the AKM to get another set of data
		I2C_SLV1_ADDR_WRITE_I2C_SLV1_RNW(0);			//Slave 0 reads from AKM data registers.
		I2C_SLV1_ADDR_WRITE_I2C_ID_1(akm_addr);			//compass address
		I2C_SLV1_REG_WRITE_I2C_SLV1_REG(AKM_REG_CNTL);	//0x0A AKM measurement mode register
		I2C_SLV1_CTRL_WRITE_I2C_SLV1_EN(1);				//Enable slave 1
		I2C_SLV1_CTRL_WRITE_I2C_SLV1_LENG(1);			//1-byte writes.
		I2C_SLV1_DO_WRITE_I2C_SLV1_DO(AKM_SINGLE_MEASUREMENT|HIGH_SENS); //Set slave 1 data.

		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV0_DLY_EN(1); //Trigger slave 0 and slave 1 actions at each sample.
		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV1_DLY_EN(1);

		if(WhoAmI < 0x38 )SELF_TEST_Y_GYRO_WRITE_I2C_MST_VDDIO(1); //For the MPU9150, the auxiliary I2C bus needs to be set to VDD.
		*/

		// uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		// readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		//	writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
		//	writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AK8963_XOUT_L);             // I2C slave 0 register address from where to begin data transfer
		//	writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x87);                     // Enable I2C and read 7 bytes



		//	mpu_set_bypass(1);
		//	AKM_CNTL_WRITE_CONT_MEAS_MODE2(akm_addr,HIGH_SENS);
	}
	delay(10);
	return *this;
}


Simple_MPU6050 & Simple_MPU6050::mpu_set_bypass(unsigned char bypass_on){
	if (bypass_on) {
		USER_CTRL_WRITE_I2C_MST_EN(0);
		delay(3);
		INT_PIN_CFG_WRITE_ACTL(0);				//7
		INT_PIN_CFG_WRITE_OPEN(0);				//6
		INT_PIN_CFG_WRITE_FSYNC_INT_MODE_EN(0);	//2
		INT_PIN_CFG_WRITE_BYPASS_EN(1);			//1
		} else {
		/* Enable I2C master mode if compass is being used. */
		USER_CTRL_WRITE_I2C_MST_EN(akm_addr>0);
		delay(3);
		INT_PIN_CFG_WRITE_ACTL(0);				//7
		INT_PIN_CFG_WRITE_OPEN(0);				//6
		INT_PIN_CFG_WRITE_FSYNC_INT_MODE_EN(0);	//2
		INT_PIN_CFG_WRITE_BYPASS_EN(0);			//1
	}
	return *this;
}

#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt
Simple_MPU6050 & Simple_MPU6050::readMagData(){
	//read mag
	static unsigned long _ETimer;
	if ( millis() - _ETimer >= (1000)) {
		_ETimer += (1000);
		AKM_CNTL_WRITE_SINGLE_MEAS_MODE(0x0C,1);
		delay(10);
		I2Cdev::readBytes(0x0C, 0x03, 6, buffer);
	}
	AKM_CNTL_WRITE_SINGLE_MEAS_MODE(0x0C,1);
	delay(100);
	int16_t RawCompass[3];
	AKM_DATA_READ_RAW_COMPASS_SWAP(akm_addr,RawCompass);
	mag[0] = (float)RawCompass[0];
	mag[1] = (float)RawCompass[1];
	mag[2] = (float)RawCompass[2];
	AKM_CNTL_WRITE_SINGLE_MEAS_MODE(0x0C,1);
//	MPU9250.readMagData(magCount);  // Read the x/y/z adc values
	      
	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	mRes = (mRes!=0)?mRes:1;
	mag[0] = (float)mag[0]*mRes*mag_sens_adj_F[0] - mag_bias[0];  // get actual magnetometer value, this depends on scale being set
	mag[1] = (float)mag[1]*mRes*mag_sens_adj_F[1] - mag_bias[1];
	mag[2] = (float)mag[2]*mRes*mag_sens_adj_F[2] - mag_bias[2];

	if(mag_scale[0]!=0) mag[0] *= mag_scale[0];
	if(mag_scale[1]!=0) mag[1] *= mag_scale[1];
	if(mag_scale[2]!=0) mag[2] *= mag_scale[2];


	// Normalise magnetometer measurement
	float nmag = sqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	if (nmag == 0.0f) return; // handle NaN
	nmag = 1.0f / nmag;        // use reciprocal for division
	mag[0] *= nmag;
	mag[1] *= nmag;
	mag[2] *= nmag;




	  // Calculate heading when the magnetometer is level, then correct for signs of axis.
	  // Atan2() automatically check the correct formula taking care of the quadrant you are in
	//  float heading = atan2(mag[1], mag[0]) * radians_to_degrees;

	  // Once you have your heading, you must then add your 'Declination Angle',
	  // which is the 'Error' of the magnetic field in your location. Mine is 0.0404
	  // Find yours here: http://www.magnetic-declination.com/
	  
	  // Output the data via the serial port.
/*
	Serial.printfloatx(F("mag xyz")     , mag[0],  15, 3, F(",   "));
	Serial.printfloatx(F("")            , mag[1],  15, 3, F(",   "));
	Serial.printfloatx(F("")            , mag[2],  15, 3, F(",   "));
	Serial.printfloatx(F("Deg")            , heading,  15, 3, F("    \t\t"));
	for (int i=0; i<abs(heading); i++)	Serial.print("*");
	Serial.print("\n");
*/
	//Serial.printfloatx(F("mag xyz")     , xmag[0],  15, 3, F(",   "));
	//Serial.printfloatx(F("")            , xmag[1],  15, 3, F(",   "));
	//Serial.printfloatx(F("")            , xmag[2],  15, 3, F("\t"));
	/*
	uint8_t rawData[6];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	Serial.print("$");
	if(AKM_ST1_READ_DATA_READY(akm_addr, &TVal).TVal){	// wait for magnetometer data ready bit to be set
	Serial.print("^");
	//  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
	//		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
	//		uint8_t c = rawData[6]; // End data read by reading ST2 register
	if(!AKM_ST2_READ_SENSOR_OVERFLOW(akm_addr,&TVal).TVal){// Check if magnetic sensor overflow set, if not then report data}
	Serial.print("~");
	//		if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
	//			AKM_DATA_READ_RAW_COMPASS(akm_addr,magCount);
	//			AKM_DATA_READ_RAW_COMPASS_SWAP(akm_addr,magCount);
	AKM_DATA_READ_RAW_COMPASS_DATA(akm_addr,rawData); // Read the six raw data
	magCount[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
	magCount[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
	magCount[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
	
	mag[0] = (float)magCount[0]*mRes*mag_sens_adj[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
	mag[1] = (float)magCount[1]*mRes*mag_sens_adj[1] - magBias[1];
	mag[2] = (float)magCount[2]*mRes*mag_sens_adj[2] - magBias[2];
	}
	}
	//AKM_CNTL_WRITE_SINGLE_MEAS_MODE(akm_addr,HIGH_SENS);// Prep for next reading
	*/
	return *this;
}


#define DPRINTBIN(Num) for (uint32_t t = (1UL<< (sizeof(Num)*8)-1); t; t >>= 1) Serial.write(Num  & t ? '1' : '0'); // Prints a binary number with leading zeros (Automatic Handling)
#define DPRINTHEX(Num) Serial.print(Num>>4,HEX);Serial.print(Num&0X0F,HEX);
#define ShowByte(Addr) {uint8_t val; I2Cdev::readBytes(0x68, Addr, 1, &val);  Serial.print("0x"); DPRINTHEX(Addr); Serial.print(" = 0x"); DPRINTHEX(val); Serial.print(" = 0B"); DPRINTBIN(val); Serial.println();}
#define ShowValue(Name, FunctionD) FunctionD; Serial.print(Name); Serial.print(" = 0x"); DPRINTHEX(D); Serial.print(" = 0B"); DPRINTBIN(D); Serial.println();
// Work in Progress:
Simple_MPU6050 & Simple_MPU6050::readMagDataThroughMPU(){
	//read mag
	uint8_t D;
	uint8_t rawData[8];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	/*
	//  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
	// I2C_SLV0_ADDR_WRITE_I2C_SLV0_RNW(1);
	// I2C_SLV0_ADDR_WRITE_I2C_ID_0(0x0C);
	ShowValue("I2C_MST_CTRL_READ_MULT_MST_EN", I2C_MST_CTRL_READ_MULT_MST_EN(D));
	ShowValue("I2C_MST_CTRL_READ_WAIT_FOR_ES", I2C_MST_CTRL_READ_WAIT_FOR_ES(D));
	ShowValue("I2C_MST_CTRL_READ_SLV_3_FIFO_EN", I2C_MST_CTRL_READ_SLV_3_FIFO_EN(D));
	ShowValue("I2C_MST_CTRL_READ_I2C_MST_P_NSR", I2C_MST_CTRL_READ_I2C_MST_P_NSR(D));
	ShowValue("I2C_MST_CTRL_READ_I2C_MST_CLK", I2C_MST_CTRL_READ_I2C_MST_CLK(D));

	//  writeByte(MPU9250_ADDRESS, I2C_SLV0_ADDR, 0x0C | 0x80);    // Set the I2C slave address of AK8963 and set for read.
	// I2C_SLV0_ADDR_WRITE_I2C_SLV0_RNW(1);
	// I2C_SLV0_ADDR_WRITE_I2C_ID_0(0x0C);
	ShowValue("I2C_SLV0_ADDR_READ_I2C_SLV0_RNW", I2C_SLV0_ADDR_READ_I2C_SLV0_RNW(D));
	ShowValue("I2C_SLV0_ADDR_READ_I2C_ID_0", I2C_SLV0_ADDR_READ_I2C_ID_0(D));


	//  writeByte(MPU9250_ADDRESS, I2C_SLV0_REG, AKM_REG_ST1);             // I2C slave 0 register address from where to begin data transfer
	//  I2C_SLV0_REG_WRITE_I2C_SLV0_REG(AKM_REG_ST1);
	ShowValue("I2C_SLV0_REG_READ_I2C_SLV0_REG", I2C_SLV0_REG_READ_I2C_SLV0_REG(D));

	//  writeByte(MPU9250_ADDRESS, I2C_SLV0_CTRL, 0x88);                     // Enable I2C and read 7 bytes
	// I2C_SLV0_CTRL_WRITE_I2C_SLV0_EN(1);
	// I2C_SLV0_CTRL_WRITE_I2C_SLV0_LENG(8);
	ShowValue("I2C_SLV0_CTRL_READ_I2C_SLV0_EN", I2C_SLV0_CTRL_READ_I2C_SLV0_EN(D));
	ShowValue("I2C_SLV0_CTRL_READ_I2C_SLV0_BYTE_SW", I2C_SLV0_CTRL_READ_I2C_SLV0_BYTE_SW(D));
	ShowValue("I2C_SLV0_CTRL_READ_I2C_SLV0_REG_DIS", I2C_SLV0_CTRL_READ_I2C_SLV0_REG_DIS(D));
	ShowValue("I2C_SLV0_CTRL_READ_I2C_SLV0_GRP", I2C_SLV0_CTRL_READ_I2C_SLV0_GRP(D));
	ShowValue("I2C_SLV0_CTRL_READ_I2C_SLV0_LENG", I2C_SLV0_CTRL_READ_I2C_SLV0_LENG(D));
	*/
	delay(2);
	//readBytes(MPU9250_ADDRESS, EXT_SENS_DATA_00, 8, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
	EXT_SENS_DATA_READ_LENGTH(8,buffer);
	mag[0] = (float)((((int16_t)buffer[1]) << 8) | buffer[0]);
	mag[1] = (float)((((int16_t)buffer[3]) << 8) | buffer[2]);
	mag[2] = (float)((((int16_t)buffer[5]) << 8) | buffer[4]);
	
	#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) Serial.print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt
	printfloatx(F("mag xyz")     , mag[0],  15, 3, F(",   "));
	printfloatx(F("")            , mag[1],  15, 3, F(",   "));
	printfloatx(F("")            , mag[2],  15, 3, F("\t"));
	//I2Cdev::writeByte(0x0C, 0x0A, 0x01); //enable the magnetometer
	return *this;
}





#define spamtimer(t)										for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
Simple_MPU6050 & Simple_MPU6050::magcalMPU(){
//https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
//https://appelsiini.net/2018/calibrate-magnetometer/
	uint8_t DelayCnt = 5;
	uint8_t Ready;
	uint16_t ii = 0, sample_count = 0, PCount = 0;
//	int32_t mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767};
	int16_t mag_min[3] = {32767, 32767, 32767};
	int16_t mag_temp[3] = {0, 0, 0};
	int16_t RawCompass[3];
	Serial.println(F("Mag Calibration: Wave device in a figure eight until done! @ 2Minutes"));
	delay(1000);
	Serial.println(F("Ready"));
	delay(1000);
	Serial.println(F("Set!"));
	delay(2000);
	Serial.println(F("GO! GO! GO!"));
	//AKM_CNTL_WRITE_CONT_MEAS_MODE2(akm_addr,1); // only works with mpu9250 por mpu9255
	AKM_CNTL_WRITE_SINGLE_MEAS_MODE(akm_addr,1);

	
	static unsigned long _ETimer;
	_ETimer = millis();
	while ( millis() - _ETimer <= (60000 * 2)) {	// shoot for ~fifteen seconds of mag data
		delay(DelayCnt); // Lets wait instead of bugging the MPU
		while(DelayCnt < 135){
			delay(1);
			AKM_ST1_READ_DATA_READY(akm_addr,&Ready); // data ready
			if(Ready)break;
			DelayCnt++;
		}
		sample_count++;  
		AKM_DATA_READ_RAW_COMPASS_SWAP(akm_addr,mag_temp);// get data
		AKM_CNTL_WRITE_SINGLE_MEAS_MODE(akm_addr,1); // Request next reading
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		spamtimer(1000){
			Serial.print((++PCount % 30)?"!":"\n");
			DelayCnt--;
		};
	}
	Serial.print(F("MS Reading Delay = "));
	Serial.println(DelayCnt +1);
	Serial.print(F("sample Count = "));
	Serial.println(sample_count);

	//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	// Get hard iron correction
	mag_bias[0]  = (float)(mag_max[0] + mag_min[0])/2.0;  // get average x mag bias in counts
	mag_bias[1]  = (float)(mag_max[1] + mag_min[1])/2.0;  // get average y mag bias in counts
	mag_bias[2]  = (float)(mag_max[2] + mag_min[2])/2.0;  // get average z mag bias in counts

		
	mag_bias[0] =  mag_bias[0] * mRes * mag_sens_adj_F[0];  // save mag biases in G for main program
	mag_bias[1] =  mag_bias[1] * mRes * mag_sens_adj_F[1];
	mag_bias[2] =  mag_bias[2] * mRes * mag_sens_adj_F[2];
	
	// Get soft iron correction estimate
	mag_scale[0]  = (float)(mag_max[0] - mag_min[0])/2.0;  // get average x mag bias in counts
	mag_scale[1]  = (float)(mag_max[1] - mag_min[1])/2.0;  // get average y mag bias in counts
	mag_scale[2]  = (float)(mag_max[2] - mag_min[2])/2.0;  // get average z mag bias in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	mag_scale[0] = avg_rad/mag_scale[0];
	mag_scale[1] = avg_rad/mag_scale[1];
	mag_scale[2] = avg_rad/mag_scale[2];

	Serial.println("Mag Calibration done!");

	PrintMagOffsets();

	return *this;
}

Simple_MPU6050 & Simple_MPU6050::setMagOffsets(float xMagB,float yMagB,float zMagB, float xMagS,float yMagS,float zMagS){
	mag_bias[0] = (float)xMagB;
	mag_bias[1] = (float)yMagB;
	mag_bias[2] = (float)zMagB;
	mag_scale[0] = (float)xMagS;
	mag_scale[1] = (float)yMagS;
	mag_scale[2] = (float)zMagS;

}


Simple_MPU6050 & Simple_MPU6050::PrintMagOffsets(){
	Serial.print(F("\n//                  X MagBias  Y MagBias  Z MagBias  X MagScale Y MagScale Z MagScale\n#define MAG_OFFSETS "));
	printfloatx("", mag_bias[0], 7, 1, ",  ");
	printfloatx("", mag_bias[1], 7, 1, ",  ");
	printfloatx("", mag_bias[2], 7, 1, ",  ");
	printfloatx("", mag_scale[0], 7, 3,",  ");
	printfloatx("", mag_scale[1], 7, 3,",  ");
	printfloatx("", mag_scale[2], 7, 3,"");
	Serial.println();
}

/*
Simple_MPU6050 & Simple_MPU6050::magcalMPU(){
uint16_t ii = 0, sample_count = 0;
int32_t mag_bias[3] = {0, 0, 0};
int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767};

Serial.println("Mag Calibration: Wave device in a figure eight until done!");
delay(4000);

sample_count = 64;
for(ii = 0; ii < sample_count; ii++) {
while(!AKM_ST1_READ_DATA_READY(akm_addr, &TVal).TVal) delay(5);
readMagData();  // Read the mag data
for (int jj = 0; jj < 3; jj++) {
if(mag[jj] > mag_max[jj]) mag_max[jj] = mag[jj];
if(mag[jj] < mag_min[jj]) mag_min[jj] = mag[jj];
}
delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
}

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

magBias[0] = (float) mag_bias[0]*mRes*mag_sens_adj[0];  // save mag biases in G for main program
magBias[1] = (float) mag_bias[1]*mRes*mag_sens_adj[1];
magBias[2] = (float) mag_bias[2]*mRes*mag_sens_adj[2];

Serial.println("Mag Calibration done!");
return *this;
}
*/

Simple_MPU6050 & Simple_MPU6050::viewMagRegisters(){
	uint8_t D;
	MPUi2cReadByte(akm_addr,0,&D);
	Serial.print((ReadCnt())? "R ":"X ");
	Serial.print(" Device ID 0x");
	DPRINTHEX(D);
	Serial.print(" 0B");
	DPRINTBIN(D);
	Serial.println();
	D = 0;

	while(!D){
		MPUi2cReadByte(akm_addr,0x02,&D);
		Serial.print((ReadCnt())? "R ":"X ");
		Serial.print(" Status 1 = 0x");
		DPRINTHEX(D);
		Serial.print(" 0B");
		DPRINTBIN(D);
		Serial.println();
		delay(1000);
	}
	Serial.println("****************");
	for(int i = 0X03;i<=0x08;i++){
		D = 0;
		if((i != 0x0B) && (i != 0x0D) && (i != 0x0E))  MPUi2cReadByte(akm_addr,i,&D);
		Serial.print((ReadCnt())? "R ":"X ");
		Serial.print("Register = 0x");
		DPRINTHEX(i);
		switch(i){
			case 0x00:
			Serial.print(" Device ID ");
			break;
			case 0x01:
			Serial.print(" Information ");
			break;
			case 0x02:
			Serial.print(" Status 1 ");
			break;
			case 0x03:
			Serial.print(" Measurement data ");
			break;
			case 0x04:
			Serial.print(" Measurement data ");
			break;
			case 0x05:
			Serial.print(" Measurement data ");
			break;
			case 0x06:
			Serial.print(" Measurement data ");
			break;
			case 0x07:
			Serial.print(" Measurement data ");
			break;
			case 0x08:
			Serial.print(" Measurement data ");
			break;
			case 0x09:
			Serial.print(" Status 2");
			break;
			case 0x0A:
			Serial.print(" Control ");
			break;
			case 0x0C:
			Serial.print(" Self-test");
			break;
			case 0x0F:
			Serial.print(" I2C disable");
			break;
			case 0x10:
			Serial.print(" X-axis sensitivity adjustment value");
			break;
			case 0x11:
			Serial.print(" Y-axis sensitivity adjustment value");
			break;
			case 0x12:
			Serial.print(" Z-axis sensitivity adjustment value");
			break;


		}
		Serial.print(" Value = 0x");
		DPRINTHEX(D);
		Serial.print(" 0B");
		DPRINTBIN(D);
		Serial.println();
	}
	Serial.println("\n");
	I2Cdev::writeByte(0x0C,0x0A ,  1 << 4 | 0x01 );// 16bit single measurement mode

	//	AKM_CNTL_WRITE_CONT_MEAS_MODE1(akm_addr,1);
	//Serial.println((WriteStatus())? "W-AKM_CNTL_WRITE_CONT_MEAS_MODE1":"X-AKM_CNTL_WRITE_CONT_MEAS_MODE1");
	return *this;
}


/*
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// "5123" equals 51.23 DegC.
int32_t bmp280_compensate_T(int32_t adc_T)
{
int32_t var1, var2, T;
var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
t_fine = var1 + var2;
T = (t_fine * 5 + 128) >> 8;
return T;
}

*/

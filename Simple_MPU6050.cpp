
#include <Wire.h>
#include <I2Cdev.h>
#include "DMP_Image.h"
#include "Simple_MPU6050.h"
#include "MPU_ReadMacros.h"
#include "MPU_WriteMacros.h"


#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has changed
volatile uint8_t _maxPackets;


Simple_MPU6050::Simple_MPU6050() {

	SetAddress(MPU6050_DEFAULT_ADDRESS);
	packet_length = 28;
	/*
	packet_length += 6;//DMP_FEATURE_SEND_RAW_ACCEL
	packet_length += 6;//DMP_FEATURE_SEND_RAW_GYRO
	packet_length += 16;//DMP_FEATURE_6X_LP_QUAT
	*/
	_maxPackets = 1024 / packet_length;
}

Simple_MPU6050 &  Simple_MPU6050::SetAddress(uint8_t address) {
	devAddr = address;

	return *this;
}

uint8_t  Simple_MPU6050::CheckAddress() {
	return devAddr;
}

/**
@brief      Get Newest packet from the FIFO. FIFO Buffer will be empty awaiting for next packet
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
	uint16_t Device = (WhoAmI < 0x39 )? 6050:6500;
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
			//			while (1) {}
		}
		return 1;
	}

	return 0;
}

Simple_MPU6050 & Simple_MPU6050::on_FIFO(void (*CB)(int16_t *, int16_t *, int32_t *, uint32_t *)) {
	on_FIFO_cb = CB;
	return *this; // return Pointer to this class
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
	if(WhoAmI < 0x39){
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

/**
@brief      Reset Fifo
*/
Simple_MPU6050 & Simple_MPU6050::reset_fifo() {
	USER_CTRL_WRITE_FIFO_RST(); //   Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::full_reset_fifo(void) { // Official way to reset fifo
	USER_CTRL_WRITE_FIFO_RST(1);
	USER_CTRL_WRITE_DMP_RST(1);
	return *this;
}



/**
@brief      Start and Stop DMP int pin triggering  //   1  Enable, 0 = Disable
*/
#define INT_ENABLE_WRITE_RAW_DMP_INT_EN(Data) MPUi2cWrite(0x38, 1, 1, Data)  //   1  Enable DMP interrupt
Simple_MPU6050 & Simple_MPU6050::DMP_InterruptEnable(uint8_t Data) {
	INT_ENABLE_WRITE_RAW_DMP_INT_EN(Data);
	dmp_on = Data;
	return *this;
};


//***************************************************************************************
//**********************         i2cdev wrapper functions          **********************
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
	if (mpuInterrupt && (Timestamp - spamtimer) >= (30)) {
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

/**
@brief      Reads Newest packet from fifo then on success triggers Callback routine
*/
Simple_MPU6050 & Simple_MPU6050::dmp_read_fifo() {
	if (!CheckForInterrupt()) return *this;
	if (!dmp_read_fifo(gyro, accel, quat, sensor_timestamp)) return *this;
	if (on_FIFO_cb) on_FIFO_cb(gyro, accel, quat, sensor_timestamp);
	return *this;
}

/**
@brief      Get the Newest packet from the FIFO. FIFO Buffer will be empty awaiting for next packet
*/
uint8_t Simple_MPU6050::dmp_read_fifo(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
	uint8_t fifo_data[MAX_PACKET_LENGTH];
	uint8_t ii = 0;
	/* Get a packet. */
	uint16_t fifo_count = 0;
	uint8_t more;
	FIFO_COUNTH_READ_FIFO_CNT(&fifo_count);
	if ((fifo_count < packet_length) || (fifo_count % packet_length)) {
		reset_fifo();
		return 0;
	}
	more = (fifo_count / packet_length);
	while (more--) {
		FIFO_READ(packet_length, fifo_data);
		timestamp = millis();
	}
	/* Parse DMP packet. */
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

#define A_OFFSET_H_READ_A_OFFS(Data)    MPUi2cReadInts(0x06, 3, Data)  //   X accelerometer offset cancellation
#define XG_OFFSET_H_READ_OFFS_USR(Data) MPUi2cReadInts(0x13, 3, Data)  //   Remove DC bias from the gyro sensor Step 0.0305 dps
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) Serial.print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt



Simple_MPU6050 & Simple_MPU6050::PrintActiveOffsets( ) {
	int16_t Data[3];
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	Serial.print(F("\n//              X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro\n#define OFFSETS "));
	if(WhoAmI < 0x39)	A_OFFSET_H_READ_A_OFFS(Data);
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
	float kP = 0.3;
	float kI = 90;
	if(Loops < 5){ // Fine tune Mode
		kP *= .1;
		kI *= .1;
	}
	PID( 0x43,  kP, kI,  Loops);
	Serial.println();
	return *this;
}

/**
@brief      Fully calibrate Accel from ZERO in about 6-7 Loops 600-700 readings
*/

Simple_MPU6050 & Simple_MPU6050::CalibrateAccel(uint8_t Loops ) {
	float kP = 0.15;
	float kI = 8;
	if(Loops <= 10){ // Fine tune Mode
	 kP *= .1;
	 kI *= .1;
	}
	PID( 0x3B, kP, kI,  Loops);
	Serial.println();
	return *this;
}


Simple_MPU6050 & Simple_MPU6050::PID(uint8_t ReadAddress, float kP,float kI, uint8_t Loops) {
	uint8_t SaveAddress = (ReadAddress == 0x3B)?((WhoAmI < 0x39 )? 0x06:0x77):0x13;
	int16_t Data;
	float Reading;
	int16_t BitZero[3];
	uint8_t shift =(SaveAddress == 0x77)?3:2;
	float Error, PTerm, ITerm[3];
	int16_t eSample;
	uint32_t eSum ;


	Serial.write('*');
	for (int i = 0; i < 3; i++) {
		I2Cdev::readWords(devAddr, SaveAddress + (i * shift), 1, &Data); // reads 1 or more 16 bit integers (Word)
		if(SaveAddress != 0x13){
			BitZero[i] = Data & 1; // Capture Bit Zero to properly handle Accelerometer calibration
			ITerm[i] = ((float)Data) * 8;
			} else {
			ITerm[i] = ((float)Data) * 4;
		}
	}
	for (int L = 0; L < Loops; L++) {
		eSample = 0;
		for (int c = 0; c < 100; c++) {// 100 PI Calculations
			eSum = 0;
			for (int i = 0; i < 3; i++) {
				I2Cdev::readWords(devAddr, ReadAddress + (i * 2), 1, &Data); // reads 1 or more 16 bit integers (Word)
				Reading = Data;// Convert int to float;
				if ((ReadAddress == 0x3B)&&(i == 2)) Reading -= 16384;  //remove Gravity
				Error = -Reading; // PID is reverse
				eSum += (Reading < 0) ? Error : Reading; //only want Positive Numbers
				PTerm = kP * Error;
				ITerm[i] += (Error * 0.001) *  kI; // Integral term 1000 Calculations a second = 0.001
				if(SaveAddress != 0x13){ //Accellerometer
					Data = round((PTerm + ITerm[i] ) / 8);
					if(abs(Error) > 400){
						if((L+c) == 0){
							Data = 0;
							ITerm[i] = Data;
						}else if((L == 0) && (c == 1)){
							Data = Error / 8;
							ITerm[i] = Data;
						}
					}
					Data = ((Data)&0xFFFE) |BitZero[i]; // Insert Bit0 Saved at beginning
				} else Data = round((PTerm + ITerm[i] ) / 4); // Gyro
				I2Cdev::writeWords(devAddr, SaveAddress + (i * shift), 1,  &Data);
			}
			if((c == 99) && eSum > 1000){
				c = 2;
				Serial.write('-');
			}
			if((eSum * ((ReadAddress == 0x3B)?.05: 1)) < 5) eSample++;// Test to see if we are really close!
			if((eSum < 100) && (c > 20) && (eSample >= 10)) break; // We have 10 really Close Calculations Cycle to Next loop
			delay(1);
		}
		Serial.write('.');
		kP *= .75;
		kI *= .75;
	}
	for (int i = 0; i < 3; i++){
		if(SaveAddress != 0x13) {
			Data = round((ITerm[i] ) / 8); //Compute PID Output
			Data = ((Data)&0xFFFE) |BitZero[i];  // Insert Bit0 Saved at beginning
		} else Data = round((ITerm[i]) / 4);
		I2Cdev::writeWords(devAddr, SaveAddress + (i * shift), 1, &Data );
		// by Using ITerm value only for the final offset prevents the last reading from having and
		// influence on the offsets

	}
	SIGNAL_PATH_FULL_RESET_WRITE_RESET();
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
	// get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
	v -> x = vRaw -> x - gravity -> x * 8192;
	v -> y = vRaw -> y - gravity -> y * 8192;
	v -> z = vRaw -> z - gravity -> z * 8192;
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
	// yaw: (about Z axis)
	data[0] = atan2(2 * q -> x * q -> y - 2 * q -> w * q -> z, 2 * q -> w * q -> w + 2 * q -> x * q -> x - 1);
	// pitch: (nose up/down, about Y axis)
	data[1] = atan(gravity -> x / sqrt(gravity -> y * gravity -> y + gravity -> z * gravity -> z));
	// roll: (tilt left/right, about X axis)
	data[2] = atan(gravity -> y / sqrt(gravity -> x * gravity -> x + gravity -> z * gravity -> z));
	return *this;
}


Simple_MPU6050 & Simple_MPU6050::ConvertToDegrees(float*ypr, float*xyz) {
	const float radians_to_degrees = 180.0 / M_PI;
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
#define PWR_MGMT_1_WRITE_DEVICE_RESET(...) MPUi2cWrite(0x6B, 1, 7, 1);delay(100);MPUi2cWrite(0x6A, 4, 3, 0b1111);delay(100);  //   1  Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear.
Simple_MPU6050 & Simple_MPU6050::load_DMP_Image(uint8_t CalibrateMode) {
	uint8_t val;
	TestConnection(1);
	Serial.println();
	PWR_MGMT_1_WRITE_DEVICE_RESET();			//PWR_MGMT_1:(0x6B Bit7 true) reset with 100ms delay and full SIGNAL_PATH_RESET:(0x6A Bits 3,2,1,0 True) with another 100ms delay
	MPUi2cWriteByte(0x6B, 0x01);				// 0000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
	MPUi2cWriteByte(0x38, 0x00);				// 0000 0000 INT_ENABLE: no Interrupt
	MPUi2cWriteByte(0x23, 0x00);				// 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
	MPUi2cWriteByte(0x1C, 0x00);				// 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
	MPUi2cWriteByte(0x37, 0x80);				// 1001 0000 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
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
	attachInterrupt(0, [] {mpuInterrupt = true;}, FALLING); //NOTE: "[]{mpuInterrupt = true;}" Is a complete funciton without a name. It is handed to the callback of attachInterrupts Google: "Lambda anonymous functions"
	//These are the features the above code initialized for you by default (ToDo Allow removal of one or more Features)
	dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO |  DMP_FEATURE_SEND_CAL_GYRO; // These are Fixed into the DMP_Image and Can't be change easily at this time.
	return *this;
}

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
	if(WhoAmI < 0x39){
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



Simple_MPU6050 & Simple_MPU6050::AKM_Init(){
	INT_PIN_CFG_WRITE_BYPASS_EN(1);
	akm_addr = 0x0C;
	FindAddress(akm_addr,0x0F);
	AKM_WHOAMI_READ(akm_addr,&akm_WhoAmI);
	if(ReadStatus()){
		Serial.print(F("Found Magnetometer at Address: 0x0"));
		Serial.println(akm_addr,HEX);


	} else 	Serial.print(F("Failed to Find Magnetometer"));
	
}
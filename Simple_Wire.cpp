/* ============================================
Simple_Wire device library code is placed under the MIT license
Copyright (c) 2022 Homer Creutz

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
*/ #include "Simple_Wire.h"

Simple_Wire::Simple_Wire() {
}

Simple_Wire::Simple_Wire(uint8_t address) {
    SetAddress(address);
}

Simple_Wire &  Simple_Wire::SetAddress(uint8_t address) {
	devAddr = address;
	return *this;
}

Simple_Wire & Simple_Wire::ReadBit(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t *Data) {
	return ReadBit( devAddr,  regAddr,  length,  bitNum,  Data);
}
Simple_Wire & Simple_Wire::ReadBit(uint8_t AltAddress, uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t *Data) {
    uint8_t b;
    ReadBytes(AltAddress, regAddr,1, &b);
    if ((I2CReadCount) != 0) {
        if(length == 1) Data[0] = b & (1 << bitNum);
        else {
            uint8_t mask = ((1 << length) - 1) << (bitNum - length + 1);
            b &= mask;
            b >>= (bitNum - length + 1);
            Data[0] = b;
        }
    }
	return *this;
}

Simple_Wire & Simple_Wire::ReadByte(uint8_t regAddr,  uint8_t *Data) {
	ReadBytes(devAddr, regAddr,  1, Data);
	return *this;
}
Simple_Wire & Simple_Wire::ReadByte(uint8_t AltAddress,uint8_t regAddr,  uint8_t *Data) {
	ReadBytes(AltAddress, regAddr,  1, Data);
	return *this;
}

Simple_Wire & Simple_Wire::ReadBytes(uint8_t regAddr, uint8_t length, uint8_t *Data) {
	ReadBytes(devAddr, regAddr,  length, Data);
	return *this;
}

Simple_Wire & Simple_Wire::ReadBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data) {
    I2CReadCount = 0;
     for (int k = 0; k < length; k += min((int)length, WIRE_BUFFER_LENGTH)) {
        beginTransmission(AltAddress);
        write(regAddr);
        endTransmission();
        beginTransmission(AltAddress);
        requestFrom((uint8_t)AltAddress, (uint8_t)min((int)length - k, WIRE_BUFFER_LENGTH));
        for (; available(); I2CReadCount++) {
            Data[I2CReadCount] = read();
        }
        endTransmission();
    }
   	return *this;
}



// ReadInt or Word
Simple_Wire & Simple_Wire::ReadInt(uint8_t regAddr, uint16_t *Data) {
	ReadInts(devAddr, regAddr, 1, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}
Simple_Wire & Simple_Wire::ReadInt(uint8_t AltAddress,uint8_t regAddr, uint16_t *Data) {
	ReadInts(AltAddress, regAddr, 1, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}

// ReadInts or Words
Simple_Wire & Simple_Wire::ReadInts(uint8_t regAddr, uint16_t size, uint16_t *Data) {
	ReadInts(devAddr, regAddr, size, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}
Simple_Wire & Simple_Wire::ReadInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size, uint16_t *Data) {
    I2CReadCount = 0;
        for (uint8_t k = 0; k < size * 2; k += min(size * 2, WIRE_BUFFER_LENGTH)) {
            beginTransmission(AltAddress);
            write(regAddr);
            endTransmission();
            beginTransmission(AltAddress);
            requestFrom(AltAddress, (uint8_t)(size * 2)); // length=words, this wants bytes

            bool msb = true; // starts with MSB, then LSB
            for (; available() && I2CReadCount < size; ) {
                if (msb) {
                    // first byte is bits 15-8 (MSb=15)
                    Data[I2CReadCount] = read() << 8;
                } else {
                    // second byte is bits 7-0 (LSb=0)
                    Data[I2CReadCount] |= read();
                    I2CReadCount++;
                }
                msb = !msb;
            }
            endTransmission();
        }
    return *this;
}

// Write Bits
Simple_Wire & Simple_Wire::WriteBit(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val) {
	return WriteBit(devAddr, regAddr,  length,  bitNum,  Val);
}
Simple_Wire & Simple_Wire::WriteBit(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val) {
    uint8_t b;
    ReadBytes(AltAddress, regAddr,1, &b);
    if(I2CReadCount){
        if (length == 1)  b = (Val != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
        else {
                uint8_t mask = ((1 << length) - 1) << (bitNum - length + 1);
                Val <<= (bitNum - length + 1); // shift Data into correct position
                Val &= mask; // zero all non-important bits in Data
                b &= ~(mask); // zero all important bits in existing byte
                b |= Val; // combine Data with existing byte
        }
        WriteBytes(AltAddress, regAddr,1, &b);
    }
	return *this;
}

// WriteByte
Simple_Wire & Simple_Wire::WriteByte(uint8_t regAddr,  uint8_t Val) {
	WriteBytes(devAddr, regAddr,  1, &Val); //Writes 1 or more 8 bit Bytes
	return *this;
}
Simple_Wire & Simple_Wire::WriteByte(uint8_t AltAddress,uint8_t regAddr,  uint8_t Val) {
	WriteBytes(AltAddress, regAddr,  1, &Val); //Writes 1 or more 8 bit Bytes
	return *this;
}

// WriteBytes
Simple_Wire & Simple_Wire::WriteBytes(uint8_t regAddr, uint8_t length, uint8_t *Data) {
	WriteBytes(devAddr, regAddr,  length, Data); //Writes 1 or more 8 bit Bytes
	return *this;
}

Simple_Wire & Simple_Wire::WriteBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data) {
    I2CWriteCount = 0;
    beginTransmission(AltAddress);
    write((uint8_t) regAddr); // send address
    for (; I2CWriteCount < length; I2CWriteCount++) {
            write((uint8_t) Data[I2CWriteCount]);
    }
    endTransmission();
    return *this;
}

// WriteInt or word
Simple_Wire & Simple_Wire::WriteInt(uint8_t regAddr,  uint16_t Val) {
	WriteInts(devAddr, regAddr, 1,  &Val);// Writes 1 or more 16 bit integers (Word)
	return *this;
}
Simple_Wire & Simple_Wire::WriteInt(uint8_t AltAddress,uint8_t regAddr,  uint16_t Val) {
	WriteInts(AltAddress, regAddr, 1,  &Val);// Writes 1 or more 16 bit integers (Word)
	return *this;
}

// WriteInts or words
Simple_Wire & Simple_Wire::WriteInts(uint8_t regAddr, uint16_t size, uint16_t *Data) {
	WriteInts(devAddr, regAddr, size ,  Data);
	return *this;
}
Simple_Wire & Simple_Wire::WriteInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size, uint16_t *Data) {
    I2CWriteCount = 0;
    beginTransmission(AltAddress);
    write(regAddr); // send address
    for (; I2CWriteCount < size; I2CWriteCount++) { 
        write((uint8_t)(Data[I2CWriteCount] >> 8));    // send MSB
        write((uint8_t)Data[I2CWriteCount]);         // send LSB
    }
    endTransmission();
	return *this;
}

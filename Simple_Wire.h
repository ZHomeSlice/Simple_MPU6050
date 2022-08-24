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
*/

#ifndef Simple_Wire_h
#define Simple_Wire_h
#include "Arduino.h"
#include <Wire.h>

#ifndef WIRE_BUFFER_LENGTH
    #if defined(I2C_BUFFER_LENGTH)
        // Arduino ESP32 core Wire uses this
        #define WIRE_BUFFER_LENGTH I2C_BUFFER_LENGTH
    #elif defined(BUFFER_LENGTH)
        // Arduino AVR core Wire and many others use this
        #define WIRE_BUFFER_LENGTH BUFFER_LENGTH
    #elif defined(SERIAL_BUFFER_SIZE)
        // Arduino SAMD core Wire uses this
        #define WIRE_BUFFER_LENGTH SERIAL_BUFFER_SIZE
    #else
        // should be a safe fallback, though possibly inefficient
        #define WIRE_BUFFER_LENGTH 32
    #endif
#endif

class Simple_Wire : public TwoWire{
    public:
        uint8_t devAddr;
        Simple_Wire();
        Simple_Wire(uint8_t AltAddress);
        Simple_Wire & SetAddress(uint8_t address);
       
        // read functions
        Simple_Wire & ReadBit(uint8_t regAddr,  uint8_t length, uint8_t bitNum, uint8_t *data);
        Simple_Wire & ReadBit(uint8_t AltAddress,uint8_t regAddr,  uint8_t length, uint8_t bitNum, uint8_t *data);
        Simple_Wire & ReadByte(uint8_t regAddr,  uint8_t *Data);
        Simple_Wire & ReadByte(uint8_t AltAddress,uint8_t regAddr, uint8_t *Data);
        Simple_Wire & ReadBytes(uint8_t regAddr, uint8_t length, uint8_t *Data);
        Simple_Wire & ReadBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data);
        Simple_Wire & ReadInt(uint8_t regAddr, uint16_t *data);
        Simple_Wire & ReadInt(uint8_t AltAddress,uint8_t regAddr, uint16_t *data);
        Simple_Wire & ReadInts(uint8_t regAddr, uint16_t size, uint16_t *Data);
        Simple_Wire & ReadInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size, uint16_t *Data);

        // write functions
        Simple_Wire & WriteBit(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val);
        Simple_Wire & WriteBit(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val);
        Simple_Wire & WriteByte(uint8_t regAddr,  uint8_t Val);
        Simple_Wire & WriteByte(uint8_t AltAddress,uint8_t regAddr,  uint8_t Val);
        Simple_Wire & WriteBytes(uint8_t regAddr, uint8_t length, uint8_t *Data);
        Simple_Wire & WriteBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data);
        Simple_Wire & WriteInt(uint8_t regAddr,  uint16_t Val);
        Simple_Wire & WriteInt(uint8_t AltAddress,uint8_t regAddr,  uint16_t Val);
        Simple_Wire & WriteInts(uint8_t regAddr, uint16_t size,  uint16_t *data);
        Simple_Wire & WriteInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size,  uint16_t *data);
        // check fundtions
        uint8_t  CheckAddress(){return(devAddr);};
        uint8_t  ReadCount() {return(I2CReadCount);};
        uint8_t  WriteCount() {return(I2CWriteCount);};
        bool  ReadSuccess() {return(I2CReadCount>0);};
        bool  WriteSucess() {return(I2CWriteCount>0);};
        
    private:
    
        uint8_t I2CReadCount;
        uint8_t I2CWriteCount;

};

#endif 
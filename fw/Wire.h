/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts

  Subset 2021 by Christian Frisson for use in moteus firmware for TorqueTuner
  From https://github.com/stm32duino/Arduino_Core_STM32 libraries/Wire/src/Wire.cpp
*/

#ifndef TwoWire_h
#define TwoWire_h

// #include "Stream.h"
// #include "Arduino.h"
// extern "C" {
// #include "utility/twi.h"
#include "twi.h"
// }

#include <stdint.h>
#include <string.h>

#define BUFFER_LENGTH 32

// class TwoWire : public Stream {
//   private:

    extern uint8_t *rxBuffer;
    extern uint8_t rxBufferAllocated;
    extern uint8_t rxBufferIndex;
    extern uint8_t rxBufferLength;

    extern uint8_t txAddress;
    extern uint8_t *txBuffer;
    extern uint8_t txBufferAllocated;
    extern uint8_t txBufferIndex;
    extern uint8_t txBufferLength;

    extern uint8_t transmitting;

    // void (*user_onRequest)(void);
    // void (*user_onReceive)(int);
    
    extern void onRequestService(/*i2c_t * */);
    extern void onReceiveService(/*i2c_t * */);
    
    void allocateRxBuffer(size_t length);
    void allocateTxBuffer(size_t length);

    void resetRxBuffer(void);
    void resetTxBuffer(void);

    /*virtual*/ size_t write(const uint8_t *, size_t);
    /*virtual*/ int available(void);
    /*virtual*/ int read(void);

    // void onReceive(void (*)(int));
    // void onRequest(void (*)(void));

// };

#endif
/*
  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
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

extern "C" {
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
}

#include "Wire.h"

#include "TorqueTuner.h"

uint8_t *rxBuffer = nullptr;
uint8_t rxBufferAllocated = 0;
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t *txBuffer = nullptr;
uint8_t txBufferAllocated = 0;
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

uint8_t transmitting = 0;

void allocateTxBuffer(size_t length)
{
  if (txBufferAllocated < length) {
    // By default we allocate BUFFER_LENGTH bytes. It is the min size of the buffer.
    if (length < BUFFER_LENGTH) {
      length = BUFFER_LENGTH;
    }
    uint8_t *tmp = (uint8_t *)realloc(txBuffer, length * sizeof(uint8_t));
    if (tmp != nullptr) {
      txBuffer = tmp;
      txBufferAllocated = length;
    } 
    // else {
    //   _Error_Handler("No enough memory! (%i)\n", length);
    // }
  }
}

/**
  * @brief  Allocate the Rx/Tx buffer to the requested length if needed
  * @note   Minimum allocated size is BUFFER_LENGTH)
  * @param  length: number of bytes to allocate
  */
void allocateRxBuffer(size_t length)
{
  if (rxBufferAllocated < length) {
    // By default we allocate BUFFER_LENGTH bytes. It is the min size of the buffer.
    if (length < BUFFER_LENGTH) {
      length = BUFFER_LENGTH;
    }
    uint8_t *tmp = (uint8_t *)realloc(rxBuffer, length * sizeof(uint8_t));
    if (tmp != nullptr) {
      rxBuffer = tmp;
      rxBufferAllocated = length;
    } 
    // else {
    //   _Error_Handler("No enough memory! (%i)\n", length);
    // }
  }
}

/**
  * @brief  This function must be called in slave Tx event callback or after
  *         beginTransmission() and before endTransmission().
  * @param  pdata: pointer to the buffer data
  * @param  quantity: number of bytes to write
  * @retval number of bytes ready to write.
  */
size_t write(const uint8_t *data, size_t quantity)
{
  size_t ret = quantity;

  if (transmitting) {
    // in master transmitter mode
    allocateTxBuffer(txBufferLength + quantity);
    // error if no memory block available to allocate the buffer
    if (txBuffer == nullptr) {
      // setWriteError();
      ret = 0;
    } else {
      // put bytes in tx buffer
      memcpy(&(txBuffer[txBufferIndex]), data, quantity);
      txBufferIndex = txBufferIndex + quantity;
      // update amount in buffer
      txBufferLength = txBufferIndex;
    }
  } else {
    // in slave send mode
    // reply to master
    if (i2c_slave_write_IT(/*i2c_t *obj, */(uint8_t *)data, quantity) != I2C_OK) {
      ret = 0;
    }
  }
  return ret;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int available(void)
{
  return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int read(void)
{
  int value = -1;

  // get each successive byte on each call
  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;

    /* Commented as not I think it is not useful
     * but kept to show that it is possible to
     * reset rx buffer when no more data available */
    if(rxBufferIndex == rxBufferLength) {
      resetRxBuffer();
    }
  }
  return value;
}

void onReceiveService(){
  uint8_t *inBytes = (uint8_t *) i2cTxRxBuffer;
  int numBytes = i2cRxNbData;

  // don't bother if user hasn't registered a callback
  // if (receiveI2C) {
    // don't bother if rx buffer is in use by a master requestFrom() op
    // i know this drops data, but it allows for slight stupidity
    // meaning, they may not have read all the master requestFrom() data yet
    if (rxBufferIndex >= rxBufferLength) {

      allocateRxBuffer(numBytes);
      // error if no memory block available to allocate the buffer
      // if (rxBuffer == nullptr)
      //   Error_Handler();
      // }

      // copy twi rx buffer into local read buffer
      // this enables new reads to happen in parallel
      memcpy(rxBuffer, inBytes, numBytes);
      // set rx iterator vars
      rxBufferIndex = 0;
      rxBufferLength = numBytes;
      // alert user program
      receiveI2C(numBytes);
    }
  // }
}
void onRequestService(){
  // don't bother if user hasn't registered a callback
  // if (sendI2C) {
    // reset tx buffer iterator vars
    // !!! this will kill any pending pre-master sendTo() activity
    txBufferIndex = 0;
    txBufferLength = 0;
    resetTxBuffer();
    // alert user program
    sendI2C();
  // }

}

/**
  * @brief  Reset Rx/Tx buffer content to 0
  */
void resetRxBuffer(void)
{
  if (rxBuffer != nullptr) {
    memset(rxBuffer, 0, rxBufferAllocated);
  }
}

void resetTxBuffer(void)
{
  if (txBuffer != nullptr) {
    memset(txBuffer, 0, txBufferAllocated);
  }
}
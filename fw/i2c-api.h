
/** \addtogroup hal */
/** @{*/
/* Forked by Christian Frisson from:
 * https://github.com/ARMmbed/mbed-os hal/i2c_api.h
 * to define custom HAL_I2C_*Callback functions
 * 
 * mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_I2C_API_H
#define MBED_I2C_API_H

// #include "stm32g4xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif

#include "device.h"
#include "pinmap.h"
// #include "PeripheralPins.h"
// #include "common_objects.h" // for i2c_s

extern HAL_StatusTypeDef i2cInit;
extern uint16_t i2cAddrMatchCode;
extern int i2cAddrCallback;
extern int i2cListenCpltCallback;
extern int i2cSlaveRxCpltCallback;
extern int i2cSlaveTxCpltCallback;

// typedef struct i2c_s i2c_t;
typedef struct {
    struct i2c_s    i2c;     /**< Target specific I2C structure */

} i2c_t;

extern i2c_t mbed_i2c_;

void i2c_init(i2c_t *obj, PinName sda, PinName scl);


#if DEVICE_I2CSLAVE

/**
 * \defgroup SynchI2C Synchronous I2C Hardware Abstraction Layer for slave
 * @{
 */

/** Configure I2C as slave or master.
 *  @param obj The I2C object
 *  @param enable_slave Enable i2c hardware so you can receive events with ::i2c_slave_receive
 *  @return non-zero if a value is available
 */
void i2c_slave_mode(i2c_t *obj, int enable_slave);

/** Configure I2C address.
 *  @param obj     The I2C object
 *  @param idx     Currently not used
 *  @param address The address to be set
 *  @param mask    Currently not used
 */
void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask);

#endif

/**@}*/

#ifdef __cplusplus
}
#endif

#endif

/** @}*/

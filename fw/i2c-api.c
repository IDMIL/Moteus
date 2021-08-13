/* Forked by Christian Frisson from:
 * https://github.com/ARMmbed/mbed-os targets/TARGET_STM/i2c_api.c
 * to define custom HAL_I2C_*Callback functions
 * 
 * mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2015, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

#include "i2c-api.h"

HAL_StatusTypeDef i2cInit;
uint16_t i2cAddrMatchCode = 0;
int i2cAddrCallback = 0;
int i2cListenCpltCallback = 0;
int i2cSlaveRxCpltCallback = 0;
int i2cSlaveTxCpltCallback = 0;

#include "mbed_assert.h"
// #include "i2c_api.h"
#include "platform/mbed_wait_api.h"

#include <string.h>
#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralPins.h"
#include "i2c_device.h" // family specific defines

#if DEVICE_I2C_ASYNCH
#define I2C_S(obj) (struct i2c_s *) (&((obj)->i2c))
#else
#define I2C_S(obj) (struct i2c_s *) (obj)
#endif

/*  Family specific description for I2C */
#define I2C_NUM (5)
static I2C_HandleTypeDef *i2c_handles[I2C_NUM];

/* Timeout values are based on core clock and I2C clock.
   The BYTE_TIMEOUT is computed as twice the number of cycles it would
   take to send 10 bits over I2C. Most Flags should take less than that.
   This is for immediate FLAG or ACK check.
*/
#define BYTE_TIMEOUT ((SystemCoreClock / obj_s->hz) * 2 * 10)

/* Declare i2c_init_internal to be used in this file */
void i2c_init_internal(i2c_t *obj, PinName sda, PinName scl);

#if defined(I2C1_BASE)
static void i2c1_irq(void)
{
    I2C_HandleTypeDef *handle = i2c_handles[0];
    HAL_I2C_EV_IRQHandler(handle);
    HAL_I2C_ER_IRQHandler(handle);
}
#endif
#if defined(I2C2_BASE)
static void i2c2_irq(void)
{
    I2C_HandleTypeDef *handle = i2c_handles[1];
    HAL_I2C_EV_IRQHandler(handle);
    HAL_I2C_ER_IRQHandler(handle);
}
#endif
#if defined(I2C3_BASE)
static void i2c3_irq(void)
{
    I2C_HandleTypeDef *handle = i2c_handles[2];
    HAL_I2C_EV_IRQHandler(handle);
    HAL_I2C_ER_IRQHandler(handle);
}
#endif
#if defined(I2C4_BASE)
static void i2c4_irq(void)
{
    I2C_HandleTypeDef *handle = i2c_handles[3];
    HAL_I2C_EV_IRQHandler(handle);
    HAL_I2C_ER_IRQHandler(handle);
}
#endif
#if defined(FMPI2C1_BASE)
static void i2c5_irq(void)
{
    I2C_HandleTypeDef *handle = i2c_handles[4];
    HAL_I2C_EV_IRQHandler(handle);
    HAL_I2C_ER_IRQHandler(handle);
}
#endif

void i2c_hw_reset(i2c_t *obj)
{
    int timeout;
    struct i2c_s *obj_s = I2C_S(obj);
    I2C_HandleTypeDef *handle = &(obj_s->handle);

    handle->Instance = (I2C_TypeDef *)(obj_s->i2c);

    // wait before reset
    timeout = BYTE_TIMEOUT;
    while ((__HAL_I2C_GET_FLAG(handle, I2C_FLAG_BUSY)) && (--timeout != 0));
#if defined I2C1_BASE
    if (obj_s->i2c == I2C_1) {
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
    }
#endif
#if defined I2C2_BASE
    if (obj_s->i2c == I2C_2) {
        __HAL_RCC_I2C2_FORCE_RESET();
        __HAL_RCC_I2C2_RELEASE_RESET();
    }
#endif
#if defined I2C3_BASE
    if (obj_s->i2c == I2C_3) {
        __HAL_RCC_I2C3_FORCE_RESET();
        __HAL_RCC_I2C3_RELEASE_RESET();
    }
#endif
#if defined I2C4_BASE
    if (obj_s->i2c == I2C_4) {
        __HAL_RCC_I2C4_FORCE_RESET();
        __HAL_RCC_I2C4_RELEASE_RESET();
    }
#endif
#if defined FMPI2C1_BASE
    if (obj_s->i2c == FMPI2C_1) {
        __HAL_RCC_FMPI2C1_FORCE_RESET();
        __HAL_RCC_FMPI2C1_RELEASE_RESET();
    }
#endif
}

void i2c_frequency(i2c_t *obj, int hz)
{
    int timeout;
    struct i2c_s *obj_s = I2C_S(obj);
    I2C_HandleTypeDef *handle = &(obj_s->handle);

    // wait before init
    timeout = BYTE_TIMEOUT;
    while ((__HAL_I2C_GET_FLAG(handle, I2C_FLAG_BUSY)) && (--timeout != 0));

#ifdef I2C_IP_VERSION_V1
    handle->Init.ClockSpeed      = hz;
    handle->Init.DutyCycle       = I2C_DUTYCYCLE_2;
#endif
#ifdef I2C_IP_VERSION_V2
    /*  Only predefined timing for below frequencies are supported */
    MBED_ASSERT((hz == 100000) || (hz == 400000) || (hz == 1000000));
    handle->Init.Timing = get_i2c_timing(hz);

    // Enable the Fast Mode Plus capability
    if (hz == 1000000) {
#if defined(I2C1_BASE) && defined(__HAL_SYSCFG_FASTMODEPLUS_ENABLE) && defined (I2C_FASTMODEPLUS_I2C1)
        if (obj_s->i2c == I2C_1) {
            HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
        }
#endif
#if defined(I2C2_BASE) && defined(__HAL_SYSCFG_FASTMODEPLUS_ENABLE) && defined (I2C_FASTMODEPLUS_I2C2)
        if (obj_s->i2c == I2C_2) {
            HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
        }
#endif
#if defined(I2C3_BASE) && defined(__HAL_SYSCFG_FASTMODEPLUS_ENABLE) && defined (I2C_FASTMODEPLUS_I2C3)
        if (obj_s->i2c == I2C_3) {
            HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C3);
        }
#endif
#if defined(I2C4_BASE) && defined(__HAL_SYSCFG_FASTMODEPLUS_ENABLE) && defined (I2C_FASTMODEPLUS_I2C4)
        if (obj_s->i2c == I2C_4) {
            HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C4);
        }
#endif
    }
#endif //I2C_IP_VERSION_V2

    /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
#if defined(I2C1_BASE) && defined (__HAL_RCC_I2C1_CONFIG)
    if (obj_s->i2c == I2C_1) {
        __HAL_RCC_I2C1_CONFIG(I2CAPI_I2C1_CLKSRC);
    }
#endif
#if defined(I2C2_BASE) && defined(__HAL_RCC_I2C2_CONFIG)
    if (obj_s->i2c == I2C_2) {
        __HAL_RCC_I2C2_CONFIG(I2CAPI_I2C2_CLKSRC);
    }
#endif
#if defined(I2C3_BASE) && defined(__HAL_RCC_I2C3_CONFIG)
    if (obj_s->i2c == I2C_3) {
        __HAL_RCC_I2C3_CONFIG(I2CAPI_I2C3_CLKSRC);
    }
#endif
#if defined(I2C4_BASE) && defined(__HAL_RCC_I2C4_CONFIG)
    if (obj_s->i2c == I2C_4) {
        __HAL_RCC_I2C4_CONFIG(I2CAPI_I2C4_CLKSRC);
    }
#endif

#ifdef I2C_ANALOGFILTER_ENABLE
    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(handle, I2C_ANALOGFILTER_ENABLE);
#endif

    // I2C configuration
    handle->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    handle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    handle->Init.OwnAddress1     = 0;
    handle->Init.OwnAddress2     = 0;
    HAL_I2C_Init(handle);

    /*  store frequency for timeout computation */
    obj_s->hz = hz;
}

i2c_t *get_i2c_obj(I2C_HandleTypeDef *hi2c)
{
    /* Aim of the function is to get i2c_s pointer using hi2c pointer */
    /* Highly inspired from magical linux kernel's "container_of" */
    /* (which was not directly used since not compatible with IAR toolchain) */
    struct i2c_s *obj_s;
    i2c_t *obj;

    obj_s = (struct i2c_s *)((char *)hi2c - offsetof(struct i2c_s, handle));
    obj = (i2c_t *)((char *)obj_s - offsetof(i2c_t, i2c));

    return (obj);
}


void i2c_ev_err_enable(i2c_t *obj, uint32_t handler)
{
    struct i2c_s *obj_s = I2C_S(obj);
    IRQn_Type irq_event_n = obj_s->event_i2cIRQ;
    IRQn_Type irq_error_n = obj_s->error_i2cIRQ;
    /*  default prio in master case is set to 2 */
    uint32_t prio = 2;

    /* Set up ITs using IRQ and handler tables */
    NVIC_SetVector(irq_event_n, handler);
    NVIC_SetVector(irq_error_n, handler);

#if DEVICE_I2CSLAVE
    /*  Set higher priority to slave device than master.
     *  In case a device makes use of both master and slave, the
     *  slave needs higher responsiveness.
     */
    if (obj_s->slave) {
        prio = 1;
    }
#endif

    NVIC_SetPriority(irq_event_n, prio);
    NVIC_SetPriority(irq_error_n, prio);
    NVIC_EnableIRQ(irq_event_n);
    NVIC_EnableIRQ(irq_error_n);
}

void i2c_ev_err_disable(i2c_t *obj)
{
    struct i2c_s *obj_s = I2C_S(obj);
    IRQn_Type irq_event_n = obj_s->event_i2cIRQ;
    IRQn_Type irq_error_n = obj_s->error_i2cIRQ;

    HAL_NVIC_DisableIRQ(irq_event_n);
    HAL_NVIC_DisableIRQ(irq_error_n);
}

uint32_t i2c_get_irq_handler(i2c_t *obj)
{
    struct i2c_s *obj_s = I2C_S(obj);
    I2C_HandleTypeDef *handle = &(obj_s->handle);
    uint32_t handler = 0;

    switch (obj_s->index) {
#if defined(I2C1_BASE)
        case 0:
            handler = (uint32_t)&i2c1_irq;
            break;
#endif
#if defined(I2C2_BASE)
        case 1:
            handler = (uint32_t)&i2c2_irq;
            break;
#endif
#if defined(I2C3_BASE)
        case 2:
            handler = (uint32_t)&i2c3_irq;
            break;
#endif
#if defined(I2C4_BASE)
        case 3:
            handler = (uint32_t)&i2c4_irq;
            break;
#endif
#if defined(FMPI2C1_BASE)
        case 4:
            handler = (uint32_t)&i2c5_irq;
            break;
#endif
    }

    i2c_handles[obj_s->index] = handle;
    return handler;
}

void i2c_init(i2c_t *obj, PinName sda, PinName scl)
{
    memset(obj, 0, sizeof(*obj));
    i2c_init_internal(obj, sda, scl);
}

void i2c_init_internal(i2c_t *obj, PinName sda, PinName scl)
{
    struct i2c_s *obj_s = I2C_S(obj);
    
    // Determine the I2C to use
    I2CName i2c_sda = (I2CName)pinmap_peripheral(sda, PinMap_I2C_SDA);
    I2CName i2c_scl = (I2CName)pinmap_peripheral(scl, PinMap_I2C_SCL);
    obj_s->sda = sda;
    obj_s->scl = scl;

    obj_s->i2c = (I2CName)pinmap_merge(i2c_sda, i2c_scl);
    MBED_ASSERT(obj_s->i2c != (I2CName)NC);

#if defined I2C1_BASE
    // Enable I2C1 clock and pinout if not done
    if (obj_s->i2c == I2C_1) {
        obj_s->index = 0;
        __HAL_RCC_I2C1_CLK_ENABLE();
        // Configure I2C pins
        obj_s->event_i2cIRQ = I2C1_EV_IRQn;
        obj_s->error_i2cIRQ = I2C1_ER_IRQn;
    }
#endif
#if defined I2C2_BASE
    // Enable I2C2 clock and pinout if not done
    if (obj_s->i2c == I2C_2) {
        obj_s->index = 1;
        __HAL_RCC_I2C2_CLK_ENABLE();
        obj_s->event_i2cIRQ = I2C2_EV_IRQn;
        obj_s->error_i2cIRQ = I2C2_ER_IRQn;
    }
#endif
#if defined I2C3_BASE
    // Enable I2C3 clock and pinout if not done
    if (obj_s->i2c == I2C_3) {
        obj_s->index = 2;
        __HAL_RCC_I2C3_CLK_ENABLE();
        obj_s->event_i2cIRQ = I2C3_EV_IRQn;
        obj_s->error_i2cIRQ = I2C3_ER_IRQn;
    }
#endif
#if defined I2C4_BASE
    // Enable I2C3 clock and pinout if not done
    if (obj_s->i2c == I2C_4) {
        obj_s->index = 3;
        __HAL_RCC_I2C4_CLK_ENABLE();
        obj_s->event_i2cIRQ = I2C4_EV_IRQn;
        obj_s->error_i2cIRQ = I2C4_ER_IRQn;
    }
#endif
#if defined FMPI2C1_BASE
    // Enable I2C3 clock and pinout if not done
    if (obj_s->i2c == FMPI2C_1) {
        obj_s->index = 4;
        __HAL_RCC_FMPI2C1_CLK_ENABLE();
        obj_s->event_i2cIRQ = FMPI2C1_EV_IRQn;
        obj_s->error_i2cIRQ = FMPI2C1_ER_IRQn;
    }
#endif

    // Configure I2C pins
    pinmap_pinout(sda, PinMap_I2C_SDA);
    pinmap_pinout(scl, PinMap_I2C_SCL);
    pin_mode(sda, OpenDrainNoPull);
    pin_mode(scl, OpenDrainNoPull);

    // I2C configuration
    // Default hz value used for timeout computation
    if (!obj_s->hz) {
        obj_s->hz = 100000;    // 100 kHz per default
    }

    // Reset to clear pending flags if any
    i2c_hw_reset(obj);
    i2c_frequency(obj, obj_s->hz);

#if DEVICE_I2CSLAVE
    // I2C master by default
    obj_s->slave = 0;
    obj_s->pending_slave_tx_master_rx = 0;
    obj_s->pending_slave_rx_maxter_tx = 0;
#endif

    // I2C Xfer operation init
    obj_s->event = 0;
    obj_s->XferOperation = I2C_FIRST_AND_LAST_FRAME;
#ifdef I2C_IP_VERSION_V2
    obj_s->pending_start = 0;
#endif
}

#if DEVICE_I2CSLAVE
/* SLAVE API FUNCTIONS */
void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask)
{
    struct i2c_s *obj_s = I2C_S(obj);
    I2C_HandleTypeDef *handle = &(obj_s->handle);

    // I2C configuration
    handle->Init.OwnAddress1     = address;
    HAL_I2C_Init(handle);

    i2c_ev_err_enable(obj, i2c_get_irq_handler(obj));

    HAL_I2C_EnableListen_IT(handle);
}

void i2c_slave_mode(i2c_t *obj, int enable_slave)
{

    struct i2c_s *obj_s = I2C_S(obj);
    I2C_HandleTypeDef *handle = &(obj_s->handle);

    if (enable_slave) {
        obj_s->slave = 1;
        HAL_I2C_EnableListen_IT(handle);
    } else {
        obj_s->slave = 0;
        HAL_I2C_DisableListen_IT(handle);
    }
}

#endif // DEVICE_I2CSLAVE

/**
  ******************************************************************************
  * @file    twi.c
  * @author  WI6LABS and Christian Frisson (McGill University, IDMIL)
  * @version V1.0.0
  * @date    01-August-2016
  * @brief   provide a subset of the TWI interface for TorqueTuner implementation in moteus firmware
  * @source  From https://github.com/stm32duino/Arduino_Core_STM32 libraries/Wire/src/utility/twi.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
  *
  ******************************************************************************
  */

// #include "utility/twi.h"
#include "twi.h"

#include "Wire.h"

int i2cRxNbData = 0; // Number of accumulated bytes received in Slave mode
uint8_t i2cTxRxBuffer[I2C_TXRX_BUFFER_SIZE];
uint8_t i2cTxRxBufferSize = 0;

#define MODE_TRANSMIT     0
#define MODE_RECEIVE      1
#define MODE_LISTEN       2

uint8_t mode = MODE_LISTEN;

/**
  * @brief  Write bytes to master
  * @param  obj : pointer to i2c_t structure
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval status
  */
i2c_status_e i2c_slave_write_IT(/*i2c_t *obj, */uint8_t *data, uint16_t size)
{
  uint8_t i = 0;
  i2c_status_e ret = I2C_OK;

  // Protection to not override the TxBuffer
  if (size > I2C_TXRX_BUFFER_SIZE) {
    ret = I2C_DATA_TOO_LONG;
  } else {
    // Check the communication status
    for (i = 0; i < size; i++) {
      i2cTxRxBuffer[i2cTxRxBufferSize + i] = *(data + i);
    }

    i2cTxRxBufferSize += size;
  }
  return ret;
}


/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  i2cAddrCallback = 1;
  i2cAddrMatchCode = AddrMatchCode;
  // i2c_t *obj = get_i2c_obj(hi2c);
  if ((mode == MODE_RECEIVE) && (i2cRxNbData != 0)) {
    onReceiveService();
    mode = MODE_LISTEN;
    i2cRxNbData = 0;
  }

  // if (AddrMatchCode == hi2c->Init.OwnAddress1) {
    if (TransferDirection == I2C_DIRECTION_RECEIVE) {
      mode = MODE_TRANSMIT;

      // if (sendI2C != NULL) {
        i2cTxRxBufferSize = 0;
        onRequestService();
      // }
      HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t *) i2cTxRxBuffer,
                                    i2cTxRxBufferSize, I2C_LAST_FRAME);
    } else {
      i2cRxNbData = 0;
      mode = MODE_RECEIVE;
      /*  We don't know in advance how many bytes will be sent by master so
       *  we'll fetch one by one until master ends the sequence */
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t *) & (i2cTxRxBuffer[i2cRxNbData]),
                                   1, I2C_NEXT_FRAME);
    }
  // }
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  i2cListenCpltCallback = 1;
  // i2c_t *obj = get_i2c_obj(hi2c);

  /*  Previous master transaction now ended, so inform upper layer if needed
   *  then prepare for listening to next request */
  if ((mode == MODE_RECEIVE) && (i2cRxNbData != 0)) {
    onReceiveService();
  }
  mode = MODE_LISTEN;
  i2cRxNbData = 0;
  HAL_I2C_EnableListen_IT(hi2c);
}

/**
  * @brief Slave RX complete callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  i2cSlaveRxCpltCallback = 1;
  // i2c_t *obj = get_i2c_obj(hi2c);
  /* One more byte was received, store it then prepare next */
  if (i2cRxNbData < I2C_TXRX_BUFFER_SIZE) {
    i2cRxNbData++;
  } 
  // else {
  //   core_debug("ERROR: I2C Slave RX overflow\n");
  // }
  /* Restart interrupt mode for next Byte */
  if (mode == MODE_RECEIVE) {
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t *) & (i2cTxRxBuffer[i2cRxNbData]),
                                 1, I2C_NEXT_FRAME);
  }
}

/**
  * @brief Slave TX complete callback
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Reset address match code event */
  i2cSlaveTxCpltCallback = 1;
  // i2c_t *obj = get_i2c_obj(hi2c);
  /* Reset transmit buffer size */
  i2cTxRxBufferSize = 0;
}


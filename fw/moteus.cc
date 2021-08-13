// Copyright 2018-2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>

#include <functional>

// // #include "mbed.h"

#include "mjlib/micro/async_exclusive.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"
#include "mjlib/multiplex/micro_server.h"
#include "mjlib/multiplex/micro_stream_datagram.h"

#include "fw/abs_port.h"
#include "fw/board_debug.h"
#include "fw/firmware_info.h"
#include "fw/git_info.h"
#include "fw/millisecond_timer.h"
#include "fw/moteus_controller.h"
#include "fw/moteus_hw.h"
#include "fw/system_info.h"
#include "fw/uuid.h"

#if defined(TARGET_STM32G4)
#include "fw/fdcan.h"
#include "fw/fdcan_micro_server.h"
#include "fw/stm32g4_async_uart.h"
#include "fw/stm32g4_flash.h"
#else
#error "Unknown target"
#endif

#include "i2c-api.h"

extern "C" {
  uint32_t kMoteusFirmwareVersion = MOTEUS_FIRMWARE_VERSION;
}

auto* const MyDWT = DWT;
auto* const MyFLASH = FLASH;

using namespace moteus;
namespace micro = mjlib::micro;
namespace multiplex = mjlib::multiplex;

#if defined(TARGET_STM32G4)
using HardwareUart = Stm32G4AsyncUart;
using Stm32Flash = Stm32G4Flash;
#else
#error "Unknown target"
#endif

extern "C" {
void SetupClock() {
#if defined(TARGET_STM32G4)
  {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 170 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  //  85 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  //  85 MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
      mbed_die();
    }

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_FDCAN |
        RCC_PERIPHCLK_USART2 |
        RCC_PERIPHCLK_USART3 |
        RCC_PERIPHCLK_ADC12 |
        RCC_PERIPHCLK_ADC345 |
        RCC_PERIPHCLK_I2C1
        ;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK; 
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      mbed_die();
    }
  }
#endif
}
}

namespace {
class ClockManager {
 public:
  ClockManager(MillisecondTimer* timer,
               micro::PersistentConfig& persistent_config,
               micro::CommandManager& command_manager)
      : timer_(timer) {
    persistent_config.Register("clock", &clock_, [this]() {
        this->UpdateConfig();
      });
    command_manager.Register("clock", [this](auto&& command, auto&& response) {
        this->Command(command, response);
      });
  }

  void UpdateConfig() {
    const int32_t trim = std::max<int32_t>(0, std::min<int32_t>(127, clock_.hsitrim));
    RCC->ICSCR = (RCC->ICSCR & ~0xff000000) | (trim << 24);
  }

  void Command(const std::string_view& command,
               const micro::CommandManager::Response& response) {
    if (command == "us") {
      snprintf(output_, sizeof(output_), "%" PRIu32 "\r\n", timer_->read_us());
      WriteMessage(output_, response);
    } else {
      WriteMessage("ERR unknown clock\r\n", response);
    }
  }

  void WriteMessage(const std::string_view& message,
                    const micro::CommandManager::Response& response) {
    micro::AsyncWrite(*response.stream, message, response.callback);
  }

 private:
  struct Config {
    int32_t hsitrim = 64;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(hsitrim));
    }
  };

  MillisecondTimer* const timer_;
  Config clock_;
  char output_[16] = {};
};

}

#if defined(TARGET_STM32G4)
extern "C" {
extern char _sccmram;
extern char _siccmram;
extern char _eccmram;
extern char _sccmram;
}
#endif

namespace moteus {
volatile uint8_t g_measured_hw_pins;
volatile uint8_t g_measured_hw_rev;
}

// TorqueTuner

#define DEVICE_I2CSLAVE 1
i2c_t mbed_i2c_;
std::string i2cErrorMessage;

// https://github.com/stm32duino/Arduino_Core_STM32 libraries/Wire/src/Wire.cpp

// /* I2C Tx/Rx buffer size */
#if !defined(I2C_TXRX_BUFFER_SIZE)
#define I2C_TXRX_BUFFER_SIZE    32
#elif (I2C_TXRX_BUFFER_SIZE >= 256)
#error I2C buffer size cannot exceed 255
#endif

int i2cRxNbData = 0; // Number of accumulated bytes received in Slave mode
uint8_t i2cTxRxBuffer[I2C_TXRX_BUFFER_SIZE];
uint8_t i2cTxRxBufferSize = 0;

#define MODE_TRANSMIT     0
#define MODE_RECEIVE      1
#define MODE_LISTEN       2

uint8_t mode = MODE_LISTEN;

uint8_t *rxBuffer = nullptr;
uint8_t rxBufferAllocated = 0;
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t *txBuffer = nullptr;
uint8_t txBufferAllocated = 0;
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

#define BUFFER_LENGTH 32

uint8_t transmitting = 0;

///@brief I2C state
typedef enum {
  I2C_OK = 0,
  I2C_DATA_TOO_LONG = 1,
  I2C_NACK_ADDR = 2,
  I2C_NACK_DATA = 3,
  I2C_ERROR = 4,
  I2C_TIMEOUT = 5,
  I2C_BUSY = 6
} i2c_status_e;

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

// https://github.com/stm32duino/Arduino_Core_STM32 libraries/Wire/src/utility/twi.c

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

// https://github.com/stm32duino/Arduino_Core_STM32 libraries/Wire/src/Wire.cpp

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
    if (i2c_slave_write_IT(/*&hi2c1, */(uint8_t *)data, quantity) != I2C_OK) {
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
    /*if(rxBufferIndex == rxBufferLength) {
      resetRxBuffer();
    }*/
  }
  return value;
}

// TorqueTuner specific

const uint8_t I2C_BUF_SIZE = 10;
const uint8_t CHECKSUMSIZE = 2;

uint8_t rx_data[I2C_BUF_SIZE+CHECKSUMSIZE];
uint8_t tx_data[I2C_BUF_SIZE+CHECKSUMSIZE];

uint16_t checksum_tx = 0;
uint16_t checksum_rx = 0;
uint16_t sum = 0;

float tmp;
int16_t angle_rounded = 0;
char mode_rec = 't';
float velocity = 0;
int16_t torque = 0;
int32_t err = 0;

uint16_t calcsum(uint8_t buf[], int length) {
  uint32_t val = 0;
  for (int k = 0; k < length; k++) {
    val += buf[k];
  }
  return val;
}

void receiveI2C(int how_many) {
  uint8_t k = 0;
  while (available()) {
    rx_data[k] = read();
    k++;
    if (k > I2C_BUF_SIZE + CHECKSUMSIZE) break;
  }
  memcpy(&checksum_rx, rx_data + I2C_BUF_SIZE, 2);

  if (checksum_rx != calcsum(rx_data, I2C_BUF_SIZE)) { // error in recieved data
    err++;
  } else {
    memcpy(&torque, rx_data, 2); // int16
    memcpy(&velocity, rx_data + 2, 4); // float
    memcpy(&mode_rec, rx_data + 6, 1); // char
    // if (mode_rec == 't' || mode_rec == 'x' || mode_rec == 'v') {
    //   mode = mode_rec;
    // };
  }

}

void sendI2C() {
  // Angle
  tmp = 0;//y_1;
  angle_rounded = static_cast<int16_t> (tmp * 10 + 0.5f);
  // angle_rounded = static_cast<int16_t> (read_angle() * 10 + 0.5);
  memcpy(tx_data, &angle_rounded, 2);

  // wrapped angle_rounded
  tmp = 0;//yw - PA;
  angle_rounded = static_cast<int16_t>(tmp * 10 + 0.5f);
  memcpy(tx_data + 2, &angle_rounded, 2); // uint

  // Velocity
  tmp = 0;//v;
  memcpy(tx_data + 4, &tmp, 4); // float

  // Checksum
  checksum_tx = calcsum(tx_data, I2C_BUF_SIZE);
  memcpy(tx_data + I2C_BUF_SIZE, &checksum_tx, 2); // uint16

  // Send tx_data to I2C master
  write(tx_data, I2C_BUF_SIZE + CHECKSUMSIZE);
}

// https://github.com/stm32duino/Arduino_Core_STM32 libraries/Wire/src/Wire.cpp

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
    // alert user program
    sendI2C();
  // }

}

// https://github.com/stm32duino/Arduino_Core_STM32 libraries/Wire/src/utility/twi.c

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

// moteus

int main(void) {
#if defined(TARGET_STM32G4)
  std::memcpy(&_sccmram, &_siccmram, &_eccmram - &_sccmram);
#endif

  FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

  SetupClock();

  // Turn on our power light.
  DigitalOut power_led(POWER_LED, 0);

  DigitalIn hwrev0(HWREV_PIN0, PullUp);
  DigitalIn hwrev1(HWREV_PIN1, PullUp);
  DigitalIn hwrev2(HWREV_PIN2, PullUp);

  // To enable cycle counting.
#ifdef MOTEUS_PERFORMANCE_MEASURE
  {
    ITM->LAR = 0xC5ACCE55;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
#endif

  const uint8_t this_hw_pins =
      0x07 & (~(hwrev0.read() |
                (hwrev1.read() << 1) |
                (hwrev2.read() << 2)));
  moteus::g_measured_hw_pins = this_hw_pins;
  const uint8_t measured_hw_rev = [&]() {
    int i = 0;
    for (auto rev_pins : kHardwareInterlock) {
      if (rev_pins == this_hw_pins) { return i; }
      i++;
    }
    return -1;
  }();
  g_measured_hw_rev = measured_hw_rev;
  // Check if the detected board revision level is in our compatible
  // set.
  const bool compatible = [&]() {
    for (auto possible_version : kCompatibleHwRev) {
      if (measured_hw_rev == possible_version) { return true; }
    }
    return false;
  }();
  MJ_ASSERT(compatible);

  // I initially used a Ticker here to enqueue events at 1ms
  // intervals.  However, it introduced jitter into the current
  // sampling interrupt, and I couldn't figure out how to get the
  // interrupt priorities right.  Thus for now we just poll to look
  // for millisecond turnover.
  MillisecondTimer timer;

  micro::SizedPool<14000> pool;

  HardwareUart rs485(&pool, &timer, []() {
      HardwareUart::Options options;
      options.tx = MOTEUS_UART_TX;
      options.rx = MOTEUS_UART_RX;
      options.dir = MOTEUS_UART_DIR;
      options.baud_rate = 3000000;
      return options;
    }());

#if defined(TARGET_STM32G4)
  FDCan fdcan([]() {
      FDCan::Options options;

      options.td = MOTEUS_CAN_TD;
      options.rd = MOTEUS_CAN_RD;

      options.slow_bitrate = 1000000;
      options.fast_bitrate = 5000000;

      options.fdcan_frame = true;
      options.bitrate_switch = true;
      options.automatic_retransmission = true;

      return options;
    }());
  FDCanMicroServer fdcan_micro_server(&fdcan);
  multiplex::MicroServer multiplex_protocol(&pool, &fdcan_micro_server, {});
#else
#error "Unknown target"
#endif

  micro::AsyncStream* serial = multiplex_protocol.MakeTunnel(1);

  micro::AsyncExclusive<micro::AsyncWriteStream> write_stream(serial);
  micro::CommandManager command_manager(&pool, serial, &write_stream);
  micro::TelemetryManager telemetry_manager(
      &pool, &command_manager, &write_stream);
  Stm32Flash flash_interface;
  micro::PersistentConfig persistent_config(pool, command_manager, flash_interface);

  SystemInfo system_info(pool, telemetry_manager);
  FirmwareInfo firmware_info(pool, telemetry_manager,
                             kMoteusFirmwareVersion, MOTEUS_MODEL_NUMBER);
  Uuid uuid(persistent_config);
  ClockManager clock(&timer, persistent_config, command_manager);

  // TorqueTuner

  i2c_init(&mbed_i2c_, MOTEUS_ABS_SDA, MOTEUS_ABS_SCL);
  i2c_slave_mode(&mbed_i2c_, 1);
  i2c_slave_address(&mbed_i2c_, 0, 8 << 1, 0);

  // moteus
  
  AbsPort abs_port(
      &pool, &persistent_config, &telemetry_manager, &timer,
      [&]() {
        AbsPort::Options options;

        options.scl = MOTEUS_ABS_SCL;
        options.sda = MOTEUS_ABS_SDA;

        return options;
      }());

  MoteusController moteus_controller(
      &pool, &persistent_config, &telemetry_manager, &timer,
      &firmware_info, &abs_port);

  BoardDebug board_debug(
      &pool, &command_manager, &telemetry_manager, &multiplex_protocol,
      moteus_controller.bldc_servo());

  persistent_config.Register("id", multiplex_protocol.config(), [](){});

  GitInfo git_info;
  telemetry_manager.Register("git", &git_info);

  persistent_config.Load();

  moteus_controller.Start();
  command_manager.AsyncStart();
  multiplex_protocol.Start(moteus_controller.multiplex_server());

  auto old_time = timer.read_ms();

  for (;;) {
    rs485.Poll();
#if defined(TARGET_STM32G4)
    fdcan_micro_server.Poll();
#endif
    moteus_controller.Poll();
    abs_port.Poll();

    const auto new_time = timer.read_ms();

    if (new_time != old_time) {
      telemetry_manager.PollMillisecond();
      system_info.PollMillisecond();
      moteus_controller.PollMillisecond();
      board_debug.PollMillisecond();
      abs_port.PollMillisecond();

      old_time = new_time;
    }

    SystemInfo::idle_count++;
  }

  return 0;
}

extern "C" {
  void abort() {
    mbed_die();
  }
}

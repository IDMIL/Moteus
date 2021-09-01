
/**
  ******************************************************************************
  * @file    TorqueTuner.c
  * @author  Mathias Kirkegaard and Christian Frisson (McGill University, IDMIL)
  * @date    13-August-2021
  * @brief   Code for I2C communication for TorqueTuner motor drivers
  ******************************************************************************
  */

#include "TorqueTuner.h"

#include "Wire.h"

#include "fw/bldc_servo.h"
#include "fw/moteus.h"

#include <stdint.h>
#include <string.h>

using namespace moteus;

#define I2C_BUF_SIZE 10
#define CHECKSUMSIZE 2

static uint8_t rx_data[I2C_BUF_SIZE+CHECKSUMSIZE];
static uint8_t tx_data[I2C_BUF_SIZE+CHECKSUMSIZE];

uint16_t checksum_tx = 0;
uint16_t checksum_rx = 0;
uint16_t sum = 0;

float tmp;
int16_t angle_rounded = 0;
char mode_rec = 't';
float velocity = 0;
int16_t torque = 0;
int32_t err = 0;

BldcServo::CommandData prev_command;

uint16_t calcsum(uint8_t buf[], int length) {
  uint32_t val = 0;
  for (int k = 0; k < length; k++) {
    val += buf[k];
  }
  return val;
}


void receiveI2C(int how_many) {
  uint8_t k = 0;
  BldcServo::CommandData stop_command;
  stop_command.mode = BldcServo::kStopped;
  if(bldcServo && bldcServo->status().fault == moteus::errc::kStartOutsideLimit ){
    stop_command.rezero_position = 0.0f;
    bldcServo->Command(stop_command);
  }
  float kNaN = std::numeric_limits<float>::quiet_NaN();
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

    BldcServo::CommandData command;
    // We default to no timeout for debug commands.
    command.timeout_s = std::numeric_limits<float>::quiet_NaN();
    command.max_torque_Nm = 0.1f; // during debug: 0.1f, default: 100.0f;
    
    command.mode = BldcServo::kPosition; // kZeroVelocity
    
    if (mode_rec == 't'){
      command.position = kNaN; // kNaN means start at the current position.
      command.velocity = 0.0f;
      command.kd_scale = 0.0f;
      command.kp_scale = 0.0f;
      command.feedforward_Nm = static_cast<float> (torque) / 208.0f; // 208.0f is an approximate value of Mechaduino uMAX
      if(bldcServo && prev_command.feedforward_Nm != command.feedforward_Nm){
        bldcServo->Command(command);
      }
    }
    if (mode_rec == 'v') {
      command.velocity = velocity;
      command.position = kNaN; // kNaN means start at the current position.
      // TODO check max_position_slip
      if(bldcServo && prev_command.velocity != command.velocity){
        bldcServo->Command(command);
      }
    }
    prev_command = command;
  }

}

void sendI2C() {
  // Angle
  tmp = 0;//y_1;
  if(bldcServo) tmp = bldcServo->status().unwrapped_position;
  angle_rounded = static_cast<int16_t> (tmp * 3600 + 0.5f);
  // angle_rounded = static_cast<int16_t> (read_angle() * 10 + 0.5);
  memcpy(tx_data, &angle_rounded, 2);

  // wrapped angle_rounded
  tmp = 0;//yw - PA;
  if(bldcServo) tmp = bldcServo->status().position;
  angle_rounded = static_cast<int16_t>(tmp * 3600 + 0.5f);
  memcpy(tx_data + 2, &angle_rounded, 2); // uint

  // Velocity
  tmp = 0;//v;
  if(bldcServo) tmp = bldcServo->status().velocity;
  memcpy(tx_data + 4, &tmp, 4); // float

  // Checksum
  checksum_tx = calcsum(tx_data, I2C_BUF_SIZE);
  memcpy(tx_data + I2C_BUF_SIZE, &checksum_tx, 2); // uint16

  // Send tx_data to I2C master
  write(tx_data, I2C_BUF_SIZE + CHECKSUMSIZE);
}
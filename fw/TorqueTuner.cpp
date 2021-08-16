
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

#include <stdint.h>
#include <string.h>

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
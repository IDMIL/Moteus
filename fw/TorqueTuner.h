
/**
  ******************************************************************************
  * @file    TorqueTuner.h
  * @author  Mathias Kirkegaard and Christian Frisson (McGill University, IDMIL)
  * @date    13-August-2021
  * @brief   Header for I2C communication for TorqueTuner motor drivers
  ******************************************************************************
  */

#ifndef __TorqueTuner_H__
#define __TorqueTuner_H__

void receiveI2C(int how_many);
void sendI2C();

#endif // __TorqueTuner_H__

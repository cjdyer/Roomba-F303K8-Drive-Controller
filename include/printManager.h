#ifndef PRINT_MANAGER_H
#define PRINT_MANAGER_H

#include <Arduino.h>
#include "ICM_20948.h"
#include "motorPID.h"

/**
 * Print the formatted IMU variables
 *
 * @param value value to be printed.
 * @param leading number of leading zeros.
 * @param decimal number of decimals.
 */ 
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);

/**
 * Print the scaled and cleaned IMU data
 *
 * @param sensor sensor with data to be printed.
 */ 
void printScaledAGMT(ICM_20948_SPI *sensor);

/**
 * Print motor information on the serial monitor
 *
 * @param sensor sensor with data to be printed.
 */ 
void printMotorData(motorPID* leftMotor, motorPID* rightMotor);

#endif
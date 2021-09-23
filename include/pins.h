#ifndef PINS_H
#define PINS_H

/* Define Motor Driver Pins */
#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

/* Define encoder pins */
#define encoderLA A0 //3
#define encoderLB A1 //2
#define encoderRA A2 //3
#define encoderRB A3 //2

/* IMU definitions */
#define SPI_PORT SPI // 13 // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10	 // Which pin you connect CS to. Used only when "USE_SPI" is defined

#endif
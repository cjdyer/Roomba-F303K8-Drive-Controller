#include "printManager.h"

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
	float aval = abs(val);
	if (val < 0)
		Serial.print("-");
	else
		Serial.print(" ");

	for (uint8_t i = 0; i < leading; i++)
	{
		uint32_t tenpow = 0;
		if (i < (leading - 1))
			tenpow = 1;

		for (uint8_t c = 0; c < (leading - 1 - i); c++)
			tenpow *= 10;

		if (aval < tenpow)
			Serial.print("0");
		else
			break;
	}

	if (val < 0)
		Serial.print(-val, decimals);
	else
		Serial.print(val, decimals);
}

void printScaledAGMT(ICM_20948_SPI *sensor)
{
	//Serial.print("Scaled. Acc (mg) [ ");
	//printFormattedFloat(sensor->accX(), 5, 2);
	//Serial.print("\t");
	//printFormattedFloat(sensor->accY(), 5, 2);
	//Serial.print("\t");
	printFormattedFloat(sensor->accZ(), 5, 2);
	Serial.print("\t");
	//printFormattedFloat(sensor->gyrX(), 5, 2);
	//Serial.print("\t");
	//printFormattedFloat(sensor->gyrY(), 5, 2);
	//Serial.print("\t");
	printFormattedFloat(sensor->gyrZ(), 5, 2);
	Serial.print("\t");
	//printFormattedFloat(sensor->magX(), 5, 2);
	//Serial.print("\t");
	//printFormattedFloat(sensor->magY(), 5, 2);
	//Serial.print("\t");
	printFormattedFloat(sensor->magZ(), 5, 2);
	//Serial.print("\t");
	//printFormattedFloat(sensor->temp(), 5, 2);
	Serial.println();
}

void printMotorData(motorPID* leftMotor, motorPID* rightMotor)
{
    Serial.print(leftMotor->speedDesired); // Tachometer
    Serial.print("\t");
    Serial.print(rightMotor->speedDesired); // Tachometer
    Serial.print("\t");
    Serial.print(leftMotor->speed); // Tachometer
    Serial.print("\t");
    Serial.print(rightMotor->speed); // Tachometer
    Serial.print("\t");
    Serial.println();
} 

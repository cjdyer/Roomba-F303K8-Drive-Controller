#include <Arduino.h>
#include "utilityFuncs.h"

// Print value (little endian) as binary to serial
void printBin(uint16_t input){
  for(int i = 15; i >= 0; i--){
    Serial.print((input >> i) & 1);
    if(i == 4 || i == 8 || i == 12){
      Serial.print(" ");
    }
  }
}

void printString(char *s){
    int i = 0;
    while (s[i] != '\0'){
        Serial.print(s[i++]);
    }
}
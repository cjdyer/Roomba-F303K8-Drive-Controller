#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <ICM_20948.h> // Sparkfun ICM_20948 IMU module

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
  #error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

/* Define Motor Driver Pins */
#define AIN1 A5
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// Define encoder pins
#define encoderLA A0 //3
#define encoderLB A1 //2
#define encoderRA A2 //3
#define encoderRB A3 //2

/* Strip lines per inch */
#define stripLPI 150.0

/* IMU definitions */
#define SPI_PORT SPI  // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                       // the ADR jumper is closed the value becomes 0

// Configure SPI for IMU
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object

volatile long encoderPosL = 0;
volatile long encoderPosR = 0;
float lastKnownPosL = 0;
float lastKnownPosR = 0;

// Set offset values to adjust motor direction if necessary. Values: 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Ticks per drive-shaft revolution (different per motor/gearbox)
const float fullRev = 1632.67;

const float wheelCirc = 3.14 * 0.8; // Circumference in metres

// Initialise motor objects
Motor motorL = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
Motor motorR = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

bool Lfired = 0;
bool Rfired = 0;

bool running = 1;
bool Lrun = 1;
bool Rrun = 1;

// Quadrature Encoder matrix - 2s shouldn't ever appear Sauce: https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; 
int LOld = 0; // Previous encoder value left motor
int ROld = 0; // Previous encoder value right motor
int LNew = 0; // New encoder value left motor
int RNew = 0; // New encoder value right motor

// Left Hall encoder. Called once for each sensor on pin-change (quadrature)
void encoderLeft_callback(){
  if(Lrun){
    LOld = LNew;
    LNew = digitalRead(encoderLA) * 2 + digitalRead(encoderLB); // Convert binary input to decimal value // (PINC & 0b0001) gets the value of bit zero in port C (A0), ((PINC & 0b0010) >> 1) gets just the value of A1
    encoderPosL = encoderPosL + QEM[LOld * 4 + LNew];
    Lfired = 1;
  }
}

// Right Hall encoder. Called once for each sensor on pin-change (quadrature)
void encoderRight_callback(){
  if(Rrun){
    ROld = RNew;
    RNew = digitalRead(encoderRA) * 2 + digitalRead(encoderRB); // Convert binary input to decimal value // (PINC & 0b0001) gets the value of bit zero in port C (A0), ((PINC & 0b0010) >> 1) gets just the value of A1
    encoderPosR = encoderPosR + QEM[ROld * 4 + RNew];
    Rfired = 1;
  }
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if (val < 0){
    Serial.print("-");

  } else {
    Serial.print(" ");
  }

  for (uint8_t indi = 0; indi < leading; indi++){
    uint32_t tenpow = 0;
    if (indi < (leading - 1)){
      tenpow = 1;
    }
    
    for (uint8_t c = 0; c < (leading - 1 - indi); c++){
      tenpow *= 10;
    }

    if (aval < tenpow){
      Serial.print("0");

    } else {
      break;
    }
  }
  if (val < 0){
    Serial.print(-val, decimals);

  } else {
    Serial.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_SPI *sensor){
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  Serial.print(" ]");
  Serial.println();
}

void setup(){
  Serial.begin(115200);
  while (!Serial){};

  // Configure Encoder Pins
  pinMode (encoderLA, INPUT);
  pinMode (encoderLB, INPUT);
  pinMode (encoderRA, INPUT);
  pinMode (encoderRB, INPUT);
  digitalWrite(encoderLA, HIGH);
  digitalWrite(encoderLB, HIGH);
  digitalWrite(encoderRA, HIGH);
  digitalWrite(encoderRB, HIGH);
  attachInterrupt(encoderLA, encoderLeft_callback, CHANGE);
  attachInterrupt(encoderLB, encoderLeft_callback, CHANGE);
  attachInterrupt(encoderRA, encoderRight_callback, CHANGE);
  attachInterrupt(encoderRB, encoderRight_callback, CHANGE);
  Serial.println("Motor Pin Configured");

  brake(motorL, motorR);
  Serial.println("Motor Brakes Applied");

  delay(200);
  lastKnownPosL = encoderPosR / stripLPI * 25.4;
  lastKnownPosL = encoderPosR / stripLPI * 25.4;

  // Initialise IMU with SPI
  myICM.begin(CS_PIN, SPI_PORT);

  bool initialized = false;
  while (!initialized){
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());

    if(myICM.status != ICM_20948_Stat_Ok){
      Serial.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }

}

void loop(){
  const int desiredDistance = 10*fullRev;

  if(running == 1){

    if (myICM.dataReady()){
      myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
      
      //printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
      printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
      delay(30);

    }else{
      Serial.println("Waiting for data");
      delay(500);
    }

    if(encoderPosR >= desiredDistance){
      if(Rrun){
        //Serial.print("Right: ");
        //Serial.println(encoderPosR);
        Rrun = 0;
      }
      motorR.brake();

    }else{
      motorR.drive(200);
    }

    if(encoderPosL >= desiredDistance){
      if(Lrun){
        //Serial.print("Right: ");
        //Serial.println(encoderPosL);
        Lrun = 0;
      }
      
      motorL.brake();
    }else{
      motorL.drive(200);
    }

    if((encoderPosL >= desiredDistance) && (encoderPosR >= desiredDistance)){
      Serial.print(encoderPosL);
      Serial.print("\t");
      Serial.println(encoderPosR);
      Serial.print("Both motors stopped at distance: Left: ");
      Serial.print((encoderPosL/desiredDistance) * wheelCirc);
      Serial.print("m ");
      Serial.print((encoderPosR/desiredDistance) * wheelCirc);
      Serial.print("m ");
      running = 0;
    }

  }else{
    brake(motorL, motorR);
  }
}
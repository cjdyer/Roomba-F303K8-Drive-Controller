#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <ICM_20948.h> // Sparkfun ICM_20948 IMU module
#include <PID_v1.h>

// Hardware Timer check
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

/* Define encoder pins */
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
double wheelSpeedL = 0; // RPM
double wheelSpeedR = 0; // RPM

double wheelSpeedL_desired = 0; // RPM
double wheelSpeedR_desired = 0; // RPM

double speedTotalL = 0; // RPM
double speedTotalR = 0; // RPM

double speedL_error = 0; // RPM
double speedR_error = 0; // RPM

double speedL_error_pre = 0; // RPM
double speedR_error_pre = 0; // RPM

double speedL_pwm= 0; // RPM
double speedR_pwm = 0; // RPM

double speedL_esum = 0; // RPM
double speedR_esum = 0; // RPM

long wheelSpeedDistanceL = 0; // RPM
long wheelSpeedDistanceR = 0; // RPM
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

// Flags for the encoder triggers - Mainly for debugging
bool Lfired = 0;
bool Rfired = 0;

// Encoder tracking flags
bool running = 1;
bool Lrun = 1;
bool Rrun = 1;

// Quadrature Encoder matrix - 2s shouldn't ever appear Sauce: https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; 
int LOld = 0; // Previous encoder value left motor
int ROld = 0; // Previous encoder value right motor
int LNew = 0; // New encoder value left motor
int RNew = 0; // New encoder value right motor

unsigned int counter1 = 0;

TIM_TypeDef *Instance1 = TIM1;
HardwareTimer *Timer1 = new HardwareTimer(Instance1);

// PID Configuration parameters
//Specify the links and initial tuning parameters
double kp = 2.0, ki = 5.0, kd = 1.0;

unsigned int start_time;
unsigned int end_time;
bool speedCheck = 0;

//PID pidRight(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID pidLeft(&wheelSpeedL, &speedL_pwm, &wheelSpeedL_desired, kp, ki, kd, DIRECT);

// Speed Calc Callback
void speedCalc_callback(void){ // Every 100ms
  // Calc Left Motor Speed
  wheelSpeedL = 600 * (encoderPosL/fullRev)/0.1; // RPM angular_vel = da/dt * 600 (convert to minutes)
  wheelSpeedR = 600 * (encoderPosR/fullRev)/0.1; // RPM
  speedTotalL += wheelSpeedL;
  speedTotalR += wheelSpeedR;
  counter1++;
  encoderPosL = 0;
  encoderPosR = 0;
  speedCheck = 1;

  //wheelSpeedL = map(wheelSpeedL, 0, 15, 0, 255);


/*
  if(Lrun){
    speedL_error = wheelSpeedL_desired - wheelSpeedL;
    
    // PID
    speedL_pwm = speedL_error*kp + speedL_esum*ki + (speedL_error - speedL_error_pre)*kd;
    speedL_error_pre = speedL_error;  //save last (previous) error
    speedL_esum += speedL_error;      //sum of error
    
    if (speedL_esum >4000)  speedL_esum = 4000;
    if (speedL_esum <-4000) speedL_esum = -4000;
  
  }else{
    motorL.brake();
    speedL_error = 0;
    speedL_error_pre = 0;
    speedL_esum = 0;
    speedL_pwm = 0;
  }
*/
/*
  if (speedL_pwm < 255 & speedL_pwm >0){
    motorL.drive(speedL_pwm);

  }else{
    if (speedL_pwm>255){
      motorL.drive(255);
    }
    else{
      motorL.drive(0);
    }
  }*/

  //Serial.println("tick"); CAN'T USE SERIAL PRINT IN STM32 BOARDS IN INTERRUPTS
}

// Left Hall encoder. Called once for each sensor on pin-change (quadrature)
void encoderLeft_callback(void){
  if(Lrun){
    LOld = LNew;
    LNew = digitalRead(encoderLA) * 2 + digitalRead(encoderLB); // Convert binary input to decimal value // (PINC & 0b0001) gets the value of bit zero in port C (A0), ((PINC & 0b0010) >> 1) gets just the value of A1
    encoderPosL += QEM[LOld * 4 + LNew];
    wheelSpeedDistanceL += QEM[LOld * 4 + LNew];
    Lfired = 1;
  }
}

// Right Hall encoder. Called once for each sensor on pin-change (quadrature)
void encoderRight_callback(void){
  if(Rrun){
    ROld = RNew;
    RNew = digitalRead(encoderRA) * 2 + digitalRead(encoderRB); // Convert binary input to decimal value // (PINC & 0b0001) gets the value of bit zero in port C (A0), ((PINC & 0b0010) >> 1) gets just the value of A1
    encoderPosR += QEM[ROld * 4 + RNew];
    wheelSpeedDistanceR += QEM[ROld * 4 + RNew];
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
  //pinMode(LED_BUILTIN, OUTPUT);
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
  Serial.println("After Delay");
  lastKnownPosL = encoderPosR / stripLPI * 25.4;
  lastKnownPosR = encoderPosL / stripLPI * 25.4;

  Serial.print("Initializing IMU... ");
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
      Serial.print("Initialized");
    }
  }

  Serial.print("Activating Timer");
  // Configure Speed calculation interrupt with Timer1
  //Timer1->setOverflow(100000, MICROSEC_FORMAT); // 10 Hz // HERTZ_FORMAT
  delay(500);
  noInterrupts();
      Timer1->setOverflow(100000, MICROSEC_FORMAT);
      Timer1->attachInterrupt(speedCalc_callback);
  interrupts();
  Timer1->resume();//*/
  start_time = micros();
}

void loop(){
  const int desiredDistance = fullRev;
  
  if(running == 1){

    /*if(Lfired){
      Serial.print(wheelSpeedL);
      Serial.print("\t");
      Serial.println(encoderPosL);
      Lfired = 0;
    }//*/
    //Serial.print(counter1);
    //Serial.print("\t");
    /*if (myICM.dataReady()){
      myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
      
      //printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
      printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
      delay(30);

    }else{
      Serial.println("Waiting for data");
      delay(500);
    }*/
    //Serial.print(wheelSpeedDistanceL);
    //Serial.print("\t");
    //Serial.println(encoderPosL);
    //Serial.print("\t");
    /*if(speedCheck){
      Serial.print(wheelSpeedL);
      Serial.print("\t");
      Serial.print(speedL_pwm);
      Serial.print("\t");
      Serial.println(wheelSpeedR);
      speedCheck = 0;
    }*/

    /*if(wheelSpeedDistanceR >= desiredDistance){
      if(Rrun){
        Rrun = 0;
      }
      motorR.brake();
    }else{
      //motorR.drive(150);
      wheelSpeedL_desired = 10;
    }//*/


//wheelSpeedL = 60 * (encoderPosL/fullRev)/0.01;

    Lrun = 1;
    pidLeft.Compute();
    for(int i=0; i <= 250; i++){
      wheelSpeedL_desired = map(i, 0, 250, 0, 20);
      //Serial.print("Left speed set to ");
      Serial.print(encoderPosL);
      Serial.print("\t");
      Serial.print(fullRev);
      Serial.print("\t");
      Serial.print((encoderPosL/fullRev));
      Serial.print("\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.print(wheelSpeedL_desired);
      Serial.print("\t");
      Serial.print(wheelSpeedL);
      Serial.print("\t");
      Serial.println(speedL_pwm);
      motorL.drive(i);
      delay(5);
    }
    for(int i=250; i >= 0; i--){
      //Serial.print("Left speed set to ");
      wheelSpeedL_desired = map(i, 0, 250, 0, 20);
      Serial.print(encoderPosL);
      Serial.print("\t");
      Serial.print(fullRev);
      Serial.print("\t");
      Serial.print((encoderPosL/fullRev));
      Serial.print("\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.print(wheelSpeedL_desired);
      Serial.print("\t");
      Serial.print(wheelSpeedL);
      Serial.print("\t");
      Serial.println(speedL_pwm);
      motorL.drive(i);
      delay(5);
    }
    running = 0;
    Lrun = 0;

    /*if(wheelSpeedDistanceL >= desiredDistance){
      if(Lrun){
        Lrun = 0;
      }
      motorL.brake();
    }else{
      wheelSpeedL_desired = 10;
      //motorL.drive(150);
    }*/

    //if((wheelSpeedDistanceL >= desiredDistance) && (wheelSpeedDistanceR >= desiredDistance)){
      //Timer1->pause();
      /*Serial.print(wheelSpeedDistanceR);
      Serial.print("\t");
      Serial.println(wheelSpeedDistanceR);
      Serial.print("Both motors stopped at distance: Left: ");
      Serial.print((wheelSpeedDistanceL/desiredDistance) * wheelCirc);
      Serial.print("m ");
      Serial.print((wheelSpeedDistanceR/desiredDistance) * wheelCirc);
      Serial.println("m ");
      
      //running = 0;
      end_time = micros();
      Serial.print(end_time);
      Serial.print(" - ");
      Serial.print(start_time);
      Serial.print(" = ");
      Serial.print(end_time-start_time);
      Serial.println("us");

      Serial.print("Average Speed (L R): ");
      Serial.print(speedTotalL/counter1);
      Serial.print(" : ");
      Serial.println(speedTotalR/counter1);*/
      delay(2000);
    //}
  }else{
    brake(motorL, motorR);
  }
}
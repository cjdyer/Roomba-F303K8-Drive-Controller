#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <ICM_20948.h> // Sparkfun ICM_20948 IMU module
#include <PID_v1.h>
#include "motorClass.h"

// Hardware Timer check
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
  #error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

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

/* Strip lines per inch */
#define stripLPI 150.0

/* IMU definitions */

#define SPI_PORT SPI  // 13 // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10     // Which pin you connect CS to. Used only when "USE_SPI" is defined

//#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
//#define AD0_VAL 1      // The value of the last bit of the I2C address.                
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                       // the ADR jumper is closed the value becomes 0

// Configure SPI for IMU
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object

//volatile long encoderPosL = 0;
//volatile long encoderPosR = 0;

// Ticks per drive-shaft revolution (different per motor/gearbox)
const float fullRev         = 1632.67;
const float wheelCirc       = 3.14 * 0.8; // Circumference in metres
const float rpm_coefficient = ((600 / 0.05) / fullRev);

// PID Configuration parameters
// Specify the links and initial tuning parameters
struct motorPID {
  volatile long encoderPos     = 0; // Current encoder position since the last clearance
  double speedTotal            = 0; // Sum of speed measurements
  double speedError            = 0; // Difference between 
  double speedError_pre        = 0; // Error 
  double speedErrorSum         = 0; // Sum of errors
  double speed                 = 0; // Calculated wheel speed in RPM
  double speedDesired          = 0; // PID Set Point
  double speedPWM              = 0; // RPM
  double kp                    = 0; //0.02; // Proportional coefficient
  double ki                    = 0; //16.0; // Integral coefficient
  double kd                    = 0; //0.01; // Derivative coefficient

  float  lastKnownPos          = 0; // Last Known Position
  float  ticks_per_millisecond = 0;
  
  long   wheelSpeedDistance    = 0; // RPM
} motorL_PID, motorR_PID;

//******************************//
//******* MOTOR SETUP **********//
//******************************//
const int offsetA = 1; //  Set offset values to adjust motor direction if necessary. Values: 1 or -1
const int offsetB = 1;

// Initialise motor objects
Motor motorL = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
Motor motorR = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

//******************************//
//******* PID SETUP ************//
//******************************//
PID pidLeft (&motorL_PID.speed, &motorL_PID.speedPWM, &motorL_PID.speedDesired, motorL_PID.kp, motorL_PID.ki, motorL_PID.kd, P_ON_M, DIRECT);
PID pidRight(&motorR_PID.speed, &motorR_PID.speedPWM, &motorR_PID.speedDesired, motorR_PID.kp, motorR_PID.ki, motorR_PID.kd, P_ON_M, DIRECT);

//******************************//
//******* TIMER SETUP **********//
//******************************//
TIM_TypeDef *Instance1 = TIM2;
HardwareTimer *Timer1 = new HardwareTimer(Instance1);

//******************************//
//**** TACHOMETER SETUP ********//
//******************************//
tachoWheel tachoR_o;
tachoWheel tachoL_o;

//******************************//
//******* EVENT FLAGS **********//
//   
//  Flags for the encoder triggers 
//  (Mainly for debugging)
//******************************//
bool running    = 1;
bool Lfired     = 0; // Left encoder has fired
bool Rfired     = 0; // Left encoder has fired
bool Lrun       = 1; // Process left encoder
bool Rrun       = 1; // Process right encoder
bool speedCheck = 0; // Timer speed calculator flag

int iter        = 1; // Loop counter
int iter_coeff  = 1;

int counter_tmp = 0; // Encoder debugging

//******************************//
//******* TIMER SETUP **********//
//  Quadrature Encoder matrix
//  2s shouldn't ever appear
//  Sauce: https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
//******************************//
int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
int LOld = 0; // Previous encoder value left motor
int ROld = 0; // Previous encoder value right motor
int LNew = 0; // New encoder value left motor
int RNew = 0; // New encoder value right motor


// Speed Calc Callback
void speedCalc_callback(void){
  tachoL_o.calcVelocity();
  tachoR_o.calcVelocity();
  motorL_PID.speed = tachoL_o.getVelocity();
  motorR_PID.speed = tachoR_o.getVelocity();
  speedCheck  = 1;
}

// Hall encoder ISRs. Called once for each sensor on pin-change (quadrature)
void encoderLeft_callback(void) { tachoL_o.encoderTick(); }
void encoderRight_callback(void){ tachoR_o.encoderTick(); }

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
  //Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial.print("\t");
  printFormattedFloat(sensor->temp(), 5, 2);
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
  attachInterrupt(encoderLA, encoderLeft_callback,  CHANGE);
  attachInterrupt(encoderLB, encoderLeft_callback,  CHANGE);
  attachInterrupt(encoderRA, encoderRight_callback, CHANGE);
  attachInterrupt(encoderRB, encoderRight_callback, CHANGE);
  Serial.println("Motor Pin Configured");

  brake(motorL, motorR);
  Serial.println("Motor Brakes Applied");

  delay(100);
  Serial.println("After Delay");
  motorL_PID.lastKnownPos = motorL_PID.encoderPos / stripLPI * 25.4;
  motorR_PID.lastKnownPos = motorR_PID.encoderPos / stripLPI * 25.4;

  pidLeft.SetMode(AUTOMATIC); //start calculation.
  pidLeft.SetOutputLimits(0,255);
  pidLeft.SetSampleTime(20);
  
  pidRight.SetMode(AUTOMATIC); //start calculation.
  pidRight.SetOutputLimits(0,255);
  pidRight.SetSampleTime(10);

  // P I D
  pidLeft. SetTunings(0.8,  11.0, 0.1);
  pidRight.SetTunings(0.05, 18.0, 0.01);

  //pidLeft. SetTunings(9.451950723484504, 8.592682475895003, 0.09451950723484505);
  //pidRight.SetTunings(0.02, 18.0, 0.01);

  //Serial.print("Initializing IMU... ");
  // Initialise IMU with SPI
  
  //*
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
  }//*/

  Serial.println("Activating Timer");
  // Configure Speed calculation interrupt with Timer1
  // Timer1->setOverflow(100000, MICROSEC_FORMAT); // 10 Hz // HERTZ_FORMAT
  delay(500);
  noInterrupts();
      //Timer1->setOverflow(100000, MICROSEC_FORMAT);
      Timer1->setOverflow(60, HERTZ_FORMAT);
      Timer1->attachInterrupt(speedCalc_callback);
  interrupts();
  Timer1->resume();//*/
}

// Print value (little endian) as binary to serial
void printBin(uint16_t input){
  for(int i = 15; i >= 0; i--){
    Serial.print((input >> i) & 1);
    if(i == 4 || i == 8 || i == 12){
      Serial.print(" ");
    }
  }
}

int stall = 50;

void loop(){
  if(running == 1){
    if(stall-- > 0){
      //Serial.print(0.001); // Tachometer
      //Serial.print("\t");
      //Serial.print(0); // Tachometer
      //Serial.print("\t");
      //Serial.println(0); // Tachometer
      motorL_PID.speedDesired = 140;
      motorR_PID.speedDesired = 140;
      delay(30);
    
    }else if(iter++ < 800){
    //}else if(iter > 0){
      /*
      if(iter == 600){
        if(stall-- > 0){
        }else{
          iter--;
          iter_coeff = -1;
        }
        
      }else{
        iter += iter_coeff;
      }
      //*/
      
      //motorL_PID.speedDesired = map(iter, 0, 600, 0, 140);
      //motorR_PID.speedDesired = map(iter, 0, 600, 0, 140);

      //*
      switch(iter){
        //case 200: motorL_PID.speedDesired = 100; motorR_PID.speedDesired = 100; break;
        case 200: motorL_PID.speedDesired = 50;  motorR_PID.speedDesired = 50; break;
        case 300: motorL_PID.speedDesired = 100;  motorR_PID.speedDesired = 100; break;
        //case 800: motorL_PID.speedDesired = 40;  motorR_PID.speedDesired = 40; break;
        default: break;
      }
      //*/
      
      //*
      pidLeft.Compute();
      pidRight.Compute();

      motorL.drive(motorL_PID.speedPWM); // Output
      motorR.drive(motorR_PID.speedPWM); // Output

      // Print information on the serial monitor
      Serial.print(motorL_PID.speedDesired); // Tachometer
      Serial.print("\t");
      Serial.print(motorL_PID.speed); // Tachometer
      Serial.print("\t");
      Serial.print(motorR_PID.speed); // Tachometer
      Serial.print("\t");
      //Serial.println();
      //*/
      
      if(myICM.dataReady()){
        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                                 //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
        printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
      }

      //motorL.drive(map(motorL_PID.speedPWM, 0, 130, 0, 250)); // Output
      delay(30);

    //*
    }else{
      Timer1->pause();
      //running = 0;
      motorR_PID.speedDesired = 0;
      motorL_PID.speedDesired = 0;
      motorL.drive(0); // Output
      motorR.drive(0); // Output
    }
    //*/

  }else{
    //brake(motorL, motorR);
    if(Lfired){
      Lfired = 0;
    }
  }
}
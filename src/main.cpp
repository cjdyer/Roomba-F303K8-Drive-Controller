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
#define SPI_PORT SPI  // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 10     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                
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
  double kp                    = 2.0; // Proportional coefficient
  double ki                    = 5.0; // Integral coefficient
  double kd                    = 1.0; // Derivative coefficient
  
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
PID pidLeft(&motorL_PID.speed, &motorL_PID.speedPWM, &motorL_PID.speedDesired, motorL_PID.kp, motorL_PID.ki, motorL_PID.kd, DIRECT);
PID pidRight(&motorR_PID.speed, &motorR_PID.speedPWM, &motorR_PID.speedDesired, motorR_PID.kp, motorR_PID.ki, motorR_PID.kd, DIRECT);

//******************************//
//******* TIMER SETUP **********//
//******************************//
TIM_TypeDef *Instance1 = TIM2;
HardwareTimer *Timer1 = new HardwareTimer(Instance1);

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

int counter_temp = 0;

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

unsigned long RPML = 0;
unsigned long RPMR = 0;




//******************************//
//**** TACHOMETER SETUP ********//
//******************************//
struct tacho
{
  const uint16_t          PulsesPerRevolution = 1633; // Encoder pulses for 1 full rotation (Quadrature)
  const unsigned long     ZeroTimeout = 100000;       // If the period between pulses is too high, or even if the pulses stopped, then we would get stuck showing the
                                                      // last value instead of a 0. Because of this we are going to set a limit for the maximum period allowed.
                                                      // If the period is above this value, the RPM will show as 0.
                                                      // The higher the set value, the longer lag/delay will have to sense that pulses stopped, but it will allow readings
                                                      // at very low RPM.
                                                      // Setting a low value is going to allow the detection of stop situations faster, but it will prevent having low RPM readings.
                                                      // The unit is in microseconds.
  const byte              numReadings = 10;            // Number of samples for smoothing. The higher, the more smoothing, but it's going to
                                                      // react slower to changes. 1 = no smoothing. Default: 2.
  volatile unsigned long  LastTimeWeMeasured;         // Time at the last pulse
  volatile unsigned long  PeriodBetweenPulses = ZeroTimeout+1000; // Stores the period between pulses in microseconds.
                                                                  // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
  volatile unsigned long  PeriodAverage = ZeroTimeout+1000;       // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.

  unsigned long   CurrentMicros = micros(); // Micros at the current cycle
  unsigned long   FrequencyRaw;             // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.
  unsigned long   FrequencyReal;            // Frequency without decimals.
  unsigned long   PeriodSum;                // Stores the summation of all the periods to do the average.
  unsigned long   RPM;                      // Raw RPM without any processing.
  unsigned long   LastTimeCycleMeasure = LastTimeWeMeasured; // Stores the last time we measure a pulse in that cycle.
                                      // We need a variable with a value that is not going to be affected by the interrupt
                                      // because we are going to do math and functions that are going to mess up if the values
                                      // changes in the middle of the cycle.
  unsigned int    PulseCounter = 1;         // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.
  unsigned int    AmountOfReadings = 1;     // Stores the last time we measure a pulse in that cycle.
                                            // We need a variable with a value that is not going to be affected by the interrupt
                                            // because we are going to do math and functions that are going to mess up if the values
                                            // changes in the middle of the cycle.
  unsigned int    ZeroDebouncingExtra;      // Stores the extra value added to the ZeroTimeout to debounce it.

  // Variables for smoothing tachometer:
  unsigned long readings[10];  // The input. // [numReadings]
  unsigned long readIndex;  // The index of the current reading.
  unsigned long total;  // The running total.
  unsigned long average;  // The RPM value after applying the smoothing.
} tachoL, tachoR;

tachoWheel tachoR_o;
tachoWheel tachoL_o;

// Speed Calc Callback
void speedCalc_callback(void){
  if(motorL_PID.encoderPos > 0)
    motorL_PID.speed = motorL_PID.encoderPos * rpm_coefficient; // RPM angular_vel = da/dt * 600 (convert to minutes)    //wheelSpeedL = 600 * (encoderPosL/fullRev)/0.01;
  
  if(motorR_PID.encoderPos > 0)
    motorR_PID.speed = motorR_PID.encoderPos * rpm_coefficient; // RPM

  motorL_PID.speedTotal += motorL_PID.speed;
  motorR_PID.speedTotal += motorR_PID.speed;

  motorL_PID.encoderPos = 0;
  motorR_PID.encoderPos = 0;
  
  speedCheck  = 1;

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

  //Serial.println("tick"); CAN'T USE SERIAL PRINT IN STM32 BOARDS IN INTERRUPTS
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
  attachInterrupt(encoderLA, encoderLeft_callback,  CHANGE);
  attachInterrupt(encoderLB, encoderLeft_callback,  CHANGE);
  attachInterrupt(encoderRA, encoderRight_callback, CHANGE);
  attachInterrupt(encoderRB, encoderRight_callback, CHANGE);
  Serial.println("Motor Pin Configured");

  brake(motorL, motorR);
  Serial.println("Motor Brakes Applied");

  delay(200);
  Serial.println("After Delay");
  motorL_PID.lastKnownPos = motorL_PID.encoderPos / stripLPI * 25.4;
  motorR_PID.lastKnownPos = motorR_PID.encoderPos / stripLPI * 25.4;

  //Serial.print("Initializing IMU... ");
  // Initialise IMU with SPI
  
  /*
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
  }*/

  Serial.println("Activating Timer");
  // Configure Speed calculation interrupt with Timer1
  // Timer1->setOverflow(100000, MICROSEC_FORMAT); // 10 Hz // HERTZ_FORMAT
  delay(500);
  noInterrupts();
      //Timer1->setOverflow(100000, MICROSEC_FORMAT);
      Timer1->setOverflow(50, HERTZ_FORMAT);
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

int stall = 25;

void loop(){
  if(running == 1){
    pidLeft.Compute();
    if(iter > 0)
    {
      
      if(iter == 250){
        if(stall-- > 0){
        }else{
          iter--;
          iter_coeff = -1;
        }
        
      }else{
        iter += iter_coeff;
      }
      motorR.drive(iter);
      motorL.drive(iter);
      //motorL_PID.speedDesired = map(iter, 0, 250, 0, 20);
      
      delay(20);    
    }
    else
    {
      running = 0;
      Timer1->pause();
      Lrun = 0;
      Rrun = 0;
    }

    RPML = tachoR_o.getVelocity();
    RPMR = tachoL_o.getVelocity();

    // Print information on the serial monitor
    Serial.print(iter);
    Serial.print("\t");
    Serial.print(RPML); // Tachometer
    Serial.print("\t");
    Serial.print(RPMR); // Tachometer
    Serial.println();
  
  }else{
    brake(motorL, motorR);
    if(Lfired){
      Lfired = 0;
    }
  }
}
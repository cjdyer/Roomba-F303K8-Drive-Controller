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

double wheelSpeedL_desired = 0;
double wheelSpeedR_desired = 0;

double speedTotalL = 0;
double speedTotalR = 0;

double speedL_error = 0;
double speedR_error = 0;

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

float ticks_per_millisecondL = 0;
float ticks_per_millisecondR = 0;

// Set offset values to adjust motor direction if necessary. Values: 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Ticks per drive-shaft revolution (different per motor/gearbox)
const float fullRev = 1632.67;
const float wheelCirc = 3.14 * 0.8; // Circumference in metres

// PID Configuration parameters
// Specify the links and initial tuning parameters
struct motorPID {
  double kp = 2.0;
  double ki = 5.0;
  double kd = 1.0;

  
};

struct motorPID motorL_PID;
struct motorPID motorR_PID;


// Initialise motor objects
Motor motorL = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
Motor motorR = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

//PID pidRight(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID pidLeft(&wheelSpeedL, &speedL_pwm, &wheelSpeedL_desired, kp, ki, kd, DIRECT);
TIM_TypeDef *Instance1 = TIM2;
HardwareTimer *Timer1 = new HardwareTimer(Instance1);

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
int iter = 1;

unsigned int start_time;
unsigned int end_time;
bool speedCheck = 0;

float rpm_coefficient = ((600 / 0.05) / fullRev);

const uint16_t PulsesPerRevolution = 1633;
const unsigned long ZeroTimeout = 100000; 
const byte numReadings = 10;
volatile unsigned long LastTimeWeMeasured;
unsigned long CurrentMicros = micros();
volatile unsigned long PeriodBetweenPulses = ZeroTimeout+1000;
volatile unsigned long PeriodAverage = ZeroTimeout+1000;
unsigned long FrequencyRaw;
unsigned long FrequencyReal;
unsigned long RPM;
unsigned int PulseCounter = 1;
unsigned long PeriodSum;
unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra;

// Variables for smoothing tachometer:
unsigned long readings[numReadings];  // The input.
unsigned long readIndex;  // The index of the current reading.
unsigned long total;  // The running total.
unsigned long average;  // The RPM value after applying the smoothing.

// Speed Calc Callback
void speedCalc_callback(void){
  if(encoderPosL > 0)
    wheelSpeedL = encoderPosL * rpm_coefficient; // RPM angular_vel = da/dt * 600 (convert to minutes)    //wheelSpeedL = 600 * (encoderPosL/fullRev)/0.01; 
  
  if(encoderPosR > 0)
    wheelSpeedR = encoderPosR * rpm_coefficient; // RPM

  speedTotalL += wheelSpeedL;
  speedTotalR += wheelSpeedR;

  encoderPosL = 0;
  encoderPosR = 0;
  
  counter1++;
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

// Left Hall encoder. Called once for each sensor on pin-change (quadrature)
void encoderLeft_callback(void){
  /*if(Lrun){
    LOld = LNew;
    // Access the Digital Input Register directly - MUCH faster than digitalRead()
    LNew = ((GPIOA->IDR >> 0) & 1) * 2 + ((GPIOA->IDR >> 1) & 1); // Convert binary input to decimal value // (PINC & 0b0001) gets the value of bit zero in port C (A0), ((PINC & 0b0010) >> 1) gets just the value of A1
    encoderPosL += QEM[LOld * 4 + LNew];
    wheelSpeedDistanceL += QEM[LOld * 4 + LNew];
    Lfired = 1; 
  }*/
  
  PeriodBetweenPulses   = micros() - LastTimeWeMeasured;
  LastTimeWeMeasured    = micros();
  
  if(PulseCounter >= AmountOfReadings)  // If counter for amount of readings reach the set limit:
  {
    PeriodAverage = PeriodSum / AmountOfReadings;
    PulseCounter = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSum = PeriodBetweenPulses;  // Reset PeriodSum to start a new averaging operation.

    int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
    // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
    // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
    // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
    // 4th and 5th values are the amount of readings range.
    RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
    AmountOfReadings = RemapedAmountOfReadings;  // Set amount of readings as the remaped value.
  }
  else
  {
    PulseCounter++;  // Increase the counter for amount of readings by 1.
    PeriodSum = PeriodSum + PeriodBetweenPulses;  // Add the periods so later we can average.
  }
} // end encoderLeft_callback

// Right Hall encoder. Called once for each sensor on pin-change (quadrature)
void encoderRight_callback(void){
  /*if(Rrun){
    ROld = RNew;
    // Access the Digital Input Register directly - MUCH faster than digitalRead()
    // encoderRA = 4th bit of GPIO port A, encoderRB = 5th bit
    RNew = ((GPIOA->IDR >> 3) & 1) * 2 + ((GPIOA->IDR >> 4) & 1); // Convert binary input to decimal value // (PINC & 0b0001) gets the value of bit zero in port C (A0), ((PINC & 0b0010) >> 1) gets just the value of A1
    encoderPosR += QEM[ROld * 4 + RNew];
    wheelSpeedDistanceR += QEM[ROld * 4 + RNew];
    Rfired = 1;
  }*/
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
  attachInterrupt(encoderLA, encoderLeft_callback,  CHANGE);
  attachInterrupt(encoderLB, encoderLeft_callback,  CHANGE);
  attachInterrupt(encoderRA, encoderRight_callback, CHANGE);
  attachInterrupt(encoderRB, encoderRight_callback, CHANGE);
  Serial.println("Motor Pin Configured");

  brake(motorL, motorR);
  Serial.println("Motor Brakes Applied");

  delay(200);
  Serial.println("After Delay");
  lastKnownPosL = encoderPosR / stripLPI * 25.4;
  lastKnownPosR = encoderPosL / stripLPI * 25.4;

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
  }
*/
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
  start_time = micros();
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

int iter_coeff = 1;

void loop(){
  if(running == 1){

    pidLeft.Compute();
    if(iter > 0)
    {
      iter += iter_coeff;
      wheelSpeedL_desired = map(iter, 0, 250, 0, 20);
      motorL.drive(iter);
      //motorR.drive(iter);
      if(iter == 255){
        iter_coeff = -1;
        //Serial.println("Switch dir");
      }
      delay(20);
    
    }
    else
    {
      running = 0;
      Timer1->pause();
      Lrun = 0;
      Rrun = 0;
    }

    // The following is going to store the two values that might change in the middle of the cycle.
    // We are going to do math and functions with those values and they can create glitches if they change in the
    // middle of the cycle.
    LastTimeCycleMeasure = LastTimeWeMeasured;  // Store the LastTimeWeMeasured in a variable.
    CurrentMicros = micros();  // Store the micros() in a variable.

    // CurrentMicros should always be higher than LastTimeWeMeasured, but in rare occasions that's not true.
    // I'm not sure why this happens, but my solution is to compare both and if CurrentMicros is lower than
    // LastTimeCycleMeasure I set it as the CurrentMicros.
    // The need of fixing this is that we later use this information to see if pulses stopped.
    if(CurrentMicros < LastTimeCycleMeasure)
    {
      LastTimeCycleMeasure = CurrentMicros;
    }

    // Calculate the frequency:
    FrequencyRaw = 10000000000 / PeriodAverage;  // Calculate the frequency using the period between pulses.

    // Detect if pulses stopped or frequency is too low, so we can show 0 Frequency:
    if(PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra)
    {  // If the pulses are too far apart that we reached the timeout for zero:
      FrequencyRaw = 0;  // Set frequency as 0.
      ZeroDebouncingExtra = 2000;  // Change the threshold a little so it doesn't bounce.
    }
    else
    {
      ZeroDebouncingExtra = 0;  // Reset the threshold to the normal value so it doesn't bounce.
    }

    FrequencyReal = FrequencyRaw / 10000;  // Get frequency without decimals.
                                        // This is not used to calculate RPM but we remove the decimals just in case
                                        // you want to print it.

    // Calculate the RPM:
    RPM = FrequencyRaw / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by
                                                // 60 seconds to get minutes.
    RPM = RPM / 10000;  // Remove the decimals.

    // Smoothing RPM:
    total = total - readings[readIndex];  // Advance to the next position in the array.
    readings[readIndex] = RPM;            // Takes the value that we are going to smooth.
    total = total + readings[readIndex];  // Add the reading to the total.
    readIndex = readIndex + 1;            // Advance to the next position in the array.

    if (readIndex >= numReadings)  // If we're at the end of the array:
    {
      readIndex = 0;  // Reset array index.
    }

    // Calculate the average:
    average = total / numReadings;  // The average value it's the smoothed result.

    // Print information on the serial monitor:
    // Comment this section if you have a display and you don't need to monitor the values on the serial monitor.
    // This is because disabling this section would make the loop run faster.
    Serial.print(iter);
    Serial.print("\t");
    Serial.print(RPM);
    Serial.print("\t");
    Serial.println(average); // Tachometer

// ###########################################
// ###########################################
// ###########################################

  }
  else
  {
    brake(motorL, motorR);
    if(Lfired){
      Lfired = 0;
    }
  }
}
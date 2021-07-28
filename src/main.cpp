#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <ICM_20948.h> // Sparkfun ICM_20948 IMU module
#include <PID_v1.h>
#include "motorClass.h"
#include "utilityFuncs.h"

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

// Configure SPI for IMU
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object

const float fullRev         = 1632.67;    // Ticks per drive-shaft revolution (different per motor/gearbox)
const float wheelCirc       = 3.14 * 0.8; // Circumference in metres
//const float rpm_coefficient = ((600 / 0.05) / fullRev);

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

enum sysModes {IDLE, TEST_IMU, TEST_DRIVE, TEST_DRIVE_SPEED, ACTIVE, STOPPED};
int sysMode = IDLE;

//******************************//
//******* MOTOR SETUP **********//
//******************************//
const int offsetA = 1; //  Set offset values to adjust motor direction if necessary. Values: 1 or -1
const int offsetB = 1;

// Initialise motor objects
Motor motorL = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // Left Motor
Motor motorR = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // Right Motor

//******************************//
//******* PID SETUP ************//
//******************************//
PID pidLeft (&motorL_PID.speed, &motorL_PID.speedPWM, &motorL_PID.speedDesired, motorL_PID.kp, motorL_PID.ki, motorL_PID.kd, P_ON_M, DIRECT);
PID pidRight(&motorR_PID.speed, &motorR_PID.speedPWM, &motorR_PID.speedDesired, motorR_PID.kp, motorR_PID.ki, motorR_PID.kd, P_ON_M, DIRECT);

//******************************//
//******* TIMER SETUP **********//
//******************************//
TIM_TypeDef   *Instance1 = TIM2; // Creates an instance of hardware Timer 2
HardwareTimer *Timer1 = new HardwareTimer(Instance1);

//******************************//
//**** TACHOMETER SETUP ********//
//******************************//
tachoWheel tachoR_o; // Right Tachometer Object (Will be integrated into the motor class)
tachoWheel tachoL_o; // Right Tachometer Object (Will be integrated into the motor class)

//******************************//
//******** ITERATORS ***********//
//******************************//
int iter        = 1;         // Loop counter
int iter_coeff  = 1;         // Multiplier for the loop counter - Changes to negative to provide a triangular iterator

//******************************//
//****** SERIAL CONFIG *********//
//******************************//
char payload[100];           // Incoming serial payload
bool stringComplete = false; // whether the string is complete
char modeSelect;             // Mode select variable - Populated by the first character of the incoming packet

// Speed Calc Callback
void speedCalc_callback(void){
  tachoL_o.calcVelocity();
  tachoR_o.calcVelocity();
  motorL_PID.speed = tachoL_o.getVelocity();
  motorR_PID.speed = tachoR_o.getVelocity();
}

// Hall encoder ISRs. Called once for each sensor on pin-change (quadrature)
void encoderLeft_callback(void) { tachoL_o.encoderTick(); }
void encoderRight_callback(void){ tachoR_o.encoderTick(); }

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if (val < 0)
    Serial.print("-");
  else
    Serial.print(" ");
  
  for (uint8_t indi = 0; indi < leading; indi++){
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
      tenpow = 1;
    
    for(uint8_t c = 0; c < (leading - 1 - indi); c++)
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

  Serial.println("Initializing IMU... ");
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
      Serial.println("Initialized");
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



int stall = 50;
int idx = 0; // Index for serial reading

// This is called immediately before every iteration of loop() to process any serial packets
void serialEvent(){
  while(Serial.available()){
    // get the new byte:
    char inChar = (char)Serial.read();
    
    // Save the first character (mode select)
    if(idx == 0){
      modeSelect = inChar;
    
    }else{
      // Add remaining chars to the payload
      payload[idx-1] = inChar;
      Serial.println(idx-1);
    }
    
    idx++;
    // Check for null terminator
    if (inChar == '\n'){
      stringComplete = true;
      idx = 0;
    }
  }
}

void loop(){
    // If a full packet has been captured at the beginning of the loop, process the result
    if(stringComplete){
            if(modeSelect == 'D'){ Serial.println("Test Drive\n"); Serial.print("Speed (payload): "); printString(payload);
        sysMode = TEST_DRIVE_SPEED;
      
      }else if(modeSelect == 'I'){ Serial.println("IMU Test");
        sysMode = TEST_IMU;

      }else if(modeSelect == 'E'){ Serial.println("Reset to Idle");
        sysMode = IDLE;

      }else if(modeSelect == 'S'){
        sysMode = IDLE;
      
      }else if(modeSelect == 'X'){
        Serial.println("System Halted");
        sysMode = STOPPED;
      }
      stringComplete = false;
    }
    
    if(sysMode == IDLE){
      
      
    }else if(sysMode == TEST_IMU){
      //*
      if(iter++ < 800){
        if(myICM.dataReady()){
          myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                                  //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
          printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
        }
      }else{
        sysMode = IDLE;
        iter = 0;
      }
      //*/
    }else if(sysMode == TEST_DRIVE_SPEED){
        
      if(payload > 0){
        motorL_PID.speedDesired = atof(payload);
        motorR_PID.speedDesired = atof(payload);

        pidLeft .Compute();
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
        Serial.println();

      }else{
        Serial.println("Brake both motors");
        motorL.drive(0); // Output
        motorR.drive(0); // Output
        delay(20);
        brake(motorL, motorR);
      }

    }else if(sysMode == TEST_DRIVE){
      Serial.println("Test Drive");
      sysMode = IDLE;
      //*
      if(stall-- > 0){
        Serial.print(0.001); // Tachometer
        Serial.print("\t");
        Serial.print(0); // Tachometer
        Serial.print("\t");
        Serial.println(0); // Tachometer
        motorL_PID.speedDesired = 140;
        motorR_PID.speedDesired = 140;
        delay(30);

      }else if(iter++ < 800){
        //}else if(iter > 0){
        /*if(iter == 600){
          if(stall-- > 0){
          }else{
            iter--;
            iter_coeff = -1;
          }
          
        }else{
          iter += iter_coeff;
        }*/
        
      }else{
        sysMode = STOPPED;
      }

      //motorL_PID.speedDesired = map(iter, 0, 600, 0, 140);
      //motorR_PID.speedDesired = map(iter, 0, 600, 0, 140);
      
      //*
      switch(iter){
        case 200: motorL_PID.speedDesired = 80; motorR_PID.speedDesired = 80; break;
        case 300: motorL_PID.speedDesired = 100; motorR_PID.speedDesired = 100; break;
        case 700: motorL_PID.speedDesired = 40;  motorR_PID.speedDesired = 40; break;
        default: break;
      }

      pidLeft .Compute();
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
        Serial.println();
      //*/

    }else if(sysMode == STOPPED){
      motorR_PID.speedDesired = 0;
      motorL_PID.speedDesired = 0;
      motorL.drive(0); // Output
      motorR.drive(0); // Output

    }else if(sysMode == ACTIVE){
      
    }
    delay(30);
}
#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <ICM_20948.h> // Sparkfun ICM_20948 IMU module
#include <PID_v1.h>
#include "motorClass.h"
#include "utilityFuncs.h"

// Another example of pull request

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

// System states/modes
enum sysModes {IDLE, TEST_IMU, TEST_DRIVE, TEST_DRIVE_SPEED, ACTIVE, STOPPED};
int sysMode = IDLE; // Current system state

//******************************//
//******* MOTOR SETUP **********//
//******************************//
const int offsetA = 1; //  Set offset values to adjust motor direction if necessary. Values: 1 or -1
const int offsetB = 1;

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
HardwareTimer *Timer = new HardwareTimer(Instance1);

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
char modeSelect;             // Mode select variable - Populated by the first character of the incoming packet
bool stringComplete = false; // Serial string completion

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

// Print the formatted IMU variables
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

// Print the scaled and cleaned IMU data
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
  Serial.print("Configuring pins and attaching interrupts... ");
  pinMode (encoderLA, INPUT);
  pinMode (encoderLB, INPUT);
  pinMode (encoderRA, INPUT);
  pinMode (encoderRB, INPUT);
  digitalWrite(encoderLA, HIGH);
  digitalWrite(encoderLB, HIGH);
  digitalWrite(encoderRA, HIGH);
  digitalWrite(encoderRB, HIGH);
  

  // Attach hardware interrupts to encoder pins
  attachInterrupt(encoderLA, encoderLeft_callback,  CHANGE);
  attachInterrupt(encoderLB, encoderLeft_callback,  CHANGE);
  attachInterrupt(encoderRA, encoderRight_callback, CHANGE);
  attachInterrupt(encoderRB, encoderRight_callback, CHANGE);
  
  // Halt both motors
  brake(motorL, motorR);
  Serial.println("Done - Brakes applied");

  delay(50);
  Serial.print("Initialising PID controllers... ");
  pidLeft.SetMode(AUTOMATIC); //start calculation.
  pidLeft.SetOutputLimits(-250,250);
  pidLeft.SetSampleTime(20);
  
  pidRight.SetMode(AUTOMATIC); //start calculation.
  pidRight.SetOutputLimits(-250,250);
  pidRight.SetSampleTime(20);
  Serial.println("Done");

  Serial.print("Setting PID characteristics... ");
  pidLeft. SetTunings(0.8,  11.0, 0.1); // kP, kI, kD
  pidRight.SetTunings(0.05, 18.0, 0.01);
  Serial.println("Done");

  Serial.println("Initializing IMU... ");
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
      Serial.println("IMU initialized");
    }
  }

  Serial.print("Configuring Timer... ");
  delay(500);
  noInterrupts();
  Timer->setOverflow(60, HERTZ_FORMAT); // Read the tachometers 60 times per second
  Timer->attachInterrupt(speedCalc_callback);
  interrupts();
  
  Timer->resume();//*/
  Serial.println("Done - Timer Active");
}

int stall = 50; // A delay value for the top of the testing triangle
int idx = 0;    // Serial buffer index

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
      if(modeSelect == 'A'){ Serial.println("Reset to Idle");
        sysMode = IDLE;

      }else if(modeSelect == 'B'){ Serial.println("IMU Test");
        sysMode = TEST_IMU;
      
      }else if(modeSelect == 'C'){ Serial.println("Test Drive\n"); Serial.print("Speed (payload): "); printString(payload);
        sysMode = TEST_DRIVE_SPEED;

      }else if(modeSelect == 'D'){ Serial.println("Test Drive Sequence\n");
        sysMode = TEST_DRIVE;
      
      }else if(modeSelect == 'X'){
        Serial.println("System Halted");
        sysMode = STOPPED;
      }
      stringComplete = false;
    }
    
          if(sysMode == IDLE){
      motorR_PID.speedDesired = 0;
      motorL_PID.speedDesired = 0;
      motorL.drive(0); // Output
      motorR.drive(0); // Output
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
        
        // TODO NOT WORKING IN REVERSE
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
        sysMode = IDLE;
      }

      // Used for the triangle test
      //motorL_PID.speedDesired = map(iter, 0, 600, 0, 140);
      //motorR_PID.speedDesired = map(iter, 0, 600, 0, 140);
      
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
      brake(motorL, motorR);

    }
    delay(30);
}
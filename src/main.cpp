// Standard Includes
#include <Arduino.h>
// #include <iostream>

// Lib Includes
#include "SparkFun_TB6612.h"
#include "ICM_20948.h" // Sparkfun ICM_20948 IMU module
#include "PID_v1.h"
#include "motorClass.h"
#include "utilityFuncs.h"

// User Includes
#include "pins.h"
#include "motorPID.h"
#include "printManager.h"

// Hardware Timer check
#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

#define SERIAL_SIZE 100 // Serial Buffer MAX
#define stripLPI 150.0  // Strip lines per inch

// Configure SPI for IMU
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object

// constexpr saves memory
constexpr float fullRev = 1632.67;		// Ticks per drive-shaft revolution (different per motor/gearbox)
constexpr float wheelCirc = 3.14 * 0.8; // Circumference in metres

int stall = 50; // A delay value for the top of the testing triangle

// PID Configuration parameters
// Specify the links and initial tuning parameters
motorPID motorL_PID;
motorPID motorR_PID;

// System states/modes
enum SystemState
{
	IDLE,
	TEST_IMU,
	TEST_DRIVE,
	TEST_DRIVE_SPEED,
	ACTIVE,
	STOPPED
};
SystemState systemState = IDLE; // Current system state

//******************************//
//******* MOTOR SETUP **********//
//******************************//
constexpr int offsetA = 1; //  Set offset values to adjust motor direction if necessary. Values: 1 or -1
constexpr int offsetB = 1;

Motor motorL = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // Left Motor
Motor motorR = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // Right Motor

//******************************//
//******* PID SETUP ************//
//******************************//
PID pidLeft(&motorL_PID.speed, &motorL_PID.speedPWM, &motorL_PID.speedDesired, motorL_PID.kp, motorL_PID.ki, motorL_PID.kd, P_ON_M, DIRECT);
PID pidRight(&motorR_PID.speed, &motorR_PID.speedPWM, &motorR_PID.speedDesired, motorR_PID.kp, motorR_PID.ki, motorR_PID.kd, P_ON_M, DIRECT);

//******************************//
//******* TIMER SETUP **********//
//******************************//
TIM_TypeDef *Instance1 = TIM2; // Creates an instance of hardware Timer 2
HardwareTimer *Timer = new HardwareTimer(Instance1);

//******************************//
//**** TACHOMETER SETUP ********//
//******************************//
tachoWheel tachoR_o; // Right Tachometer Object (Will be integrated into the motor class)
tachoWheel tachoL_o; // Right Tachometer Object (Will be integrated into the motor class)

//******************************//
//******** ITERATORS ***********//
//******************************//
int iter = 1;		// Loop counter
int iter_coeff = 1; // Multiplier for the loop counter - Changes to negative to provide a triangular iterator

//******************************//
//****** SERIAL CONFIG *********//
//******************************//
char payload[SERIAL_SIZE];			 // Incoming serial payload
char modeSelect;			 // Mode select variable - Populated by the first character of the incoming packet
bool stringComplete = false; // Serial string completion
char serial_index = 0;	// Serial buffer index

// Speed Calc Callback
void speedCalc_callback(void)
{
	tachoL_o.calcVelocity();
	tachoR_o.calcVelocity();
	motorL_PID.speed = tachoL_o.getVelocity();
	motorR_PID.speed = tachoR_o.getVelocity();
}

// Hall encoder ISRs. Called once for each sensor on pin-change (quadrature)
void encoderLeft_callback(void) { tachoL_o.encoderTick(); }
void encoderRight_callback(void) { tachoR_o.encoderTick(); }

void setup()
{
	Serial.begin(115200);

	while (!Serial) { }

	// Configure Encoder Pins
	//Serial.print("Configuring pins and attaching interrupts... ");
	pinMode(encoderLA, INPUT);
	pinMode(encoderLB, INPUT);
	pinMode(encoderRA, INPUT);
	pinMode(encoderRB, INPUT);
	digitalWrite(encoderLA, HIGH);
	digitalWrite(encoderLB, HIGH);
	digitalWrite(encoderRA, HIGH);
	digitalWrite(encoderRB, HIGH);

	// Attach hardware interrupts to encoder pins
	attachInterrupt(encoderLA, encoderLeft_callback, CHANGE);
	attachInterrupt(encoderLB, encoderLeft_callback, CHANGE);
	attachInterrupt(encoderRA, encoderRight_callback, CHANGE);
	attachInterrupt(encoderRB, encoderRight_callback, CHANGE);

	// Halt both motors
	brake(motorL, motorR);
	//Serial.println("Done - Brakes applied");

	delay(50);
	//Serial.print("Initialising PID controllers... ");
	pidLeft.SetMode(AUTOMATIC); //start calculation.
	pidLeft.SetOutputLimits(-250, 250);
	pidLeft.SetSampleTime(20);

	pidRight.SetMode(AUTOMATIC); //start calculation.
	pidRight.SetOutputLimits(-250, 250);
	pidRight.SetSampleTime(20);
	//Serial.println("Done");

	//Serial.print("Setting PID characteristics... ");
	pidLeft.SetTunings(0.8, 11.0, 0.1); // kP, kI, kD
	pidRight.SetTunings(0.05, 18.0, 0.01);
	//Serial.println("Done");

	//Serial.println("Initializing IMU... ");
	myICM.begin(CS_PIN);

	bool initialized = false;
	while (!initialized)
	{
		//Serial.print(F("Initialization of the sensor returned: "));
		//Serial.println(myICM.statusString());

		if (myICM.status != ICM_20948_Stat_Ok)
		{
			//Serial.println("Trying again...");
			delay(500);
		}
		else
		{
			initialized = true;
			//Serial.println("IMU initialized");
		}
	}

	//Serial.print("Configuring Timer... ");
	delay(500);
	noInterrupts();
	Timer->setOverflow(60, HERTZ_FORMAT); // Read the tachometers 60 times per second
	Timer->attachInterrupt(speedCalc_callback);
	interrupts();

	Timer->resume();
	//Serial.println("Done - Timer Active");
	//Serial.print("s");

	//while ((char)Serial.read() != 's') { }
	//Serial.print("r");
}

// This is called immediately before every iteration of loop() to process any serial packets
void serialEvent()
{
	while (Serial.available())
	{
		char inChar = (char)Serial.read(); // get the new byte:

		if (serial_index == 0)
			modeSelect = inChar; // Save the first character (mode select)
		else
			payload[serial_index - 1] = inChar; // Add remaining chars to the payload
		
		if (inChar == '\n') // Check for null terminator
		{
			stringComplete = true;
			serial_index = 0;
		}
		else 
			serial_index++;
	}
}

void loop()
{
	// If a full packet has been captured at the beginning of the loop, process the result
	if (stringComplete)
	{
		if (modeSelect == 'A')
		{
			Serial.println("Reset to Idle");
			systemState = IDLE;
		}
		else if (modeSelect == 'B')
		{
			Serial.println("IMU Test");
			systemState = TEST_IMU;
		}
		else if (modeSelect == 'C')
		{
			Serial.println("Test Drive\n");
			Serial.print("Speed (payload): ");
			printString(payload);
			systemState = TEST_DRIVE_SPEED;
		}
		else if (modeSelect == 'D')
		{
			Serial.println("Test Drive Sequence\n");
			systemState = TEST_DRIVE;
		}
		else if (modeSelect == 'X')
		{
			Serial.println("System Halted");
			systemState = STOPPED;
		}
		stringComplete = false;
	}

	if (systemState == IDLE)
	{
		motorR_PID.speedDesired = 0;
		motorL_PID.speedDesired = 0;
		motorL.drive(0); // Output
		motorR.drive(0); // Output
	}
	else if (systemState == TEST_IMU)
	{
		//if (iter++ < 5000)
		//{
			if (myICM.dataReady())
			{
				myICM.getAGMT();		 // The values are only updated when you call 'getAGMT'
										 //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
				printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
			}
		//}
		//else
		//{
		//	sysMode = IDLE;
		//	iter = 0;
		//}
		//*/
	}
	else if (systemState == TEST_DRIVE_SPEED)
	{

		if (payload > 0)
		{
			motorL_PID.speedDesired = atof(payload);
			motorR_PID.speedDesired = atof(payload);

			pidLeft.Compute();
			pidRight.Compute();

			// TODO NOT WORKING IN REVERSE
			motorL.drive(motorL_PID.speedPWM); // Output
			motorR.drive(motorR_PID.speedPWM); // Output

			printMotorData(&motorL_PID, &motorR_PID);
		}
		else
		{
			Serial.println("Brake both motors");
			motorL.drive(0); // Output
			motorR.drive(0); // Output
			delay(20);
			brake(motorL, motorR);
		}
	}
	else if (systemState == TEST_DRIVE)
	{
		if (stall-- > 0)
		{
			Serial.print(0.001); // Tachometer
			Serial.print("\t");
			Serial.print(0); // Tachometer
			Serial.print("\t");
			Serial.println(0); // Tachometer
			motorL_PID.speedDesired = 140;
			motorR_PID.speedDesired = 140;
			delay(30);
		}
		else if (iter++ < 800)
		{
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
		}
		else
		{
			systemState = IDLE;
		}

		// Used for the triangle test
		//motorL_PID.speedDesired = map(iter, 0, 600, 0, 140);
		//motorR_PID.speedDesired = map(iter, 0, 600, 0, 140);

		switch (iter)
		{
		case 200:
			motorL_PID.speedDesired = 80;
			motorR_PID.speedDesired = 80;
			break;
		case 300:
			motorL_PID.speedDesired = 100;
			motorR_PID.speedDesired = 100;
			break;
		case 700:
			motorL_PID.speedDesired = 40;
			motorR_PID.speedDesired = 40;
			break;
		default:
			break;
		}

		pidLeft.Compute();
		pidRight.Compute();

		motorL.drive(motorL_PID.speedPWM); // Output
		motorR.drive(motorR_PID.speedPWM); // Output

		printMotorData(&motorL_PID, &motorR_PID); // Print information on the serial monitor
	}
	else if (systemState == STOPPED)
	{
		motorR_PID.speedDesired = 0;
		motorL_PID.speedDesired = 0;
		motorL.drive(0); // Output
		motorR.drive(0); // Output
		brake(motorL, motorR);
	}
	delay(10); // 30 was pretty high. Changed to 10 for smoother communication and operation
}
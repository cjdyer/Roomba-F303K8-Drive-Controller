#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <ICM_20948.h>

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
  #error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

/* MOTOR DEFINITIONS */
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
Motor motorL = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorR = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

bool Lfired = 0;
bool Rfired = 0;

bool running = 1;

// Quadrature Encoder matrix - 2s shouldn't ever appear Sauce: https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; 
int LOld = 0; // Previous encoder value left motor
int ROld = 0; // Previous encoder value right motor
int LNew = 0; // New encoder value left motor
int RNew = 0; // New encoder value right motor

void encoderLeft_callback(){
  LOld = LNew;
  LNew = (digitalRead(encoderLA) * 2) + digitalRead(encoderLB); // Convert binary input to decimal value // (PINC & 0b0001) gets the value of bit zero in port C (A0), ((PINC & 0b0010) >> 1) gets just the value of A1
  encoderPosL = encoderPosL + QEM[LOld * 4 + LNew];
  Lfired = 1;
}

void encoderRight_callback(){
  ROld = RNew;
  RNew = (digitalRead(encoderRA) * 2) + digitalRead(encoderRA); // Convert binary input to decimal value // (PINC & 0b0001) gets the value of bit zero in port C (A0), ((PINC & 0b0010) >> 1) gets just the value of A1
  encoderPosR = encoderPosR + QEM[ROld * 4 + RNew];
  Rfired = 1;
}

/*
// Define Timer instances
TIM_TypeDef *Instance1 = TIM1;
TIM_TypeDef *Instance2 = TIM2;

// Define Timer Objects
HardwareTimer *Timer1 = new HardwareTimer(Instance1);
HardwareTimer *Timer2 = new HardwareTimer(Instance2);
*/

// Callback Functions
/*
void ledFlash_callback(void)  {counter1++; digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));} // Toggle pin. 10hz toogle --> 5Hz PWM
void print_callback(void)     {counter2++;}
void btn1_callback(void)      {counter3++;}
void btn2_callback(void)      {counter4++;}
*/

void setup(){
    Serial.begin(115200);
    
    /*
    // configure pin in output mode
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(5, INPUT_PULLDOWN);
    pinMode(6, INPUT_PULLDOWN);
    noInterrupts();

    Timer1->setOverflow(2, HERTZ_FORMAT); // 10 Hz
    Timer1->attachInterrupt(ledFlash_callback);

    Timer2->setOverflow(1, HERTZ_FORMAT); // 10 Hz
    Timer2->attachInterrupt(print_callback);

    attachInterrupt(5, btn1_callback, RISING);
    attachInterrupt(6, btn2_callback, RISING);
    interrupts();
    Timer1->resume();
    Timer2->resume();

    Serial.println("T1 \t T2 \t D5\t D6");
    */

  // Configure Encoder Pins
  pinMode (encoderLA, INPUT);
  pinMode (encoderLB, INPUT);
  pinMode (encoderRA, INPUT);
  pinMode (encoderRB, INPUT);
  digitalWrite(encoderLA, HIGH);
  digitalWrite(encoderLB, HIGH);
  digitalWrite(encoderRA, HIGH);
  digitalWrite(encoderRB, HIGH);
  attachInterrupt(encoderLA, encoderLeft_callback, RISING);
  attachInterrupt(encoderLB, encoderLeft_callback, RISING);
  attachInterrupt(encoderRA, encoderRight_callback, RISING);
  attachInterrupt(encoderRB, encoderRight_callback, RISING);

  brake(motorL, motorR);

  delay(200);
  lastKnownPosL = encoderPosR / stripLPI * 25.4;
  lastKnownPosL = encoderPosR / stripLPI * 25.4;
}

void loop(){
  if(running == 1){
    /*Serial.print(counter1);
    Serial.print("\t");
    Serial.print(counter2);
    Serial.print("\t");
    Serial.print(counter3);
    Serial.print("\t");
    Serial.println(counter4);

    if(counter3 >= 10){Timer1->pause();}
    if(counter4 >= 10){Timer2->pause();}

    delay(10);*/

    if(encoderPosR > fullRev)
      motorL.brake();
    else
      motorL.drive(100);
    
    if(encoderPosL > fullRev)
      motorR.brake();
    else
      motorR.drive(100);

    if((encoderPosL > fullRev) && (encoderPosR > fullRev)){
      Serial.print("Both motors stopped at distance: Left: ");
      Serial.print((encoderPosL/fullRev) * wheelCirc);
      Serial.print("m ");
      Serial.print((encoderPosR/fullRev) * wheelCirc);
      Serial.print("m ");
      motorR.brake();
      motorL.brake();
      running = 0;
    }
  }else{
    brake(motorL, motorR);
  }
}
#include <Arduino.h>

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
  #error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

unsigned int counter1 = 0;
unsigned int counter2 = 0;
unsigned int counter3 = 0;
unsigned int counter4 = 0;

// Define Timer instances
TIM_TypeDef *Instance1 = TIM1;
TIM_TypeDef *Instance2 = TIM2;

// Define Timer Objects
HardwareTimer *Timer1 = new HardwareTimer(Instance1);
HardwareTimer *Timer2 = new HardwareTimer(Instance2);

// Callback Functions
void ledFlash_callback(void)  {counter1++; digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));} // Toggle pin. 10hz toogle --> 5Hz PWM
void print_callback(void)     {counter2++;}
void btn1_callback(void)      {counter3++;}
void btn2_callback(void)      {counter4++;}

void setup(){

    Serial.begin(115200);
    // Instantiate HardwareTimer object. Thanks to 'new' instanciation, HardwareTimer is not destructed when setup() function is finished.
    //HardwareTimer *MyTim = new HardwareTimer(Instance);
    

    // configure pin in output mode
    pinMode(pin, OUTPUT);
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
}

void loop()
{
  Serial.print(counter1);
  Serial.print("\t");
  Serial.print(counter2);
  Serial.print("\t");
  Serial.print(counter3);
  Serial.print("\t");
  Serial.println(counter4);

  if(counter3 >= 10){
    Timer1->pause();
  }
  
  if(counter4 >= 10){
    Timer2->pause();
  }

  delay(10);
  /* Nothing to do all is done by hardware. Even no interrupt required. */
}
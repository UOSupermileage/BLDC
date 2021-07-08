#include "msp430f5529.h"
#include <Energia.h>
#include "pinDeclerations.h"
#include "coreFunctions.h"
#include "spiFunctions.h"

/*
   UOE Racing: BLDC Motor Controller Program 1.1.0

   CONTRIBUTORS:
     - Atinderpaul Kanwar - ap_kanwar@outlook.com
     - Ryan Fleck - Ryan.Fleck@protonmail.com

   CHANGELOG:
   Version 1.1.2:
     - Update pins for UOE 2020rA board.
     - Temporarily disable other SPI devices.

   Version 1.1.1:
     - Add initMotorState function to read halls during setup.
     - FIX TODO: Jumpstart motor on throttle.
     - Add red running-light to indicate program is ready.
     - Remove next-state code, no longer used.

   Version 1.1.0:
     - Change pins to interrupt-capable pins.
     - Reverse sequence of hall status modification operations.

   Version 1.0.0:
     - Initial implementation of interrupt code.

*/


/*
   UOE 2020rA BOARD SETUP INSTRUCTIONS:

   Yellow bundle:
   A -> White wire.
   B -> Purple wire.
   C -> Orange wire.
*/

using namespace PINS;
using namespace coreFunctions;
//using namespace spiFunctions;

// Constants
#define PWM_VALUE_DEBUG 128 // During testing, this value is used as throttle while the MSP430 buttons are pressed.
#define PWM_FREQUENCY 20000 // 20kHz
#define DEFAULT_THROTTLE_LOOP_COUNT 1000 // must be smaller than 65,535
#define MIN_PWM_VALUE 0
#define MAX_PWM_VALUE 255

//State Variable Values
#define zeroDegrees B110 //6
#define twentyDegrees B010 //2
#define fortyDegrees B011 //3
#define sixtyDegrees B001 //1
#define eightyDegrees B101 //5
#define hundredDegrees B100 //4
#define errorState1 B111
#define errorState2 B000

// Function call repitition macros
#define postInterruptAction() motorSpin()

//Throttle Variables and Pin
int loopCount = 0; //Variable to store number of loops gone through
int loopNum = DEFAULT_THROTTLE_LOOP_COUNT; //Number of loops to trigger a throttle read
int pwm_value = 100; //Throttle value, always start this off at 0

//Motor Variables
volatile byte state = 0; //This variable stores the current state of the motor

//Debug Variables
bool DEBUG = false;


/*
   SETUP CODE
     - Configures MSP430 pins
     - Calls function to set up SPI
     - Sets interrupts
*/
void setup() {
  //Begin by increasing the speed of the chip
  IncreaseClockSpeed_25MHz();

  //Configure SPI devices
  pinMode(CS_TEMP, OUTPUT);
  pinMode(CS_SD, OUTPUT);
  pinMode(CS_DRV, OUTPUT);
  pinMode(EXTERNAL_SPI_ENABLE, OUTPUT);
  pinMode(SCLK, OUTPUT);
  digitalWrite(CS_TEMP, HIGH);
  digitalWrite(CS_SD, HIGH);
  digitalWrite(CS_DRV, HIGH);
  digitalWrite(EXTERNAL_SPI_ENABLE, HIGH);


  //Gate Driver setup
  pinMode(ENABLE, OUTPUT);
  pinMode(nFAULT, INPUT);
  digitalWrite(ENABLE, HIGH);
  analogFrequency(PWM_FREQUENCY); //sets the PWM frequency for the motor controller
  //Control pins for driver setup
  pinMode(LOWA, OUTPUT);
  pinMode(LOWB, OUTPUT);
  pinMode(LOWC, OUTPUT);
  pinMode(HIGHA, OUTPUT);
  pinMode(HIGHB, OUTPUT);
  pinMode(HIGHC, OUTPUT);
  digitalWrite(LOWA, LOW);
  digitalWrite(LOWB, LOW);
  digitalWrite(LOWC, LOW);
  digitalWrite(HIGHA, LOW);
  digitalWrite(HIGHB, LOW);
  digitalWrite(HIGHC, LOW);
  //This is the start of the fault clearing status. It might be a good idea to remove this at a latter date
  delayMicroseconds(20);
  digitalWrite(ENABLE, LOW);
  delayMicroseconds(20);
  digitalWrite(ENABLE, HIGH);
  delayMicroseconds(20);
  digitalWrite(ENABLE, LOW);

  //Motor Hall pin setup
  pinMode(HALLA, INPUT);
  pinMode(HALLB, INPUT);
  pinMode(HALLC, INPUT);

  //Throttle Input setup
  pinMode(THROTTLE, INPUT);

  // Debug stuff
  pinMode(P1_1, INPUT_PULLUP);
  pinMode(P2_1, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  Serial.begin(28800);
  digitalWrite(RED_LED, HIGH);

  //Attach the interupts for the motor state
  attachInterrupt(digitalPinToInterrupt(HALLA), changeSA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLB), changeSB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLC), changeSC, CHANGE);
  interrupts(); //This is for safe keeping in case interupts were disabled by something

  //Configure the driver chip over SPI
  digitalWrite(ENABLE, HIGH);
  DRV8323_SPI_Setup();
  DRV8323_CSA();

  // Set initial state.
  initMotorState();
} // setup() END



/*
   LOOP CODE
*/
volatile int button1_state = 1;
volatile int button2_state = 1;

void loop() {
  delay(30);

  // Button DOWN will be read as 0, not 1!
  button1_state = digitalRead(P1_1);
  button2_state = digitalRead(P2_2);

      if (!button1_state) {
        digitalWrite(GREEN_LED, HIGH);
        motorSpin();
      } else {
        digitalWrite(GREEN_LED, LOW);
      }
  //pwm_value = readThrottle();

} // loop() END

//Throttle Function
int readThrottle(void){
    float rawThrottle = analogRead(THROTTLE);
    rawThrottle = 0.0 + ((255.0 - 0.0) / (4096.0 - 1200.0)) * (rawThrottle - 1200.0);
    return (int) rawThrottle;
}

/*
   MOTOR SPIN FUNCTION
     - Called after an interrupt has been encountered
     - Writes new values to the HALLs
*/
void motorSpin() {
  switch (state) {
    case zeroDegrees: //010
      //setLow('B');
      digitalWrite(LOWB, HIGH);
      digitalWrite(HIGHB, LOW);
      //setHighZ('A');
      digitalWrite(LOWA, LOW);
      digitalWrite(HIGHA, LOW);
      //setHigh('C');
      digitalWrite(LOWC, HIGH);
      analogWrite(HIGHC, pwm_value);
      break;

    case twentyDegrees: //011
      //setLow('B');
      digitalWrite(LOWB, HIGH);
      digitalWrite(HIGHB, LOW);
      //setHighZ('C');
      digitalWrite(LOWC, LOW);
      digitalWrite(HIGHC, LOW);
      //setHigh('A');
      digitalWrite(LOWA, HIGH);
      analogWrite(HIGHA, pwm_value);
      break;

    case fortyDegrees: //001
      //setLow('C');
      digitalWrite(LOWC, HIGH);
      digitalWrite(HIGHC, LOW);
      //setHighZ('B');
      digitalWrite(LOWB, LOW);
      digitalWrite(HIGHB, LOW);
      //setHigh('A');
      digitalWrite(LOWA, HIGH);
      analogWrite(HIGHA, pwm_value);
      break;

    case sixtyDegrees: //101
      //setLow('C');
      digitalWrite(LOWC, HIGH);
      digitalWrite(HIGHC, LOW);
      //setHighZ('A');
      digitalWrite(LOWA, LOW);
      digitalWrite(HIGHA, LOW);
      //setHigh('B');
      digitalWrite(LOWB, HIGH);
      analogWrite(HIGHB, pwm_value);
      break;

    case eightyDegrees: //100
      //setLow('A');
      digitalWrite(LOWA, HIGH);
      digitalWrite(HIGHA, LOW);
      //setHighZ('C');
      digitalWrite(LOWC, LOW);
      digitalWrite(HIGHC, LOW);
      //setHigh('B');
      digitalWrite(LOWB, HIGH);
      analogWrite(HIGHB, pwm_value);
      break;

    case hundredDegrees: //110
      //setLow('A');
      digitalWrite(LOWA, HIGH);
      digitalWrite(HIGHA, LOW);
      //setHighZ('B');
      digitalWrite(LOWB, LOW);
      digitalWrite(HIGHB, LOW);
      //setHigh('C');
      digitalWrite(LOWC, HIGH);
      analogWrite(HIGHC, pwm_value);
      break;

    default :
      coast();
      break;

  } // SWITCH END
}// motorSpin() END



/*
   COAST FUNCTION
     - Removes all forces from the BLDC by disabling FETs, allowing the motor to coast
*/
void coast (void) {
  //setHighZ('A');
  digitalWrite(LOWA, LOW);
  digitalWrite(HIGHA, LOW);
  //setHighZ('B');
  digitalWrite(LOWB, LOW);
  digitalWrite(HIGHB, LOW);
  //setHighZ('C');
  digitalWrite(LOWC, LOW);
  digitalWrite(HIGHC, LOW);
}



/*
   INITMOTORSTATE FUNCTION
     - Checks the halls and sets the state byte at the time of startup.
*/
void initMotorState() {
  state = B000;
  state |= digitalRead(HALLA) ? B100 : B000;
  state |= digitalRead(HALLB) ? B010 : B000;
  state |= digitalRead(HALLC) ? B001 : B000;
}



/*
   INTERRUPT FUNCTIONS:
   changeSX: Interrupt sequence for HALL X.
     Each interrupt function is triggered during a voltage change on the relevant
     pin.
*/
void changeSA() {
  if (digitalRead(HALLA) == HIGH) {
    state |= B100;
  }
  else {
    state &= B001;
  }
  postInterruptAction();
}

void changeSB() {
  if (digitalRead(HALLB) == HIGH) {
    state |= B010;
  }
  else {
    state &= B100;
  }
  postInterruptAction();
}

void changeSC() {
  if (digitalRead(HALLC) == HIGH) {
    state |= B001;
  }
  else {
    state &= B010;
  }
  postInterruptAction();
}

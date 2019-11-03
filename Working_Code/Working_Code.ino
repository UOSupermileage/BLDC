#include <SPI.h>

#define ENABLE 5

// SPI PINS
#define nSCS 12
#define SCLK 7
#define nFAULT 8
#define MISO 14
#define MOSI 15

// HALL PINS
#define HALLA 10
#define HALLB 9
#define HALLC 31

#define LOWA 39
#define LOWB 37
#define LOWC 35
#define HIGHA 40
#define HIGHB 38
#define HIGHC 36

#define aPIN 32

#define THROTTLE 24

// Constants
#define PWM_VALUE_DEBUG 100 // For button-press debugging.
#define PWM_FREQUENCY 20000 // 20kHz
#define DEFAULT_THROTTLE_LOOP_COUNT 1000 // must be smaller than 65,535
#define MIN_PWM_VALUE 0
#define MAX_PWM_VALUE 255

//Throttle Variables and Pin
int loopCount = 0; //Variable to store number of loops gone through
int loopNum = DEFAULT_THROTTLE_LOOP_COUNT; //Number of loops to trigger a throttle read
int pwm_value = 0; //Throttle value, always start this off at 0

//State Variable Values
#define zeroDegrees B110 //6
#define twentyDegrees B010 //2
#define fortyDegrees B011 //3
#define sixtyDegrees B001 //1
#define eightyDegrees B101 //5
#define hundredDegrees B100 //4
#define errorState1 B111
#define errorState2 B000

byte state = errorState1;
byte old_state = state;
byte expected_next_state = state;
int unexpected_jump_cnt = 0;

int desiredTorque = 0;
bool new_state_OK = false;

/* Coast
  Allows the motor to coast by turning off all Fets
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

// SPI SETUP
void controllerSetup(void) {
  delay(200); // Why is this here?
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  SPI.setModule(0);

  // Writing to Control Register
  // Selecting 3xPWM Mode Function, using controller driver control register:
  // BIT#  15  14 13 12 11 10    09        08            07    06 05     04    03        02    01   00
  // Name  R/W < Addr3-0 > res chgpump gatedrivevolt overtemp pwmMode  pwmcom 1pwmdir  coast brake clearfault
  //       W=0 (MSB first)
  // Value  0  Ctrl=0010    0     0         0             0   3xPWM=01    0     0         0     0    0
  // This corresponds to a 16-bit value to put motor controller into 3xPWM mode: 0(001 0)000 0(01)0 0000 => 0x1020
  // This is actually executed as single-byte writes of 0x10 then 0x20

  digitalWrite(nSCS, LOW); //Pull slave select low to begin the transaction
  delayMicroseconds(2);
  SPI.transfer(0x10);
  byte return_value = SPI.transfer(0x20);
  Serial.println(return_value, HEX);
  Serial.println();
  delayMicroseconds(2);
  digitalWrite(nSCS, HIGH); //Pull slave select high to end the transaction

  // delayMicroseconds(10);

  // Reading Control Register
  // Reads back the same driver control register value, to make sure that it got set as required, by setting bit 15 to R/W=1 (read)
  // This corresponds to a 16-bit value: 1(001 0)000 0(01)0 0000 => 0x9020 or byte writes of 0x90 then 0x20

  // digitalWrite(nSCS, LOW); //Pull slave select low to begin the transaction
  // delayMicroseconds(2);
  // SPI.transfer(0x90);
  // return_value = SPI.transfer(0x20);
  // delayMicroseconds(2);
  // digitalWrite(nSCS, HIGH); //Pull slave select high to end the transaction

  SPI.end();
}

// SETUP CODE
void setup() {
  digitalWrite(ENABLE, HIGH);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  delayMicroseconds(2);
  digitalWrite(ENABLE, LOW); // Keep enable low for between 4 and 40 microseconds
  delayMicroseconds(20);
  digitalWrite(ENABLE, HIGH);

  digitalWrite(nSCS, HIGH);
  pinMode(nSCS, OUTPUT);
  digitalWrite(nSCS, HIGH);

  digitalWrite(SCLK, LOW);
  pinMode(SCLK, OUTPUT);
  digitalWrite(SCLK, LOW);

  pinMode(nFAULT, INPUT);
  // pinMode(MISO, INPUT_PULLUP);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);

  pinMode(HALLA, INPUT);
  pinMode(HALLB, INPUT);
  pinMode(HALLC, INPUT);

  digitalWrite(LOWA, LOW);
  digitalWrite(LOWB, LOW);
  digitalWrite(LOWC, LOW);
  digitalWrite(HIGHA, LOW);
  digitalWrite(HIGHB, LOW);
  digitalWrite(HIGHC, LOW);
  pinMode(LOWA, OUTPUT);
  pinMode(LOWB, OUTPUT);
  pinMode(LOWC, OUTPUT);
  pinMode(HIGHA, OUTPUT);
  pinMode(HIGHB, OUTPUT);
  pinMode(HIGHC, OUTPUT);
  pinMode(aPIN, OUTPUT);

  pinMode(THROTTLE, INPUT);
  pinMode(P1_1, INPUT_PULLUP);
  pinMode(P2_1, INPUT_PULLUP);

  state = errorState1;
  old_state = errorState2;
  expected_next_state = errorState2;

  Serial.begin(9600);
  Serial.println("Starting Motor");

  analogFrequency(PWM_FREQUENCY);

  controllerSetup();
}

// LOOP CODE
void loop() {

  // stay in the while loop below unless the Hall sensor values change, then pass through the state machine switch statement once
  // At initial startup, we require the same set of valid Hall sensor values twice in a row before we turn on the motor

  digitalWrite(aPIN, HIGH);

  //Read HALL Values
  int rHallA = digitalRead(HALLA);
  int rHallB = digitalRead(HALLB);
  int rHallC = digitalRead(HALLC);
  //   Serial.println("Hall Input = " + String(rHallA) + String(rHallB) + String(rHallC));

  old_state = state; // Remember the previous state value, so that we can detect a change in the Hall effect sensor inputs

  // Determine a composite new state value using Hall effect sensor inputs in different bit positions
  // Sensor inputs from A, B and C are stored in state bits 2,1 and 0, respectively
  state = 0;
  if (rHallA == HIGH) {
    state = state | B100;  // set bit 2
  }
  if (rHallB == HIGH) {
    state = state | B010;  // set bit 1
  }
  if (rHallC == HIGH) {
    state = state | B001;  // set bit 0
  }

  // Temporary logic using either button on the board to run the motor.
  if (digitalRead(P1_1) & digitalRead(P2_1)) {
    //If neither button has been pressed, throttle can be set to zero.
    pwm_value = 0;
  } else {
    // If one or the other is pressed, the bitwise AND will be zero, so throttle should be set.
    pwm_value = PWM_VALUE_DEBUG;
  }

  //  if(loopCount >= loopNum){
  //    int analogReadValue = analogRead(THROTTLE); // Read the POT Value
  //    int mapValue = map(analogReadValue, 1550, 4096, 0, 255); // Map the pot value, assuming that the range is between 1550-4096 (12-bit value) to 0-255
  //    // If the new map value is ever a negitive integer, change the value to 0
  //    if (mapValue <= MIN_PWM_VALUE) {
  //      mapValue == MIN_PWM_VALUE;
  //    }
  //
  //    // The throttle is never allowed to go over the MAX PWM VALUE
  //    if (mapValue > MAX_PWM_VALUE) {
  //      pwm_value = MAX_PWM_VALUE;
  //    }
  //    else {
  //      pwm_value = (mapValue + pwm_value) / 2; // The PWM value keeps increasing at a rate of 2
  //    }
  //    loopCount=0;
  //  }
  //  //IF you havn't reached loopNum then increment and continue to the next loop
  //  else{
  //    loopCount++;
  //  }

  // Switch-Case Statement follow the REVERSE1 code, (which previously required a push to start spinning the motor clockwise    (Check clockwise_motor brach on GITLAB)
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
      expected_next_state = twentyDegrees;
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
      expected_next_state = fortyDegrees;
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
      expected_next_state = sixtyDegrees;
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
      expected_next_state = eightyDegrees;
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
      expected_next_state = hundredDegrees;
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
      expected_next_state = zeroDegrees;
      break;

    default :
      Serial.println("Error - Set to Coast");
      Serial.println(state);
      coast();
      expected_next_state = state;  // assume the same errored state is the most likely one next
      break;
  } // SWITCH END

  digitalWrite(aPIN, LOW);

} // LOOP END

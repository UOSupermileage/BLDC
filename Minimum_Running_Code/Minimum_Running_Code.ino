#include <SPI.h>
#include "msp430f5529.h"

/*
 * This is a reduced copy of the Polling_Code, for testing basic motor function.
 * Confirmed running 2020-03-15 on 2020 BLDC Driver Board Rev. A
 */

#define ENABLE 5

// TESTING SPEED
#define PWM_VALUE_DEBUG 100

// SPI PINS
#define SCLK 7
#define nFAULT 33

#define CS_TEMP 32
#define CS_SD 11
#define EXTERNAL_SPI_ENABLE 15

#define CS_DRV 8

// HALL PINS
#define HALLA 19
#define HALLB 13
#define HALLC 12

/*
 * A -> White wire.
 * B -> Purple wire.
 * C -> Orange wire.
 */

#define LOWA 39
#define LOWB 37
#define LOWC 35

#define HIGHA 40
#define HIGHB 38
#define HIGHC 36

#define PWM_FREQUENCY 20000 // 20kHz
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


int pwm_value = 0; //Throttle value, always start this off at 0

byte state = errorState1;
byte old_state = state;
byte expected_next_state = state;
int unexpected_jump_cnt = 0;

int desiredTorque = 0;
bool new_state_OK = false;


void setup() {
  pwm_value = PWM_VALUE_DEBUG;

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Debugging lights red when setup is running.

  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);

  delay(200);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);

  // After setup light is on, increase clockspeed.
  

  // Disable other SPI devices:
  pinMode(CS_TEMP, OUTPUT);
  pinMode(CS_SD, OUTPUT);
  pinMode(EXTERNAL_SPI_ENABLE, OUTPUT);
  digitalWrite(CS_TEMP, HIGH);
  digitalWrite(CS_SD, HIGH);
  digitalWrite(EXTERNAL_SPI_ENABLE, HIGH);


  digitalWrite(ENABLE, HIGH);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  delayMicroseconds(2);
  digitalWrite(ENABLE, LOW); // Keep enable low for between 4 and 40 microseconds
  delayMicroseconds(20);
  digitalWrite(ENABLE, HIGH);

  digitalWrite(CS_DRV, HIGH);
  pinMode(CS_DRV, OUTPUT);
  digitalWrite(CS_DRV, HIGH);

  digitalWrite(SCLK, LOW);
  pinMode(SCLK, OUTPUT);
  digitalWrite(SCLK, LOW);

  pinMode(nFAULT, INPUT);

  pinMode(HALLA, INPUT);
  pinMode(HALLB, INPUT);
  pinMode(HALLC, INPUT);

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

  pinMode(P1_1, INPUT_PULLUP);
  pinMode(P2_1, INPUT_PULLUP);

  state = errorState1;
  old_state = errorState2;
  expected_next_state = errorState2;

  Serial.begin(9600);
  Serial.println("Starting Motor");

  analogFrequency(PWM_FREQUENCY);

  drv8323_SPI_Setup();

  // Increase clockspeed *AFTER* SPI setup.
  IncreaseClockSpeed_25MHz();

  // Switch leds to GREEN when running.
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);
}


void loop() {

  //Read HALL Values
  int rHallA = digitalRead(HALLA);
  int rHallB = digitalRead(HALLB);
  int rHallC = digitalRead(HALLC);

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
      coast();
      expected_next_state = state;  // assume the same errored state is the most likely one next
      break;
  } // SWITCH END
}


void drv8323_SPI_Setup(void) {
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

  digitalWrite(CS_DRV, LOW); //Pull slave select low to begin the transaction
  delayMicroseconds(2);
  SPI.transfer(0x10);
  byte return_value = SPI.transfer(0x20);
  Serial.println(return_value, HEX);
  Serial.println();
  delayMicroseconds(2);
  digitalWrite(CS_DRV, HIGH); //Pull slave select high to end the transaction

  // delayMicroseconds(10);

  // Reading Control Register
  // Reads back the same driver control register value, to make sure that it got set as required, by setting bit 15 to R/W=1 (read)
  // This corresponds to a 16-bit value: 1(001 0)000 0(01)0 0000 => 0x9020 or byte writes of 0x90 then 0x20

  // digitalWrite(CS_DRV, LOW); //Pull slave select low to begin the transaction
  // delayMicroseconds(2);
  // SPI.transfer(0x90);
  // return_value = SPI.transfer(0x20);
  // delayMicroseconds(2);
  // digitalWrite(CS_DRV, HIGH); //Pull slave select high to end the transaction

  SPI.end();
}



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
 * CPU TUNING FUNCTIONS:
 *   - SetVCoreUp - Increases core voltage to allow higher clock speed.
 *   - IncreaseClockSpeed_25MHz - Increases clock speed incrementally to 25 MHz.
 */
void SetVcoreUp (unsigned int level)
{
 /*
  * From TI example MSP430F55xx_UCS_10.c
  * Increses CPU voltage.
  */
 
  // Open PMM registers for write
  PMMCTL0_H = PMMPW_H;
      
  // Set SVS/SVM high side new level
  SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
  
  // Set SVM low side to new level
  SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
  
  // Wait till SVM is settled
  while ((PMMIFG & SVSMLDLYIFG) == 0);
  
  // Clear already set flags
  PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
  
  // Set VCore to new level
  PMMCTL0_L = PMMCOREV0 * level;
  
  // Wait till new level reached
  if ((PMMIFG & SVMLIFG))
    while ((PMMIFG & SVMLVLRIFG) == 0);
    
  // Set SVS/SVM low side to new level
  SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
  
  // Lock PMM registers for write access
  PMMCTL0_H = 0x00;
}

void IncreaseClockSpeed_25MHz()
{
    /* Increase Vcore one step at a time. */
    SetVcoreUp (0x01);
    SetVcoreUp (0x02);  
    SetVcoreUp (0x03);  

    UCSCTL3 = SELREF_2; // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2; // Set ACLK = REFO
  
    __bis_SR_register(SCG0); // Disable the FLL control loop
    UCSCTL0 = 0x0000; // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_7; // Select DCO range 50MHz operation
    UCSCTL2 = FLLD_0 + 762; // Set DCO Multiplier for 25MHz
    
    /**
     * (N + 1) * FLLRef = Fdco
     * (762 + 1) * 32768 = 25MHz
     * Set FLL Div = fDCOCLK/2
     * 
     * Worst-case settling time for the DCO when the DCO range bits have been
     * changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
     * UG for optimization.
     * 32 x 32 x 25 MHz / 32,768 Hz ~ 780k MCLK cycles for DCO to settle
     */
    __bic_SR_register(SCG0); // Enable the FLL control loop
    __delay_cycles(782000);
  
    // Loop until XT1,XT2 & DCO stabilizes
    // -> In this case only DCO has to stabilize
    do
    {
      // Clear XT2,XT1,DCO fault flags
      UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
      
      // Clear fault flags
      SFRIFG1 &= ~OFIFG;
      
    }while (SFRIFG1&OFIFG); // Test oscillator fault flag
}

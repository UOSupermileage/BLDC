//Interrupt Driven Code

#include <SPI.h>
#include "msp430g2553.h"

#define ENABLE 5  // port 3.6 output

// SPI PINS
#define nSCS 12   // port 2.5 output
#define SCLK 7    // port 4.5 output
#define nFAULT 8  // port 3.4 input
#define MISO 14   // port 4.7 input
#define MOSI 15   // port 4.6 output

// HALL PINS
#define HALLA 10  // port 1.2 input
#define HALLB 9   // port 1.3 input
#define HALLC 31  // port 3.2 input

#define LOWA 39   // port 6.0 output
#define LOWB 37   // port 6.2 output
#define LOWC 35   // port 6.4 output
#define HIGHA 40  // port 2.1 output
#define HIGHB 38  // port 6.1 output
#define HIGHC 36  // port 6.3 output

#define PWM_VALUE 50
#define PWM_FREQUENCY 20000 // 50kHz

//State Variable Values
#define zeroDegrees B110
#define twentyDegrees B010
#define fortyDegrees B011
#define sixtyDegrees B001
#define eightyDegrees B101
#define hundredDegrees B100
#define errorState1 B111
#define errorState2 B000

byte state = errorState1;
byte old_state = state;
byte expected_next_state = state;
byte pwm_value = PWM_VALUE; 
int unexpected_jump_cnt=0;

int desiredTorque = 0;
bool new_state_OK = false;

/* Coast
*Allows the motor to coast by turning off all Fets
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
  digitalWrite(nSCS, HIGH);

//  delayMicroseconds(10);

//  // Reading Fault Register 1
//  digitalWrite(nSCS, LOW); //Pull slave select low to begin the transaction
//  delayMicroseconds(2);
//  SPI.transfer(0x80);
//  SPI.transfer(0x20);
//  delayMicroseconds(2);
//  digitalWrite(nSCS, HIGH);
//
//  delayMicroseconds(10);
//
//  // Reading Fault Register 2
//  digitalWrite(nSCS, LOW); //Pull slave select low to begin the transaction
//  delayMicroseconds(2);
//  SPI.transfer(0x88);
//  SPI.transfer(0x20);
//  delayMicroseconds(2);
//  digitalWrite(nSCS, HIGH);

//   // Reading Control Register
  // read back the same driver control register value, to make sure that it got set as required, by setting bit 15 to R/W=1 (read)
  // This corresponds to a 16-bit value: 1(001 0)000 0(01)0 0000 => 0x9020 or byte writes of 0x90 then 0x20
//   digitalWrite(nSCS, LOW); //Pull slave select low to begin the transaction
//   delayMicroseconds(2);
//   SPI.transfer(0x90);
//   return_value = SPI.transfer(0x20);
//   delayMicroseconds(2);
//   digitalWrite(nSCS, HIGH);
  
  SPI.end();
}

void setup() {
  // put your setup code here, to run once:
  digitalWrite(ENABLE, HIGH);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  delayMicroseconds(2);
  digitalWrite(ENABLE, LOW); // strobe enable low for between 4 and 40 microseconds
  delayMicroseconds(20);
  digitalWrite(ENABLE, HIGH);

  digitalWrite(nSCS, HIGH);
  pinMode(nSCS, OUTPUT);
  digitalWrite(nSCS, HIGH);

  digitalWrite(SCLK, LOW);
  pinMode(SCLK, OUTPUT);
  digitalWrite(SCLK, LOW);
  
  pinMode(nFAULT, INPUT);
  //pinMode(MISO, INPUT_PULLUP);
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

  state = errorState1;
  old_state = errorState2;
  expected_next_state = errorState2;
  
  Serial.begin(14400);
  Serial.println("Starting Motor");

  analogFrequency(PWM_FREQUENCY);
  //analogWrite(HIGHA, PWM_VALUE);
  //analogWrite(HIGHB, PWM_VALUE);
  //analogWrite(HIGHC, PWM_VALUE);
  
  controllerSetup();

//  Serial.println("SPI Write Complete");
  
}

//#define ENABLE 5  // port 3.6 output

//#define nSCS 12   // port 2.5 output
//#define SCLK 7    // port 4.5 output
//#define nFAULT 8  // port 3.4 input
//#define MISO 14   // port 4.7 input
//#define MOSI 15   // port 4.6 output

// HALL PINS
//#define HALLA 10  // port 1.2 input
//#define HALLB 9   // port 1.3 input
//#define HALLC 31  // port 3.2 input

//#define LOWA 39   // port 6.0 output
//#define LOWB 37   // port 6.2 output
//#define LOWC 35   // port 6.4 output
//#define HIGHA 40  // port 2.1 output
//#define HIGHB 38  // port 6.1 output
//#define HIGHC 36  // port 6.3 output

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;         // Stop WDT  
  CCTL0 = CCIE;                     // CCR0 interrupt enabled
  TACTL = TASSEL_2 + MC_1 + ID_3;   // SMCLK/8, upmode
  //CCR0 =  500;               // 250 Hz rate for the timer poke rate (always set this lower than 65535)
  //CCR0 =  1000;               // 125 Hz rate for the timer poke rate (always set this lower than 65535)
  CCR0 =  5000;               // 25 Hz rate for the timer poke rate (always set this lower than 65535)
  //CCR0 =  10000;              // 12.5 Hz rate for the timer poke rate (always set this lower than 65535)
  //CCR0 =  60000;              // 2.08 Hz rate for the timer poke rate (always set this lower than 65535)
  P1OUT &= 0x00;              // Shut down everything
  P1DIR &= 0x00;              // Assume all of port 1 is input
  // P1DIR |= BIT0 + BIT6;    // P1.0 and P1.6 pins output the rest are input 
  // P1REN |= BIT3;           // Enable internal pull-up/down resistors
  // P1OUT |= BIT3;           // Select pull-up mode for P1.3
  P1IES |= BIT2 + BIT3;       // P1.3 P1.2 Hi/lo edge
  P1IFG &= ~BIT3;             // P1.3 IFG cleared
  P1IFG &= ~BIT2;             // P1.2 IFG cleared
  P1IE |= BIT2 + BIT3;        // P1.2 P1.3 interrupts enabled
  
  P3OUT &= 0x00;              // Shut down everything
  P3DIR &= 0x00               // Assume all of port 3 is input
  P1IES |= BIT2;              // P3.2 Hi/lo edge
  P1IFG &= ~BIT2;             // P3.2 IFG cleared
  P3IE |= BIT2;               // P3.2 interrupt enabled
  
  // _BIS_SR(CPUOFF + GIE);      // Enter LPM0 w/ interrupt 
  while(1) {                   //Loop forever; interrupt-driven main loop now!
    // read the throttle value from the user here, converting it to a byte with a value ranging 0..255
    // set the pwm duty cycle using the averaged current sense inputs for A, B and C using timer B to schedule the increments
    // to the pwm_value variable (is this variable visible in the ISR?) in a controlled and metered fashion
    // The more current headroom, the smaller the delay between those increments and vice versa.  
    // The pwm_value variable can be decreased to lower values immediately, though
  }
} 
  
// Timer A0 interrupt service routine 
#pragma vector=TIMERA0_VECTOR 
__interrupt void Timer_A (void) 
{
  // We do this every so often, whether or not the sensor inputs change (i.e. required for startup)
  
  // determine a composite new state value using Hall effect sensor inputs in different bit positions
  // Sensor inputs from A, B and C are stored in state bits 2,1 and 0, respectively

  // Note that the port 1 and port 2 ISRs can reset this timer, so that it might never get executed
  // In the absence of sensor activity, however, this ISR will run periodically and initiate motor movement.
  // If this routine is never or rarely called, we can assume that the motor is already travelling fast enough
  // to sustain sensor transitions at a high enough rate
  
  state = 0;
  if(rHallA == HIGH) {
    state = state | B100;  // set bit 2
  }
  if(rHallB == HIGH) {
    state = state | B010;  // set bit 1
  }
  if (rHallC == HIGH) {
    state = state | B001;  // set bit 0
  }  

  // get the motor moving, based on the sensor inputs
  switch(state) {
    case zeroDegrees:
    //setLow('C');
      digitalWrite(LOWC, HIGH);
      digitalWrite(HIGHC, LOW);    
    //setHighZ('A');
      digitalWrite(LOWA, LOW);
      digitalWrite(HIGHA, LOW);    
    //setHigh('B');
      digitalWrite(LOWB, HIGH);
      analogWrite(HIGHB, pwm_value);
    expected_next_state = twentyDegrees;
    break;

    case twentyDegrees:
    //setLow('A');
      digitalWrite(LOWA, HIGH);
      digitalWrite(HIGHA, LOW);    
    //setHighZ('C');
      digitalWrite(LOWC, LOW);
      digitalWrite(HIGHC, LOW);    
    //setHigh('B');
      digitalWrite(LOWB, HIGH);
      analogWrite(HIGHB, pwm_value);
    expected_next_state = fortyDegrees;
    break;
    
    case fortyDegrees:
    //setLow('A');
      digitalWrite(LOWA, HIGH);
      digitalWrite(HIGHA, LOW);    
    //setHighZ('B');
      digitalWrite(LOWB, LOW);
      digitalWrite(HIGHB, LOW);    
    //setHigh('C');
      digitalWrite(LOWC, HIGH);
      analogWrite(HIGHC, pwm_value);
    expected_next_state = sixtyDegrees;
    break;

    case sixtyDegrees:
    //setLow('B');
      digitalWrite(LOWB, HIGH);
      digitalWrite(HIGHB, LOW);    
    //setHighZ('A');
      digitalWrite(LOWA, LOW);
      digitalWrite(HIGHA, LOW);    
    //setHigh('C');
      digitalWrite(LOWC, HIGH);
      analogWrite(HIGHC, pwm_value);
    expected_next_state = eightyDegrees;
    break;

    case eightyDegrees:
    //setLow('B');
      digitalWrite(LOWB, HIGH);
      digitalWrite(HIGHB, LOW);    
    //setHighZ('C');
      digitalWrite(LOWC, LOW);
      digitalWrite(HIGHC, LOW);    
    //setHigh('A');
      digitalWrite(LOWA, HIGH);
      analogWrite(HIGHA, pwm_value);
    expected_next_state = hundredDegrees;
    break;

    case hundredDegrees:
    //setLow('C');
      digitalWrite(LOWC, HIGH);
      digitalWrite(HIGHC, LOW);    
    //setHighZ('B');
      digitalWrite(LOWB, LOW);
      digitalWrite(HIGHB, LOW);    
    //setHigh('A');
      digitalWrite(LOWA, HIGH);
      analogWrite(HIGHA, pwm_value);
    expected_next_state = zeroDegrees;
    break;

    default :
    Serial.println("Error - Set to Coast");
    coast();
    expected_next_state = state;  // assume the same errored state is the most likely one next
    break;
  } // SWITCH END
  
} // Timer A ISR

//                        _ABC
//#define zeroDegrees     B110 // look for falling A, falling B and rising C
//#define twentyDegrees   B010 // look for rising A, falling B and rising C
//#define fortyDegrees    B011 // look for rising A, falling B and falling C
//#define sixtyDegrees    B001 // look for rising A, rising B and falling C
//#define eightyDegrees   B101 // look for falling A, rising B and falling C
//#define hundredDegrees  B100 // look for falling A, rising B and rising C

// An alternative would be to only look for one interrupt at a time???

//#define errorState1     B111 // look for rising A, rising B and rising C
//#define errorState2     B000 // look for rising A, rising B and rising C

// HALL PINS
//#define HALLA 10  // port 1.2 input
//#define HALLB 9   // port 1.3 input
//#define HALLC 31  // port 3.2 input


// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  
  CCTL0 &= ~CCIE;  // CCR0 interrupt disabled
  
  if((P1IFG & BIT2) != 0) {
    // Hall A edge interrupt
    
    if ( (P3IES & BIT2) != 0 ) {
      // This should never happen (i.e. interrupt on Hall C, so we clear it
      // and effectively prioritize the interrupt currently being handled as the highest priority input
      P3IE &= ~BIT2;    // P3.2 interrupt disabled for Hall B
      P3IFG &= ~BIT2;   // P3.2 IFG cleared
      P3IE |= BIT2;     // P3.2 interrupt re-enabled          
      Serial.println("Unexpected C (in A/B)");         
    }
    
    if ( (P1IES & BIT2) != 0) {  
      // Hi/lo edge was expected on Hall A, so we are in state eightyDegrees
      //setLow('B');
      digitalWrite(LOWB, HIGH);
      digitalWrite(HIGHB, LOW);    
      //setHighZ('C');
      digitalWrite(LOWC, LOW);
      digitalWrite(HIGHC, LOW);    
      //setHigh('A');
      digitalWrite(LOWA, HIGH);
      analogWrite(HIGHA, pwm_value);
      expected_next_state = hundredDegrees;
      P1IE &= ~BIT2;    // P1.2 interrupt disabled for Hall A
      if ((P1IFG & BIT3) != 0) {
        // This case should not happen (i.e. interrupt on both Hall A and B, simultaneously) but we handle it anyway
        P1IE &= ~BIT3;    // P1.3 interrupt disabled for Hall B
      }
      // P3IE &= ~BIT2;    // P3.2 interrupt disabled for Hall C
      P1IES &= ~BIT2;   // P1.2 reverse Lo/Hi edge expected next
    }
    else {
      // Lo/Hi edge expected on Hall A, so we are in state twentyDegrees
      //setLow('A');
      digitalWrite(LOWA, HIGH);
      digitalWrite(HIGHA, LOW);    
      //setHighZ('C');
      digitalWrite(LOWC, LOW);
      digitalWrite(HIGHC, LOW);    
      //setHigh('B');
      digitalWrite(LOWB, HIGH);
      analogWrite(HIGHB, pwm_value);
      expected_next_state = fortyDegrees;
      // expect the other edge next time
      P1IE &= ~BIT2;    // P1.2 interrupt disabled
      if ((P1IFG & BIT3) != 0) {
        // This case should not happen (i.e. interrupt on both Hall A and B, simultaneously) but we handle it anyway
        P1IE &= ~BIT3;    // P1.3 interrupt disabled for Hall B
      }
      P1IES |= BIT2;    // P1.2 Hi/Lo edge expected next                
    }
    if ((P1IFG & BIT3) != 0) {
      // This case should not happen (i.e. interrupt on both Hall A and B, simultaneously) 
      // If it does, we handle it anyway by ignoring/clearing the Hall B interrupt
      P1IFG &= ~BIT3;   // P1.3 IFG cleared
      P1IE |= BIT3;     // P1.3 interrupt re-eabled for Hall B  
      Serial.println("Unexpected B (in A)");         
    }
    P1IFG &= ~BIT2;   // P1.2 IFG cleared
    P1IE |= BIT2;     // P1.2 interrupt re-enabled          
  } // Hall A edge
  
  else if ((P1IFG & BIT3) != 0) { 
    // Hall B edge
    // This interrupt is prioritzed behind the Hall A interrupt and will not be handled if A is active
    // Both are never supposed to be active simultaneously, so this should not be an issue
    if ( (P1IES & BIT3) != 0) {  
      // Hi/lo edge was being expected on Hall B, so we are in state zeroDegrees
      //setLow('C');
      digitalWrite(LOWC, HIGH);
      digitalWrite(HIGHC, LOW);    
      //setHighZ('A');
      digitalWrite(LOWA, LOW);
      digitalWrite(HIGHA, LOW);    
      //setHigh('B');
      digitalWrite(LOWB, HIGH);
      analogWrite(HIGHB, pwm_value);
      expected_next_state = twentyDegrees;
      P1IE &= ~BIT3;    // P1.3 interrupt disabled
      P1IES &= ~BIT3;   // P1.3 Lo/Hi edge expected next          
    }
    else {
      // Lo/Hi edge expected on Hall B, so we are in state sixtyDegrees
      //setLow('B');
      digitalWrite(LOWB, HIGH);
      digitalWrite(HIGHB, LOW);    
      //setHighZ('A');
      digitalWrite(LOWA, LOW);
      digitalWrite(HIGHA, LOW);    
      //setHigh('C');
      digitalWrite(LOWC, HIGH);
      analogWrite(HIGHC, pwm_value);
      expected_next_state = eightyDegrees;
      P1IE &= ~BIT3;    // P1.3 interrupt disabled
      P1IES |= BIT3;    // P1.3 Hi/Lo edge expected next                
    }   
    P1IFG &= ~BIT3;   // P1.3 IFG cleared
    P1IE |= BIT3;     // P1.3 interrupt re-enabled          
  } // Hall B edge
  else {
  // not expecting to have any other sources of interrupt on the port, besides the ones from Hall A and Hall B
    Serial.println("Unexpected interrupt on port 1");
  }
  
  TACTL = TASSEL_2 + MC_1 + ID_3 + TACLR;   // set SMCLK/8, upmode and clear the value in Timer A
  CCTL0 = CCIE;             // CCR0 interrupt re-enabled
  
} // port 1 ISR

// Port 3 interrupt service routine
#pragma vector=PORT3_VECTOR
__interrupt void Port_3(void)
{
  
  CCTL0 &= ~CCIE;  // CCR0 interrupt disabled
  
  if((P3IFG & BIT2) != 0) {
    // Hall C edge interrupt
    if ( (P3IES & BIT2) != 0) {  
      // Hi/lo edge was expected on Hall C, so we are in state fortyDegrees
      //setLow('A');
      
      if ( (P1IES & BIT3) != 0 ) {
        // This should never happen (i.e. interrupt on Hall B, so we clear it
        // and effectively prioritize the interrupt currently being handled as the highest priority input
        P1IE &= ~BIT3;    // P1.3 interrupt disabled for Hall B
        P1IFG &= ~BIT3;   // P1.3 IFG cleared
        P1IE |= BIT3;     // P1.3 interrupt re-enabled          
        Serial.println("Unexpected B (in C)");         
      }

      if ( (P1IES & BIT2) != 0 ) {
        // This should never happen (i.e. interrupt on Hall A, so we clear it
        // and effectively prioritize the interrupt currently being handled as the highest priority input
        P1IE &= ~BIT2;    // P1.3 interrupt disabled for Hall A
        P1IFG &= ~BIT2;   // P1.3 IFG cleared
        P1IE |= BIT2;     // P1.3 interrupt re-enabled          
        Serial.println("Unexpected A (in C)");         
      }

      digitalWrite(LOWA, HIGH);
      digitalWrite(HIGHA, LOW);    
      //setHighZ('B');
      digitalWrite(LOWB, LOW);
      digitalWrite(HIGHB, LOW);    
      //setHigh('C');
      digitalWrite(LOWC, HIGH);
      analogWrite(HIGHC, pwm_value);
      expected_next_state = sixtyDegrees;
      P3IE &= ~BIT2;    // P3.2 interrupt disabled
      P3IES &= ~BIT2;   // P3.2 Lo/Hi edge expected next
          
    }
    else if ((P3IFG & BIT2) == 0) {
      // Lo/Hi edge expected on Hall C, so we are in state hundredDegrees
      //setLow('C');
      
      if ( (P1IES & BIT3) != 0 ) {
        // This should never happen (i.e. interrupt on Hall B, so we clear it
        // and effectively prioritize the interrupt currently being handled as the highest priority input
        P1IE &= ~BIT3;    // P1.3 interrupt disabled for Hall B
        P1IFG &= ~BIT3;   // P1.3 IFG cleared
        P1IE |= BIT3;     // P1.3 interrupt re-enabled          
        Serial.println("Unexpected B (in C)");         
      }

      if ( (P1IES & BIT2) != 0 ) {
        // This should never happen (i.e. interrupt on Hall A, so we clear it
        // and effectively prioritize the interrupt currently being handled as the highest priority input
        P1IE &= ~BIT2;    // P1.3 interrupt disabled for Hall A
        P1IFG &= ~BIT2;   // P1.3 IFG cleared
        P1IE |= BIT2;     // P1.3 interrupt re-enabled 
        Serial.println("Unexpected A (in C)");         
      }

      digitalWrite(LOWC, HIGH);
      digitalWrite(HIGHC, LOW);    
      //setHighZ('B');
      digitalWrite(LOWB, LOW);
      digitalWrite(HIGHB, LOW);    
      //setHigh('A');
      digitalWrite(LOWA, HIGH);
      analogWrite(HIGHA, pwm_value);
      expected_next_state = zeroDegrees;
      P3IE &= ~BIT2;    // P3.2 interrupt disabled
      P3IES |= BIT2;    // P3.2 Hi/Lo edge expected next  
                          
    }
    else {
      // not expecting to have more than one interrupt on port 3, besides the one from Hall C
      Serial.println("Unexpected interrupt on port 3");
    }
  } // Hall C edge
  P3IFG &= ~BIT2;   // P3.2 IFG cleared 
  P3IE |= BIT2;     // P3.2 interrupt re-enabled  
    
  TACTL = TASSEL_2 + MC_1 + ID_3 + TACLR;   // set SMCLK/8, upmode and clear the value in Timer A
  CCTL0 = CCIE;             // CCR0 interrupt re-enabled
          
} // Port 3 ISR

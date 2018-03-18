#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define INHA 10
#define INLA 8
#define INHB 6 
#define INLB 5
#define INHC 4
#define INLC 3

#define HALLA 14
#define HALLB 15
#define HALLC 16

#define SDI 11
#define SDO 12
#define SCS 23
#define SCLK 13
#define FAULT 18

#define joystick 9

#define EN_GATE 17

//make sure to define the prototype first and include the libs
char *binaryToHex(char *binaryString);

// Changing Binary to Hex
char *binaryToHex(char *binaryString) {
  //converts the binary string into a long of base 2
  int binaryVal = (int)strtol(binaryString, NULL, 2);
  char tmpHexString[5];
  //converts the long into hex
  sprintf(tmpHexString, "%x", binaryVal);
  //allocates memory outside of the function
  char *hexString = (char*)malloc(5);
  strcpy(hexString, tmpHexString);
  return hexString;
}

void setup() {
  pinMode(INHA, OUTPUT);
  pinMode(INLA, OUTPUT);
  pinMode(INHB, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INHC, OUTPUT);
  pinMode(INLC, OUTPUT);

  pinMode(HALLA, INPUT);
  pinMode(HALLB, INPUT);
  pinMode(HALLC, INPUT);

  pinMode(SDI, INPUT);
  pinMode(SDO, OUTPUT);
  pinMode(SCS, INPUT);
  pinMode(SCLK, INPUT);
  pinMode(FAULT, OUTPUT);

  

//  digitalWrite(HALLC, LOW);

//  pinMode(dataReadyPin, INPUT);
//  pinMode(chipSelectPin, OUTPUT);

  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setBitOrder(MSBFIRST);

}

void loop() {
  // Reading PWM Value
  analogRead(joystick);

  //Writing new value to the register
  digitalWrite(SS, LOW);
  SPI.transfer(0x02);
  digitalWrite(SS, HIGH);

  //Reading the new register value
  int received =  SPI.transfer(0x02);
  Serial.print("New Register 0x02 Value: ");
  Serial.println(received, HEX);




 
  // Switching through phases
  if(digitalRead(HALLA) == HIGH && digitalRead(HALLB) == HIGH && digitalRead(HALLC) == LOW) {
    //Phase A
    digitalWrite(INLA, LOW);
    digitalWrite(INHA, LOW);
    
    //Phase B
    digitalWrite(INLB, HIGH);
    digitalWrite(INLB, HIGH);

    //Phase C
    digitalWrite(INLC, HIGH);
    digitalWrite(INLC, LOW);

    delay(analogRead(joystick));
  }
  else if(digitalRead(HALLA) == LOW && digitalRead(HALLB) == HIGH && digitalRead(HALLC) == LOW) {
    //Phase A
    digitalWrite(INLA, HIGH);
    digitalWrite(INHA, LOW);
    
    //Phase B
    digitalWrite(INLB, HIGH);
    digitalWrite(INLB, HIGH);

    //Phase C
    digitalWrite(INLC, LOW);
    digitalWrite(INLC, LOW);

    delay(analogRead(joystick));
  }
  else if(digitalRead(HALLA) == LOW && digitalRead(HALLB) == HIGH && digitalRead(HALLC) == HIGH) {
    //Phase A
    digitalWrite(INLA, HIGH);
    digitalWrite(INHA, LOW);
    
    //Phase B
    digitalWrite(INLB, LOW);
    digitalWrite(INLB, LOW);

    //Phase C
    digitalWrite(INLC, HIGH);
    digitalWrite(INLC, HIGH);

    delay(analogRead(joystick));
  }
  else if(digitalRead(HALLA) == LOW && digitalRead(HALLB) == LOW && digitalRead(HALLC) == HIGH) {
    //Phase A
    digitalWrite(INLA, LOW);
    digitalWrite(INHA, LOW);
    
    //Phase B
    digitalWrite(INLB, HIGH);
    digitalWrite(INLB, LOW);

    //Phase C
    digitalWrite(INLC, HIGH);
    digitalWrite(INLC, HIGH);

    delay(analogRead(joystick));
  }
  else if(digitalRead(HALLA) == HIGH && digitalRead(HALLB) == LOW && digitalRead(HALLC) == HIGH) {
    //Phase A
    digitalWrite(INLA, HIGH);
    digitalWrite(INHA, HIGH);

    //Phase B
    digitalWrite(INLB, HIGH);
    digitalWrite(INLB, LOW);

    //Phase C
    digitalWrite(INLC, LOW);
    digitalWrite(INLC, LOW);

    delay(analogRead(joystick));
  }
  else if(digitalRead(HALLA) == HIGH && digitalRead(HALLB) == LOW && digitalRead(HALLC) == LOW) {
    //Phase A
    digitalWrite(INLA, HIGH);
    digitalWrite(INLA, HIGH);

    //Phase B
    digitalWrite(INLB, LOW);
    digitalWrite(INLB, LOW);

    //Phase C
    digitalWrite(INLC, HIGH);
    digitalWrite(INLC, LOW);

    delay(analogRead(joystick));
  }

}

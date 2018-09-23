//Test Script adapted from final code
//Code by Camry Mabon

//Libary Inclusions
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Pin Assignments
#define INHA 10
#define INLA 9
#define INHB 6 
#define INLB 5
#define INHC 4
#define INLC 3
#define HALLA 20
#define HALLB 21
#define HALLC 22
#define SDI 12
#define SDO 11
#define SCS 14
#define SCLK 13
#define FAULT 8

#define joystick 23

#define EN_GATE 7

int TEST=1;

//Function Declearations 
void windingHIGH(int phase, int PWM); //For the three phase functions phase A is 1, B is 2, C is 3
void windingLOW(int phase); 
void windingHIZ(int phase);
void testSq(); //generates the hall effect changes

//Hall Variables
bool hallA=0;
bool hallB=0;
bool hallC=0;

//These are the defualt values for the SPI write in setup
unsigned int driver_control_register = 0x1020
;

void setup() {
  if(TEST == 0){
    Serial.begin(9600);
  }
  
  //This is the setup for the all of the pins
  pinMode(INHA, OUTPUT);//Logic inputs for the driver chip
  pinMode(INLA, OUTPUT);
  pinMode(INHB, OUTPUT);
  pinMode(INLB, OUTPUT);
  pinMode(INHC, OUTPUT);
  pinMode(INLC, OUTPUT);

  pinMode(HALLA, INPUT);//Pinmode setups for Halls
  pinMode(HALLB, INPUT);
  pinMode(HALLC, INPUT);

  pinMode(SDI, INPUT);//Pins for SPI
  pinMode(SDO, OUTPUT);
  pinMode(SCS, INPUT);
  pinMode(SCLK, INPUT);
  
  pinMode(EN_GATE, OUTPUT);

  digitalWrite(SCS, HIGH);  //Write the SCS pin high and puase to make sure it has time to latch
  digitalWrite(EN_GATE, HIGH); //Enable the chip 
  
  pinMode(FAULT, INPUT); //This pin handles fualt reporting from the chip

  digitalWrite(13,LOW);
 //This stuff below is the setup for the SPI and the initial transfer of defualt register values to the chip
  SPI.begin();
  SPI.beginTransaction(SPISettings(1400000, MSBFIRST, SPI_MODE1));
  digitalWrite(SCS, LOW);
  SPI.transfer16(driver_control_register);
  digitalWrite(SCS, HIGH);
  delay(10);
  SPI.endTransaction();
  SPI.end();
  analogWriteFrequency(3, 100000);
  analogWriteFrequency(6, 100000);
  analogWriteFrequency(10, 100000);
}

int throttle=0;

void loop() {

throttle = 75;

//Get hall Values
bool hallA = digitalRead(HALLA);
bool hallB = digitalRead(HALLB);
bool hallC = digitalRead(HALLC);

//algorithim 

if(hallA && hallB && (!hallC)){
	windingHIZ(1);
	windingLOW(3);
	windingHIGH(2, throttle);
}
else if((!hallA) && hallB && (!hallC)){
	windingLOW(2);
	windingLOW(1);
	windingHIZ(3)
	windingHIGH(2, throttle);
}
else if((!hallA) && hallB && hallC){
	windingHIZ(2);
	windingLOW(1);
	windingHIGH(3, throttle);
}
else if((!hallA) && (!hallB) && hallC){
	windingLOW(3);
	windingLOW(2);
	windingHIZ(1);
	windingHIGH(3, throttle);
}
else if(hallA && (!hallB) && hallC){
	windingHIZ(3);
	windingLOW(2);
	windingHIGH(1, throttle);
}
else if(hallA && (!hallB) && (!hallC)){
	windingLOW(1);
	windingLOW(3);
	windingHIZ(2);
	winding(1, throttle);
}


}
//Phase Functions it is again noted that For the three phase functions phase A is 1, B is 2, C is 3
// All of this information can be found in the datasheet
void windingHIGH(int phase, int PWM){
  switch(phase){
    case 1 :
      digitalWrite(INLA, LOW);
      analogWrite(INHA, PWM);
      break;
    case 2 :
      digitalWrite(INLB, LOW);
      analogWrite(INHB, PWM);
      break;
    case 3 :
      digitalWrite(INLC, LOW);
      analogWrite(INHC, PWM);
      break;
  }
}
void windingLOW(int phase){
  switch(phase){
    case 1 :
      digitalWrite(INLA, HIGH);
      analogWrite(INHA, 0);
      break;
    case 2 :
      digitalWrite(INLB, HIGH);
      analogWrite(INHB, 0);
      break;
    case 3 :
      digitalWrite(INLC, HIGH);
      analogWrite(INHC, 0);
      break;
  }
}
void windingHIZ(int phase){
  switch(phase){
    case 1 :
      digitalWrite(INLA, LOW);
      analogWrite(INHA, 0);
      break;
    case 2 :
      digitalWrite(INLB, LOW);
      analogWrite(INHB, 0);
      break;
    case 3 :
      digitalWrite(INLC, LOW);
      analogWrite(INHC, 0);
      break;
  }
}

//testSq variables
int times=55;
int stage=1;


void testSq(){
  if(0){
    times++;
  }
  else{
    if(stage==1){
      hallA=1;
      hallB=0;
      hallC=1;
    }
    if(stage==2){
      hallA=1;
      hallB=0;
      hallC=0;
    }
    if(stage==3){
      hallA=1;
      hallB=1;
      hallC=0;
    }
    if(stage==4){
      hallA=0;
      hallB=1;
      hallC=0;
    }
    if(stage==5){
      hallA=0;
      hallB=1;
      hallC=1;
    }
    if(stage==6){
      hallA=0;
      hallB=0;
      hallC=1;
    }
    if(stage==7){
      stage=0;
    }
    stage++;
    times=0;
  }
}

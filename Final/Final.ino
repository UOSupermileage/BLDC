#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bitset>

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
//Defines below deal with PWM Generation
#define thrMax 1023
#define minCoast 15 //Throtle value at which to enter coast mode this is out of 255


//Function Declearations 
void windingHIGH(int phase, int PWM); //For the three phase functions phase A is 1, B is 2, C is 3
void windingLOW(int phase); 
void windingHIZ(int phase);
int coast(); //Enables or disables the coast mode of the motor
void clrFUALT(); //Clears any fualt condidtion on the motor

//These are the defualt values for all of the registers
//DO NOT CHANGE THESE REGISTERS UNLESS YOU HAVE THE DATASHEET INFRONT OF YOU
//first bit is r/w, next 4 are the adress, MSB TO LSB 
char CNTRLREG[16]={0,0,0,1,0,0,0,0,1,0,1,0,0,0,0,0}; //Control register of the driver, defualt is set to write
char GATEHCTRLREG[16]={0,0,0,1,1,0,1,1,1,1,1,1,1,1,1,1}; //This register controls the currents used to turn the  highmosfets on and off
char GATELCTRLREG[16]={0,0,1,0,0,1,1,1,1,1,1,1,1,1,1,1}; //This register controls the currents used to turn the low mosfets on and off
char OCPCTRLREG[16]={0,0,1,0,1,0,0,1,0,1,0,1,1,0,0,1}; //This register controls the Over current protection stuff (DEAD TIME IS ALSO IN HERE) 
char CSACTRLREG[16]={0,0,1,1,0,0,1,0,1,0,0,0,0,0,1,1};//This is the current sense amplifier Stuff

//Variables for keeping track of settings
int coasting=0; //If set to 1 then the motor is currently coasting

void setup() {
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
  
  pinMode(FAULT, INPUT); //This pin handles fualt reporting from the chip

  
  //This stuff below is the setup for the SPI and the initial transfer of defualt register values to the chip
  SPI.beginTransaction(SPISettings(1400000, MSBFIRST, SPI_MODE1));
  digitalWrite(SCS, LOW);
  SPI.transfer16((int)strtol(CNTRLREG, NULL, 2));
  digitalWrite(SCS, HIGH);
  delay(1);//The chip needs the NSC delay line to be high for 400ns between words
  digitalWrite(SCS, LOW);
  SPI.transfer16((int)strtol(GATEHCTRLREG,NULL, 2));
  digitalWrite(SCS, HIGH);
  delay(1);//The chip needs the NSC delay line to be high for 400ns between words
  digitalWrite(SCS, LOW);
  SPI.transfer16((int)strtol(GATELCTRLREG,NULL, 2));
  digitalWrite(SCS, HIGH);
  delay(1);//The chip needs the NSC delay line to be high for 400ns between words
  digitalWrite(SCS, LOW);
  SPI.transfer16((int)strtol(OCPCTRLREG,NULL, 2));
  digitalWrite(SCS, HIGH);
  delay(1);//The chip needs the NSC delay line to be high for 400ns between words
  digitalWrite(SCS, LOW);
  SPI.transfer16((int)strtol(CSACTRLREG,NULL, 2));
  digitalWrite(SCS, HIGH);
  SPI.endTransaction();
}

void loop() {
  /* -OVER VIEW OF MAIN LOOP-
	1. Check for any fualt conditions on the chip the error report pin
	2a. IF there is an error we need to read which one and handle it
	2b. If no error then we are good to go ahead
	3. Read the value of the throttle
	4a.If less then coast cutoff then check if the motor is in coast settings, if it is do nothing, else we need to put 
	it into coast mode
	4b. If more then set the PWM stregth to that, and check if the motor is in coast. If not then start the algorithim,
		if it is take it out of that state and then begin 
  
	Note about the algorithim -> This code is going to run a hell of a lot faster then the motor, we should check if
								motor has turned to a new phase before rewritting the output values to save both headaches
								and improve the general quality and efficency of the code. 
  */
  
  
  //Part 1 &2  
  if(digitalRead(FAULT)==0){//If this condidtion fails then there is no fualt and we can proceed
	  //Code here needs to read in the fualt register and figure out what is happening
	  //First thing is to get the data from the registers
	  SPI.beginTransaction(SPISettings(1400000, MSBFIRST, SPI_MODE1));
	  int roneValue; //Value from first register
	  int rtwoValue; //Value from second register
	  digitalWrite(SCS, LOW);
	  roneValue=SPI.transfer16(0x0800);//Read command for the first fualt register
	  digitalWrite(SCS, HIGH);
	  delay(1);
	  digitalWrite(SCS, LOW);
	  rtwoValue=SPI.transfer16(0x8800);//Read command for the second fualt register
	  digitalWrite(SCS, HIGH);
	  SPI.endTransaction();
	  //Convert both into strings so we can look at the individual bit
    bitset<16> regOne(roneValue);
	  char regTwo[16]=std::bitset<16>(rtwoValue);
	  //Need to look for certain flags and react appropiately
	  char OvercurrentA;
	  if(regOne[10]==1 || regOne[11]==1){ //Over current Phase A
		OvercurrentA='1';  
	  }
	  char OvercurrentB;
	  if(regOne[12]==1 || regOne[13]==1){
		OvercurrentB='1';  
	  }
	  char OvercurrentC;
	  if(regOne[14]==1 || regOne[15]==1){
		OvercurrentC='1'  
	  }
	  char OTW= regTwo[8]; //This is the over tempature warning
	  char OTSHTDWN = regOne[9]; //This is overtempature shutdown
	  
  }
  //Part 3 of the code
  int throttle = analogRead(joystick);
  map(throttle, 0, thrMax, 0, 255); //Need it in a range that analog write can understand 

  //Part 4
  if(throttle <= minCoast && coasting==0){
	  coasting=coast(); //If throttle is low engough to coast then coasting begins
  }
  else if(coasting==1){
	coasting=coast(); //If the motor is coasting   
  }
  // Switching through phases
  if(hallA && hallB && !hallC && coasting==0 ){ //Row 1 of Table
		windingHIZ(1);
		windingLOW(3);
		windingHIGH(2, throttle);
	}
	else if(!hallA && hallB && !hallC && coasting==0 ){ //Row 2 of table
		windingLOW(1); //The order that windings are changed is important. There always need to be a path for the motor to spin inbetween switching 
		windingHIZ(3);
		windingHIGH(2, throttle);
	}
	else if(!hallA && hallB && hallC && coasting==0 ){ //Row 3 of Table
		windingLOW(1);
		windingHIZ(2);
		windingHIGH(3, throttle);
	}
	else if(!hallA && !hallB && hallC && coasting==0 ){ //Row 4 of Table
		windingLOW(2);
		windingHIZ(1);
		windingHIGH(3, throttle);
	}
	else if(hallA && !hallB && hallC && coasting==0 ){ //Row 5 of Table
		windingLOW(2);
		windingHIZ(3);
		windingHIGH(1, throttle);
	}
	else if(hallA && !hallB && !hallC && coasting==0 ){ //Row 6 of Table
		windingLOW(3);
		windingHIZ(2);
		windingHIGH(1, throttle);

}

//Phase Functions it is again noted that For the three phase functions phase A is 1, B is 2, C is 3
// All of this information can be found in the datasheet
void windingHIGH(int phase, int PWM){
	switch(phase){
		case 1 :
			digitalWrite(INLA, HIGH);
			analogWrite(INHA, PWM);
		case 2 :
			digitalWrite(INLB, HIGH);
			analogWrite(INHB, PWM);
		case 3 :
			digitalWrite(INLC, HIGH);
			analogWrite(INHC, PWM);
	}
}
void windingLOW(int phase){
	switch(phase){
		case 1 :
			digitalWrite(INLA, HIGH);
			digitalWrite(INHA, LOW);
		case 2 :
			digitalWrite(INLB, HIGH);
			digitalWrite(INHB, LOW);
		case 3 :
			digitalWrite(INLC, HIGH);
			digitalWrite(INHC, LOW);
	}
}
void windingHIZ(int phase){
	switch(phase){
		case 1 :
			digitalWrite(INLA, LOW);
			digitalWrite(INHA, LOW);
		case 2 :
			digitalWrite(INLB, LOW);
			analogWrite(INHB, LOW);
		case 3 :
			digitalWrite(INLC, LOW);
			digitalWrite(INHC, LOW);
	}
}

//Motor State functions
int coast(){
	int coastenabled; //Value to store return value to tell caller if coast was enabled or disabled
  coastenabled=CNTRLREG[14];
	if(coastenabled==1){//Find the current state of coast and then toggle it to the other
		CNTRLREG[14]=0;
		coastenabled=0;
	}
	else{
		CNTRLREG[14]=1;
		coastenabled=1;
	}
	CNTRLREG[14]=1; //Set the register coast setting to be engabled 
	SPI.beginTransaction(SPISettings(1400000, MSBFIRST, SPI_MODE1));
	digitalWrite(SCS, LOW);
	SPI.transfer16((int)strtol(CNTRLREG, NULL, 2));
	digitalWrite(SCS, HIGH);
	SPI.endTransaction();
	delay(1);
	return coastenabled;
}

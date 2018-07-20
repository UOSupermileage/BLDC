//This is an ardunio program that will read all of the data off of the 
//DRV8323 per register in an attempt to verify its exsistence

#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>

#define SDI 12
#define SDO 11
#define SCS 14
#define SCLK 13
#define FAULT 8
#define EN_GATE 7

void setup(){
	Serial.begin(9600);
	digitalWrite(EN_GATE, HIGH);
	Serial.println("Program has begon");
	Serial.println("Current state of the Fualt pin is" + digitalRead(FAULT));
	delay(10000);
}

void loop(){
	int address = 0x00;
	SPI.beginTransaction(SPISettings(1400000, MSBFIRST, SPI_MODE1));
	unsigned short data = 0x00;
	data = SPI.transfer16(0x8000);
	Serial.println(data);
	data = SPI.transfer16(0x8800);
	Serial.println(data);
	data = SPI.transfer16(0x9000);
	Serial.println(data);
	data = SPI.transfer16(0x9800);
	Serial.println(data);
	data = SPI.transfer16(0xA000);
	Serial.println(data);
	data = SPI.transfer16(0xA800);
	Serial.println(data);
	data = SPI.transfer16(0xB000);
	Serial.println(data);
	Serial.println("That is all")
	break;
}

/*
 * spiFunctions.cpp
 *
 *  Created on: Jul 8, 2021
 *      Author: Camry
 */

#include "spiFunctions.h"
#include <SPI.h>
#include "pinDeclerations.h"

/*
 * Sets the correct current sense amplifier settings for the driver chip.
 * CURENTLY SET TO 20V/V
 */
void DRV8323_CSA(void){
    SPI.begin(CS_DRV);
    SPI.setClockDivider(SPI_CLOCK_DIV64);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE1);
    SPI.setModule(0);
    digitalWrite(CS_DRV, LOW);
    delayMicroseconds(2);
    SPI.transfer(0x32);
    SPI.transfer(0x83);
    digitalWrite(CS_DRV, HIGH);
    SPI.end();

}

/*
   SPI SETUP FUNCTION
*/
void DRV8323_SPI_Setup(void) {
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

  SPI.end();
}



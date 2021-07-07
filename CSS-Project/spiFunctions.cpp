/*
 * spiFunctions.cpp
 *
 *  Created on: Jul 5, 2021
 *      Author: Camry
 */
#include "msp430f5529.h"
#include "spiFunctions.h"
#include <SPI.h>
#include "pinDeclerations.h"

using namespace PINS;
/*
 * Spi setup functions that sets up all of the correct settings on the SPI buss so it functions
 */
void spiFunctions::setupSPI(void){
   SPI.setBitOrder(MSBFIRST);
   SPI.setClockDivider(SPI_CLOCK_DIV64);
   SPI.setBitOrder(MSBFIRST);
   SPI.setDataMode(SPI_MODE1);
   SPI.setModule(0);
}


/*
   Driver setup function

   This function sets the correct drive mode of the chip to spin the motor. Make sure that you call the spiSetup() first
*/
void spiFunctions::DRV8323_SPI_Setup(void) {
  delay(200); // Why is this here?
  SPI.begin(CS_DRV);

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
  Serial.println("Value returned from the driver setup:");
  Serial.println(return_value, HEX);
  Serial.println();
  delayMicroseconds(2);
  digitalWrite(CS_DRV, HIGH); //Pull slave select high to end the transaction
  SPI.end();
}

/*
 * This function will grab the value of the driver chips error register and return it into errorOut
 */
void spiFunctions::DRV8323_ErrorRegisterRead(uint16_t &errorOut){
    SPI.begin(CS_DRV);
    uint8_t temp = 0;
    uint8_t temp1 = 0;
    digitalWrite(CS_DRV, LOW);
    delayMicroseconds(2);
    temp = SPI.transfer(0b10000000);
    temp1 = SPI.transfer(0x00);
    digitalWrite(CS_DRV, HIGH);
    errorOut = temp << 8; //This shifts the first 8 bits of the reply into the proper posistion
    errorOut = errorOut + temp1; // Plops in the remaining 8 bits of the reply

}


/*
 * coreFunctions.cpp
 *
 *  Created on: Jul 5, 2021
 *      Author: Camry
 */

#include "coreFunctions.h"
#include "msp430f5529.h"

/*
   CPU Tuning Functions

   SetVCoreUp - Increases core voltage to allow higher clock speed.

   IncreaseClockSpeed_25MHz - Increases clock speed incrementally to 25 MHz.
*/
void coreFunctions::SetVcoreUp (unsigned int level)
{
  /*
     From TI example MSP430F55xx_UCS_10.c
     Increses CPU voltage.
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

void coreFunctions::IncreaseClockSpeed_25MHz()
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
     (N + 1) * FLLRef = Fdco
     (762 + 1) * 32768 = 25MHz
     Set FLL Div = fDCOCLK/2

     Worst-case settling time for the DCO when the DCO range bits have been
     changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
     UG for optimization.
     32 x 32 x 25 MHz / 32,768 Hz ~ 780k MCLK cycles for DCO to settle
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

  } while (SFRIFG1 & OFIFG); // Test oscillator fault flag
}




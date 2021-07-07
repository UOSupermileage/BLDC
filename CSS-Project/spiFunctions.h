/*
 * spiFunctions.h
 *
 *  Created on: Jul 5, 2021
 *      Author: Camry
 */
#include <inttypes.h>

#ifndef SPIFUNCTIONS_H_
#define SPIFUNCTIONS_H_

namespace spiFunctions{

void setupSPI();

void DRV8323_SPI_Setup();

void DRV8323_ErrorRegisterRead();

void DRV8323_ErrorRegisterRead(uint16_t &errorOut);
}

#endif /* SPIFUNCTIONS_H_ */

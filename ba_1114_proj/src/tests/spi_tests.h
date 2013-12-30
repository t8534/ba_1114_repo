/*
 * spi_tests.h
 *
 *  Created on: 26-12-2013
 *      Author: arek
 */

#ifndef SPI_TESTS_H_
#define SPI_TESTS_H_

#include "type.h"

#define SPI0    0U
#define SPI1    1U

boolean_t SPI_LoopbackInternalTest(uint8_t portNum);
boolean_t SPI_LoopbackHardwareTest(void);


#endif /* SPI_TESTS_H_ */

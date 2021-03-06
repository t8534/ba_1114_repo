/*
 * spi_tests.h
 *
 *  Created on: 26-12-2013
 *      Author: arek
 */

#ifndef SPI_TESTS_H_
#define SPI_TESTS_H_

#include "type.h"


#define	SPI0    0U
#define	SPI1    1U

void SPITESTS_SPI1DataProcessingISP(uint8_t val);
boolean_t SPITESTS_LoopbackInternalTest(uint8_t portNum);
boolean_t SPITESTS_LoopbackHardwarePoolingTest(void);
boolean_t SPITESTS_LoopbackHardwareISPTest(void);

#endif /* SPI_TESTS_H_ */

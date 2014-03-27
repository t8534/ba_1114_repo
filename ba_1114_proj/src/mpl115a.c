//******************************************************************************
//
//    THE SOFTWARE INCLUDED IN THIS FILE IS FOR GUIDANCE ONLY.

//    BTC KORPORACJA SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
//    OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
//    FROM USE OF THIS SOFTWARE.
//
//******************************************************************************

/*
 * The sensor MPL115a is connected by SPI1 on lpcexpresso 1114 board.
 * SPI0 is shared with debugger pins.
 *
 * SPI                                         MPL115a Kamami
 *
 * SPI MOSI1 - PIO2_3  - PIO2_3 J6-45   -----  SDA/SDI (2)
 * SPI MISO1 - PIO2_2  - PIO2_2 J6-14   -----  SDO     (4)
 * SPI SCK1  - PIO2_1  - PIO2_1 J16-13  -----  SCL/CLK (5)
 * SPI SSEL1 - PIO2_0  - PIO2_0 J6-12   -----  CS_RES  (2)
 *
 *
 *
 */


// TODO
//
// 1.
// Tune timings

// 2.
// Update with original code from app note
//
//


#include "driver_config.h"  //todo describe this include MUST BE, and if SSP driver will be not ON in there,
                            // than all the ssp.h and .c file will be not add to compile, so during compilation
                            // of this file, there will be error. This approach is very buggy.
#include "gpio.h"
#include "ssp.h"

#include "mpl115a.h"


#define	SPI0    0U
#define	SPI1    1U

#define SPI_READ    0x80

uint8_t MPL115A2ADC[4]; 				//Place to store temperature and pressure ADCs values.
										//These values are stored in registers 0x0..0x3 of MPL115A.

uint8_t MPL115A2Coeffs[12];				//Place to store equation coefficients
										//These values are stored in registers 0x4..0xF of MPL115A.



//************************************************************************************************
// Device dependent routines
//************************************************************************************************

// This is only  SSP0 !
void CSLow(void)
{
    /* Set SPI0 SSEL pin to output low. */
    //todo is it not automatically set by SSP periph
    GPIOSetValue(PORT2, 0, 0);
    return;
}


void CSHi(void)
{
    /* Set SPI0 SSEL pin to output high. */
    //todo is it not automatically set by SSP periph
    GPIOSetValue(PORT2, 0, 1);
    return;
}


// SPI write and read
uint8_t SPISend (uint8_t outb)
{
    uint8_t res = 0;

    res = SSPSendRecvByte(outb);

    return res;
}


//************************************************************************************************
// Device dependent routines - END
//************************************************************************************************


// MPL115AWrite 
// Writes value of b to MPL115A <address> register 
void MPL115AWrite(uint8_t address, uint8_t b)
{
	CSLow();
	SPISend(address << 1);
	SPISend(b);
	CSHi();
}

// MPL115ARead
// Reads <address> register
uint8_t MPL115ARead(uint8_t address)
{
	uint8_t res;
	CSLow();
	SPISend(SPI_READ | (address << 1));
	res = SPISend(0xFF);
	CSHi();

	return res;
}

// MPL115AIntitalize
// Initialization of communication interface 
void MPL115AIntitalize()
{
#if 0
	CSHi();
    SSP_IOConfig(SPI1);
    SSP_Init(SPI1);
	CSHi();  // todo which one ?
#endif


	SSP_Dev_t SSP_Dev;

	SSP_Dev.Device = LPC_SSP1;
	SSP_Dev.FrameFormat = SSP_FRAME_SPI;
	SSP_Dev.DataSize = SSP_DATABITS_8;
	SSP_Dev.CPOL = ;
	SSP_Dev.CPHA = ;
	SSP_Dev.LoopBackMode = SSP_LOOPBACK_OFF;
	SSP_Dev.Mode = SSP_MASTER_MODE;
	SSP_Dev.ClockRateHz = ???;	/* Clock rate,in Hz, should not exceed TODO: (SPI peripheral clock)/8 */
	SSP_Dev.SlaveOutputDisable;
	SSP_Dev.transferType = SSP_TRANSFER_POLLING;
	SSP_Dev.InterruptCondition = ???;
	SSP_Dev.ISR_Processing = NULL;
	SSP_Dev.IO_pins.MOSI_pin =   ;
	SSP_Dev.IO_pins.MISO_pin =   ;
	SSP_Dev.IO_pins.SCK_pin =   ;
	SSP_Dev.IO_pins.SSEL_pin =   ;

	SSP_Init(&SSP_Dev);

}

// MPL115AReadPressureAndTempADC
// Read ADCs values
int MPL115AReadPressureAndTempADC()
{
	uint32_t del;
	//Send command to convert pressure and temperature
	MPL115AWrite(0x12, 0x00);
	
	for (del = 0; del < 500000; del++);	// wait for A-D conversion to take place
	                                    // todo can be too short ! check it

	//Read converted values
	MPL115A2ADC[0] = MPL115ARead(0x0);
	MPL115A2ADC[1] = MPL115ARead(0x1);
	MPL115A2ADC[2] = MPL115ARead(0x2);
	MPL115A2ADC[3] = MPL115ARead(0x3);

	return 0;  //todo remove returned int
}

// MPL115AReadCoeffs
// Read coefficients from MPL115A
int MPL115AReadCoeffs()
{
	MPL115A2Coeffs[0] = MPL115ARead(0x4);
	MPL115A2Coeffs[1] = MPL115ARead(0x5);
	MPL115A2Coeffs[2] = MPL115ARead(0x6);
	MPL115A2Coeffs[3] = MPL115ARead(0x7);
	MPL115A2Coeffs[4] = MPL115ARead(0x8);
	MPL115A2Coeffs[5] = MPL115ARead(0x9);
	MPL115A2Coeffs[6] = MPL115ARead(0xA);
	MPL115A2Coeffs[7] = MPL115ARead(0xB);
	MPL115A2Coeffs[8] = MPL115ARead(0xC);
	MPL115A2Coeffs[9] = MPL115ARead(0xD);
	MPL115A2Coeffs[10] = MPL115ARead(0xE);
	MPL115A2Coeffs[11] = MPL115ARead(0xF);

	return 0;  //todo remove returned int
}

// MPL115ACalculatePressure
// This function calculates pressure from ADC values and coefficients
void MPL115ACalculatePressure(double *CalcPress)
{	
	char sia0MSB, sia0LSB, sib1MSB, sib1LSB, sib2MSB, sib2LSB, sic12MSB, sic12LSB, sic11MSB, sic11LSB, sic22MSB, sic22LSB;
	int16_t sia0, sib1, sib2, sic12, sic11, sic22;
	unsigned int uiPadc, PressCntdec, uiTadc, TempCntdec, siPcomp;
	int32_t lt1, lt2, lt3, si_c11x1, si_a11, si_c12x2, si_a1, si_c22x2, si_a2, si_a1x1, si_y1, si_a2x2;
	double decPcomp;


	//Pressure calculation based on Freescale AN3785
	sia0MSB = MPL115A2Coeffs[0];
	sia0LSB = MPL115A2Coeffs[1];
	sia0 = (int16_t)sia0MSB <<8; 				//s16 type //Shift to MSB
	sia0 += (int16_t)sia0LSB & 0x00FF; 			//Add LSB to 16bit number
	//coeff b1 16bit
	sib1MSB= MPL115A2Coeffs[2];
	sib1LSB= MPL115A2Coeffs[3];
	sib1 = sib1MSB <<8; 						//Shift to MSB
	sib1 += sib1LSB & 0x00FF; 					//Add LSB to 16bit number
	//coeff b2 16bit
	sib2MSB= MPL115A2Coeffs[4];
	sib2LSB= MPL115A2Coeffs[5];
	sib2 = sib2MSB <<8;							//Shift to MSB
	sib2 += sib2LSB & 0x00FF; 					//Add LSB to 16bit number
	//coeff c12 14bit
	sic12MSB= MPL115A2Coeffs[6];
	sic12LSB= MPL115A2Coeffs[7];
	sic12 = sic12MSB <<8; 						//Shift to MSB only by 8 for MSB
	sic12 += sic12LSB & 0x00FF;
	//coeff c11 11bit
	sic11MSB= MPL115A2Coeffs[8];
	sic11LSB= MPL115A2Coeffs[9];
	sic11 = sic11MSB <<8; 						//Shift to MSB only by 8 for MSB
	sic11 += sic11LSB & 0x00FF;
	//coeff c22 11bit
	sic22MSB= MPL115A2Coeffs[10];
	sic22LSB= MPL115A2Coeffs[11];
	sic22 = sic22MSB <<8; 						//Shift to MSB only by 8 for MSB
	sic22 += sic22LSB & 0x00FF;



	PressCntdec = (MPL115A2ADC[0] << 8) | MPL115A2ADC[1];
	TempCntdec = (MPL115A2ADC[2] << 8) | MPL115A2ADC[3];

	uiPadc=PressCntdec>>6; 						//Note that the PressCntdec is the raw value from the MPL115A data address. Its shifted >>6 since its 10 bit.
	uiTadc=TempCntdec>>6;						//Note that the TempCntdec is the raw value from the MPL115A data address. Its shifted >>6 since its 10 bit.
	//******* STEP 1 c11x1= c11 * Padc
	lt1 = (int32_t)sic11; 						// s(16,27) s(N,F+zeropad) goes from s(11,10)+11ZeroPad = s(11,22) => Left Justified = s(16,27)
	lt2 = (int32_t)uiPadc; 						// u(10,0) s(N,F)
	lt3 = lt1 * lt2; 							// s(26,27) /c11*Padc
	si_c11x1 = (int32_t)(lt3); 					// s(26,27)- EQ 1 =c11x1 /checked
	//divide this hex number by 2^30 to get the correct decimal value.
	//b1 =s(14,11) => s(16,13) Left justified
	//******* STEP 2 a11= b1 + c11x1
	lt1 = ((int32_t)sib1<<14); 					// s(30,27) b1=s(16,13) Shift b1 so that the F matches c11x1(shift by 14)
	lt2 = (int32_t)si_c11x1; 					// s(26,27) //ensure fractional bits are compatible
	lt3 = lt1 + lt2; 							// s(30,27) /b1+c11x1
	si_a11 = (int32_t)(lt3>>14); 				// s(16,13) - EQ 2 =a11 Convert this block back to s(16,X)
	//******* STEP 3 c12x2= c12 * Tadc
	// sic12 is s(14,13)+9zero pad = s(16,15)+9 => s(16,24) left justified
	lt1 = (int32_t)sic12; 						// s(16,24)
	lt2 = (int32_t)uiTadc; 						// u(10,0)
	lt3 = lt1 * lt2; 							// s(26,24)
	si_c12x2 = (int32_t)(lt3); 					// s(26,24) - EQ 3 =c12x2 /checked
	//******* STEP 4 a1= a11 + c12x2
	lt1 = ((int32_t)si_a11<<11); 				// s(27,24) This is done by s(16,13) <<11 goes to s(27,24) to match c12x2's F part
	lt2 = (int32_t)si_c12x2; 					// s(26,24)
	lt3 = lt1 + lt2; 							// s(27,24) /a11+c12x2
	si_a1 =(int32_t)(lt3>>11); 					// s(16,13) - EQ 4 =a1 /check
	//******* STEP 5 c22x2= c22 * Tadc
	// c22 is s(11,10)+9zero pad = s(11,19) => s(16,24) left justified
	lt1 = (int32_t)sic22; 						// s(16,30) This is done by s(11,10) + 15 zero pad goes to s(16,15)+15, to s(16,30)
	lt2 = (int32_t)uiTadc; 						// u(10,0)
	lt3 = lt1 * lt2; 							// s(26,30) /c22*Tadc
	si_c22x2 = (int32_t)(lt3); 					// s(26,30) - EQ 5 /=c22x2
	//******* STEP 6 a2= b2 + c22x2
	//WORKS and loses the least in data. One extra execution. Note how the 31 is really a 32 due to possible overflow.
	// b2 is s(16,14) User shifted left to => s(31,29) to match c22x2 F value
	lt1 = ((int32_t)sib2<<15); 					//s(31,29)

	lt2 = ((int32_t)si_c22x2>>1); 				//s(25,29) s(26,30) goes to >>16 s(10,14) to match F from sib2
	lt3 = lt1+lt2; 								//s(32,29) but really is a s(31,29) due to overflow the 31 becomes a 32.
	si_a2 = ((int32_t)lt3>>16); 				//s(16,13)
	//******* STEP 7 a1x1= a1 * Padc
	lt1 = (int32_t)si_a1; 						// s(16,13)
	lt2 = (int32_t)uiPadc; 						// u(10,0)
	lt3 = lt1 * lt2; 							// s(26,13) /a1*Padc
	si_a1x1 = (int32_t)(lt3); 					// s(26,13) - EQ 7 /=a1x1 /check
	//******* STEP 8 y1= a0 + a1x1
	// a0 = s(16,3)
	lt1 = ((int32_t)sia0<<10); 					// s(26,13) This is done since has to match a1x1 F value to add. So S(16,3) <<10 = S(26,13)
	lt2 = (int32_t)si_a1x1; 					// s(26,13)
	lt3 = lt1 + lt2; 							// s(26,13) /a0+a1x1
	si_y1 = ((int32_t)lt3>>10); // s(16,3) - EQ 8 /=y1 /check
	//******* STEP 9 a2x2= a2 *Tadc
	lt1 = (int32_t)si_a2; 						// s(16,13)
	lt2 = (int32_t)uiTadc;						// u(10,0)
	lt3 = lt1 * lt2; 							// s(26,13) /a2*Tadc
	si_a2x2 = (int32_t)(lt3); 					// s(26,13) - EQ 9 /=a2x2
	//******* STEP 10 pComp = y1 +a2x2
	// y1= s(16,3)
	lt1 = ((int32_t)si_y1<<10); 				// s(26,13) This is done to match a2x2 F value so addition can match. s(16,3) <<10
	lt2 = (int32_t)si_a2x2; 					// s(26,13)
	lt3 = lt1 + lt2; 							// s(26,13) /y1+a2x2
	// FIXED POINT RESULT WITH ROUNDING:
	siPcomp = ((int16_t)(lt3>>13)); 			// goes to no fractional parts since this is an ADC count.
	//decPcomp is defined as a floating point number.
	//Conversion to Decimal value from 1023 ADC count value. ADC counts are 0 to 1023. Pressure is 50 to 115kPa correspondingly.
	decPcomp = ((65.0/1023.0)*siPcomp)+50.0;
	*CalcPress = decPcomp;
}



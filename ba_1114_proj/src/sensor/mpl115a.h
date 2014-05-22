//******************************************************************************
//
//    THE SOFTWARE INCLUDED IN THIS FILE IS FOR GUIDANCE ONLY.
//    BTC KORPORACJA SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
//    OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
//    FROM USE OF THIS SOFTWARE.
//
//******************************************************************************
#include "type.h"

#define MPL115A_ERROR 1

void MPL115AIntitalize(void);
int MPL115AReadPressureAndTempADC(void);
int MPL115AReadCoeffs(void);
void MPL115ACalculatePressure(double *CalcPress);


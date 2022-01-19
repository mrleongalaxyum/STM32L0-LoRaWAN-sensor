//=============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//=============================================================================
/// \file    system.c (V1.0)
/// \author  RFU
/// \date    24-Jan-2016
/// \brief   System functions
//=============================================================================

#include "system.h"
#include "stdint.h"

//-----------------------------------------------------------------------------
/* -- adapt this delay for your uC -- */
void DelayMicroSeconds(uint32_t nbrOfUs)
{
	HAL_Delay(nbrOfUs / 1000);
}

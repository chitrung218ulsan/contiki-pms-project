/*
 * cc26xx_adc.c
 *
 *  Created on: Jun 12, 2016
 *      Author: user
 */
#include "contiki.h"
#include "ti-lib.h"
#include "driverlib/aux_adc.h"
#include "driverlib/aux_wuc.h"


#include <stdio.h>
#include <stdint.h>

void cc26xx_adc_init(uint32_t ADCInput)
{

	//intialisation of ADC
	ti_lib_aon_wuc_aux_wakeup_event(AONWUC_AUX_WAKEUP);
	while(!(ti_lib_aon_wuc_power_status_get() & AONWUC_AUX_POWER_ON))
	{ }

	// Enable clock for ADC digital and analog interface (not currently enabled in driver)
	// Enable clocks
	ti_lib_aux_wuc_clock_enable(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK);
	while(ti_lib_aux_wuc_clock_status(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK) != AUX_WUC_CLOCK_READY)
	{ }

	printf("clock selected\r\n");

	// Connect AUX IO7 (DIO23, but also DP2 on XDS110) as analog input.
	AUXADCSelectInput(ADCInput);
	printf("input selected\r\n");

	// Set up ADC range
	// AUXADC_REF_FIXED = nominally 4.3 V
	AUXADCEnableSync(AUXADC_REF_FIXED,  AUXADC_SAMPLE_TIME_2P7_US, AUXADC_TRIGGER_MANUAL);
	printf("init adc --- OK\r\n");
}
/*
 *
 */
void cc26xx_adc_start_convert()
{
	  AUXADCGenManualTrigger();
	  printf("trigger --- OK\r\n");
}
/*
 *
 */
uint16_t cc26xx_get_adc_value()
{
	uint16_t singleSample = AUXADCReadFifo();
	return singleSample;
}
void cc26xx_adc_stop()
{
	 AUXADCDisable();
	 printf("disable --- OK\r\n");
}





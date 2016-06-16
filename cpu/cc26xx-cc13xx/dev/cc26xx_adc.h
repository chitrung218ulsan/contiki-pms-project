/*
 * cc26xx_adc.h
 *
 *  Created on: Jun 12, 2016
 *      Author: user
 */

#ifndef CC26XX_ADC_H_
#define CC26XX_ADC_H_

void cc26xx_adc_init();
void cc26xx_adc_start_convert();
void cc26xx_adc_stop();
uint16_t cc26xx_get_adc_value();


#endif /* CC26XX_ADC_H_ */

/*
 * module-cc2630-example.c
 *
 *  Created on: Jun 10, 2016
 *      Author: user
 */


#include "contiki.h"
#include "net/mac/nullrdc.h"
#include "net/rime/device-module-cc26xx.h"
#include "net/rime/ftsp.h"
#include <stdio.h>
#include <string.h>
//PROCESS(test_human_detection_process, "Test human detection algorithm");
PROCESS(device_process, "device process");

AUTOSTART_PROCESSES(&device_process);

PROCESS_THREAD(device_process, ev, data)
{
	static struct etimer et1;
	PROCESS_BEGIN();

	ftsp_init();
	ftsp_set_mode(TS_USER_MODE);
#if RM_SCD
	device_module_open();
	RM_SCD_InitSystem();
#endif
#if HD_SCD
	device_module_open();
	//static struct human_detection_str humanDetection;
	//human_detection_init(&humanDetection);
#endif
#if PMS_SCD || TEST_CODE
	static uint8_t pms_start = 0;


	//SystemInit();
	etimer_set(&et1, 10*CLOCK_SECOND );
#endif

	while(1)
	{
			PROCESS_WAIT_EVENT();

	#if PMS_SCD || TEST_CODE

			if(etimer_expired(&et1))
			{
				if(ftsp_get_seqNum() <= 50)
				//if(ftsp_get_seqNum() <= 15)
				{
					etimer_set(&et1, 2*CLOCK_SECOND);
					ftsp_send();
				}
				else
				{
					 if(pms_start == 0)
					 {
						 //SystemInit();
						 device_module_open();
						 pms_start = 1;
						 //break;
					 }
				}
			}
	#endif
	}

	PROCESS_END();
}

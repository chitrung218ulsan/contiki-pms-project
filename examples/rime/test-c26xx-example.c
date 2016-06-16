/*
 * test-c26xx-example.c
 *
 *  Created on: Jun 13, 2016
 *      Author: user
 */

#include "contiki.h"
#include <stdio.h>
#include <string.h>
//PROCESS(test_human_detection_process, "Test human detection algorithm");
PROCESS(test_cc26xx_process, "test cc26xx device process");

AUTOSTART_PROCESSES(&test_cc26xx_process);

PROCESS_THREAD(test_cc26xx_process, ev, data)
{
	static struct etimer et1;
	PROCESS_BEGIN();
	etimer_set(&et1, 5*CLOCK_SECOND );
	while(1)
	{
		PROCESS_WAIT_EVENT();
		if(etimer_expired(&et1))
		{
			printf("Test  CC26xx chip \n");
		}
		etimer_reset(&et1);
	}
	PROCESS_END();
}



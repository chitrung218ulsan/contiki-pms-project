/*
 * device-module-cc26xx.c
 *
 *  Created on: Jun 10, 2016
 *      Author: user
 */
#include "contiki.h"
#include "net/rime/rime.h"
#include "net/netstack.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "sys/rtimer.h"

#include "device-module-cc26xx.h"
#include "net/mac/contikimac/contikimac.h"

#include "net/rime/ftsp.h"
#include "net/linkaddr.h"
#include "sys/clock.h"

#include "net/mac/nullrdc.h"

#include "lib/ifft.h"

/*
 * For chip CC2630, we can include file project-conf and define USE_CC2630_CONFIG or USE_CC2420_CONFIG
 * For chip CC2420, we have to define directly in file contiki-conf.h in the folder flatform
 */


#ifdef USE_CC2630_CONFIG
#define USE_CC2630 USE_CC2630_CONFIG
#else USE_CC2630 0
#endif

#if USE_CC2630
#include "cc26xx_adc.h"
#include "rf-core/rf-ble.h"

#include "ti-lib.h"
#endif


#define DEVICE_MODULE_CYCLE_TIME	2*CLOCK_SECOND


extern uint32_t nullrdc_rm_cycle;
extern uint32_t nullrdc_pms_count_sent;

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from);

static CC_CONST_FUNCTION struct broadcast_callbacks module_device_broadcast_call = {broadcast_recv};

struct broadcast_conn broadcast;

#if PMS_SCD || TEST_CODE
//void device_module_PMS_send_command_to_RM(uint8_t commandType);
//void device_module_PMS_send_data_to_SRT();
//static void device_module_rm_ack_expire_handle (void *ptr);
//static void ctimer_turn_off_air_exp_func(void*ptr);
static void device_module_pms_send_adv_time_syn();
static void device_module_set_period_first(void *ptr);
static void device_module_set_period_general(void*ptr);
static void device_module_PMS_send_command_to_RM_directly(uint8_t commandType);
static void device_module_reset_sync_period_c(void *ptr);
static void device_module_pms_send_adv_time_resyn();
static void device_module_send_adv_sync_2(void *ptr);
static void check_current_off_experation_func(void*ptr);
static void check_current_on_experation_func(void*ptr);
#endif /*PMS_SCD || TEST_CODE */

#if HD_SCD
static void device_module_check_PIR();
static int16_t q_div(int16_t a,int16_t b);
static int16_t q_mul(int16_t a,int16_t b);
static uint8_t device_module_process_data_PIR (int16_t *PIRDataset);
#endif



#if RM_SCD
static void device_module_set_period_first(void *ptr);
static void device_module_set_period_general(void*ptr);
#endif /*RM_SCD*/



// Variable


/*
 * Function:device_module_open
 */


#if HD_SCD

#define MW_FFT_SIZE 128
#define PIR_DATA_POINT	50
uint16_t cycleCount ;
extern uint8_t nullrdc_hd_try_nums;


static int16_t HD_MWre[MW_FFT_SIZE];

static int16_t HD_MWimg[MW_FFT_SIZE];

static int16_t FFT_buffer[MW_FFT_SIZE];

static int16_t PIR_buffer[PIR_DATA_POINT];

static uint8_t device_module_HD_check;
static uint8_t device_module_curState;
static uint8_t device_module_MW_is_checking;

static uint8_t device_module_HD_retransmit_room_status;
static const int16_t check_max_no_movement[20] =
{
	700, 400, 200, 125, 70, 40, 30, 30, 20, 15, 15, 10, 5, 5, 6, 3, 3, 3, 2, 2

};


PROCESS(MW_check_process,"Test_MW_check_process");
//PROCESS(FFT_process,"Test_FFT_check_process");
//PROCESS(ADC_process,"Test_ADC_check_process");

PROCESS_THREAD(MW_check_process,ev,data)
{

	PROCESS_BEGIN();
	printf("start MW_check_process thread \n");
	static int i = 0;
	static uint8_t check_count = 0;
	static uint8_t FFT_HD_movement_detect = 0;
	while(1)
	{
		PROCESS_WAIT_EVENT();

#if USE_CC2630
		check_count = 0;
		FFT_HD_movement_detect = 0;
		device_module_MW_is_checking = 1;

		// provide power for PIR
		ti_lib_ioc_pin_type_gpio_output(MW_CTRL_POWER_IOID);

		ti_lib_gpio_pin_write(MW_CTRL_POWER_PIN, 1);

		// start ADC
		cc26xx_adc_init(ADC_COMPB_IN_AUXIO5); // DIO25 for MW

		for(check_count = 0;check_count < 10;check_count ++)
		{
			for(i=0;i<128;i++)
			{
				cc26xx_adc_start_convert();

				HD_MWre[i] = cc26xx_get_adc_value();
			}

			cc26xx_adc_stop();

			// Cut off power for PIR

			ti_lib_gpio_pin_write(MW_CTRL_POWER_PIN, 0);

			// Processing FFT
			for(i = 0;i < MW_FFT_SIZE;i++)
			{

				FFT_buffer[i] = -128 + (HD_MWre[i] >> 4);

			}

			ifft(FFT_buffer, HD_MWimg, MW_FFT_SIZE);

			for (i = 1; i < MW_FFT_SIZE/2; i++)
			{
				if(i <=20)
				{
					if(check_max_no_movement[i-1] < FFT_buffer[i] )
					{
						//check_max_no_movement[i] = FFT_buffer[i];
						FFT_HD_movement_detect ++;
						//printf("compare i: %d ,amplitude: %d \n",i,FFT_buffer[i]);
					}
					//printf("compare i: %d ,amplitude: %d \n",i,check_max_no_movement[i]);
				}
			}


			if( FFT_HD_movement_detect >=1 )
			{
				printf("FFT done ->>>>>>> movement \n");
				break;
			}
			else
			{
				printf("FFT done ->>>>>>> no movement \n");
			}
		} // for


		device_module_MW_is_checking = 0;

		if(FFT_HD_movement_detect >=1)
		{
			device_module_curState = E_STATE;

			device_module_HD_check = 30;

			device_module_turn_on_Radio();

			device_module_HD_send_status_to_PMS_diretly(device_module_curState);

		}
		else
		{
			device_module_HD_check = 0;

			device_module_curState = NE_STATE;

			device_module_turn_on_Radio();

			device_module_HD_send_status_to_PMS_diretly(device_module_curState);
		}
#endif
	}
	PROCESS_END();
}

static void device_module_check_PIR()
{
#if USE_CC2630
	int i = 0;

	uint8_t PIR = 0;


	// provide power for PIR
	ti_lib_ioc_pin_type_gpio_output(PIR_CTRL_POWER_IOID);

	ti_lib_gpio_pin_write(PIR_CTRL_POWER_PIN, 1);


	// Get data of PIR
	cc26xx_adc_init(ADC_COMPB_IN_AUXIO4); // DIO26 for PIR

	for(i=0;i<PIR_DATA_POINT;i++)
	{
		cc26xx_adc_start_convert();

		PIR_buffer[i] = cc26xx_get_adc_value();
	}

	//stop ADC
	cc26xx_adc_stop();
	//Cut off power for PIR
	ti_lib_gpio_pin_write(PIR_CTRL_POWER_PIN, 0);

	PIR = device_module_process_data_PIR(PIR_buffer);

	if(PIR == 1)
	{

		device_module_HD_check = 30;
		if(device_module_curState == NE_STATE)
		{
			//send to pms from NE_STATE to E_STATE
			device_module_curState = E_STATE;

			device_module_turn_on_Radio();

			device_module_HD_send_status_to_PMS_diretly(device_module_curState);
		}
		else
		{
			//ctimer_stop(&turnOffCPUAndRadioTimer);
			//device_module_off_CPU_Radio(NULL);
		}
	}
	else if(PIR == 0 && device_module_HD_check > 0)
	{
		device_module_HD_check -= 1;

		if(device_module_HD_check == 0 && device_module_curState == E_STATE)
		{
			printf("At device_module_check_PIR, PIR: 0, device_module_HD_check: 0 \n");
			// send status to pms from E_STATE to NE_STATE
			//call PMW to check
			process_post_synch(&MW_check_process,PROCESS_EVENT_CONTINUE,NULL);

		}
		else
		{
			//ctimer_stop(&turnOffCPUAndRadioTimer);
			//device_module_off_CPU_Radio(NULL);
		}
	}
#endif
}
/*
 *
 *
 */
static int16_t q_div(int16_t a,int16_t b)
{
	int16_t result;
	int32_t temp;
	temp = (int32_t)a << 3;
	if((temp >=0 && b >=0) || (temp <0 && b <0))
	{
		temp += b/2;
	}
	else
		temp -= b/2;

	result = (int16_t)(temp/b);

	return result;
}
/*
 *
 *
 */
static int16_t sat16(int32_t x)
{
	if(x > 32768)
		return 32768;
	else if(x < -32768)
		return -32768;
	else
		return (int16_t)x;
}
/*
 *
 */
static int16_t q_mul(int16_t a,int16_t b)
{
	int16_t result;
	int32_t temp;
	temp = (int32_t)a * (int32_t)b;
	temp = temp + 4;
	temp = temp >> 3;
	result = sat16(temp);

	return result;
}
/*
 *
 *
 */
static uint8_t device_module_process_data_PIR (int16_t *PIRDataset)
{
	uint8_t PIR=0;

	uint8_t outCnt = 0;

	int i;

	int16_t tempPIR = 0;
	int16_t tempPIR1 = 0;
	int16_t temp = 0;

	for(i=0 ; i<50;i++)
	{
		temp = *(PIRDataset++);

		tempPIR = q_div(temp,4095);

		tempPIR1 = q_mul(tempPIR,20);

		//printf("At device_module_human_detection_test_PIR_1, data: %d, q_div: %d, q_mul:%d \n",temp,tempPIR,tempPIR1);

		if(tempPIR1 < 1)
		{
			outCnt++;
		}
	}
	if(outCnt > 5)
	{
		PIR = 1;
	}
	printf("PIR Out: %d \n",PIR);
	return PIR;
}
/*
 *
 */
void device_module_set_period_general(void*ptr)
{

	cycleCount += 1;
	if(cycleCount % 300 == 0)
	{
		device_module_turn_on_Radio();
		ctimer_stop(&periodTimer);
		ftsp_reset_seqNum();
		return;
	}


	if(device_module_MW_is_checking == 0)
	{
		ctimer_reset(&periodTimer);

		device_module_check_PIR();

		//process_post(&MW_check_process,PROCESS_EVENT_CONTINUE,NULL);

	}
	else
	{

		ctimer_reset(&periodTimer);
	}
}
void device_module_set_period_first(void *ptr)
{
	ctimer_set( &periodTimer, DEVICE_MODULE_CYCLE_TIME, device_module_set_period_general, NULL);
}

void device_module_HD_send_status_to_PMS_diretly(uint8_t roomStatus)
{
	//TurnOnCC2420(); for chip CC2420
	// Turn on radio chip of CC2630
	nullrdc_hd_try_nums =0;
	struct hd_message_str messagePkt;
	messagePkt.frameType = HD_ROOM_STATUS;
	messagePkt.nodeId = linkaddr_node_addr.u8[0];
	messagePkt.roomStatus = roomStatus;
	if(DEBUG_ENABLE)
	{
		printf("Human-detection: HD node %d sends room status to PMS node\n",linkaddr_node_addr.u8[0]);
	}
	uint8_t ret = nullRdc_HD_send_room_status_to_PMS(&messagePkt,sizeof(struct hd_message_str));
	if(ret != RADIO_TX_OK)
	{
		// set flag to retransmit the room status
	}
	//call function to turn off radio chip
}

void device_module_off_CPU_Radio(void *ptr)

{


	/*
	TurnOFFCC2420();
	//_BIS_SR(GIE | SCG0 | SCG1 | CPUOFF);
	_BIS_SR(LPM3_bits);
	printf("HD node %d turns off Radio and CPU, cycle: %d \n",rimeaddr_node_addr.u8[0],cycleCount);
	*/

	//To do for new chip CC2630
}

#endif /* HD_SCD */

#if PMS_SCD || TEST_CODE

extern uint8_t nullrdc_pms_try_nums;
uint16_t count_PMS = 0;
uint16_t cycleCount = 0;

static void device_module_pms_send_adv_time_resyn()
{
	struct pms_message_adv_time_syn messagePkt;
	messagePkt.frameType = PMS_ADV_TIME_SYN;
	messagePkt.nodeId = linkaddr_node_addr.u8[0];
	messagePkt.startSynTime = get_global_time() + 1*CLOCK_SECOND;
	timeStamp = messagePkt.startSynTime;

	ctimer_stop(&periodTimer_1);
	ctimer_set(&periodTimer_1, 1*CLOCK_SECOND, device_module_set_period_first, NULL);

	packetbuf_copyfrom(&messagePkt, sizeof(struct pms_message_adv_time_syn));

	broadcast_send(&broadcast);

	ctimer_set(&periodAdvSyncTimer, 50, device_module_send_adv_sync_2, NULL);

	if(DEBUG_ENABLE)
	{
		printf("PMS node %d sends ADV Time SYN at globalTime: %d \n",linkaddr_node_addr.u8[0],
				get_global_time());
	}

}


static void device_module_reset_sync_period_c(void *ptr)
{

	if(ftsp_get_seqNum() <= 15)
	 {
			ftsp_send();
			ctimer_set(&periodSyncTimer, 50, device_module_reset_sync_period_c, NULL);
	 }
	 else
	 {
		  printf("Start send ADV SYNC \n");
		  device_module_pms_send_adv_time_resyn();
	 }
}

static void device_module_set_period_general(void*ptr)
{
	rtimer_clock_t rtime_now;
	rtime_now = RTIMER_NOW();
	ctimer_reset(&periodTimer);
	//only send command to RM within first 50 ms
	dutyCycleOn = 1;
	// call function to send command to RM

	// For test only periodically, Mungtv
	//device_module_PMS_send_command_to_RM_directly(PMS_COMMAND_IR_OFF_TV_REQ);
	count_PMS++;
	cycleCount ++;

	if(cycleCount % 300 == 0)
	{
		if(DEBUG_ENABLE)
		{
			printf("Cycle Count Value: %u \n",cycleCount);
		}
		ctimer_stop(&periodTimer);
		ftsp_reset_seqNum();
		ctimer_set(&periodSyncTimer, 25, device_module_reset_sync_period_c, NULL);
		return;
	}


	if(offCommand)
	{
		//ctimer_set(&ctimer_PMS_send_command_to_RM,2,ctimer_PMS_send_command_to_RM_exp_func,NULL);
		device_module_PMS_send_command_to_RM_directly(PMS_COMMAND_IR_OFF_TV_REQ);
		offCommand = 0;

		//set timer to check current
		ctimer_set(&check_currren_off_timer,32*CLOCK_SECOND,check_current_off_experation_func,NULL);
	}
	else if(onCommand)
	{
		device_module_PMS_send_command_to_RM_directly(PMS_COMMAND_IR_ON_TV_REQ);
		onCommand = 0;
		//set timer to check current
		ctimer_set(&check_currren_on_timer,3*CLOCK_SECOND,check_current_on_experation_func,NULL);
	}

	//ctimer_set(&turnOffCPUAndRadioTimer,cycleTime,device_module_off_CPU_Radio,NULL);

	if(DEBUG_ENABLE){
		//printf("PMS node Id: %d wakes up, cycle: %u,send cmd %u at globalTime: %u \n", rimeaddr_node_addr.u8[0], cycleCount, nullrdc_pms_count_sent, get_global_time());
	}
}


static void device_module_set_period_first(void *ptr)
{
	ctimer_set( &periodTimer, DEVICE_MODULE_CYCLE_TIME, device_module_set_period_general, NULL);
}

static void device_module_send_adv_sync_2(void *ptr)
{
	if(synSeq >= 3)
	{
		synSeq = 0;
		ctimer_stop(&periodAdvSyncTimer);
		return;
	}
	synSeq++;

	struct pms_message_adv_time_syn messagePkt;
	messagePkt.frameType = PMS_ADV_TIME_SYN;
	messagePkt.nodeId = linkaddr_node_addr.u8[0];
	messagePkt.startSynTime = timeStamp;

	packetbuf_copyfrom(&messagePkt, sizeof(struct pms_message_adv_time_syn));

	broadcast_send(&broadcast);

	if(DEBUG_ENABLE)
	{
		printf("PMS node %d sends ADV Time SYN at globalTime: %d \n",linkaddr_node_addr.u8[0],
				get_global_time());
	}

	ctimer_set(&periodAdvSyncTimer, 50, device_module_send_adv_sync_2, NULL);
}

static void device_module_pms_send_adv_time_syn()
{
	struct pms_message_adv_time_syn messagePkt;
	messagePkt.frameType = PMS_ADV_TIME_SYN;
	messagePkt.nodeId = linkaddr_node_addr.u8[0];
	messagePkt.startSynTime = get_global_time() + 10*CLOCK_SECOND;

	timeStamp = messagePkt.startSynTime;

	ctimer_stop(&periodTimer_1);
	ctimer_set(&periodTimer_1, 10*CLOCK_SECOND, device_module_set_period_first, NULL);

	if(DEBUG_ENABLE)
	{
		printf("PMS node %d sends ADV Time SYN at globalTime: %d \n",linkaddr_node_addr.u8[0],
				get_global_time());
	}

	packetbuf_copyfrom(&messagePkt, sizeof(struct pms_message_adv_time_syn));

	broadcast_send(&broadcast);

	ctimer_set(&periodAdvSyncTimer, 50, device_module_send_adv_sync_2, NULL);
}


void device_module_PMS_update_Room_Status_From_HD(void *message){

	if(DEBUG_ENABLE)
	{
		printf("PMS_SCD %d receives HD_ROOM_STATUS from HD node %d, room_status:%d \n",
				linkaddr_node_addr.u8[0],pms_hd_NodeId,pms_room_status);
	}

	struct hd_message_str *messagePkt;
	messagePkt = (struct hd_message_str*) message;
	pms_hd_NodeId = messagePkt->nodeId;

	if(messagePkt->roomStatus == 2)
	{
		/*// Do nothing depending on the status of PMC
		if(!(PMC_SCD.Status == STANDBY || PMC_SCD.Status == OPERATING|| PMC_SCD.Status == MEASURING))
		{
			printf("PMC receive HD turn off command, do nothing at status %d\n", PMC_SCD.Status);
			return;
		}
		*/
#if PMS_SCD_NO_REMOTE

		printf("PMC connects to device without remove ->>>>> turn off device without remote \n");

		//call function to cut off the power

		//ProcessCutOffMode();
#endif
		//if connecting with device having remote

		//send command to RM to off device and cutoff

#if PMS_SCD_REMOTE_TV

		offCommand = 1;
#endif

#if PMS_SCD_REMOTE_AIR

		//device_module_PMS_send_command_to_RM(PMS_COMMAND_IR_OFF_AIR_REQ);

		//set timer to cut off air since the air is turned off by TV PMS
		offCommand = 1;

#endif

	}
	pms_room_status = messagePkt->roomStatus;
}

static void device_module_PMS_send_command_to_RM_directly(uint8_t commandType)
{
	struct pms_command_message_str messagePkt;
	messagePkt.frameType = commandType;
	messagePkt.nodeId = linkaddr_node_addr.u8[0];


	nullrdc_pms_try_nums = 0;
	if(DEBUG_ENABLE)
	{
		printf("PMS node %d sends command: %d to RM node\n",linkaddr_node_addr.u8[0],commandType);
	}
	uint8_t ret = nullRdc_PMS_send_command_to_RM(&messagePkt,sizeof(struct pms_command_message_str));
	if(ret != RADIO_TX_OK)
	{
		if(commandType == PMS_COMMAND_IR_OFF_TV_REQ)
		{
			offCommand = 1;
		}
	}

	if(DEBUG_ENABLE)
	{
		//printf("PMS node %d sends command: %d to RM node\n",linkaddr_node_addr.u8[0],commandType);
	}
}


static void check_current_off_experation_func(void*ptr)
{
	// Check the status of device wheather it is off or not
	/*
	if(PMC_SCD.Status != CUTOFF)
	{
		//send command to turn off once more time
		offCommand = 1;
	}
	*/
}
static void check_current_on_experation_func(void *ptr)
{
	// Check the status of device wheather it is on or not
/*
	if(PMC_SCD.Status == CUTOFF)
	{

		onCommand = 1;
	}
*/
}

#endif /*PMS_SCD */


#if RM_SCD

    uint16_t cycleCount ;


	void device_mode_RM_send_IR_TV(uint8_t commandType)
	{
		if(commandType == PMS_COMMAND_IR_OFF_TV_REQ)
		{
			// call function to send IR to turn on or turn off device
			//TVControl();
		}
		else if(commandType == PMS_COMMAND_IR_ON_TV_REQ){

			// call function to send IR to turn on or turn off device
			//TVControl();
		}
		if(DEBUG_ENABLE)
		{
			printf("RM node %d send IR TV \n",linkaddr_node_addr.u8[0]);
		}
	}

	static void device_module_set_period_first(void *ptr)
	{
		ctimer_set( &periodTimer, DEVICE_MODULE_CYCLE_TIME, device_module_set_period_general, NULL);
	}

	static void device_module_set_period_general(void*ptr)
	{


		//TurnOnCC2420();
		//rtimer_set(&radioOnTimer, RTIMER_NOW() + 1, 1, (rtimer_callback_t)device_module_set_timer_radio_on, NULL);

		device_module_turn_on_Radio();

		ctimer_reset(&periodTimer);
		cycleCount = cycleCount + 1;
		nullrdc_rm_cycle = cycleCount;

		if(cycleCount % 300 == 0)
		{
			ctimer_stop(&periodTimer);
			ftsp_reset_seqNum();
			return;
		}


		if(DEBUG_ENABLE){
			//printf("RM node Id: %d, cycle: %u, wakes up at globalTime: %d  \n",rimeaddr_node_addr.u8[0],cycleCount, get_global_time());
		}
		//ctimer_set(&periodTimer_2,(1*CLOCK_SECOND)-1,device_module_set_period_general_1,NULL);

		//after 1000 ms -> off Radio and CPU to save energy
		ctimer_set(&turnOffCPUAndRadioTimer,1*CLOCK_SECOND,device_module_save_power,NULL);


		rtimer_clock_t rtime_now;
		rtime_now = RTIMER_NOW();
		//printf("RM node Id: %d wakes up at globalTime: %u \n",rimeaddr_node_addr.u8[0],rtime_now);

	}


#endif /*RM_SCD*/




static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{

	struct header_str *header = (struct header_str *)packetbuf_dataptr();
	uint8_t frameType = header->frameType;
#if HD_SCD
	if(DEBUG_ENABLE)
	{

	}
	if(frameType == PMS_ADV_TIME_SYN)
	{
		struct pms_message_adv_time_syn *messagePkt;
		messagePkt = (struct pms_message_adv_time_syn*) packetbuf_dataptr();
		if(DEBUG_ENABLE)
		{
			printf("HD_SCD %d receives PMS_ADV_TIME_SYN from PMS_SCD node %d \n",
					linkaddr_node_addr.u8[0],messagePkt->nodeId);
		}

		//
		ctimer_stop(&periodTimer_1);
		ctimer_set(&periodTimer_1, messagePkt->startSynTime-get_global_time(), device_module_set_period_first, NULL);
		//TurnOFFCC2420(); // for cc2420
		//call function to turn off radio for chip CC2630

	}
#endif

#if RM_SCD
	if(frameType == HD_ROOM_STATUS)
	{
		/*
		struct hd_message_str *messagePkt;
		messagePkt = (struct hd_message_str*) packetbuf_dataptr();
		if(DEBUG_ENABLE)
		{
			printf("RM_SCD %d receives HD_ROOM_STATUS from HD node %d, room_status: %d \n",
					rimeaddr_node_addr.u8[0],messagePkt->nodeId,messagePkt->roomStatus);
		}
		if(rm_room_status == 1 && messagePkt->roomStatus == 2)
		{
			// call IR function to turn off device
			//TurnOffSS();
			// call function to send command to ask PMS_SCD to cut off stanby power
			//device_module_RM_send_command_to_PMS(RM_COMMAND_OFF_REQ);
		}
		rm_room_status = messagePkt->roomStatus;

		*/

	}
	else if(frameType == PMS_COMMAND_IR_ON_REQ)
	{
		/*struct pms_command_message_str *messagePkt;
		messagePkt = (struct pms_command_message_str*) packetbuf_dataptr();

		// call IR function to turn on device
		int i;
		for(i=0;i<5;i++)
		{
			SetTemp27(Hauzen);
		}

		device_module_RM_send_ACK_to_PMS(RM_ACK_ON);
		if(DEBUG_ENABLE)
		{
			printf("RM_SCD %d receives PMS_COMMAND_IR_ON_REQ from PMS node %d \n",
					rimeaddr_node_addr.u8[0],messagePkt->nodeId);
		}
		*/
	}
	else if(frameType == PMS_COMMAND_IR_OFF_TV_REQ)
	{
		// call function to turn off TV
		//printf("turn off TV all\n");
		//device_module_RM_send_IR_notification();
		//device_module_RM_send_ACK_to_PMS(RM_ACK_OFF);
		//printf("Send ack ok \n");

		//TVControl();

		//printf("TV Control OK 1\n");

		//ctimer_set(&ctimer_turn_off_air, 10*CLOCK_SECOND,ctimer_turn_off_expiration_func,NULL);
		//printf("TV Control OK 2\n");
	}
	else if(frameType == PMS_COMMAND_IR_OFF_AIR_REQ)
	{
		// call function to turn off AIR
		//printf("turn off AIR all\n");

		//device_module_RM_send_ACK_to_PMS(RM_ACK_OFF);

		//device_module_RM_send_IR_notification();
		//TVControl();

		//ctimer_set(&ctimer_turn_off_air, 5*CLOCK_SECOND,ctimer_turn_off_expiration_func,NULL);
		//printf("TV Control OK 2\n");
	}
	else if(frameType == PMS_COMMAND_IR_ON_AIR_REQ)
	{
		//SetTemp27(Hauzen);
		//device_module_RM_send_ACK_to_PMS(RM_ACK_ON);
	}
	else if(frameType == PMS_COMMAND_IR_ON_TV_REQ)
	{
		// call function to turn on TV
		//TVControl();

		//ctimer_set(&ctimer_turn_on_air, 10*CLOCK_SECOND,ctimer_turn_on_expiration_func,NULL);

		//device_module_RM_send_ACK_to_PMS(RM_ACK_ON);
	}
	else if(frameType == PMS_ADV_TIME_SYN)
	{
		struct pms_message_adv_time_syn *messagePkt;
		messagePkt = (struct pms_message_adv_time_syn*) packetbuf_dataptr();

		ctimer_stop(&periodTimer_1);

		ctimer_set(&periodTimer_1, messagePkt->startSynTime-get_global_time(), device_module_set_period_first, NULL);

		//TurnOFFCC2420();
		// Turn off Radio chip
		//

		if(DEBUG_ENABLE)
		{
			printf("RM_SCD %d receives PMS_ADV_TIME_SYN from PMS_SCD node %d \n",
					linkaddr_node_addr.u8[0],messagePkt->nodeId);
		}
	}
#endif /*RM_SCD*/

#if PMS_SCD

	if(DEBUG_ENABLE)
	{

	}
	/*if(frameType == HD_ROOM_STATUS)
	{
		struct hd_message_str *messagePkt;
		messagePkt = (struct hd_message_str*) packetbuf_dataptr();
		pms_hd_NodeId = messagePkt->nodeId;
		if(messagePkt->roomStatus == 2)
		{
			// call IR function to turn off device
			//TurnOffSS();
			// call function to send command to ask PMS_SCD to cut off stanby power
			//device_module_RM_send_command_to_PMS(RM_COMMAND_OFF_REQ);

			//if connecting with device without remote
			if(!(PMC_SCD.Status == STANDBY || PMC_SCD.Status == OPERATING|| PMC_SCD.Status == MEASURING))
			{
				printf("PMC receive HD turn off command, do nothing at status %d\n", PMC_SCD.Status);
				return;
			}
#if PMS_SCD_NO_REMOTE
			printf("turn off Device without remote \n");
			ProcessCutOffMode();
#endif


			//if connecting with device having remote

			//send command to RM to off device and cutoff
#if PMS_SCD_REMOTE_TV

			device_module_PMS_send_command_to_RM(PMS_COMMAND_IR_OFF_TV_REQ);
#endif

#if PMS_SCD_REMOTE_AIR

			//device_module_PMS_send_command_to_RM(PMS_COMMAND_IR_OFF_AIR_REQ);

			//set timer to cut off air since the air is turned off by TV PMS
			ctimer_set(&ctimer_turn_off_air_at_pms,13*CLOCK_SECOND,ctimer_turn_off_air_exp_func,NULL);
#endif

		}
		pms_room_status = messagePkt->roomStatus;

		if(DEBUG_ENABLE)
		{
			printf("PMS_SCD %d receives HD_ROOM_STATUS from HD node %d, room_status:%d \n",
					rimeaddr_node_addr.u8[0],hd_NodeId,room_status);
		}
	}*/
	/*else if(frameType == RM_COMMAND_OFF_REQ)
	{
		struct rm_message_str *messagePkt;
		messagePkt = (struct rm_message_str*) packetbuf_dataptr();
		if(DEBUG_ENABLE)
		{
			printf("PMS_SCD %d receives RM_COMMAND_OFF_REQ from RM node %d to cut off standby power \n",
					rimeaddr_node_addr.u8[0],messagePkt->nodeId);
		}
		// check if PMS_SCD connects with device having remote
		// before cutoff
		ProcessCutOffMode();
	}
	else if(frameType == RM_ACK_OFF)
	{
		struct rm_message_str *messagePkt;
		messagePkt = (struct rm_message_str*) packetbuf_dataptr();
		if(DEBUG_ENABLE)
		{
			printf("PMS_SCD %d receives ACK for off device from RM node %d \n",
					rimeaddr_node_addr.u8[0],messagePkt->nodeId);
		}
		ctimer_stop(&rm_ack_timer);
		pms_send_count = 0;

		//ProcessCutOffMode();

	}

	else if(frameType == RM_ACK_ON)
	{
		ctimer_stop(&rm_ack_timer);
		pms_send_count = 0;
		if(DEBUG_ENABLE)
		{
			struct rm_message_str *messagePkt;
			messagePkt = (struct rm_message_str*) packetbuf_dataptr();
			printf("PMS_SCD %d receives ACK for on device from RM node %d \n",
					rimeaddr_node_addr.u8[0],messagePkt->nodeId);
		}
	}
	*/
	else if(frameType == RM_SENDING_IR)
	{

	}
#endif /*PMS_SCD*/


#if SRT_SCD
	if(DEBUG_ENABLE)
	{

	}
#endif

}

void device_module_open()
{
	broadcast_open(&broadcast, 150, &module_device_broadcast_call);
	nullrdc_pms_count_sent = 0;

	int i = 0;

	uint32_t battery_voltage ;

	device_module_get_battery_voltage(&battery_voltage);

	printf("Current battery voltage: %d \n",battery_voltage);

#if RM_SCD

	cycleCount = 0;
#endif

#if PMS_SCD || TEST_CODE
	printf("PMC SCD Start Module \n");
	//flag_turn_on_wait_to_ack = 0;
	onCommand = 0;
	offCommand = 0;
	cycleCount = 0;
	synSeq = 0;
	// send global time syn message
	device_module_pms_send_adv_time_syn();
#endif /* PMS_SCD */



#if HD_SCD
	printf("Human Detection Sensor Start \n");
	process_start(&MW_check_process,NULL);
	cycleCount = 0;
	//process_start(&FFT_process,NULL);
	//process_start(&ADC_process,NULL);
	/*for(i=0;i<MW_FFT_SIZE;i++)
	{
		HD_MWre[i] = 0;
		HD_MWre[i] = 0;
	}*/
	device_module_curState = E_STATE;
	device_module_HD_check = 5;
	device_module_MW_is_checking = 0;
	device_module_HD_retransmit_room_status = 0;
	//ctimer_set(&periodTimer, 1*CLOCK_SECOND,device_module_set_period_general,NULL);
#endif

}

/*
 * Function: Turn off Radio chip
 */

void device_module_turn_off_Radio()
{
	NETSTACK_RADIO.off();
}

void device_module_turn_on_Radio()
{
	NETSTACK_RADIO.on();
}

void device_module_save_power()
{
	device_module_turn_off_Radio();
}

void device_module_get_battery_voltage(uint32_t *battery_voltage)
{
	*(battery_voltage) = 0;
#if USE_CC2630
	 ti_lib_aon_batmon_enable();
	 *battery_voltage =ti_lib_aon_batmon_battery_voltage_get();
	 ti_lib_aon_batmon_disable();
#endif

}

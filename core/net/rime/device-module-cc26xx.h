/*
 * device-module-cc26xx.h
 *
 *  Created on: Jun 10, 2016
 *      Author: user
 */

#ifndef DEVICE_MODULE_CC26XX_H_
#define DEVICE_MODULE_CC26XX_H_
#include "contiki.h"
#include "net/rime/rime.h"
#include "net/mac/nullrdc.h"



#define DEBUG_ENABLE 1

#define PMS_SCD_NO_REMOTE 0
#define PMS_SCD_REMOTE_TV	0
#define PMS_SCD_REMOTE_AIR	0
#define COMMAND_TYPE	101
#define DATA_REQUEST	106
#define DATA_RESPONSE	107


#define PMS_CMD_ON 1
#define PMS_CMD_OFF 2

enum
{
	HD_ROOM_STATUS,
	RM_COMMAND_ON_REQ,
	RM_COMMAND_OFF_REQ,
	PMS_COMMAND_IR_ON_REQ,
	PMS_COMMAND_IR_OFF_TV_REQ,
	PMS_COMMAND_IR_OFF_AIR_REQ,

	PMS_COMMAND_IR_ON_TV_REQ,
	PMS_COMMAND_IR_ON_AIR_REQ,
	RM_ACK_ON,
	RM_ACK_OFF,
	PMS_ADV_TIME_SYN,
	RM_SENDING_IR,
	MESSAGE_UNDEFINED
};


struct header_str
{
	uint8_t frameType;
};


#if HD_SCD || RM_SCD || PMS_SCD || TEST_CODE
struct hd_message_str
{
	uint8_t frameType;
	uint8_t nodeId;
	uint8_t roomStatus;
};
struct pms_message_adv_time_syn{
	uint8_t frameType;
    uint8_t nodeId;
    uint32_t startSynTime;
};

struct ctimer periodTimer_1;
struct ctimer periodTimer;
struct ctimer turnOffCPUAndRadioTimer;
struct ctimer periodTimer_2;

struct ctimer periodResetSyncTimer;
struct ctimer periodSyncTimer;

struct ctimer periodAdvSyncTimer;

#endif


#if HD_SCD
#define E_STATE		1
#define NE_STATE	2

#define PIR_CTRL_POWER_IOID	IOID_22
#define PIR_CTRL_POWER_PIN (1 << PIR_CTRL_POWER_IOID)

#define MW_CTRL_POWER_IOID	IOID_21
#define MW_CTRL_POWER_PIN (1 << MW_CTRL_POWER_IOID)
#endif


#if RM_SCD

static void ctimer_turn_on_expiration_func (void *ptr);
struct rm_message_str_ir_send_notifi
{
	uint8_t frameType;
	uint8_t nodeId;
};
struct rm_message_str
{
	uint8_t frameType;
	uint8_t nodeId;

};
struct pms_command_message_str
{
	uint8_t frameType;
	uint8_t nodeId;
	//uint8_t flag_off_all ; //1 :all, 0: single
};


struct rtimer radioOnTimer;

uint16_t cycleCount;
#endif


#if SRT_SCD
/*struct srt_message
{
	uint8_t frameType;
	uint8_t nodeId;
};*/
struct pms_message_str
{
	uint8_t frameType;
	uint8_t hdNodeId;
	uint8_t roomStatus;
	uint8_t pmsNodeId;
	uint16_t powerConsumption;
};
#endif


//#if PMS_SCD
#if PMS_SCD || TEST_CODE


struct rm_message_str_ir_send_notifi
{
	uint8_t frameType;
	uint8_t nodeId;
};
struct pms_message_str
{
	uint8_t frameType;
	uint8_t hdNodeId;
	uint8_t roomStatus;
	uint8_t pmsNodeId;
	uint16_t powerConsumption;
};
struct srt_message
{
	uint8_t frameType;
	uint8_t nodeId;
};
struct rm_message_str
{
	uint8_t frameType;
	uint8_t nodeId;

};
struct pms_command_message_str
{
	uint8_t frameType;
	uint8_t nodeId;
	//uint8_t flag_off_all ;
};
static uint8_t pms_room_status;
static uint8_t pms_hd_NodeId;
static uint8_t dutyCycleOn = 0;

uint8_t onCommand;
static uint8_t offCommand;

static uint8_t synSeq;
static uint32_t timeStamp;
struct ctimer check_currren_off_timer;
struct ctimer check_currren_on_timer;

struct ctimer ctimer_PMS_send_command_to_RM;

#endif /* PMS_SCD */

extern struct ctimer rm_ack_timer;
extern uint8_t flag_turn_on_wait_to_ack;
extern struct ctimer ctimer_prevent_ir_from_rm;

void device_module_open();

void device_module_turn_off_Radio();

void device_module_turn_on_Radio();

void device_module_save_power();

void device_module_get_battery_voltage(uint32_t *battery_voltage);


/*
 * Function Prototype
 */
#if HD_SCD
PROCESS_NAME(MW_check_process);
//PROCESS_NAME(FFT_process);
//PROCESS_NAME(ADC_process);
void device_module_HD_send_status_to_PMS_diretly(uint8_t roomStatus);
void device_module_set_period_first(void *ptr);
void device_module_off_CPU_Radio(void *ptr);
void device_module_set_period_general(void*ptr);
//uint8_t device_module_check_MW();
//uint8_t device_module_human_detection_test_PIR (uint16_t PIRDataset[]);

//static void device_module_check_PIR_1();
//uint8_t device_module_human_detection_test_PIR_1 (int32_t *PIRDataset);
//static uint8_t device_module_check_MW_1();
#endif

#if RM_SCD
//static void device_module_RM_send_command_to_PMS(uint8_t commandType);
//static void device_module_RM_send_ACK_to_PMS(uint8_t commandType);
//static void device_module_RM_send_IR_notification();
//static void device_module_set_period_first(void *ptr);
//static void device_module_set_period_general(void*ptr);
//static void device_module_off_CPU_Radio(void *ptr);
void device_mode_RM_send_IR_TV(uint8_t commandType);
#endif

#if PMS_SCD || TEST_CODE
void device_module_PMS_update_Room_Status_From_HD(void *message);
#endif /*PMS_SCD || TEST_CODE*/


#endif /* DEVICE_MODULE_CC26XX_H_ */

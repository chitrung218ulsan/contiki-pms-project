/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A null RDC implementation that uses framer for headers.
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Niclas Finne <nfi@sics.se>
 */

#ifndef NULLRDC_H_
#define NULLRDC_H_

#include "net/mac/rdc.h"

#include "net/linkaddr.h"

#define TEST_CODE 0
#define ANCHOR_SCD 0

#define RM_SCD	0
#define PMS_SCD	0
#define HD_SCD	1
#define SRT_SCD	0

#define PMS_CMD_ON 1
#define PMS_CMD_OFF 2

#define SSMA_RDC_TIMER_SWITCH       0x1     /* bit 0000 0001 is used for ON/OFF*/
#define SSMA_RDC_TIMER_ON           0x1
#define SSMA_RDC_TIMER_OFF          0x0
#define SSMA_RDC_MAX_MAC_TRANSMISSIONS 3
#define SSMA_CTS_TRANSMISSION	10
#define SSMA_ACK_TRANSMISSION	23

#define SSMA_RDC_TIMER_TYPE 0xE     /* bit 0000 1110 is used for Timer type */
#define SSMA_RDC_T_RTS      0x0     /* bit 0000 0000 */
#define SSMA_RDC_T_BACKOFF  0x2     /* bit 0000 0010 */
#define SSMA_RDC_T_XMIT     0x4     /* bit 0000 0100 */
#define SSMA_RDC_T_REMOTE   0x6     /* bit 0000 0110 */
#define SSMA_RDC_T_CANCEL    0x8     /* bit 0000 1000 */
#define SSMA_RDC_T_UNDEFINED 0xE    /* bit 0000 1110 */

// For broadcast part
#define SSMA_RDC_mBSS 330  // 10 ms
#define SSMA_RDC_PDelay 1  // Progagation Delay
#define SSMA_RDC_CW 3 //Contention Window
#define SSMA_RDC_mBSSN 5 // The number of mini slot in one sharable slot

typedef enum
{
    SSMA_RDC_RTS,
    SSMA_RDC_CTS,
    SSMA_RDC_UNICAST,
    SSMA_RDC_BROADCAST
} SsmaRdcFrameType;

typedef enum
{
    SSMA_RDC_S_PASSIVE,
    SSMA_RDC_S_RTS,
    SSMA_RDC_S_BACKOFF,
    SSMA_RDC_S_REMOTE,
    SSMA_RDC_S_XMIT,
    SSMA_RDC_S_YIELD,
    SSMA_RDC_S_IN_XMITING_RTS,
    SSMA_RDC_S_IN_XMITING_CTS,
    SSMA_RDC_S_IN_XMITING_UNICAST,
    SSMA_RDC_S_IN_XMITING_BROADCAST
} SsmaRdcStateType;

typedef struct _ssma_rdc_timer
{
	uint8_t   isRunning;
    uint8_t   seq;
    uint8_t   flag;
    //struct rtimer back_off_timer;
    struct ctimer back_off_ctimer;
} SsmaRdcTimer;

struct struct_ssma_rdc_str
{
	uint8_t state;
	struct rdc_buf_list *curbuf;
	SsmaRdcTimer timer;
	uint8_t    data_size;
	linkaddr_t currentNextHopAddress;

	uint8_t rts_count;
	uint8_t rts_call;

	uint8_t isSendingPeriod;
	uint8_t isSendData;

	//For statistic
	uint8_t number_rts_sent;
	uint8_t number_rts_receive;
	uint8_t number_rts_overheard;
	uint8_t  number_rts_fail;

	uint8_t number_cts_sent;
	uint8_t number_cts_receive;
	uint8_t number_cts_overheard;
	uint8_t  number_cts_fail;

	uint8_t  number_data_sent;
	uint8_t  number_data_fail;

	uint8_t number_call_one;
	uint8_t number_call_passive;
};

struct ssma_qbuf_metadata {
  mac_callback_t sent;
  void *cptr;
  uint8_t max_transmissions;
};

struct ssma_rdc_hdr {
    uint8_t frameType;  /* RTS, CTS */
    uint8_t tickDelay;
    linkaddr_t srcAddr;
    linkaddr_t destAddr;
};

struct ssmab_command
{
	uint8_t seq;
	int8_t src;
	uint8_t depth;
	uint8_t frameType;

	uint8_t nodeId;

	uint8_t message[20];
	// For time synchronization
	unsigned long global_time;
};
#if HD_SCD
struct hd_message
{
	uint8_t frameType;
	uint8_t nodeId;
	uint8_t roomStatus;
};
extern uint8_t nullrdc_hd_try_nums;
uint8_t nullRdc_HD_send_room_status_to_PMS(void* message,uint8_t size);

#endif

#if RM_SCD
struct rm_message
{
	uint8_t frameType;
	uint8_t nodeId;

};
extern uint16_t countRM;
#endif


#if SRT_SCD
struct srt_message
{
	uint8_t frameType;
	uint8_t nodeId;
};
struct pms_message
{
	uint8_t frameType;
	uint8_t hdNodeId;
	uint8_t roomStatus;
	uint8_t pmsNodeId;
	uint16_t powerConsumption;
};
#endif


#if PMS_SCD || TEST_CODE
struct pms_message
{
	uint8_t frameType;
	uint8_t hdNodeId;
	uint8_t roomStatus;
	uint8_t pmsNodeId;
	uint16_t powerConsumption;
};
static uint8_t room_status;
static uint8_t hd_NodeId;

static void nullRdc_pms_send_data();
uint8_t nullRdc_PMS_send_command_to_RM(void* message,uint8_t size);
#endif

uint8_t isReceiveBroadcast;
uint8_t isSendingCommandPeriod;
uint8_t isReceivingCommandPeriod;
uint16_t cmdCount;
uint16_t totalReceiveCount;
uint16_t sendCount;
uint16_t num_superframe;
uint8_t indexmBSS;

uint8_t numTries;
uint8_t orphan_count;

uint32_t nullrdc_rm_cycle;
extern uint8_t nullrdc_pms_try_nums;
uint32_t nullrdc_pms_count_sent;

//
#define HD_COMMAND	102
#define RM_COMMAND	103
#define RM_COMMAND_ACK	104
#define HD_COMMAND_ACK	105
#define DATA_REQUEST	106
#define DATA_RESPONSE	107




uint8_t srt_hd_count;
//uint8_t srt_hd_type[3];
uint8_t srt_hd_nodeId[3];

uint8_t srt_hd_roomStatus[3];

uint8_t srt_pms_count;
//uint8_t srt_hd_type[3];
uint8_t srt_pms_nodeId[3];

uint16_t srt_pms_powerConsumption[3];


// For broadcast part
#define NULL_DATA_TYPE 5
#define COMMAND_TYPE 101



#define SRT_COMMAND_COUNT_MAX 5
uint8_t ssmab_index;
uint8_t nodeId;
// For command broadcast part
uint8_t srt_command_count;
uint8_t srt_cmd_type[SRT_COMMAND_COUNT_MAX];
uint8_t srt_cmd_nodeId[SRT_COMMAND_COUNT_MAX];
uint8_t receiveControlMsg[100];
uint8_t commandPktSize;

/*
static void SsmaRdcPassive();
static void SsmaRdcSetState(int state);
static void SsmaRdcSendRts();
static void SsmaRdcSendCts(linkaddr_t destAddr, uint8_t tickDelay);
static void SsmaRdcLayer(void *ptr);
static void SsmaRdcCancelTimer( uint8_t timerType);
static void SsmaRdcSetTimer( uint8_t timerType, uint8_t timerValue);
void RdcEndSendingSlot();

void initPeriod();
void initCyclePeriod();
void initCyclePeriodSink();

void CancelAllByAck();
void free_current_packet();

*/
void nullRdc_send_control(uint8_t command[], uint8_t size);
void nullRdc_relay_control();



 //static void SsmaRdcRemote(struct ssma_rdc_hdr controlHeader, linkaddr_t fromAddr, linkaddr_t toAddr);
 //void nullRdc_send_control(uint8_t command[], uint8_t size);
 //static void nullRdc_pms_send_data();
 void nullRdc_motes_send_command(void* message,uint8_t size);
 //void nullRdc_send_data_request_to_pms();
 //void nullRdc_PMS_send_command_to_RM(void* message,uint8_t size);




extern const struct rdc_driver nullrdc_driver;

#endif /* NULLRDC_H_ */

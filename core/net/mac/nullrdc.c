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

#include "net/mac/mac-sequence.h"
#include "net/mac/nullrdc.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "net/rime/rimestats.h"
#include "lib/random.h"
#include <string.h>
#include <stdio.h>
#include "net/rime/device-module-cc26xx.h"


#if CONTIKI_TARGET_COOJA
#include "lib/simEnvChange.h"
#include "sys/cooja_mt.h"
#endif /* CONTIKI_TARGET_COOJA */

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef NULLRDC_CONF_ADDRESS_FILTER
#define NULLRDC_ADDRESS_FILTER NULLRDC_CONF_ADDRESS_FILTER
#else
#define NULLRDC_ADDRESS_FILTER 0
#endif /* NULLRDC_CONF_ADDRESS_FILTER */

#ifndef NULLRDC_802154_AUTOACK
#ifdef NULLRDC_CONF_802154_AUTOACK
#define NULLRDC_802154_AUTOACK NULLRDC_CONF_802154_AUTOACK
#else
#define NULLRDC_802154_AUTOACK 0
#endif /* NULLRDC_CONF_802154_AUTOACK */
#endif /* NULLRDC_802154_AUTOACK */

#ifndef NULLRDC_802154_AUTOACK_HW
#ifdef NULLRDC_CONF_802154_AUTOACK_HW
#define NULLRDC_802154_AUTOACK_HW NULLRDC_CONF_802154_AUTOACK_HW
#else
#define NULLRDC_802154_AUTOACK_HW 0
#endif /* NULLRDC_CONF_802154_AUTOACK_HW */
#endif /* NULLRDC_802154_AUTOACK_HW */

#if NULLRDC_802154_AUTOACK
#include "sys/rtimer.h"
#include "dev/watchdog.h"

#ifdef NULLRDC_CONF_ACK_WAIT_TIME
#define ACK_WAIT_TIME NULLRDC_CONF_ACK_WAIT_TIME
#else /* NULLRDC_CONF_ACK_WAIT_TIME */
#define ACK_WAIT_TIME                      RTIMER_SECOND / 2500
#endif /* NULLRDC_CONF_ACK_WAIT_TIME */
#ifdef NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#define AFTER_ACK_DETECTED_WAIT_TIME NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME
#else /* NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#define AFTER_ACK_DETECTED_WAIT_TIME       RTIMER_SECOND / 1500
#endif /* NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME */
#endif /* NULLRDC_802154_AUTOACK */

#ifdef NULLRDC_CONF_SEND_802154_ACK
#define NULLRDC_SEND_802154_ACK NULLRDC_CONF_SEND_802154_ACK
#else /* NULLRDC_CONF_SEND_802154_ACK */
#define NULLRDC_SEND_802154_ACK 0
#endif /* NULLRDC_CONF_SEND_802154_ACK */

#if NULLRDC_SEND_802154_ACK
#include "net/mac/frame802154.h"
#endif /* NULLRDC_SEND_802154_ACK */

#define ACK_LEN 3


#if RM_SCD || TEST_CODE
uint16_t countRM = 0;
#endif

#if PMS_SCD || TEST_CODE
uint8_t nullrdc_pms_try_nums = 0;
#endif

#if HD_SCD
uint8_t nullrdc_hd_try_nums = 0;
#endif

/*---------------------------------------------------------------------------*/
static int
send_one_packet(mac_callback_t sent, void *ptr)
{
  int ret;
  int last_sent_ok = 0;

  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
#endif /* NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW */

  if(NETSTACK_FRAMER.create() < 0) {
    /* Failed to allocate space for headers */
    PRINTF("nullrdc: send failed, too large header\n");
    ret = MAC_TX_ERR_FATAL;
  } else {
#if NULLRDC_802154_AUTOACK
    int is_broadcast;
    uint8_t dsn;
    dsn = ((uint8_t *)packetbuf_hdrptr())[2] & 0xff;

    NETSTACK_RADIO.prepare(packetbuf_hdrptr(), packetbuf_totlen());

    is_broadcast = packetbuf_holds_broadcast();

    if(NETSTACK_RADIO.receiving_packet() ||
       (!is_broadcast && NETSTACK_RADIO.pending_packet())) {

      /* Currently receiving a packet over air or the radio has
         already received a packet that needs to be read before
         sending with auto ack. */
      ret = MAC_TX_COLLISION;
    } else {
      if(!is_broadcast) {
        RIMESTATS_ADD(reliabletx);
      }

      switch(NETSTACK_RADIO.transmit(packetbuf_totlen())) {
      case RADIO_TX_OK:
        if(is_broadcast) {
          ret = MAC_TX_OK;
        } else {
          rtimer_clock_t wt;

          /* Check for ack */
          wt = RTIMER_NOW();
          watchdog_periodic();
          while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + ACK_WAIT_TIME)) {
#if CONTIKI_TARGET_COOJA
            simProcessRunValue = 1;
            cooja_mt_yield();
#endif /* CONTIKI_TARGET_COOJA */
          }

          ret = MAC_TX_NOACK;
          if(NETSTACK_RADIO.receiving_packet() ||
             NETSTACK_RADIO.pending_packet() ||
             NETSTACK_RADIO.channel_clear() == 0) {
            int len;
            uint8_t ackbuf[ACK_LEN];

            if(AFTER_ACK_DETECTED_WAIT_TIME > 0) {
              wt = RTIMER_NOW();
              watchdog_periodic();
              while(RTIMER_CLOCK_LT(RTIMER_NOW(),
                                    wt + AFTER_ACK_DETECTED_WAIT_TIME)) {
      #if CONTIKI_TARGET_COOJA
                  simProcessRunValue = 1;
                  cooja_mt_yield();
      #endif /* CONTIKI_TARGET_COOJA */
              }
            }

            if(NETSTACK_RADIO.pending_packet()) {
              len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
              if(len == ACK_LEN && ackbuf[2] == dsn) {
                /* Ack received */
                RIMESTATS_ADD(ackrx);
                ret = MAC_TX_OK;
              } else {
                /* Not an ack or ack not for us: collision */
                ret = MAC_TX_COLLISION;
              }
            }
          } else {
	    PRINTF("nullrdc tx noack\n");
	  }
        }
        break;
      case RADIO_TX_COLLISION:
        ret = MAC_TX_COLLISION;
        break;
      default:
        ret = MAC_TX_ERR;
        break;
      }
    }

#else /* ! NULLRDC_802154_AUTOACK */

    switch(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen())) {
    case RADIO_TX_OK:
      ret = MAC_TX_OK;
      break;
    case RADIO_TX_COLLISION:
      ret = MAC_TX_COLLISION;
      break;
    case RADIO_TX_NOACK:
      ret = MAC_TX_NOACK;
      break;
    default:
      ret = MAC_TX_ERR;
      break;
    }

#endif /* ! NULLRDC_802154_AUTOACK */
  }
  if(ret == MAC_TX_OK) {
    last_sent_ok = 1;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);
  return last_sent_ok;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  send_one_packet(sent, ptr);
}
/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  while(buf_list != NULL) {
    /* We backup the next pointer, as it may be nullified by
     * mac_call_sent_callback() */
    struct rdc_buf_list *next = buf_list->next;
    int last_sent_ok;

    queuebuf_to_packetbuf(buf_list->buf);
    last_sent_ok = send_one_packet(sent, ptr);

    /* If packet transmission was not successful, we should back off and let
     * upper layers retransmit, rather than potentially sending out-of-order
     * packet fragments. */
    if(!last_sent_ok) {
      return;
    }
    buf_list = next;
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{

	uint8_t first_byte = *((uint8_t *)packetbuf_dataptr()); // Get first byte value of packet
	int original_datalen = packetbuf_datalen();
	//printf("Nullrdc: First byte receives %u \n", first_byte);

#if NULLRDC_SEND_802154_ACK
  int original_datalen;
  uint8_t *original_dataptr;

  original_datalen = packetbuf_datalen();
  original_dataptr = packetbuf_dataptr();
#endif

#if NULLRDC_802154_AUTOACK
  if(packetbuf_datalen() == ACK_LEN) {
    /* Ignore ack packets */
    PRINTF("nullrdc: ignored ack\n"); 
  } else
#endif /* NULLRDC_802154_AUTOACK */
	  if(first_byte != DATA_RESPONSE && original_datalen == (sizeof(struct ssma_rdc_hdr)))
	  {
		  return;
	  }
	  else if(first_byte == COMMAND_TYPE && original_datalen > 20)
	  {
		  return;
	  }
#if PMS_SCD
	  else if (first_byte == HD_COMMAND || first_byte == RM_COMMAND)
	  {
		  return;
	  }
#endif

#if PMS_SCD || TEST_CODE
	  else if(first_byte == DATA_REQUEST)
	  {
		ssmab_index = 0;
		// send roomStatus and power consumption to sensor router
		printf("nullRdc: PMS receives data request from Anchor node \n");
		nullRdc_pms_send_data();
		return;
	  }
	  else if (first_byte == HD_ROOM_STATUS)
	  {
		 printf("nullRdc: Pms receives HD_ROOM_STATUS from HD node \n");
		// sending ACK
		uint8_t ackdata[2] = {0, 0};
		ackdata[0] = HD_COMMAND_ACK;
		ackdata[1] = linkaddr_node_addr.u8[0];
		NETSTACK_RADIO.send(ackdata, 2);
		//////////////////////////////////////////////////////////////
		struct hd_message_str messagePtr;
		packetbuf_copyto(&messagePtr);

		// Update Room Status
		device_module_PMS_update_Room_Status_From_HD(&messagePtr);
		return;
	  }
#endif /*PMS_SCD || TEST_CODE*/

#if RM_SCD

	  else if(first_byte == RM_COMMAND_ACK)
	  {
		//if node is RM_SCD
		// call function to send IR Communication
		PRINTF("nullRdc: RM receives ACK from pms node to turn on device \n");
		//SetTemp25(Hauzen);
		return;
	  }
	  else if(first_byte == PMS_COMMAND_IR_OFF_TV_REQ || first_byte == PMS_COMMAND_IR_ON_TV_REQ)
	  {
		  uint8_t ackdata[2] = {0, 0};
		  ackdata[0] = RM_COMMAND_ACK;
		  ackdata[1] = linkaddr_node_addr.u8[0];
		  NETSTACK_RADIO.send(ackdata, 2);
		/*if(cycleCount % 300 != 0)
		{
			//TurnOFFCC2420();

		}*/
		countRM++;

		PRINTF("nullRdc RM receive command %u \n", countRM);

		device_mode_RM_send_IR_TV(PMS_COMMAND_IR_ON_TV_REQ);



		return;
	  }
#endif /*RM_SCD*/

#if ANCHOR_SCD || TEST_CODE
	  	else if(first_byte == DATA_RESPONSE)
	  	{
	  		// save data
	  		struct pms_message pmsPtr;
	  		packetbuf_copyto(&pmsPtr);
	  		if(srt_hd_count >=3 || srt_pms_count>=3)
	  		{
	  			PRINTF("nullRdc: srt_hd_count >=3 \n");
				PRINTF("nullRdc: Anchor node receives data response, hdnodeId: %d, roomStatus: %d, pmsNodeId: %d, pmsPower: %d \n",
						pmsPtr.hdNodeId,pmsPtr.roomStatus,pmsPtr.pmsNodeId,pmsPtr.powerConsumption	);
	  			return;
	  		}
	  		//srt_hd_type[srt_hd_count] = HD_DATA_TYPE;
	  		if(srt_hd_count < 1)
	  		{
				srt_hd_nodeId[srt_hd_count] = pmsPtr.hdNodeId;
				srt_hd_roomStatus[srt_hd_count] = pmsPtr.roomStatus;
				srt_hd_count++;
	  		}
	  		if(linkaddr_node_addr.u8[0] == 10 && (pmsPtr.pmsNodeId == 35 || pmsPtr.pmsNodeId == 27 || pmsPtr.pmsNodeId == 4))
			{
	  			srt_pms_nodeId[srt_pms_count] = pmsPtr.pmsNodeId;
				srt_pms_powerConsumption[srt_pms_count] = pmsPtr.powerConsumption;
				srt_pms_count++;
			}

	  		else if(linkaddr_node_addr.u8[0] == 19 && (pmsPtr.pmsNodeId == 25 || pmsPtr.pmsNodeId == 14 || pmsPtr.pmsNodeId == 8))
	  		{
	  			srt_pms_nodeId[srt_pms_count] = pmsPtr.pmsNodeId;
				srt_pms_powerConsumption[srt_pms_count] = pmsPtr.powerConsumption;
				srt_pms_count++;
	  		}
	  		//srt_pms_type[srt_pms_count] = PMS_DATA_TYPE;


			PRINTF("nullRdc: Anchor node receives data response, hdnodeId: %d, roomStatus: %d, pmsNodeId: %d, pmsPower: %d \n",
					pmsPtr.hdNodeId,pmsPtr.roomStatus,pmsPtr.pmsNodeId,pmsPtr.powerConsumption	);
	  		return;
	  	}
#endif


  if(NETSTACK_FRAMER.parse() < 0) {
    PRINTF("nullrdc: failed to parse %u\n", packetbuf_datalen());
#if NULLRDC_ADDRESS_FILTER
  }
  else if(!linkaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
                                         &linkaddr_node_addr) &&
            !packetbuf_holds_broadcast()) {
    PRINTF("nullrdc: not for us\n");
#endif /* NULLRDC_ADDRESS_FILTER */
  }
  else
  {
    int duplicate = 0;

#if NULLRDC_802154_AUTOACK || NULLRDC_802154_AUTOACK_HW
#if RDC_WITH_DUPLICATE_DETECTION
    /* Check for duplicate packet. */
    duplicate = mac_sequence_is_duplicate();
    if(duplicate) {
      /* Drop the packet. */
      PRINTF("nullrdc: drop duplicate link layer packet %u\n",
             packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO));
    } else {
      mac_sequence_register_seqno();
    }
#endif /* RDC_WITH_DUPLICATE_DETECTION */
#endif /* NULLRDC_802154_AUTOACK */
 
#if NULLRDC_SEND_802154_ACK
    {
      frame802154_t info154;
      frame802154_parse(original_dataptr, original_datalen, &info154);
      if(info154.fcf.frame_type == FRAME802154_DATAFRAME &&
         info154.fcf.ack_required != 0 &&
         linkaddr_cmp((linkaddr_t *)&info154.dest_addr,
                      &linkaddr_node_addr)) {
        uint8_t ackdata[ACK_LEN] = {0, 0, 0};

        ackdata[0] = FRAME802154_ACKFRAME;
        ackdata[1] = 0;
        ackdata[2] = info154.seq;
        NETSTACK_RADIO.send(ackdata, ACK_LEN);
      }
    }
#endif /* NULLRDC_SEND_ACK */
    if(!duplicate)
    {
      NETSTACK_MAC.input();
    }
  }
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
off(int keep_radio_on)
{
  if(keep_radio_on) {
    return NETSTACK_RADIO.on();
  } else {
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  on();
}

/*----------------------------------------------------------------------------*/
#if HD_SCD

uint8_t nullRdc_HD_send_room_status_to_PMS(void* message,uint8_t size)
{

	uint8_t ret;

	if(nullrdc_hd_try_nums >=3 )
	{
		nullrdc_hd_try_nums = 0;
		//device_module_off_CPU_Radio(NULL);
		ret = RADIO_TX_NOACK;
		return ret;
	}

	nullrdc_hd_try_nums ++;


	switch(NETSTACK_RADIO.send(message, size))
	{

		case RADIO_TX_OK:
		{

			printf("Mote send message OK \n");

			// waiting here to receive ACK
			rtimer_clock_t wt = RTIMER_NOW();
			while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + 100)) { }

			uint8_t ackbuf[2] = {0,0};
			uint8_t len = NETSTACK_RADIO.read(ackbuf, 2);
			if(len == 2 && ackbuf[0] == HD_COMMAND_ACK)
			{
				printf("HD node %d receives ACK from PMS at: %d \n",linkaddr_node_addr.u8[0],RTIMER_NOW());
			 //NETSTACK_RADIO.off();
			 //device_module_off_CPU_Radio(NULL);
				ret = RADIO_TX_OK;
			}
			else {
				printf("HD node does not receive ACK and retry: %d \n",nullrdc_hd_try_nums);

				nullRdc_HD_send_room_status_to_PMS(message,size);

			}

			break;
		}
		case RADIO_TX_ERR:
		{

			printf("Mote send message fail RADIO_TX_ERR \n");
			ret = RADIO_TX_ERR;
		  break;
		}
		case RADIO_TX_COLLISION:
		{
			rtimer_clock_t t0;
			t0 = RTIMER_NOW() + (random_rand()%5);
			while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
			nullRdc_HD_send_room_status_to_PMS(message, size);
		   break;
		}
		case RADIO_TX_NOACK:
		{

			printf("Mote send message fail RADIO_TX_NOACK \n");
			ret = RADIO_TX_NOACK;
		  break;
		}
		default:
		{

			printf("Mote send message fail \n");
			ret = RADIO_TX_NOACK;
		  break;
		}
	}
	return ret;

}
#endif /*HD_SCD*/

#if PMS_SCD || TEST_CODE


uint8_t nullRdc_PMS_send_command_to_RM(void* message,uint8_t size)
{

	uint8_t ret = RADIO_TX_NOACK;

	if(nullrdc_pms_try_nums >=3)
	{
		ret = RADIO_TX_NOACK;
		return ret;
	}
	nullrdc_pms_try_nums++;

	rtimer_clock_t t0 = RTIMER_NOW()+ 10 + (random_rand()%10);

	while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }

	switch(NETSTACK_RADIO.send(message, size))
	{
		case RADIO_TX_OK:
		{
			nullrdc_pms_count_sent++;

			PRINTF("NullRdc: PMS sends command to RM OK at %u \n",get_global_time());

			// waiting here to receive ACK
			rtimer_clock_t wt = RTIMER_NOW();
			while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + 100)) { }

			uint8_t ackbuf[2] = {0,0};
			uint8_t len = NETSTACK_RADIO.read(ackbuf, 2);

			if(len == 2 && ackbuf[0] == RM_COMMAND_ACK)
			{
				printf("PMS node %d receives ACK from RM at: %d \n",linkaddr_node_addr.u8[0],RTIMER_NOW());
			 //NETSTACK_RADIO.off();
			 //device_module_off_CPU_Radio(NULL);
				ret = RADIO_TX_OK;
			}
			else {
				printf("HD node does not receive ACK and retry: %d \n",nullrdc_pms_try_nums);

				t0 = RTIMER_NOW()+ 10 + (random_rand()%10);
				while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }

				nullRdc_PMS_send_command_to_RM(message,size);

			}
		  break;
		}
		case RADIO_TX_ERR:
		{
			PRINTF("NullRdc: PMS sends command to RM ERR at %u \n",get_global_time());
			ret = RADIO_TX_ERR;
		  break;
		}
		case RADIO_TX_COLLISION:
		{
			PRINTF("NullRdc: PMS sends command to RM COLLISION at %u \n",get_global_time());
			nullRdc_PMS_send_command_to_RM(message, size);
			/*
			rtimer_clock_t t0;
			t0 = RTIMER_NOW() + (random_rand()%10);
			while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
			NETSTACK_RADIO.send(message, size);
			*/
		   break;
		}
		case RADIO_TX_NOACK:
		{
			PRINTF("Mote send message fail RADIO_TX_NOACK \n");
			ret = RADIO_TX_NOACK;
		  break;
		}
		default:
		{
			ret = RADIO_TX_NOACK;
			PRINTF("Mote send message fail \n");
		  break;
		}
	 }
	return ret;
}

static void nullRdc_pms_send_data()
{
	struct pms_message dataStr;
	dataStr.frameType = DATA_RESPONSE;
	dataStr.hdNodeId = hd_NodeId;
	dataStr.roomStatus = room_status;
	dataStr.pmsNodeId = linkaddr_node_addr.u8[0];

	//dataStr.powerConsumption= GetPower(); // call function from PMS device to get power

	PRINTF("nullRdc: Pms node sends data to anchor node\n");

	nullRdc_motes_send_command(&dataStr,sizeof(struct pms_message));
}
#endif

#if RM_SCD || TEST_CODE

#endif

#if ANCHOR_SCD
void nullRdc_send_data_request_to_pms()
{
	struct srt_message messagePkt;
	messagePkt.frameType = DATA_REQUEST;
	messagePkt.nodeId = linkaddr_node_addr.u8[0];
	PRINTF("nullRdc: Anchor node sends request to pms node\n");
	nullRdc_motes_send_command(&messagePkt,sizeof(struct srt_message));
}
#endif /*ANCHOR_SCD */

/*****************************************************************************************************/
//Common function used for all devices
void nullRdc_motes_send_command(void* message,uint8_t size)
{
	rtimer_clock_t t0;
	t0 = RTIMER_NOW() + 100*(random_rand()%10);
	while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }

	switch(NETSTACK_RADIO.send(message, size))
	{

		case RADIO_TX_OK:
		{
			PRINTF("Mote send message OK \n");
		  break;
		}
		case RADIO_TX_ERR:
		{
			PRINTF("Mote send message fail RADIO_TX_ERR \n");
		  break;
		}
		case RADIO_TX_COLLISION:
		{
			rtimer_clock_t t0;
			t0 = RTIMER_NOW() + 20*(random_rand()%10);
			while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
			nullRdc_motes_send_command(message, size);
		   break;
		}
		case RADIO_TX_NOACK:
		{
			PRINTF("Mote send message fail RADIO_TX_NOACK \n");
		  break;
		}
		default:
		{
			PRINTF("Mote send message fail \n");
		  break;
		}
	 }
}
/*******************************************************************************************/
/*
 *nullRdc_send_control
 */
void nullRdc_send_control(uint8_t command[], uint8_t size)
{
	rtimer_clock_t t0;


	t0 = RTIMER_NOW() +20+ random_rand()%30;
	while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
	switch(NETSTACK_RADIO.send(command, size))
	{
		case RADIO_TX_OK:
		{
			PRINTF("Send Control OK \n");
		  break;
		}
		case RADIO_TX_ERR:
		{
			PRINTF("Send Control fail RADIO_TX_ERR \n");
		  break;
		}
		case RADIO_TX_COLLISION:
		{
			//PRINTF("Send Control fail RADIO_TX_COLLISION \n");
			t0 = RTIMER_NOW() + random_rand()%10;
			while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
			nullRdc_send_control(command, size);
		   break;
		}
		case RADIO_TX_NOACK:
		{
			PRINTF("Send Control fail RADIO_TX_NOACK \n");
			break;
		}
		default:
		{
			PRINTF("Send Control fail \n");
		  break;
		}
	 }
}
/*
 *
 */
void nullRdc_relay_control(){
	rtimer_clock_t t0;
	if(isSendingCommandPeriod == 0 || isReceiveBroadcast == 0)
	{
		PRINTF( "Can not relay control message %u, %u \n", isSendingCommandPeriod, isReceiveBroadcast);
		return;
	}
	// Wait to the working slot
	//uint8_t i = random_rand()%SSMA_RDC_CW;
	//t_end = RTIMER_NOW() + (rtimer_clock_t)SSMA_RDC_CW*SSMA_RDC_mBSS;

	//if(i == 0)
	{
	    // wait a random delay that consider the propagation delay
		t0 = RTIMER_NOW() + 50 + random_rand()%50;
		while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
	}

	/*
	else
	{
		t0 = RTIMER_NOW() + (rtimer_clock_t)i*SSMA_RDC_mBSS;
		while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
	    // wait a random delay that consider the propagation delay
		t0 = RTIMER_NOW() + random_rand()%20;
		while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
	}
	*/
/*
	if(receiveControlMsg.frameType == 102)
	{
		time_attach = 1;
	}
	else
	{
		time_attach = 0;
	}
*/
	switch(NETSTACK_RADIO.send(&receiveControlMsg, commandPktSize)){
		case RADIO_TX_OK:
		{
			PRINTF("Relay Control OK \n");
			NETSTACK_RADIO.off();
			sendCount++;
		  break;
		}
		case RADIO_TX_ERR:
		{
			//PRINTF("Send Control fail RADIO_TX_ERR \n");
		  break;
		}
		case RADIO_TX_COLLISION:
		{
			//PRINTF("Relay fail at %d \n", i);
			t0 = RTIMER_NOW() + random_rand()%10;
			while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
			nullRdc_relay_control();
		   break;
		}
		default:
		{
			PRINTF("Send Control fail \n");
		  break;
		}
	 }
}

/*---------------------------------------------------------------------------*/
const struct rdc_driver nullrdc_driver = {
  "nullrdc",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/

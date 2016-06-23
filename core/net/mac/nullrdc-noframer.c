/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *         A MAC protocol that does not do anything.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "net/mac/nullrdc-noframer.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include <string.h>

#include "deluge.h"



#define DEBUG	1
#if DEBUG
#include <stdio.h>
#define PRINTF(...)	printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

uint8_t nullrdc_noframer_sent_count;
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  int ret;
  if(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen()) == RADIO_TX_OK) {
    ret = MAC_TX_OK;
  } else {
    ret =  MAC_TX_ERR;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);
}
/*---------------------------------------------------------------------------*/
static void
send_list(mac_callback_t sent, void *ptr, struct rdc_buf_list *buf_list)
{
  if(buf_list != NULL) {
    queuebuf_to_packetbuf(buf_list->buf);
    send_packet(sent, ptr);
  }
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
  PRINTF("NULLRDC_NOFRAMMER, packet input \n");

  char *msg;
  int len;
  struct deluge_msg_profile *profile;
  struct deluge_msg_packet *packet;
  struct deluge_msg_request *request;

  msg = packetbuf_dataptr();
  len = packetbuf_datalen();
  if(len < 1)
    return;


  switch(msg[0])
  {
	  case DELUGE_CMD_SUMMARY:
	  {
		  PRINTF("In packet_input: DELUGE_CMD_SUMMARY \n");
		  break;
	  }
	  case DELUGE_CMD_REQUEST:
	  {
		  PRINTF("In packet_input: DELUGE_CMD_REQUEST \n");
		  if(len >= sizeof(struct deluge_msg_request))
		  {
			  request = (struct deluge_msg_request *)msg;
			  if(linkaddr_cmp(&request->receiver,&linkaddr_node_addr))
			  {
				   handle_request_mc_deluge((struct deluge_msg_request *)msg);
			  }
			  //handle_request((struct deluge_msg_request *)msg);

		  }

		  break;
	  }
	  case DELUGE_CMD_PACKET:
	  {
		  PRINTF("In packet_input: DELUGE_CMD_PACKET \n");
		  packet = (struct deluge_msg_packet *)msg;
		  if(len >= sizeof(struct deluge_msg_packet))
		  {
			    linkaddr_copy(&current_object.parent_addr, &packet->sender);
		      	handle_packet_mc_deluge((struct deluge_msg_packet *)msg, &packet->sender);
		  }

		  break;
	  }
	  case DELUGE_CMD_PROFILE:
	  {
		  PRINTF("In packet_input: DELUGE_CMD_PROFILE \n");
		  profile = (struct deluge_msg_profile *)msg;
		  if(len >= sizeof(*profile) &&
			 len >= sizeof(*profile) + profile->npages * profile->version_vector[0])

			//handle_profile((struct deluge_msg_profile *)msg);
			handle_profile_mc_deluge((struct deluge_msg_profile *)msg);
		  break;
	  }
	  default:
		  break;
  }



  NETSTACK_MAC.input();
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




/*---------------------------------------------------------------------------*/

uint8_t nullrdc_noframer_send_packet(void* message,uint8_t size, uint8_t retry)
{
	if(nullrdc_noframer_sent_count >= 3)
	{
		nullrdc_noframer_sent_count = 0;
		return RADIO_TX_COLLISION;
	}
	uint8_t ret = RADIO_TX_ERR;

	nullrdc_noframer_sent_count++;

	switch(NETSTACK_RADIO.send(message, size))
	{
			case RADIO_TX_OK:
			{
				PRINTF("Mote send message OK \n");
				ret = RADIO_TX_OK;
				break;
			}
			case RADIO_TX_ERR:
			{
				PRINTF("Mote send message fail RADIO_TX_ERR \n");
				ret = RADIO_TX_ERR;
				break;
			}
			case RADIO_TX_COLLISION:
			{
				PRINTF("Mote send message fail RADIO_TX_COLLISION \n");

				 /*rtimer_clock_t t0;
				 t0 = RTIMER_NOW() + 2+ (random_rand()%10);
				while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 )) { }*/
				if(retry == 1)
				{
					nullrdc_noframer_send_packet(message,size, 0);
				}

				ret = RADIO_TX_COLLISION;
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
				PRINTF("Mote send message fail \n");
				ret = RADIO_TX_ERR;
			  break;
			}
	}
	return ret;
}
/*---------------------------------------------------------------------------*/
uint8_t nullrdc_noframer_check_channel()
{
	int i;
	NETSTACK_RADIO.off();
	if(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet())
	{
		NETSTACK_RADIO.on();
		return 0;
	}
	for(i = 0; i < 6; ++i)
	{

	  NETSTACK_RADIO.on();
	  rtimer_clock_t t0 = RTIMER_NOW();


	  while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + 5))
	  {
			 if(NETSTACK_RADIO.channel_clear() == 0)
			 {
				  NETSTACK_RADIO.on();
				  return 0;
			 }
	  }

	  NETSTACK_RADIO.off();
	  t0 = RTIMER_NOW();
	  while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + 16)) { }
	}
	NETSTACK_RADIO.on();
	return 1;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver nullrdc_noframer_driver = {
  "nullrdc-noframer",
  init,
  send_packet,
  send_list,
  packet_input,
  on,
  off,
  channel_check_interval,
};
/*---------------------------------------------------------------------------*/

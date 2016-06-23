/*
 * Copyright (c) 2007, Swedish Institute of Computer Science
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
 *	An implementation of the Deluge protocol.
 *      (Hui and Culler: The dynamic behavior of a data 
 *      dissemination protocol for network programming at scale,
 *      ACM SenSys 2004)
 * \author
 * 	Nicolas Tsiftes <nvt@sics.se>
 */
/*
 * MC_Deluge:
 * Author: Trung Ngo
 */
#include "contiki.h"
#include "net/rime/rime.h"
#include "cfs/cfs.h"
#include "loader/elfloader.h"
#include "lib/crc16.h"
#include "lib/random.h"
#include "sys/node-id.h"
#include "deluge.h"

#include "dev/cc2420/cc2420.h"
#include "net/mac/nullrdc-noframer.h"

#if NETSIM
#include "ether.h"
#include <stdio.h>
#endif

#include "dev/leds.h"
#include <stdlib.h>
#include <string.h>

#define DEBUG	1
#if DEBUG
#include <stdio.h>
#define PRINTF(...)	printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Implementation-specific variables. */
static struct broadcast_conn deluge_broadcast;
static struct unicast_conn deluge_uc;
struct deluge_object current_object;
static process_event_t deluge_event;

/* Deluge variables. */
static int deluge_state;
static int old_summary;
static int neighbor_inconsistency;
static unsigned r_interval;
static unsigned recv_adv;
static int broadcast_profile;

/* Deluge timers. */
static struct ctimer rx_timer;
static struct ctimer tx_timer;
static struct ctimer summary_timer;
static struct ctimer profile_timer;

static struct ctimer tx_timer_mc_deluge;
static struct ctimer node_tx_timer_send_page_mc_deluge;
static struct ctimer node_rx_timer_send_page_mc_deluge;
static struct ctimer set_period_first_timer;
static struct ctimer tx_node_resend_page_timer;
static struct rtimer node_send_page_rtimer;

/* Deluge objects will get an ID that defaults to the current value of
   the next_object_id parameter. */
static deluge_object_id_t next_object_id;

/* Rime callbacks. */
static void broadcast_recv(struct broadcast_conn *, const linkaddr_t *);
static void unicast_recv(struct unicast_conn *, const linkaddr_t *);

static const struct broadcast_callbacks broadcast_call = {broadcast_recv, NULL};
static const struct unicast_callbacks unicast_call = {unicast_recv, NULL};

void
handle_profile_mc_deluge(struct deluge_msg_profile *msg);
static void
send_profile_mc_deluge(struct deluge_object *obj);

static void
sink_send_page_mc_deluge(void *arg);

static void
send_page_mc_deluge(struct deluge_object *obj, unsigned pagenum);

void
handle_packet_mc_deluge(struct deluge_msg_packet *msg, linkaddr_t *sender);

static void
send_request_mc_deluge(void *arg);

static void node_send_page_mc_deluge(struct deluge_object *obj, unsigned pagenum);

void
handle_request_mc_deluge(struct deluge_msg_request *msg);

static void node_send_page_mc_deluge_set_timer(void *arg);

static void node_send_page_set_general_timer_mc_deluge(void *arg);

static void set_period_first_mc_deluge(void *arg);

static void node_receive_page_set_first_mc_deluge(void *arg);

static void node_receive_page_set_general_mc_deluge(void *arg);

static void
send_page_by_request_mc_deluge(struct deluge_object *obj, unsigned pagenum);

static void
tx_callback_mc_deluge(void *arg);

static void node_resend_page_mc_deluge(void *arg);

PROCESS_NAME(node_send_page_process);

/* The Deluge process manages the main Deluge timer. */
PROCESS(deluge_process, "Deluge");

static void
transition(int state)
{
  if(state != deluge_state) {
    switch(deluge_state) {
    case DELUGE_STATE_MAINTAIN:
      ctimer_stop(&summary_timer);
      ctimer_stop(&profile_timer);
      break;
    case DELUGE_STATE_RX:
      ctimer_stop(&rx_timer);
      break;
    case DELUGE_STATE_TX:
      ctimer_stop(&tx_timer);
      break;
    }
    deluge_state = state;
  }
}

static int
write_page(struct deluge_object *obj, unsigned pagenum, unsigned char *data)
{
  cfs_offset_t offset;

  offset = pagenum * S_PAGE;

  if(cfs_seek(obj->cfs_fd, offset, CFS_SEEK_SET) != offset) {
    return -1;
  }
  return cfs_write(obj->cfs_fd, (char *)data, S_PAGE);
}

static int
read_page(struct deluge_object *obj, unsigned pagenum, unsigned char *buf)
{
  cfs_offset_t offset;

  offset = pagenum * S_PAGE;

  if(cfs_seek(obj->cfs_fd, offset, CFS_SEEK_SET) != offset) {
    return -1;
  }
  return cfs_read(obj->cfs_fd, (char *)buf, S_PAGE);
}

static void
init_page(struct deluge_object *obj, int pagenum, int have)
{
  struct deluge_page *page;
  unsigned char buf[S_PAGE];

  page = &obj->pages[pagenum];
  page->flags = 0;
  page->last_request = 0;
  page->last_data = 0;

  if(have) {
    page->version = obj->version;
    page->packet_set = ALL_PACKETS;
    page->flags |= PAGE_COMPLETE;
    read_page(obj, pagenum, buf);
    page->crc = crc16_data(buf, S_PAGE, 0);
  } else {
    page->version = obj->version;
    page->packet_set = 0;
  }
}

static cfs_offset_t
file_size(const char *file)
{
  int fd;
  cfs_offset_t size;

  fd = cfs_open(file, CFS_READ);
  if(fd < 0) {
    return (cfs_offset_t)-1;
  }

  size = cfs_seek(fd, 0, CFS_SEEK_END);
  cfs_close(fd);

  return size;
}

static int
init_object(struct deluge_object *obj, char *filename, unsigned version)
{
  static struct deluge_page *page;
  int i;

  obj->cfs_fd = cfs_open(filename, CFS_READ | CFS_WRITE);
  if(obj->cfs_fd < 0) {
    return -1;
  }

  obj->filename = filename;
  obj->object_id = next_object_id++;
  obj->size = file_size(filename);
  obj->version = obj->update_version = version;
  obj->current_rx_page = 0;
  obj->nrequests = 0;
  obj->tx_set = 0;


  PRINTF("Deluge: object size: %d, num of page: %d \n",obj->size,OBJECT_PAGE_COUNT(*obj));
  obj->pages = malloc(OBJECT_PAGE_COUNT(*obj) * sizeof(*obj->pages));
  if(obj->pages == NULL) {
    cfs_close(obj->cfs_fd);
    return -1; 
  }

  for(i = 0; i < OBJECT_PAGE_COUNT(current_object); i++) {
    page = &current_object.pages[i];
    init_page(&current_object, i, 1);
  }

  memset(obj->current_page, 0, sizeof(obj->current_page));

  return 0;
}

static int
highest_available_page(struct deluge_object *obj)
{
  int i;

  for(i = 0; i < OBJECT_PAGE_COUNT(*obj); i++) {
    if(!(obj->pages[i].flags & PAGE_COMPLETE)) {
      break;
    }
  }

  return i;
}

static void
send_request(void *arg)
{
  struct deluge_object *obj;
  struct deluge_msg_request request;

  obj = (struct deluge_object *)arg;

  request.cmd = DELUGE_CMD_REQUEST;
  request.pagenum = obj->current_rx_page;
  request.version = obj->pages[request.pagenum].version;
  request.request_set = ~obj->pages[obj->current_rx_page].packet_set;
  request.object_id = obj->object_id;

  PRINTF("Sending request for page %d, version %u, request_set %u\n", 
	request.pagenum, request.version, request.request_set);
  packetbuf_copyfrom(&request, sizeof(request));
  unicast_send(&deluge_uc, &obj->summary_from);

  /* Deluge R.2 */
  if(++obj->nrequests == CONST_LAMBDA) {
    /* XXX check rate here too. */
    obj->nrequests = 0;
    transition(DELUGE_STATE_MAINTAIN);
  } else {
    ctimer_reset(&rx_timer);
  }
}

static void
advertise_summary(struct deluge_object *obj)
{
  struct deluge_msg_summary summary;

  if(recv_adv >= CONST_K) {
    ctimer_stop(&summary_timer);
    return;
  }

  summary.cmd = DELUGE_CMD_SUMMARY;
  summary.version = obj->update_version;
  summary.highest_available = highest_available_page(obj);
  summary.object_id = obj->object_id;

  PRINTF("Advertising summary for object id %u: version=%u, available=%u\n",
	(unsigned)obj->object_id, summary.version, summary.highest_available);

  packetbuf_copyfrom(&summary, sizeof(summary));
  broadcast_send(&deluge_broadcast);
}

static void
handle_summary(struct deluge_msg_summary *msg, const linkaddr_t *sender)
{
  int highest_available, i;
  clock_time_t oldest_request, oldest_data, now;
  struct deluge_page *page;

  highest_available = highest_available_page(&current_object);

  if(msg->version != current_object.version ||
      msg->highest_available != highest_available) {
    neighbor_inconsistency = 1;
  } else {
    recv_adv++;
  }

  if(msg->version < current_object.version) {
    old_summary = 1;
    broadcast_profile = 1;
  }

  /* Deluge M.5 */
  if(msg->version == current_object.update_version &&
     msg->highest_available > highest_available) {
    if(msg->highest_available > OBJECT_PAGE_COUNT(current_object)) {
      PRINTF("Error: highest available is above object page count!\n");
      return;
    }

    oldest_request = oldest_data = now = clock_time();
    for(i = 0; i < msg->highest_available; i++) {
      page = &current_object.pages[i];
      if(page->last_request < oldest_request) {
	oldest_request = page->last_request;
      }
      if(page->last_request < oldest_data) {
	oldest_data = page->last_data;
      }
    }

    if(((now - oldest_request) / CLOCK_SECOND) <= 2 * r_interval ||
	((now - oldest_data) / CLOCK_SECOND) <= r_interval) {
      return;
    }

    linkaddr_copy(&current_object.summary_from, sender);
    transition(DELUGE_STATE_RX);

    if(ctimer_expired(&rx_timer)) {
      ctimer_set(&rx_timer,
	CONST_OMEGA * ESTIMATED_TX_TIME + ((unsigned)random_rand() % T_R),
	send_request, &current_object);
    }
  }
}

static void
send_page(struct deluge_object *obj, unsigned pagenum)
{
  unsigned char buf[S_PAGE];
  struct deluge_msg_packet pkt;
  unsigned char *cp;

  pkt.cmd = DELUGE_CMD_PACKET;
  pkt.pagenum = pagenum;
  pkt.version = obj->pages[pagenum].version;
  pkt.packetnum = 0;
  pkt.object_id = obj->object_id;
  pkt.crc = 0;

  read_page(obj, pagenum, buf);

  /* Divide the page into packets and send them one at a time. */
  for(cp = buf; cp + S_PKT <= (unsigned char *)&buf[S_PAGE]; cp += S_PKT) {
    if(obj->tx_set & (1 << pkt.packetnum)) {
      pkt.crc = crc16_data(cp, S_PKT, 0);
      memcpy(pkt.payload, cp, S_PKT);
      packetbuf_copyfrom(&pkt, sizeof(pkt));
      broadcast_send(&deluge_broadcast);
    }
    pkt.packetnum++;
  }
  obj->tx_set = 0;
}

static void
tx_callback(void *arg)
{
  struct deluge_object *obj;

  obj = (struct deluge_object *)arg;
  if(obj->current_tx_page >= 0 && obj->tx_set) {
    send_page(obj, obj->current_tx_page);
    /* Deluge T.2. */
    if(obj->tx_set) {
      packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM);
      ctimer_reset(&tx_timer);
    } else {
      packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM_END);
      obj->current_tx_page = -1;
      transition(DELUGE_STATE_MAINTAIN);
    }
  }
}

static void
handle_request(struct deluge_msg_request *msg)
{


  int highest_available;

  if(msg->pagenum >= OBJECT_PAGE_COUNT(current_object)) {
	 PRINTF("In Handle_Request Func: msg->pagenum >= OBJECT_PAGE_COUNT(current_object) \n");
    return;
  }

  if(msg->version != current_object.version) {
    neighbor_inconsistency = 1;
  }

  highest_available = highest_available_page(&current_object);
  PRINTF("In Handle_Request Func: Node %u receives request->>>pagenum:%u,request_set:%u, highest_avaible:%u,msg->version:%u,current_object.version:%u \n",node_id,msg->pagenum,msg->request_set,
		  highest_available,msg->version,current_object.version);
  /* Deluge M.6 */
  if(msg->version == current_object.version && msg->pagenum <= highest_available)

  {
    current_object.pages[msg->pagenum].last_request = clock_time();

    /* Deluge T.1 */
    if(msg->pagenum == current_object.current_tx_page) {
      current_object.tx_set |= msg->request_set;
    } else {
      current_object.current_tx_page = msg->pagenum;
      current_object.tx_set = msg->request_set;
    }

    transition(DELUGE_STATE_TX);
    //ctimer_set(&tx_timer, CLOCK_SECOND, tx_callback, &current_object);
    ctimer_set(&tx_timer, 2, tx_callback, &current_object);
  }
}

static void
handle_packet(struct deluge_msg_packet *msg)
{
  struct deluge_page *page;
  uint16_t crc;
  struct deluge_msg_packet packet;

  memcpy(&packet, msg, sizeof(packet));

  PRINTF("Incoming packet for object id %u, version %u, page %u, packet num %u!\n",
	(unsigned)packet.object_id, (unsigned)packet.version,
	(unsigned)packet.pagenum, (unsigned)packet.packetnum);

  if(packet.pagenum != current_object.current_rx_page) {
    return;
  }

  if(packet.version != current_object.version) {
    neighbor_inconsistency = 1;
  }

  page = &current_object.pages[packet.pagenum];

  if(packet.version == page->version && !(page->flags & PAGE_COMPLETE)) {
    memcpy(&current_object.current_page[S_PKT * packet.packetnum],
	packet.payload, S_PKT);

    crc = crc16_data(packet.payload, S_PKT, 0);
    if(packet.crc != crc) {
      PRINTF("packet crc: %hu, calculated crc: %hu\n", packet.crc, crc);
      return;
    }

    page->last_data = clock_time();
    page->packet_set |= (1 << packet.packetnum);

    if(page->packet_set == ALL_PACKETS)
    {
      /* This is the last packet of the requested page; stop streaming. */
      packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM_END);

      write_page(&current_object, packet.pagenum, current_object.current_page);
      page->version = packet.version;
      page->flags = PAGE_COMPLETE;
      PRINTF("Page %u completed\n", packet.pagenum);

      current_object.current_rx_page++;

      if(packet.pagenum == OBJECT_PAGE_COUNT(current_object) - 1)
      {
	current_object.version = current_object.update_version;
	leds_on(LEDS_RED);
	PRINTF("Update completed for object %u, version %u\n", 
	       (unsigned)current_object.object_id, packet.version);
      }
      else if(current_object.current_rx_page < OBJECT_PAGE_COUNT(current_object))
      {
        if(ctimer_expired(&rx_timer)) {
        	ctimer_set(&rx_timer,
        			CONST_OMEGA * ESTIMATED_TX_TIME + (random_rand() % T_R),
        			send_request, &current_object);
        }
      }
      /* Deluge R.3 */
      transition(DELUGE_STATE_MAINTAIN);
    } else {
      /* More packets to come. Put lower layers in streaming mode. */
      packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM);
    }
  }
}

static void
send_profile(struct deluge_object *obj)
{
  struct deluge_msg_profile *msg;
  unsigned char buf[sizeof(*msg) + OBJECT_PAGE_COUNT(*obj)];
  int i;

  if(broadcast_profile && recv_adv < CONST_K) {
    broadcast_profile = 0;

    msg = (struct deluge_msg_profile *)buf;
    msg->cmd = DELUGE_CMD_PROFILE;
    msg->version = obj->version;
    msg->npages = OBJECT_PAGE_COUNT(*obj);
    msg->object_id = obj->object_id;
    for(i = 0; i < msg->npages; i++) {
      msg->version_vector[i] = obj->pages[i].version;
    }

    packetbuf_copyfrom(buf, sizeof(buf));
    broadcast_send(&deluge_broadcast);
    PRINTF("Node %u sends profile \n",node_id);
  }
}

static void
handle_profile(struct deluge_msg_profile *msg)
{
  int i;
  int npages;
  struct deluge_object *obj;
  char *p;

  obj = &current_object;
  if(msg->version <= current_object.update_version) {
    return;
  }

  PRINTF("Received profile of version %u with a vector of %u pages.\n",
	msg->version, msg->npages);

  leds_off(LEDS_RED);
  current_object.tx_set = 0;

  npages = OBJECT_PAGE_COUNT(*obj);
  obj->size = msg->npages * S_PAGE;

  p = malloc(OBJECT_PAGE_COUNT(*obj) * sizeof(*obj->pages));
  if(p == NULL) {
    PRINTF("Failed to reallocate memory for pages!\n");
    return;
  }

  memcpy(p, obj->pages, npages * sizeof(*obj->pages));
  free(obj->pages);
  obj->pages = (struct deluge_page *)p;

  if(msg->npages < npages) {
    npages = msg->npages;
  }

  for(i = 0; i < npages; i++) {
    if(msg->version_vector[i] > obj->pages[i].version) {
      obj->pages[i].packet_set = 0;
      obj->pages[i].flags &= ~PAGE_COMPLETE;
      obj->pages[i].version = msg->version_vector[i];
    }
  }
  obj->current_rx_page = highest_available_page(obj);
  obj->update_version = msg->version;

  for(; i < msg->npages; i++) {
    init_page(obj, i, 0);
  }

  //obj->current_rx_page = highest_available_page(obj);
  //obj->update_version = msg->version;

  transition(DELUGE_STATE_RX);

  ctimer_set(&rx_timer,
	CONST_OMEGA * ESTIMATED_TX_TIME + ((unsigned)random_rand() % T_R),
	send_request, obj);
}

static void
command_dispatcher(const linkaddr_t *sender)
{
  char *msg;
  int len;
  struct deluge_msg_profile *profile;

  msg = packetbuf_dataptr();  
  len = packetbuf_datalen();
  if(len < 1)
    return;

  switch(msg[0]) {
  case DELUGE_CMD_SUMMARY:
    if(len >= sizeof(struct deluge_msg_summary))
      handle_summary((struct deluge_msg_summary *)msg, sender);
    break;
  case DELUGE_CMD_REQUEST:
    if(len >= sizeof(struct deluge_msg_request))
      //handle_request((struct deluge_msg_request *)msg);
    	//handle_request_mc_deluge((struct deluge_msg_request *)msg);
    break;
  case DELUGE_CMD_PACKET:
    if(len >= sizeof(struct deluge_msg_packet))
      //handle_packet((struct deluge_msg_packet *)msg);
    	linkaddr_copy(&current_object.parent_addr, sender);
    	//handle_packet_mc_deluge((struct deluge_msg_packet *)msg, sender);
    break;
  case DELUGE_CMD_PROFILE:
    profile = (struct deluge_msg_profile *)msg;
    if(len >= sizeof(*profile) &&
       len >= sizeof(*profile) + profile->npages * profile->version_vector[0])

      //handle_profile((struct deluge_msg_profile *)msg);
    	handle_profile_mc_deluge((struct deluge_msg_profile *)msg);
    break;
  default:
    PRINTF("Incoming packet with unknown command: %d\n", msg[0]);
  }
}

static void
unicast_recv(struct unicast_conn *c, const linkaddr_t *sender)
{
  command_dispatcher(sender);
}

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *sender)
{
  command_dispatcher(sender);
}

int
deluge_disseminate(char *file, unsigned version)
{
  /* This implementation disseminates at most one object. */
  if(next_object_id > 0 || init_object(&current_object, file, version) < 0) {
    return -1;
  }
  process_start(&deluge_process, file);

  return 0;
}

PROCESS_THREAD(deluge_process, ev, data)
{
  static struct etimer et;
  static unsigned time_counter;
  static unsigned r_rand;

  PROCESS_EXITHANDLER(goto exit);

  PROCESS_BEGIN();

  deluge_event = process_alloc_event();

  //broadcast_open(&deluge_broadcast, DELUGE_BROADCAST_CHANNEL, &broadcast_call);
  //unicast_open(&deluge_uc, DELUGE_UNICAST_CHANNEL, &unicast_call);
  r_interval = T_LOW;

  PRINTF("Maintaining state for object %s of %d pages\n",
	current_object.filename, OBJECT_PAGE_COUNT(current_object));

  deluge_state = DELUGE_STATE_MAINTAIN;

  /* Trung add */
  if(node_id == 1)
  {

	  current_object.count_tx_page = 0;

	  ctimer_set(&profile_timer, 1 * CLOCK_SECOND, (void *)(void *)send_profile_mc_deluge, &current_object);
  }
  ctimer_set(&set_period_first_timer,10 * CLOCK_SECOND,(void *)(void *)set_period_first_mc_deluge,NULL);

  //set level and channel
  if(node_id == 1)
  {
	  current_object.level = 1;
  }
  if(node_id == 2 || node_id == 3 || node_id == 7 || node_id == 5 || node_id == 6)
  {
	  current_object.level = 2;
  }
  if(node_id == 4 || node_id ==8|| node_id == 9 || node_id == 10 || node_id == 11 || node_id == 12)
  {
 	  current_object.level = 3;
  }
  /*if(node_id == 11 || node_id ==12 || node_id == 13 || node_id == 14 || node_id == 15 || node_id == 23)
  {
	  current_object.level = 4;
  }
  if(node_id == 18 || node_id == 19 || node_id == 20 || node_id == 21 || node_id == 22)
    {
  	  current_object.level = 5;
    }*/
  /*------------------------------------------------------------------------------*/
  for(r_interval = T_LOW;;) {
    if(neighbor_inconsistency) {
      /* Deluge M.2 */
      r_interval = T_LOW;
      neighbor_inconsistency = 0;
    } else {
      /* Deluge M.3 */
      r_interval = (2 * r_interval >= T_HIGH) ? T_HIGH : 2 * r_interval;
    }

    r_rand = r_interval / 2 + ((unsigned)random_rand() % (r_interval / 2));
    recv_adv = 0;
    old_summary = 0;

    /* Deluge M.1 */
    //ctimer_set(&summary_timer, r_rand * CLOCK_SECOND, (void *)(void *)advertise_summary, &current_object);

    /* Deluge M.4 */
    //ctimer_set(&profile_timer, r_rand * CLOCK_SECOND, (void *)(void *)send_profile, &current_object);

    LONG_TIMER(et, time_counter, r_interval);
  }

exit:
  unicast_close(&deluge_uc);
  broadcast_close(&deluge_broadcast);
  if(current_object.cfs_fd >= 0) {
    cfs_close(current_object.cfs_fd);
  }
  if(current_object.pages != NULL) {
    free(current_object.pages);
  }

  PROCESS_END();
}

/*-------------------------------------------------------------------------------------------------------*/
void
handle_profile_mc_deluge(struct deluge_msg_profile *msg)
{
  int i;
  int npages;
  struct deluge_object *obj;
  char *p;
  obj = &current_object;

  PRINTF("Node %u receives profile, version:%d  \n",node_id,msg->version);

  if(msg->version <= current_object.update_version) {
    return;
  }

  PRINTF("In handle_profile_mc_deluge: Received profile of version %u with a vector of %u pages.\n",
	msg->version, msg->npages);

  leds_off(LEDS_RED);
  current_object.tx_set = 0;

  npages = OBJECT_PAGE_COUNT(*obj);
  obj->size = msg->npages * S_PAGE;

  p = malloc(OBJECT_PAGE_COUNT(*obj) * sizeof(*obj->pages));
  if(p == NULL) {
    PRINTF("Failed to reallocate memory for pages!\n");
    return;
  }

  memcpy(p, obj->pages, npages * sizeof(*obj->pages));
  free(obj->pages);
  obj->pages = (struct deluge_page *)p;

  if(msg->npages < npages) {
    npages = msg->npages;
  }

  for(i = 0; i < npages; i++) {
    if(msg->version_vector[i] > obj->pages[i].version) {
      obj->pages[i].packet_set = 0;
      obj->pages[i].flags &= ~PAGE_COMPLETE;
      obj->pages[i].version = msg->version_vector[i];
    }
  }
  obj->current_rx_page = highest_available_page(obj);
  obj->update_version = msg->version;

  for(; i < msg->npages; i++) {
    init_page(obj, i, 0);
  }

  //obj->current_rx_page = highest_available_page(obj);
  //obj->update_version = msg->version;

  // Rebroadcast its profile
  rtimer_clock_t t0;

  t0 = RTIMER_NOW() + 10+ (random_rand()%100);
  while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }

  send_profile_mc_deluge(obj);

}

static void
send_profile_mc_deluge(struct deluge_object *obj)
{
  struct deluge_msg_profile *msg;
  unsigned char buf[sizeof(*msg) + OBJECT_PAGE_COUNT(*obj)];
  int i;

  //if(broadcast_profile && recv_adv < CONST_K)
  //{
    broadcast_profile = 0;

    msg = (struct deluge_msg_profile *)buf;
    msg->cmd = DELUGE_CMD_PROFILE;
    //msg->version = obj->version; // when a node receives a page completely, it sends its profile with the updated version.
    msg->version = obj->update_version;
    msg->npages = OBJECT_PAGE_COUNT(*obj);
    msg->object_id = obj->object_id;
    for(i = 0; i < msg->npages; i++) {
      msg->version_vector[i] = obj->pages[i].version;
    }

    //packetbuf_copyfrom(buf, sizeof(buf));
    //broadcast_send(&deluge_broadcast);

    uint8_t ret = nullrdc_noframer_send_packet(buf,sizeof(buf), 0);

    PRINTF("Node %u sends profile, result:%u \n",node_id,ret);


  //}
    if(node_id == 1)
    {
    	//transition(DELUGE_STATE_TX);
    	//ctimer_set(&tx_timer, 2*CLOCK_SECOND, sink_send_page_mc_deluge, obj);
    }
}

static void
sink_send_page_mc_deluge(void * arg)
{
  struct deluge_object *obj;

  obj = (struct deluge_object *)arg;
  obj->count_tx_set = (1 << 4) -1;

  if(obj->count_tx_page >= 0 && obj->count_tx_set && obj->count_tx_page <= OBJECT_PAGE_COUNT(*obj)-1)
  {
	  send_page_mc_deluge(obj, obj->count_tx_page);

	/* Deluge T.2. */
	if(obj->count_tx_set) {
	  packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM);
	  //ctimer_reset(&tx_timer);
	} else {
	  packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM_END);

	  obj->count_tx_page ++;


	  //set timer to continue send page


	  ctimer_set(&tx_timer_mc_deluge, 2*CLOCK_SECOND, sink_send_page_mc_deluge, obj);

	}
  }

}

static void
send_page_mc_deluge(struct deluge_object *obj, unsigned pagenum)
{
  unsigned char buf[S_PAGE];
  struct deluge_msg_packet pkt;
  unsigned char *cp;

  pkt.cmd = DELUGE_CMD_PACKET;
  linkaddr_copy(&pkt.sender,&linkaddr_node_addr);
  //pkt.sender.u8[0] = node_id;
  pkt.pagenum = pagenum;
  pkt.version = obj->pages[pagenum].version;
  pkt.packetnum = 0;
  pkt.object_id = obj->object_id;
  pkt.crc = 0;

  read_page(obj, pagenum, buf);

  /* Divide the page into packets and send them one at a time. */
  for(cp = buf; cp + S_PKT <= (unsigned char *)&buf[S_PAGE]; cp += S_PKT) {
    if(obj->count_tx_set & (1 << pkt.packetnum)) {
      pkt.crc = crc16_data(cp, S_PKT, 0);
      memcpy(pkt.payload, cp, S_PKT);
      //rtimer_clock_t t0;
      //t0 = RTIMER_NOW() + 4;
      //while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }

      //packetbuf_copyfrom(&pkt, sizeof(pkt));
      //broadcast_send(&deluge_broadcast);
      nullrdc_noframer_sent_count = 0;
      nullrdc_noframer_send_packet(&pkt, sizeof(pkt),1);
    }
    pkt.packetnum++;
  }
  obj->count_tx_set = 0;
}

void
handle_packet_mc_deluge(struct deluge_msg_packet *msg, linkaddr_t *sender)
{
  struct deluge_page *page;
  uint16_t crc;
  struct deluge_msg_packet packet;

  memcpy(&packet, msg, sizeof(packet));

  PRINTF("Incoming packet for object id %u, version %u, page %u, packet num %u!\n",
	(unsigned)packet.object_id, (unsigned)packet.version,
	(unsigned)packet.pagenum, (unsigned)packet.packetnum);

  if(packet.pagenum != current_object.current_rx_page) {
	  PRINTF("In handle_packet_mc_deluge: current_rx_page: %u \n",current_object.current_rx_page);
	  if(!ctimer_expired(&node_tx_timer_send_page_mc_deluge))
	  {
		  //uint8_t rand_time = 10 + (random_rand()%100);

		        //node_send_page_mc_deluge(&current_object,current_object.current_page_completed);
		  //ctimer_set(&node_tx_timer_send_page_mc_deluge, 1*CLOCK_SECOND+rand_time, node_send_page_mc_deluge_set_timer, & current_object.current_page_completed);
	  }
    return;
  }
  if(packet.pagenum == current_object.current_page_completed)
  {
	  if(!ctimer_expired(&node_tx_timer_send_page_mc_deluge))
	  {

	  }
  }

  if(packet.version != current_object.version) {
    neighbor_inconsistency = 1;
  }

  page = &current_object.pages[packet.pagenum];


  if(packet.version == page->version && !(page->flags & PAGE_COMPLETE))
  {
	PRINTF("In handle_packet_mc_deluge: Receiving packet from node %u \n",sender->u8[0]);

    memcpy(&current_object.current_page[S_PKT * packet.packetnum],
	packet.payload, S_PKT);

    crc = crc16_data(packet.payload, S_PKT, 0);
    if(packet.crc != crc) {
      PRINTF("packet crc: %hu, calculated crc: %hu\n", packet.crc, crc);
      return;
    }

    page->last_data = clock_time();
    page->packet_set |= (1 << packet.packetnum);

    if(page->packet_set == ALL_PACKETS)
    {
      /* This is the last packet of the requested page; stop streaming. */
      packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM_END);

      write_page(&current_object, packet.pagenum, current_object.current_page);
      page->version = packet.version;
      page->flags = PAGE_COMPLETE;
      PRINTF("Page %u completed\n", packet.pagenum);


      current_object.current_rx_page++;

      current_object.current_page_completed = current_object.current_rx_page - 1;

      /*uint8_t rand_time = 5 + (random_rand()%10);


  	  ctimer_set(&node_tx_timer_send_page_mc_deluge, rand_time, node_send_page_mc_deluge_set_timer, & current_object.current_page_completed);

		*/
      if(packet.pagenum == OBJECT_PAGE_COUNT(current_object) - 1)
      {
			current_object.version = current_object.update_version;
			leds_on(LEDS_RED);
			PRINTF("Update completed for object %u, version %u\n",
				   (unsigned)current_object.object_id, packet.version);


      }
      else if(current_object.current_rx_page < OBJECT_PAGE_COUNT(current_object))
      {
        if(ctimer_expired(&rx_timer))
        {
        	//PRINTF("Set Time to send request \n");
        	//ctimer_set(&rx_timer,0.5*CLOCK_SECOND,send_request_mc_deluge, &current_object);
        }
      }
      /* Deluge R.3 */
      transition(DELUGE_STATE_MAINTAIN);
    }
    else
    {
      /* More packets to come. Put lower layers in streaming mode. */
      packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM);
      if(ctimer_expired(&rx_timer))
	  {
		PRINTF("Set Time to send request \n");
		current_object.current_page_completed = current_object.current_rx_page;
		uint8_t rand_time =8 + (random_rand()%10);
		ctimer_set(&rx_timer,rand_time,send_request_mc_deluge, &current_object);
	  }
    }
  }
}

static void
send_request_mc_deluge(void *arg)
{
  struct deluge_object *obj;
  struct deluge_msg_request request;

  obj = (struct deluge_object *)arg;
  int i = 0;
  uint8_t pagenum_request;


  if((obj->pages[obj->current_page_completed].flags & PAGE_COMPLETE))
  {
	  return;
  }

  pagenum_request = obj->current_page_completed;

  request.cmd = DELUGE_CMD_REQUEST;
  //request.pagenum = obj->current_rx_page;
  linkaddr_copy(&request.receiver,&obj->parent_addr);

  request.pagenum = pagenum_request;
  request.version = obj->pages[request.pagenum].version;
  //request.request_set = ~obj->pages[obj->current_rx_page].packet_set;
  request.request_set = ~obj->pages[request.pagenum].packet_set;
  request.object_id = obj->object_id;

  PRINTF("Sending request for page %d, version %u, request_set %u\n",
	request.pagenum, request.version, request.request_set);

  //packetbuf_copyfrom(&request, sizeof(request));
  //broadcast_send(&deluge_broadcast);
  //unicast_send(&deluge_uc, &obj->parent_addr);


  nullrdc_noframer_sent_count = 0;
  nullrdc_noframer_send_packet(&request, sizeof(request),0);
}

static void node_send_page_mc_deluge(struct deluge_object *obj, unsigned pagenum)
{
	if(!(obj->pages[pagenum].flags & PAGE_COMPLETE))
	{
		return;
	}
	 obj->count_tx_set = (1 << 4) -1;

	 if(node_id == 1)
	 {
		 //rtimer_clock_t t0;
		 //t0 = RTIMER_NOW() + 1+ (random_rand()%100);
		 //while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0)) { }
		 send_page_mc_deluge(obj,pagenum);
	 }
	 else{

		 uint8_t rand_value = random_rand()%100 ;
		 rtimer_set(&node_send_page_rtimer, RTIMER_NOW()+rand_value,1,node_send_page_mc_deluge_rtimer_expired,NULL);
	 }

	 /*if(nullrdc_noframer_check_channel() == 1)
	 {
		 send_page_mc_deluge(obj,pagenum);
	 }
	 else
	 {
		 PRINTF("In node_send_page_mc_deluge: channel is busy, set time to resend \n");
		 ctimer_set(&tx_node_resend_page_timer, 10, node_resend_page_mc_deluge, NULL);
	 }*/

}

void
handle_request_mc_deluge(struct deluge_msg_request *msg)
{


  int highest_available;

  if(msg->pagenum >= OBJECT_PAGE_COUNT(current_object)) {
	 PRINTF("In Handle_Request Func: msg->pagenum >= OBJECT_PAGE_COUNT(current_object) \n");
    return;
  }

  if(msg->version != current_object.version) {
    neighbor_inconsistency = 1;
  }

  highest_available = highest_available_page(&current_object);
  PRINTF("In Handle_Request Func: Node %u receives request->>>pagenum:%u,request_set:%u, highest_avaible:%u,msg->version:%u,current_object.version:%u \n",node_id,msg->pagenum,msg->request_set,
		  highest_available,msg->version,current_object.update_version);
  /* Deluge M.6 */
  if(msg->version == current_object.update_version && msg->pagenum <= highest_available)

  {
    current_object.pages[msg->pagenum].last_request = clock_time();

    /* Deluge T.1 */
    if(msg->pagenum == current_object.current_tx_page) {
      current_object.tx_set |= msg->request_set;
    } else {
      current_object.current_tx_page = msg->pagenum;
      current_object.tx_set = msg->request_set;
    }

    //transition(DELUGE_STATE_TX);
    //ctimer_set(&tx_timer, CLOCK_SECOND, tx_callback, &current_object);
    ctimer_set(&tx_timer, 2, tx_callback_mc_deluge, &current_object);
  }
}

static void node_send_page_mc_deluge_set_timer(void *arg)
{
	ctimer_reset(&node_tx_timer_send_page_mc_deluge);

	PRINTF("Switch to sending channel \n");

	if(current_object.level == 2)
	{
		cc2420_set_channel(20);
	}
	if(current_object.level == 3)
	{
		cc2420_set_channel(20);
	}
	if(current_object.level == 4)
	{
		cc2420_set_channel(15);
	}
	if(current_object.level == 5)
	{
		cc2420_set_channel(15);
	}


	node_send_page_mc_deluge(&current_object,current_object.current_page_completed);

	if(current_object.current_page_completed == OBJECT_PAGE_COUNT(current_object) - 1)
	{
		ctimer_stop(&node_tx_timer_send_page_mc_deluge);
		return;
	}

}

static void set_period_first_mc_deluge(void *arg)
{
	if(node_id == 1) //SINK
	{
		cc2420_set_channel(15);
		ctimer_set(&tx_timer, 0, sink_send_page_mc_deluge, &current_object);
	}
	else
	{
		uint8_t rand_time = 5 + (random_rand()%10);

		      //node_send_page_mc_deluge(&current_object,current_object.current_page_completed);
		if(current_object.level == 2)
		{
			cc2420_set_channel(15);
			ctimer_set(&node_tx_timer_send_page_mc_deluge, 1*CLOCK_SECOND, node_send_page_set_general_timer_mc_deluge, NULL);
			ctimer_set(&node_rx_timer_send_page_mc_deluge, 0, node_receive_page_set_first_mc_deluge, NULL);
		}
		else if(current_object.level == 3)
		{
			ctimer_set(&node_tx_timer_send_page_mc_deluge, 2*CLOCK_SECOND, node_send_page_set_general_timer_mc_deluge, NULL);
			ctimer_set(&node_rx_timer_send_page_mc_deluge, 1*CLOCK_SECOND, node_receive_page_set_first_mc_deluge, NULL);
		}
		else if(current_object.level == 4)
		{
			ctimer_set(&node_tx_timer_send_page_mc_deluge, 3*CLOCK_SECOND, node_send_page_set_general_timer_mc_deluge, NULL);
			ctimer_set(&node_rx_timer_send_page_mc_deluge, 2*CLOCK_SECOND, node_receive_page_set_first_mc_deluge, NULL);
		}
		else if(current_object.level == 5)
		{
			ctimer_set(&node_tx_timer_send_page_mc_deluge, 4*CLOCK_SECOND, node_send_page_set_general_timer_mc_deluge, NULL);
			ctimer_set(&node_rx_timer_send_page_mc_deluge, 3*CLOCK_SECOND, node_receive_page_set_first_mc_deluge, NULL);
		}
	}
}
static void node_send_page_set_general_timer_mc_deluge(void *arg)
{
	PRINTF("Switch to sending channel \n");

	if(current_object.level == 2)
	{
		cc2420_set_channel(20);
	}
	if(current_object.level == 3)
	{
		cc2420_set_channel(20);
	}
	if(current_object.level == 4)
	{
		cc2420_set_channel(15);
	}
	if(current_object.level == 5)
	{
		cc2420_set_channel(15);
	}

	node_send_page_mc_deluge(&current_object,current_object.current_page_completed);

	ctimer_set(&node_tx_timer_send_page_mc_deluge, 2*CLOCK_SECOND, node_send_page_mc_deluge_set_timer, NULL);

}

static void node_receive_page_set_first_mc_deluge(void *arg)
{

	PRINTF("Switch to receiving channel \n");

	if(current_object.level == 2)
	{
		cc2420_set_channel(15);
	}
	if(current_object.level == 3)
	{
		cc2420_set_channel(20);
	}
	if(current_object.level == 4)
	{
		cc2420_set_channel(20);
	}
	if(current_object.level == 5)
		{
			cc2420_set_channel(15);
		}
	ctimer_set(&node_rx_timer_send_page_mc_deluge, 2*CLOCK_SECOND, node_receive_page_set_general_mc_deluge, NULL);

}

static void node_receive_page_set_general_mc_deluge(void *arg)
{
	// // set receiving channel accordingly
	ctimer_reset(&node_rx_timer_send_page_mc_deluge);

	PRINTF("Switch to receiving channel \n");

	if(current_object.level == 2)
	{
		cc2420_set_channel(15);
	}
	if(current_object.level == 3)
	{
		cc2420_set_channel(20);
	}
	if(current_object.level == 4)
	{
		cc2420_set_channel(20);
	}
	if(current_object.level == 5)
	{
		cc2420_set_channel(15);
	}

}


static void
send_page_by_request_mc_deluge(struct deluge_object *obj, unsigned pagenum)
{
  unsigned char buf[S_PAGE];
  struct deluge_msg_packet pkt;
  unsigned char *cp;

  pkt.cmd = DELUGE_CMD_PACKET;
  linkaddr_copy(&pkt.sender,&linkaddr_node_addr);
  pkt.pagenum = pagenum;
  pkt.version = obj->pages[pagenum].version;
  pkt.packetnum = 0;
  pkt.object_id = obj->object_id;
  pkt.crc = 0;

  read_page(obj, pagenum, buf);

  /* Divide the page into packets and send them one at a time. */
  for(cp = buf; cp + S_PKT <= (unsigned char *)&buf[S_PAGE]; cp += S_PKT) {
    if(obj->tx_set & (1 << pkt.packetnum)) {
      pkt.crc = crc16_data(cp, S_PKT, 0);
      memcpy(pkt.payload, cp, S_PKT);
      //packetbuf_copyfrom(&pkt, sizeof(pkt));
      //broadcast_send(&deluge_broadcast);
      nullrdc_noframer_sent_count = 0;
      nullrdc_noframer_send_packet(&pkt, sizeof(pkt),0);
    }
    pkt.packetnum++;
  }
  obj->tx_set = 0;
}

static void
tx_callback_mc_deluge(void *arg)
{
  struct deluge_object *obj;

  obj = (struct deluge_object *)arg;
  if(obj->current_tx_page >= 0 && obj->tx_set) {
    send_page_by_request_mc_deluge(obj, obj->current_tx_page);
    /* Deluge T.2. */
    if(obj->tx_set) {
      packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM);
      ctimer_reset(&tx_timer);
    } else {
      packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
			 PACKETBUF_ATTR_PACKET_TYPE_STREAM_END);
      obj->current_tx_page = -1;
      transition(DELUGE_STATE_MAINTAIN);
    }
  }
}

static void node_resend_page_mc_deluge(void *arg)
{
	node_send_page_mc_deluge(&current_object,current_object.current_page_completed);
}


/*-------------------------------------------------------------------------------------------*/

PROCESS(node_send_page_process,"node_send_page_process");

PROCESS_THREAD(node_send_page_process,ev,data)
{

	PROCESS_BEGIN();
	while(1)
	{
		 PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
	}
	PROCESS_END();
}

static void node_send_page_mc_deluge_rtimer_expired(void *arg)
{

}

/*
* ftsp.h
*
* Created on: Feb 23, 2010
* Author: Federico Ferrari
*/
#ifndef FTSP_H_
#define FTSP_H_
#include "contiki.h"
#include "net/rime/broadcast.h"
#include "sys/rtimer.h"
#include "sys/clock.h"



#ifdef RADIO_CC2420_CONFIG
#define RADIO_CC2420 RADIO_CC2420_CONFIG
#else
#define RADIO_CC2420 0
#endif

#ifdef RADIO_CC2630_CONFIG
#define RADIO_CC2630 RADIO_CC2630_CONFIG
#else
#define RADIO_CC2630 0
#endif

uint8_t state;

#define FTSP_CHANNEL 140
#define FTSP_ATTRIBUTES { PACKETBUF_ATTR_EPACKET_ID, PACKETBUF_ATTR_BIT * 8 }, \
						{ PACKETBUF_ATTR_TIMESTAMP, PACKETBUF_ATTR_BIT },\



struct ftsp_message
{
unsigned int    root_id;
unsigned int	node_id;
uint8_t padding[20];
unsigned long   global_time;
};
struct ftsp_message_rcv
{
unsigned int    root_id;
unsigned int	node_id;
uint8_t padding[20];
unsigned long   global_time;
unsigned long	local_time;
uint8_t seq_num;

};
typedef struct table_item
{
uint8_t state;
rtimer_long_clock_t local_time;
long time_offset; // global-time - local_time
} table_item;
enum
{
MAX_ENTRIES = 8, // number of entries in the table
BEACON_RATE = 5, // how often send the beacon msg (in seconds)
ROOT_TIMEOUT = 4, // time to declare itself the root if no msg was received (in sync periods)
IGNORE_ROOT_MSG = 4, // after becoming the root ignore other roots messages (in send period)
ENTRY_VALID_LIMIT = 4, // number of entries to become synchronized
ENTRY_SEND_LIMIT = 3, // number of entries to send sync messages
ENTRY_THROWOUT_LIMIT = 30, // if time sync error is bigger than this (in 32 kHz ticks) clear the table
};
enum
{
ENTRY_EMPTY = 0,
ENTRY_FULL = 1,
};
enum
{
STATE_IDLE = 0x00,
STATE_PROCESSING = 0x01,
STATE_SENDING = 0x02,
STATE_INIT = 0x04,
};
enum
{
TS_TIMER_MODE = 0,
TS_USER_MODE = 1,
};
enum
{
FTSP_OK = 1,
FTSP_ERR = 0,
};
enum{
	SINK_ID = 1,
};
process_event_t ftsp_table_updated;
process_event_t ftsp_msg_sent;
//#define get_local_time() RTIMER_NOW()
//#define get_local_time() RTIMER_NOW()
#define get_local_time() clock_time()
//#define get_local_time() rtimer_arch_now_long()
rtimer_long_clock_t get_global_time(void);

int is_synced(void);
rtimer_long_clock_t local_to_global(rtimer_long_clock_t time);
rtimer_long_clock_t global_to_local(rtimer_long_clock_t time);
void ftsp_init(void);
void ftsp_stop(void);
void ftsp_send(void);
uint8_t ftsp_get_mode(void);
void ftsp_set_mode(uint8_t mode_);
uint8_t ftsp_get_seqNum();
void ftsp_reset_seqNum();

// mungtv 20150619
void get_global_time_departure();
void ftsp_maintain_syn(unsigned int	node_id, unsigned long global_time);
/*---------------------------------------------------------------------------*/
PROCESS_NAME(ftsp_process);
PROCESS_NAME(ftsp_msg_process);
PROCESS_NAME(ftsp_send_process);
/*---------------------------------------------------------------------------*/



#endif /*FTSP_H_*/

#ifndef REPLY_SERVER_H
#define REPLY_SERVER_H

#include "inttypes.h"
#include "lwip.h"
#include "lwip/tcp.h"
#include "lwip/ip.h"

/* ECHO protocol states */
//enum tcp_echoserver_states
//{
//  ES_NONE = 0,
//  ES_ACCEPTED,
//  ES_RECEIVED,
//  ES_CLOSING
//};

///* structure for maintaing connection infos to be passed as argument
//   to LwIP callbacks*/
//struct tcp_echoserver_struct
//{
//  u8_t state;             /* current connection state */
//  u8_t retries;
//  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
//  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
//};


struct tcp_pcb *initServer(uint16_t port);



#endif

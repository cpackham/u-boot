/*
 * net/ndisc.h
 *
 * (C) Copyright 2013 Allied Telesis Labs NZ
 *
 * This file is released under the terms of GPL v2 and any later version.
 * See the file COPYING in the root directory of the source tree for details.
 */

/* IPv6 destination address of packet waiting for ND */
extern struct in6_addr net_nd_sol_packet_ip6;
/* IPv6 address we are expecting ND advert from */
extern struct in6_addr net_nd_rep_packet_ip6;
/* MAC destination address of packet waiting for ND */
extern uchar *net_nd_packet_mac;
/* pointer to packet waiting to be transmitted after ND is resolved */
extern uchar *net_nd_tx_packet;
extern uchar net_nd_packet_buf[PKTSIZE_ALIGN + PKTALIGN];
/* size of packet waiting to be transmitted */
extern int net_nd_tx_packet_size;
/* the timer for ND resolution */
extern ulong net_nd_timer_start;
/* the number of requests we have sent so far */
extern int net_nd_try;


void ndisc_receive(struct ethernet_hdr *et, struct ip6_hdr *ip6, int len);
void ip6_NDISC_Request(void);
void ip6_NDISC_TimeoutCheck(void);

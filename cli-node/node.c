/*
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

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>


#define DEBUG DEBUG_PRINT
#ifdef USING_TARGET_CC2530
#include "net/uip-debug.h"
#include "debug.h"
#else//USING_TARGET_CC2530
#include "net/ip/uip-debug.h"
#endif//USING_TARGET_CC2530

#include "dev/watchdog.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "dev/button-sensor.h"


#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])

#define MAX_PAYLOAD_LEN 64

static char buf[MAX_PAYLOAD_LEN];

static struct uip_udp_conn *server_conn;
static uint16_t len;

#define SERVER_REPLY          1
/*---------------------------------------------------------------------------*/
PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/

static void 
get_my_adrress(uip_ipaddr_t *ipaddr) {

  uint16_t ip[4];
#ifdef  USING_TARGET_CC2530
  ip[0] = ((uint16_t)rimeaddr_node_addr.u8[0])<<8|(uint16_t)rimeaddr_node_addr.u8[1];
  ip[1] = ((uint16_t)rimeaddr_node_addr.u8[2])<<8|(uint16_t)rimeaddr_node_addr.u8[3];
  ip[2] = ((uint16_t)rimeaddr_node_addr.u8[4])<<8|(uint16_t)rimeaddr_node_addr.u8[5];
  ip[3] = ((uint16_t)rimeaddr_node_addr.u8[6])<<8|(uint16_t)rimeaddr_node_addr.u8[7];
#else // USING_TARGET_CC2530
  uint8_t longaddr[8];
  memset(longaddr, 0, sizeof(longaddr));
  linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);
  //printf("My MAC =  %02x%02x:%02x%02x:%02x%02x:%02x%02x \n",
         //longaddr[0], longaddr[1], longaddr[2], longaddr[3],
         //longaddr[4], longaddr[5], longaddr[6], longaddr[7]);
  ip[0] = ((uint16_t)longaddr[0])<<8|(uint16_t)longaddr[1];
  ip[1] = ((uint16_t)longaddr[2])<<8|(uint16_t)longaddr[3];
  ip[2] = ((uint16_t)longaddr[4])<<8|(uint16_t)longaddr[5];
  ip[3] = ((uint16_t)longaddr[6])<<8|(uint16_t)longaddr[7];
#endif // USING_TARGET_CC2530

  uip_ip6addr(ipaddr, 0xaaaa, 0, 0, 0, 0x0212, ip[1], ip[2], ip[3]);

}


/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  static char resp[MAX_PAYLOAD_LEN];
  int16_t rssi, lqi;

  memset(resp, 0, MAX_PAYLOAD_LEN);
  memset(buf, 0, MAX_PAYLOAD_LEN);
  if(uip_newdata()) {
    leds_on(LEDS_RED);
    len = uip_datalen();
    memcpy(buf, uip_appdata, len);
    PRINTF("%u bytes from [", len);
    PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    PRINTF("]:%u\n", UIP_HTONS(UIP_UDP_BUF->srcport));

    printf("TTL = %d\n", UIP_IP_BUF->ttl);

    rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
    //lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);
    printf("RSSI = %d\n",rssi);
    //PRINTF("LQI = %0d\n",lqi);
  
#if SERVER_REPLY

    if (buf[0]== 1) {
      PRINTF ("Execute CMD = led_on\n");
      leds_on(LEDS_GREEN);
      sprintf(resp, "Replied = led_on OK\n");
    }
    else if (buf[0] == 2) {
      PRINTF ("Execute CMD = led_off\n");
      leds_off(LEDS_GREEN);
      sprintf(resp, "Replied = led_off OK\n");
    }
    else if (buf[0] == 3) {
      PRINTF ("Execute CMD = get_rssi\n");
      sprintf(resp, "Replied = %d\n", rssi);
    }

    //sprintf(resp, "Reply: RSSI = %d\n", rssi);
    //printf("cbi gui 1 \n");
    uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    server_conn->rport = UIP_UDP_BUF->srcport;
    //printf("cbi gui 2 \n");
    uip_udp_packet_send(server_conn, resp, MAX_PAYLOAD_LEN);
    /* Restore server connection to allow data from any node */
    printf("da gui \n");
    uip_create_unspecified(&server_conn->ripaddr);
    server_conn->rport = 0;
#endif
  }
  leds_off(LEDS_RED);
  return;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  uip_ipaddr_t my_ipaddr;

  PROCESS_BEGIN();
  printf("Starting UDP server\n");

  server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  udp_bind(server_conn, UIP_HTONS(3000));

  PRINTF("Listen port: 3000, TTL=%u\n", server_conn->ttl);

  PRINTF("My IPV6 address: ");  
  get_my_adrress(&my_ipaddr); 
  PRINT6ADDR(&my_ipaddr);PRINTF("\n");

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*
|-------------------------------------------------------------------|
| HCMC University of Technology                                     |
| Telecommunications Departments                                    |
| Wireless Embedded Firmware                                        |
| Version: 1.0                                                      |
| Author: ng.sontung.1995@gmail.com                                 |
| Date: 11/2017														|
| HW Support: CC2538DK 												|
|																	|
| @Compile: gcc cli.c  -o cli                                       |
|-------------------------------------------------------------------|*/

#include "my-include.h"

#include "stdint.h"
#include <stdio.h>

#include <math.h>
#include <string.h>

#include "cli.h"

#define DEBUG 1
#if DEBUG
#define PRINTF(...)    printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define TIMEOUT_PRINT_ALL 10

/*-----------------------------------------------------------------|*/
#define MAXBUF 64   //Max length of buffer 
#define PORT "3000"    //The port on which to listen for incoming data 

struct  pollfd fd;
int     res;
#define TIMEOUT_ACK 3000

static  int     rev_bytes;
static  struct  sockaddr_in6 rev_sin6;
static  int     rev_sin6len;

static  char    rev_buffer[MAXBUF];
static 	char 	data_rev[MAXBUF];
static  char    rev_buffer_cpy[MAXBUF];

static  int     port;

#define LEN_CHAR_IPV6 26

static  char    dst_ipv6addr[LEN_CHAR_IPV6];

static  char    str_port[5];
static  char    cmd[20];
static  char    arg[32];

static  time_t rawtime;
static  time_t time_begin;
static  time_t time_end;
static  time_t current_time;

static  struct tm * timeinfo;

/* delay measurement */
struct timeval t0;
struct timeval t1;
float elapsed;
/*-----------------------------------------------------------------|*/
float timedifference_msec(struct timeval t0, struct timeval t1){
    return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
}
/*-----------------------------------------------------------------|*/

int main(int argc, char* argv[])
{ 
    int sock;
    int status, i;
    struct addrinfo sainfo, *psinfo;
    struct sockaddr_in6 sin6;
    int sin6len;
    char buffer[MAXBUF];

    sin6len = sizeof(struct sockaddr_in6);

    memset(buffer, 0, MAXBUF);
    port = 3000;
    sprintf(dst_ipv6addr,"aaaa::212:7401:1:101");

    if(argc < 4) {
        printf("Specify an IPv6 addr or port number or Cmd \n"), exit(1);
    }

    else if (argc==4) {
        sprintf(dst_ipv6addr,"%s",argv[1]);      
        strcpy(str_port,argv[2]);
        strcpy(cmd,argv[3]);
        port = atoi(str_port);
        //sprintf(buffer,"%s",cmd);

        /* REQ-TYPE*/
        if (strcmp(cmd,CLI_LED_ON)==0) {
            printf("led_on\n");
            buffer[0] = 1;
        }
        else if (strcmp(cmd,CLI_LED_OFF)==0) {
            printf("led_off\n");
            buffer[0] = 2;
        }
        else if (strcmp(cmd,CLI_RSSI)==0) {
            printf("get rssi\n");
            buffer[0] = 3;
        }  
        else {
          printf("Unknown cmd \n");
          exit(1);
        }  
    }     

    sock = socket(PF_INET6, SOCK_DGRAM,0);

    memset(&sin6, 0, sizeof(struct sockaddr_in6));
    sin6.sin6_port = htons(port);
    sin6.sin6_family = AF_INET6;
    sin6.sin6_addr = in6addr_any;

    status = bind(sock, (struct sockaddr *)&sin6, sin6len);

    if(-1 == status)
        error("bind"), exit(1);

    memset(&sainfo, 0, sizeof(struct addrinfo));
    memset(&sin6, 0, sin6len);

    sainfo.ai_flags = 0;
    sainfo.ai_family = PF_INET6;
    sainfo.ai_socktype = SOCK_DGRAM;
    sainfo.ai_protocol = IPPROTO_UDP;
    status = getaddrinfo(dst_ipv6addr, str_port, &sainfo, &psinfo);

    status = sendto(sock, &buffer, sizeof(buffer), 0,(struct sockaddr *)psinfo->ai_addr, sin6len);

    gettimeofday(&t0, 0);

    printf("\nSend REQUEST (%d bytes ) to [%s]:%s\n",status, dst_ipv6addr,str_port);
    printf(".......... done\n");

    /*wait for a reply */
    fd.fd = sock;
    fd.events = POLLIN;
    
    res = poll(&fd, 1, TIMEOUT_ACK); 
    if (res == -1) {
        printf(" Error !!!\n");
    }
    else if (res == 0)   {
        printf(" Wait ACK: timeout !!!\n");
    }
    else{
        rev_bytes = recvfrom((int)sock, rev_buffer, MAXBUF, 0,(struct sockaddr *)(&rev_sin6), (socklen_t *) &rev_sin6len);
        if (rev_bytes<0) {
            perror("Problem in recvfrom \n");
            exit(1);
        }
        else{
            printf("Got REPLY (%d bytes): %s\n",rev_bytes, rev_buffer);
            gettimeofday(&t1, 0);
            elapsed = timedifference_msec(t0, t1);
    		printf("Cmd execution delay %.2f (ms) \n\n", elapsed);        
        }
    }

    PRINTF("----------------------------------------------------------------\n\n");

    shutdown(sock, 2);
    close(sock); 

     // free memory
    freeaddrinfo(psinfo);
    psinfo = NULL;

    return 0; 
}  
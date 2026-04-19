
/* vtp1_sro_new.c - first readout list for VTP boards (polling mode) */


#ifdef Linux_armv7l

#define DMA_TO_BIGBUF /*if want to dma directly to the big buffers*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <sys/types.h>
#ifndef VXWORKS
#include <sys/time.h>
#endif

//#define _BSD_SOURCE
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>

#include "daqLib.h"
#include "vtpLib.h"
#include "vtpConfig.h"

#include "circbuf.h"

/*****************************/
/* former 'crl' control keys */

/* readout list VTP1_SRO_NEW */
#define ROL_NAME__ "VTP1_SRO_NEW"

/* polling */
#define POLLING_MODE


/* name used by loader */
#define INIT_NAME vtp1_sro_new__init

#include "rol.h"

void usrtrig(unsigned long, unsigned long);
void usrtrig_done();

/* vtp readout */
#include "VTP_source.h"

/************************/
/************************/

#define NUM_VTP_CONNECTIONS 2   /* can be up to 4 */
#define VTP_NET_MODE        0   /*  0=TCP 1=UDP   */
#define ROCID               80
static int roc_id; /* it is not our, it is VME crate rocid */

/* define an array of Payload port Config Structures */
PP_CONF ppInfo[16];

/* slot->payload translation table (payload=0 not used) */
static int slot2payload[21] = 
{
/*0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20 - slots*/
  0,  0,  0, 15, 13, 11,  9,  7,  5,  3,  1,  0,  0,  2,  4,  6,  8, 10, 12, 14, 16 /* payloads */
};

#define N_EMUDATA 0
//#define N_EMUDATA 2
unsigned int emuData[] = {0xC0DA2019,0x00000001};


static char rcname[5];
static int block_level = 1;


#define ABS(x)      ((x) < 0 ? -(x) : (x))

#define TIMERL_VAR \
  static hrtime_t startTim, stopTim, dTim; \
  static int nTim; \
  static hrtime_t Tim, rmsTim, minTim=10000000, maxTim, normTim=1

#define TIMERL_START \
{ \
  startTim = gethrtime(); \
}

#define TIMERL_STOP(whentoprint_macros,histid_macros) \
{ \
  stopTim = gethrtime(); \
  if(stopTim > startTim) \
  { \
    nTim ++; \
    dTim = stopTim - startTim; \
    /*if(histid_macros >= 0)   \
    { \
      uthfill(histi, histid_macros, (int)(dTim/normTim), 0, 1); \
    }*/														\
    Tim += dTim; \
    rmsTim += dTim*dTim; \
    minTim = minTim < dTim ? minTim : dTim; \
    maxTim = maxTim > dTim ? maxTim : dTim; \
    /*logMsg("good: %d %ud %ud -> %d\n",nTim,startTim,stopTim,Tim,5,6);*/ \
    if(nTim == whentoprint_macros) \
    { \
      logMsg("timer: %7llu microsec (min=%7llu max=%7llu rms**2=%7llu)\n", \
                Tim/nTim/normTim,minTim/normTim,maxTim/normTim, \
                ABS(rmsTim/nTim-Tim*Tim/nTim/nTim)/normTim/normTim,5,6); \
      nTim = Tim = 0; \
    } \
  } \
  else \
  { \
    /*logMsg("bad:  %d %ud %ud -> %d\n",nTim,startTim,stopTim,Tim,5,6);*/ \
  } \
}

/* for compatibility */
int
getTdcTypes(int *typebyslot)
{
  return(0);
}
int
getTdcSlotNumbers(int *slotnumbers)
{
  return(0);
}

static void
__download()
{
  int stat;

#ifdef POLLING_MODE
  rol->poll = 1;
#else
  rol->poll = 0;
#endif

  printf("\n>>>>>>>>>>>>>>> ROCID=%d, CLASSID=%d <<<<<<<<<<<<<<<<\n",rol->pid,rol->classid);
  printf("CONFFILE >%s<\n\n",rol->confFile);

  /* Clear some global variables etc for a clean start */
  CTRIGINIT;

  /* init trig source VTP */
  CDOINIT(VTP, 1);

  /************/
  /* init daq */

  daqInit();
  DAQ_READ_CONF_FILE;

  printf("INFO: User Download 1 Executed\n");

  return;
}




int
get_addr_and_netmask_using_ifaddrs(const char* ifa_name, char *addr, char *netmask)
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *s;
    int found = 0;

    if (getifaddrs(&ifap) == -1) {
        perror("getifaddrs");
        exit(EXIT_FAILURE);
    }

    for (ifa = ifap; ifa && !found; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL)
            continue;

        if (strcasecmp(ifa_name, ifa->ifa_name))
            continue;

        /* IPv4 */
        if (ifa->ifa_addr->sa_family != AF_INET)
            continue;

        sa = (struct sockaddr_in *) ifa->ifa_addr;
        s = inet_ntoa(sa->sin_addr);
        strcpy(addr, s);

        sa = (struct sockaddr_in *) ifa->ifa_netmask;
        s = inet_ntoa(sa->sin_addr);
        strcpy(netmask, s);

        found = 1;
    }

    freeifaddrs(ifap);

    if (found)
        return EXIT_SUCCESS;
    return EXIT_FAILURE;
}






static void
__prestart()
{
  int i, slot, nslots, nslots_half, payload, payloads[16], stream, nstreams, stat, inst, ret;
  unsigned long jj, adc_id, sl;
  char *env, *myname;
  char /*name[128],*/ host[128], host_in[128], stream_name[NUM_VTP_CONNECTIONS][128];
  unsigned char vtp_mac[NUM_VTP_CONNECTIONS][6];
  int port_in;

  FILE *fdin;
  char filein[256], str[256];
  char *tmp, host_no_vtp[40], *ch;
  int board_type, len;

  unsigned int slotMask = 0;
  unsigned int fadcSlotMask = 0;
  unsigned int dcrbSlotMask = 0;
  int ppmask = 0;
  int netMode = VTP_NET_MODE; // 0=TCP, 1=UDP
  int localport = 0; /*sergey: will be set to 10001 in vtpLib.c*/

  *(rol->nevents) = 0;

#ifdef POLLING_MODE
  /* Register a sync trigger source (polling mode)) */
  CTRIGRSS(VTP, 1, usrtrig, usrtrig_done);
  rol->poll = 1; /* not needed here ??? */
#else
  /* Register a async trigger source (interrupt mode) */
  CTRIGRSA(VTP, 1, usrtrig, usrtrig_done);
  rol->poll = 0; /* not needed here ??? */
#endif

  sprintf(rcname,"RC%02d",rol->pid);
  printf("rcname >%4.4s<\n",rcname);


  /*****************/
  /*get my hostname*/
  myname = getenv("HOST");
  printf("myname befor >%s<\n",myname);
  // remove everything starting from first dot
  ch = strstr(myname,".");
  if(ch != NULL) *ch = '\0';
  printf("myname after >%s<\n",myname);


  /******************************************************************************/
  /*read board masks from the temporary file - until figure out better solution */
  len = strlen(myname);
  strncpy(host_no_vtp,myname,len-3);
  host_no_vtp[len-3] = '\0';
  printf("host_no_vtp >%s<\n",host_no_vtp);
  sprintf(filein,"%s/sro/%s.%s",getenv("CLON_PARMS"),host_no_vtp,"txt");
  if((fdin=fopen(filein,"r")) == NULL)
  {
    printf("Cannot open input file >%s< - exit\n",filein);
    return;
  }
  else
  {
    printf("Opened input file >%s< for reading\n\n",filein);
  }
  while( fscanf(fdin,"%d %x %x",&roc_id,&slotMask,&board_type) != EOF )
  {
    printf("info from txt file: roc_id=%d, slotMask is 0x%08x, board type is 0x%01x\n",roc_id,slotMask,board_type);
    if(board_type==0x1)
    {
      fadcSlotMask = slotMask;
      if(fadcSlotMask>0) printf("==> roc_id=%d, fadcSlotMask is 0x%08x, board type is 0x%01x\n",roc_id,fadcSlotMask,board_type);
    }
    else if(board_type==0x2)
    {
      dcrbSlotMask = slotMask;
      if(dcrbSlotMask>0) printf("==> roc_id=%d, dcrbSlotMask is 0x%08x, board type is 0x%01x\n",roc_id,dcrbSlotMask,board_type);
    }
    else
    {
      printf("==> ERROR: unknown board type 0x%01x, or unspecified one - exit\n",board_type);
      exit(1);
    }
  }
  printf("\n");
  fclose(fdin);





  
  
  /*******************/
  /*get configuration*/
  printf("calling VTP_READ_CONF_FILE ..\n");fflush(stdout);

  //VTP_READ_CONF_FILE;

  vtpSetExpid(expid);
  vtpInitGlobals();
  if(dcrbSlotMask>0)
  {
    vtpSetV7(FW_FILENAME_V7_DCRB); /*default is FW_FILENAME_V7_FADC*/
    vtpSetRefClk(125); /*default is 250*/
  }
  vtpConfig("");
  if(strncasecmp(rol->confFile,"none",4)) vtpConfig(rol->confFile);






  /**/
  nstreams = 0;
  for(inst=0; inst<NUM_VTP_CONNECTIONS; inst++)
  {
    sprintf(stream_name[inst],"%s-s%d",myname, inst+1);
    printf("stream_name >%s<\n",stream_name[inst]);

    vtpMacAddress(stream_name[inst], vtp_mac[inst]);
    
    port_in = 0;
    codaGetStreamin(stream_name[inst], host, &port_in, host_in);
    printf("\ncodaGetStreamin: our_name >%s, <our_host >%s<, host_in >%s<, port_in=%d\n",stream_name[inst],host,host_in,port_in);
    if(port_in != 0) nstreams ++;
  }
  printf("Nstreams = %d\n",nstreams);


  /*check if we have anything*/
  slotMask = fadcSlotMask | dcrbSlotMask;
  if(slotMask==0)
  {
    printf("ERROR: there are no known board types in the crate - exit\n");
    exit(1);
  }
  else
  {
    printf("\n===== will use slotMask=0x%08x =====\n\n",slotMask);
  }

 
  /*assign slots to streams*/
  nslots = 0;
  for(slot=0; slot<21; slot++)
  {
    if( ((slotMask>>slot)&0x1)==1 )
    {
      payloads[nslots ++] = slot2payload[slot];
      printf("Slot=%2d -> Payload=%2d\n",slot,payloads[nslots-1]);
    }
  }
  printf("Nslots = %d\n",nslots);

  if(nstreams==1)
  {
    nslots_half = nslots;
    printf("One stream: nslots=%d, nslots_half=%d\n",nslots,nslots_half);

    for(i=0; i<nslots-1; i++)
    {
      vtpPayloadConfig(payloads[i],ppInfo,1,1,0,1);
      printf("(%2d)->payload=%2d reports to stream 1\n",i,payloads[i]);
    }
    ppmask = vtpPayloadConfig(payloads[nslots-1],ppInfo,1,1,0,1);
    printf("(%2d)->payload=%2d reports to stream 1\n",nslots-1,payloads[nslots-1]);
  }
  else if(nstreams==2)
  {
    nslots_half = nslots / 2;
    printf("Two streams: nslots=%d, nslots_half=%d\n",nslots,nslots_half);

    for(i=0; i<nslots_half; i++)
    {
      vtpPayloadConfig(payloads[i],ppInfo,1,1,0,1);
      printf("(%2d)->payload=%2d reports to stream 1\n",i,payloads[i]);
    }
    for(i=nslots_half; i<nslots-1; i++)
    {
      vtpPayloadConfig(payloads[i],ppInfo,1,1,0,2);
      printf("(%2d)->payload=%2d reports to stream 2\n",i,payloads[i]);
    }
    ppmask = vtpPayloadConfig(payloads[nslots-1],ppInfo,1,1,0,2);
    printf("(%2d)->payload=%2d reports to stream 2\n",nslots-1,payloads[nslots-1]);
  }
  else
  {
    printf("ERROR: nstreams=%d - return\n",nstreams);
    return;
  }

  /* configure payload ports 
        params:
            payload
            ppInfo[]
            module  - Module ID:  None=0, FADC250=1, MPD=2, ..., UNKNOWN=15
            lag (1-bonded lines, 0-four independent lines)
            bank    - Bank Info:  bits0-1   BankID for LANE1 or for a Bonded Link
                This is used by     bits8-9   BankID for LANE2
                the ROC Event       bits16-17 BankID for LANE3
                Builder             bits24-25 BankID for LANE4
            stream  - Stream Info stream number (1-4) where PP data will be output
                          otherwise 0 for triggered readout
 */



  /* enable Serdes for existing FADCs */
  vtpEnableTriggerPayloadMask(ppmask);


  /* Update the Streaming EB configuration for the new firmware to get the correct PP Mask and ROCID
     PP mask, nstreams, frame_len (ns), ROCID, ppInfo  */
  vtpStreamingSetEbCfg(ppmask, nstreams/*??? NUM_VTP_CONNECTIONS*/, 0xffff, roc_id, ppInfo);
  //emuData[4] = ROCID;  /* define ROCID in the EMU Connection data as well*/

  /* Enable the Streaming EB to allow Async Events. Disable Stream processing for the moment */
  stat = vtpStreamingEbEnable(VTP_STREB_ASYNC_FIFO_EN);
  if(stat != OK)
    printf("Error in vtpStreamingEbEnable()\n");

  // Reset the MIG - DDR memory write tagging - for Streaming Ebio 
  vtpStreamingMigReset();

  // Reset the data links between V7 Streaming EB and the Zync TCP client 
  // Set the Network output mode
  vtpStreamingEbioReset(netMode);

  /* establish connections */
  printf("LOOP OVER inst's\n");
  for(inst=0; inst<NUM_VTP_CONNECTIONS; inst++)
  {
    unsigned char ipaddr[4];
    unsigned char subnet[4];
    unsigned char gateway[4];
    unsigned char mac[6];
    unsigned char udpaddr[4], tcpaddr[4];
    unsigned int tcpport, udpport;
    unsigned int a[4], b[4];
    /*
    unsigned char ipaddr_[4];
    unsigned char subnet_[4];
    unsigned char gateway_[4];
    unsigned char tcpaddr_[4], udpaddr_[4];
    unsigned int tcpport_, udpport_;
    */
    printf("stream_name >%s<\n",stream_name[inst]);

    port_in = 0;
    codaGetStreamin(stream_name[inst], host, &port_in, host_in);
    printf("\ncodaGetStreamin: our_host >%s<, host_in >%s<, port_in=%d\n",host,host_in,port_in);

    if(port_in>0)
    {
      printf("port_in=%d - connecting\n",port_in);

      for(i=0; i<6; i++) mac[i] = vtp_mac[inst][i];
      /*
      printf("\n1 ipaddr_=%d.%d.%d.%d\n",ipaddr_[0],ipaddr_[1],ipaddr_[2],ipaddr_[3]);
      printf("1 subnet_=%d.%d.%d.%d\n",subnet_[0],subnet_[1],subnet_[2],subnet_[3]);
      printf("1 gateway_=%d.%d.%d.%d\n",gateway_[0],gateway_[1],gateway_[2],gateway_[3]);
      printf("1 mac=%02x:%02x:%02x:%02x:%02x:%02x\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
      printf("1 tcpaddr_=%d.%d.%d.%d\n",tcpaddr_[0],tcpaddr_[1],tcpaddr_[2],tcpaddr_[3]);
      printf("1 tcp_ports: local=%d, dest=%d\n\n",(tcpport_>>16)&0xFFFF,tcpport_&0xFFFF);
      */
      
    {
      struct hostent *hp, *gethostbyname();
      struct sockaddr_in sin;
      int s, slen;
      int socketnum;
      char *str;


      hp = gethostbyname(host);
      if(hp == 0 && (sin.sin_addr.s_addr = inet_addr(host)) == -1)
      {
	printf("unknown host >%s<\n",host);
	return;
      }
      str = inet_ntoa(*((struct in_addr *)hp->h_addr_list[0]));
      //printf("hp->h_addr >%s<\n",str);
      sscanf(str, "%d.%d.%d.%d", a, a+1, a+2, a+3);
      //printf("a[]= %d %d %d %d\n",a[0],a[1],a[2],a[3]);




      hp = gethostbyname(host_in);
      if(hp == 0 && (sin.sin_addr.s_addr = inet_addr(host_in)) == -1)
      {
	printf("unknown host_in >%s<\n",host_in);
	return;
      }
      /*The  inet_ntoa() function converts the Internet host address in, given in network byte order,
        to a string in IPv4 dotted-decimal notation.  The string is returned
	in a statically allocated buffer, which subsequent calls will overwrite.*/
      str = inet_ntoa(*((struct in_addr *)hp->h_addr_list[0]));
      //printf("hp->h_addr >%s<\n",str);
      sscanf(str, "%d.%d.%d.%d", b, b+1, b+2, b+3);
      //printf("b[]= %d %d %d %d\n",b[0],b[1],b[2],b[3]);




      /* our ip address */
      for(i=0; i<4; i++) ipaddr[i] = (unsigned char)a[i];

      /* our network mask and gateway */
      if( (ipaddr[2]>=160) && (ipaddr[2]<=163) )
      {
        subnet[0]=255; subnet[1]=255; subnet[2]=252; subnet[3]=0;
        gateway[0]=ipaddr[0]; gateway[1]=ipaddr[1]; gateway[2]=ipaddr[2]; gateway[3]=1;
      }
      else if(ipaddr[2]==167)
      {
        subnet[0]=255; subnet[1]=255; subnet[2]=255; subnet[3]=0;
        gateway[0]=ipaddr[0]; gateway[1]=ipaddr[1]; gateway[2]=ipaddr[2]; gateway[3]=99;
      }
      else if(ipaddr[2]==68)
      {
        subnet[0]=255; subnet[1]=255; subnet[2]=255; subnet[3]=0;
        gateway[0]=ipaddr[0]; gateway[1]=ipaddr[1]; gateway[2]=ipaddr[2]; gateway[3]=100;
      }
      else if(ipaddr[2]==179)
      {
        subnet[0]=255; subnet[1]=255; subnet[2]=255; subnet[3]=192;
        gateway[0]=ipaddr[0]; gateway[1]=ipaddr[1]; gateway[2]=ipaddr[2]; gateway[3]=66;
      }
      else
      {
        subnet[0]=255; subnet[1]=255; subnet[2]=255; subnet[3]=0;
        gateway[0]=ipaddr[0]; gateway[1]=ipaddr[1]; gateway[2]=ipaddr[2]; gateway[3]=1;
      }


      /* destination ip address */
      for(i=0; i<4; i++) tcpaddr[i] = (unsigned char)b[i];

      /* destination port */
      tcpport = port_in;
    }

	/*
{
#include <sys/ioctl.h>
#include <net/if.h> 
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>

    struct ifreq ifr;
    struct ifconf ifc;
    char buf[1024];
    int success = 0;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock == -1) {  };

    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if (ioctl(sock, SIOCGIFCONF, &ifc) == -1) {  }

    struct ifreq* it = ifc.ifc_req;
    const struct ifreq* const end = it + (ifc.ifc_len / sizeof(struct ifreq));

    for (; it != end; ++it) {
        strcpy(ifr.ifr_name, it->ifr_name);
        if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
            if (! (ifr.ifr_flags & IFF_LOOPBACK)) { // don't count loopback
                if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0) {
                    success = 1;
                    break;
                }
            }
        }
        else { }
    }

    unsigned char mac_address[6];

    if (success)
	{
      memcpy(mac_address, ifr.ifr_hwaddr.sa_data, 6);
      printf("\nMAC: %02x:%02x:%02x:%02x:%02x:%02x\n\n",
        mac_address[0],mac_address[1],mac_address[2],mac_address[3],mac_address[4],mac_address[5]);
	}
    else
	{
      printf("\nCANNOT GET MAC\n\n");
	}
}
*/

	/*
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr;

    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr->sa_family==AF_INET) {
            sa = (struct sockaddr_in *) ifa->ifa_netmask;
            addr = inet_ntoa(sa->sin_addr);
            printf("My Interface: %s\tAddress: %s\n", ifa->ifa_name, addr);
        }
    }
    freeifaddrs(ifap);
}


{
    char *addr = malloc(NI_MAXHOST);
    char *netmask = malloc(NI_MAXHOST);

    if (!get_addr_and_netmask_using_ifaddrs ("eth0", addr, netmask))
        printf("++ [%s] %s %s\n", __func__, addr, netmask);
    else
        printf("++ interface error.\n");

    free(addr);
    free(netmask);
}
	*/


      printf("\nset ipaddr=%d.%d.%d.%d\n",ipaddr[0],ipaddr[1],ipaddr[2],ipaddr[3]);
      printf("set subnet=%d.%d.%d.%d\n",subnet[0],subnet[1],subnet[2],subnet[3]);
      printf("set gateway=%d.%d.%d.%d\n",gateway[0],gateway[1],gateway[2],gateway[3]);
      printf("set mac=%02x:%02x:%02x:%02x:%02x:%02x\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
      printf("set tcpaddr=%d.%d.%d.%d\n",tcpaddr[0],tcpaddr[1],tcpaddr[2],tcpaddr[3]);
      printf("set tcp ports: local=%d, dest=%d\n\n",(tcpport>>16)&0xFFFF,tcpport&0xFFFF);

      /* set network configuration parameters */
      ret = vtpStreamingSetNetCfg(
          inst,
          netMode,
          ipaddr,
          subnet,
          gateway,
          mac,

	  tcpaddr,
	  tcpport,
	  localport
        );

      printf("\nvtpStreamingSetNetCfg returned %d\n\n",ret);fflush(stdout);
      if(ret<0) exit(1);
      
      /* read them back */
      ret = vtpStreamingGetNetCfg(
	  inst,
          ipaddr,
          subnet,
          gateway,
          mac,
	  
          udpaddr,
	  tcpaddr,
          &udpport,
	  &tcpport
      );

      printf("\nvtpStreamingSGetNetCfg returned %d\n\n",ret);fflush(stdout);

      printf("\nget ipaddr=%d.%d.%d.%d\n",ipaddr[0],ipaddr[1],ipaddr[2],ipaddr[3]);
      printf("get subnet=%d.%d.%d.%d\n",subnet[0],subnet[1],subnet[2],subnet[3]);
      printf("get gateway=%d.%d.%d.%d\n",gateway[0],gateway[1],gateway[2],gateway[3]);
      printf("get mac=%02x:%02x:%02x:%02x:%02x:%02x\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
      printf("get udpaddr=%d.%d.%d.%d\n",udpaddr[0],udpaddr[1],udpaddr[2],udpaddr[3]);
      printf("get tcpaddr=%d.%d.%d.%d\n",tcpaddr[0],tcpaddr[1],tcpaddr[2],tcpaddr[3]);
      printf("get udpport=0x%08x  tcp ports: local=%d, dest=%d\n\n",udpport,(tcpport>>16)&0xFFFF,tcpport&0xFFFF);

      printf("inst=%d\n",inst);
      vtpStreamingTcpConnect(inst, (netMode+1), emuData, N_EMUDATA);
    }
    else
    {
      printf("port_in=%d - NOT connecting\n",port_in);
    }

  }

  // Send a Prestart Event
  //vtpStreamingEvioWriteControl(0,EV_PRESTART,rol->runNumber,rol->runType);

  printf("INFO: User Prestart 1 executed\n");

  /* from parser (do we need that in rol2 ???) */
  *(rol->nevents) = 0;
  rol->recNb = 0;

  return;
}


static void
__go()
{
  int i, stat;
  char *env;

  if(vtpSerdesCheckLinks() == ERROR)
  {
    printf("ERROR","VTP Serdes links not up");
  }

  printf("Calling vtpSerdesStatusAll()\n");
  vtpSerdesStatusAll();


  vtpV7SetResetSoft(1);
  vtpV7SetResetSoft(0);
  //  vtpEbResetFifo();

  vtpStats(0);
  vtpSDPrintScalers();


  /*Send Go Event*/
  //vtpStreamingEvioWriteControl(0,EV_GO,0,0);

  /* Enable StreamingEB Stream processing */
  stat = vtpStreamingEbEnable(VTP_STREB_PP_STREAM_EN);
  if(stat != OK)
    printf("Error in vtpStreamingEbEnable()\n");


  printf("INFO: User Go 1 Enabling\n");
  CDOENABLE(VTP,1,1);
  printf("INFO: User Go 1 Enabled\n");

  return;
}

static void
__end()
{
  int ii, stat, inst, total_count, rem_count;
  unsigned int nFrames;

  CDODISABLE(VTP,1,0);

  vtpStats(0);

  sleep(2);  /* wait a bit to make sure the VTP sends all its data */




  /* Send an End Event */
  /* Enable StreamingEB AsyncFiFo processing */
  stat = vtpStreamingEbEnable(VTP_STREB_ASYNC_FIFO_EN);
  if(stat != OK)
    printf("Error in vtpStreamingEbEnable()\n");

  /*Send End Event to instance 0*/
  nFrames = vtpStreamingFramesSent(0);
  //vtpStreamingEvioWriteControl(0,EV_END,rol->runNumber,nFrames);
  printf("rocEnd: Wrote End Event (total %d frames)\n",nFrames);
  sleep(2);

  /* Disable Streaming EB - careful. If the User Sync is not high this can drop packets from a frame using UDP */
  vtpStreamingEbReset();




  /* Disconnect Streaming sockets */
  for(inst=0; inst<NUM_VTP_CONNECTIONS; inst++)
  {
    vtpStreamingTcpConnect(inst,0,0,0);
  }


  /* Reset all Socket Connections on the TCP Server - Server Mode only*/
  vtpStreamingTcpReset(0);


  vtpSDPrintScalers();



  printf("INFO: User End 1 Executed\n");

  return;
}

static void
__pause()
{
  CDODISABLE(VTP,1,0);

  printf("INFO: User Pause 1 Executed\n");

  return;
}


void
usrtrig(unsigned long EVTYPE, unsigned long EVSOURCE)
{
  return;

}

void
usrtrig_done()
{
  return;
}

void
__done()
{
  /* from parser */
  poolEmpty = 0; /* global Done, Buffers have been freed */

  /*printf("__done reached\n");*/


  /* Acknowledge TI */
  CDOACK(VTP,1,1);

  return;
}
  
static void
__status()
{
  return;
}  

/* This routine is automatically executed just before the shared libary
 *    is unloaded.
 *
 *       Clean up memory that was allocated 
 *       */
__attribute__((destructor)) void end (void)
{
  static int ended=0;

  if(ended==0)
    {
      printf("ROC Cleanup\n");

      ended=1;
    }

}

#else

void
vtp1_dummy()
{
  return;
}

#endif


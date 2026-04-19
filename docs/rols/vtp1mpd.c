
/* vtp1mpd.c - first readout list for VTP boards used as MPD readout (polling mode) */


#ifdef Linux_armv7l

#define DMA_TO_BIGBUF /*if want to dma directly to the big buffers*/


#define USE_MPD


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <sys/types.h>
#ifndef VXWORKS
#include <sys/time.h>
#endif

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

/* readout list VTP1MPD */
#define ROL_NAME__ "VTP1MPD"

/* polling */
#define POLLING_MODE


/* name used by loader */
#define INIT_NAME vtp1mpd__init


//#include "rolInt.h"
int codaConfigTableGetIP(char *config, char *name, char *ip_name);

#include "rol.h"
extern char configname[128]; /* coda_component.c */

void usrtrig(unsigned long, unsigned long);
void usrtrig_done();

/* vtp readout */
#include "VTP_source.h"

#ifdef USE_MPD
#include "vtp_roc_mpdro.c"
#endif

#define NUM_VTP_CONNECTIONS 1   /* */


/************************/
/************************/

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

void
rocStatus()
{
  extern int mpdOutputBufferCheck();

#ifdef USE_MPD
  /* Put out some Status' for debug */
  //mpdGStatus(0);
  vtpMpdPrintStatus(mpdGetVTPFiberMask(),0);
#endif

  vtpRocStatus(0);

#ifdef USE_MPD
  //mpdOutputBufferCheck();
#endif
}



static void
__download()
{
  int stat;
  char *ch, tmp[256];
  char buf[1000];


#ifdef POLLING_MODE
  rol->poll = 1;
#else
  rol->poll = 0;
#endif

  printf("\n>>>>>>>>>>>>>>> ROCID=%d, CLASSID=%d <<<<<<<<<<<<<<<<\n",rol->pid,rol->classid);
  printf("CONFFILE >%s<, USRSTRING >%s<, configname >%s<\n",rol->confFile,rol->usrString,configname);
  printf("name >%s<, tclName >%s<, listName >%s<\n\n",rol->name,rol->tclName,rol->listName);



  /* Clear some global variables etc for a clean start */
  CTRIGINIT;

  /* init trig source VTP */
  CDOINIT(VTP, 1);

  /************/
  /* init daq */

  daqInit();
  DAQ_READ_CONF_FILE;





  printf("calling VTP_READ_CONF_FILE_MPD ..\n");fflush(stdout);
  VTP_READ_CONF_FILE_MPD;
  //vtpSetExpid(expid); inside above




 /* print some connection info from the ROC */
  //sergey: figure it out later
  //printf(" **Info from ROC Connection Structure**\n");
  //printf("   ROC Type = %s\n", rol->rlinkP->type);
  //printf("   EMU name = %s\n", rol->rlinkP->name);
  //printf("   EMU IP   = %s\n", rol->rlinkP->net);
  //printf("   EMU port = %d\n", rol->rlinkP->port);

  /* Configure the ROC*/
  //sergey *(rol->async_roc) = 1;  // don't send Control events to the EB


  vtpRocReset(0);
  printf(" Set ROC ID = %d \n",rol->pid);
  vtpRocConfig(rol->pid, 0, 8, 0);  /* Use defaults for other parameters MaxRecSize, Max#Blocks, timeout*/
  //emuData[4] = rol->pid;  /* define ROCID=rol->pid in the EB Connection data as well*/


#ifdef USE_MPD
  printf("\nINFO: call vtpMpdDownload\n");
  vtpMpdDownload();
#endif

  rocStatus();


  printf("INFO: User Download 1 Executed\n");

  return;
}


static void
__prestart()
{
  int i, ret, nstreams, inst;
  unsigned long jj, adc_id, sl;
  char *myhost = getenv("HOST");
  char tmp[256];

  char *env, *myname, *ch;
  char conffilename[128], host[128], host_in[128], ipname[128];
  int port_in;


  unsigned char ipaddr[4];
  unsigned char subnet[4];
  unsigned char gateway[4];
  unsigned char mac[6];
  unsigned char tcpaddr[4];
  unsigned int tcpport;
  unsigned int a[4], b[4];

  struct hostent *hp, *gethostbyname();
  struct sockaddr_in sin;
  int s, slen;
  int socketnum;
  char *str;


  printf("PRESTART !!!!!!!!!!!!!!!!!\n");
  printf("PRESTART !!!!!!!!!!!!!!!!!\n");
  printf("PRESTART !!!!!!!!!!!!!!!!!\n");
  printf("PRESTART !!!!!!!!!!!!!!!!!\n");
  printf("PRESTART !!!!!!!!!!!!!!!!!\n");


  extern void vtpMpdApvConfigStatus();
  unsigned int emuip, emuport;

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

  vtpSetExpid(expid);


  //VTPflag = 0;

#if 0 //moved after connection
#ifdef USE_MPD
  sprintf(conffilename,"gem_config_apv_%s.txt",myname);
  printf("INFO: Calling vtpMpdPrestart(%s)\n",conffilename);
  vtpMpdPrestart(conffilename);
  /*
  printf("INFO: Call vtpMpdPrestart\n");
  if(rol->pid==49) vtpMpdPrestart("gem_config_apv_gem0vtp.txt");
  else if(rol->pid==52) vtpMpdPrestart("gem_config_apv_gem1vtp.txt");
  else {printf("ERROR: unknown rocid=%d - exit\n",rol->pid); exit(1);}
  */
#endif
#endif
  
  /* Reset the ROC */
  vtpRocReset(0);

  /* Initialize the TI Interface */
  vtpTiLinkInit();


  /**************************/
  /* start connecting to EB */

  /* get ipname from config table, it should be set by coda_roc */
  codaConfigTableGetIP(configname, rol->name, ipname);
  printf("ipname=>%s<\n",ipname);

  /* get mac address for 'ipname' */
  vtpMacAddress(ipname, mac);

  /* get port_in and host_in from 'links' table - we assumes that EB already updated it */
  strcpy(host,ipname);
  port_in = 0;
  codaLinksTableGetHostPort(ipname, host_in, &port_in);
  printf("\nPPPPP: our_name >%s<, our_host >%s<, host_in >%s<, port_in=%d\n",ipname,host,host_in,port_in);

  if(port_in>0)
  {
    printf("port_in=%d - connecting\n",port_in);

    hp = gethostbyname(host);
    if(hp == 0 && (sin.sin_addr.s_addr = inet_addr(host)) == -1)
    {
      printf("unknown host >%s<\n",host);
      return;
    }
    str = inet_ntoa(*((struct in_addr *)hp->h_addr_list[0]));
    printf("hp->h_addr >%s<\n",str);
    sscanf(str, "%d.%d.%d.%d", a, a+1, a+2, a+3);
    printf("a[]= %d %d %d %d\n",a[0],a[1],a[2],a[3]);

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
    printf("hp->h_addr >%s<\n",str);
    sscanf(str, "%d.%d.%d.%d", b, b+1, b+2, b+3);
    printf("b[]= %d %d %d %d\n",b[0],b[1],b[2],b[3]);

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

    printf("\nset: ipaddr=%d.%d.%d.%d\n",ipaddr[0],ipaddr[1],ipaddr[2],ipaddr[3]);
    printf("set: subnet=%d.%d.%d.%d\n",subnet[0],subnet[1],subnet[2],subnet[3]);
    printf("set: gateway=%d.%d.%d.%d\n",gateway[0],gateway[1],gateway[2],gateway[3]);
    printf("set: mac=%02x:%02x:%02x:%02x:%02x:%02x\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    printf("set: tcpaddr=%d.%d.%d.%d\n",tcpaddr[0],tcpaddr[1],tcpaddr[2],tcpaddr[3]);
    printf("set: tcp ports: local=%d, dest=%d\n\n",(tcpport>>16)&0xFFFF,tcpport&0xFFFF);

    /* set network configuration parameters */
    ret = vtpRocSetTcpCfg(ipaddr, subnet, gateway, mac, tcpaddr, tcpport);
    printf("\nvtpRocSetTcpCfg returned %d\n\n",ret);fflush(stdout);
    if(ret<0) exit(1);
      
    /* read them back */
    ret = vtpRocGetTcpCfg(ipaddr, subnet, gateway, mac, tcpaddr, &tcpport);
    printf("\nvtpRocGetTcpCfg returned %d\n\n",ret);fflush(stdout);

    printf("\nget: ipaddr=%d.%d.%d.%d\n",ipaddr[0],ipaddr[1],ipaddr[2],ipaddr[3]);
    printf("get: subnet=%d.%d.%d.%d\n",subnet[0],subnet[1],subnet[2],subnet[3]);
    printf("get: gateway=%d.%d.%d.%d\n",gateway[0],gateway[1],gateway[2],gateway[3]);
    printf("get: mac=%02x:%02x:%02x:%02x:%02x:%02x\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    printf("get: tcpaddr=%d.%d.%d.%d\n",tcpaddr[0],tcpaddr[1],tcpaddr[2],tcpaddr[3]);
    printf("get: tcp ports: local=%d, dest=%d\n\n",(tcpport>>16)&0xFFFF,tcpport&0xFFFF);

    /* Make the Connection . Pass Data needed to complete connection with the EMU */
#ifdef USE_MPD
    vtpRocTcpConnect(1,emuData,N_EMUDATA);
#else
    unsigned int emuData[8];
    vtpRocTcpConnect(1,emuData,0);
#endif
  }
  else
  {
    printf("port_in=%d - NOT connecting\n",port_in);
  }

  /* end connecting to EB */
  /************************/



  /* Reset and Configure the MIG and ROC Event Builder */
  vtpRocMigReset();


#ifdef USE_MPD
  sprintf(conffilename,"gem_config_apv_%s.txt",myname);
  printf("INFO: Calling vtpMpdPrestart(%s)\n",conffilename);
  vtpMpdPrestart(conffilename);
  /*
  printf("INFO: Call vtpMpdPrestart\n");
  if(rol->pid==49) vtpMpdPrestart("gem_config_apv_gem0vtp.txt");
  else if(rol->pid==52) vtpMpdPrestart("gem_config_apv_gem1vtp.txt");
  else {printf("ERROR: unknown rocid=%d - exit\n",rol->pid); exit(1);}
  */
#endif

#ifdef USE_MPD

  memset(ppInfo, 0, sizeof(ppInfo));

  /* The fibermask from vtp_mpdro.c */
  uint64_t fibermask = mpdGetVTPFiberMask();
  uint32_t lanemask = 0, payloadport = 0;
  uint32_t bankmask = 0;
  int ppmask=0;

  uint64_t iquad;
  int ifiber, slot, bankid = 1;

  for(iquad = 0; iquad < (VTP_MPD_MAX+3)/4; iquad++)
    {
      if(fibermask & ((unsigned long long)0xF << (iquad*4)))
	{
	  /* e.g. 0 quad -> fiber 0-3 -> VME slot 3 */
	  /* ... convert to payload port */
          slot = (iquad<8) ? (iquad+3) : (iquad+5);
	  payloadport = vmeSlot2vxsPayloadPort(slot);

	  bankmask = 0;
	  for(ifiber = 0; ifiber < 4; ifiber++)
	    {
	      if( (fibermask >> (iquad * 4ull) ) & (1 << ifiber))
		bankmask |= (bankid << (ifiber * 8));
	    }

	  /* Configure payloadport with MPD in lanes */
	  printf("Configure VME slot %2d (payload port %2d) with MPD bankmask 0x%08x\n",
		 slot, payloadport, bankmask);
	  ppmask |= vtpPayloadConfig(payloadport, ppInfo, 2,
			   0, bankmask);
	}
    }

  printf("vtpPayloadConfig ppmask = 0x%04x\n",ppmask);

#endif



  /* Initialize and program the ROC Event Builder*/
  vtpRocEbStop();

#ifdef USE_MPD
  vtpRocEbInit(VTPMPD_BANK,6,7);   // define bank1 tag = 3562, bank2 tag = 6, bank3 tag = 7
  vtpRocEbConfig(ppInfo,0);  // blocklevel=0 will skip setting the block level
#endif

  /* Reset the data Link between V7 ROC EB and the Zync FPGA ROC */
  vtpRocEbioReset();


  /* Set TI readout to Hardware mode */
  vtpTiLinkSetMode(1);

  /* Enable Async&EB Events for ROC   bit2 - Async, bit1 - Sync, bit0 V7-EB */
  vtpRocEnable(0x5);


  /* Print Run Number and Run Type */
  printf(" Run Number = %d, Run Type = %d \n",rol->runNumber,rol->runType);





  /*Send Prestart Event*/
  vtpRocEvioWriteControl(EV_PRESTART,rol->runNumber,rol->runType);


  //printf("sleeping ...\n");fflush(stdout);
  //sleep(10);
  //printf("... woke up\n");fflush(stdout);


  rocStatus();


#ifdef USE_MPD

  vtpMpdApvConfigStatus();

#endif






  /* Insert Config Files into User Event 137 */
  /* Pointer gymnastics ahead. */
//#define UEVENT137
#ifdef UEVENT137
  uint32_t *ueBuffer; /* User event buffer */
  int maxsize = 3 * 1024 * 1024;

  ueBuffer = malloc(maxsize + 1024*1024);
  if(ueBuffer == NULL)
    {
      perror("malloc");
      return;
    }

  /* Point the data pointer to the start of the buffer */
  uint32_t *uebufp = ueBuffer;
  uebufp += 2; /* Bump by 2 words for the Event Length and Header */

  unsigned int uetype = 137; /*  1/alpha  How has this not been taken yet? */
  int inum = 0, nwords = 0;

  /* Fill the buffer with a string bank of the file contents */
  nwords = rocFile2Bank(VTP_CONFIG_FILENAME,
			(uint8_t *)uebufp,
			ROCID, inum++, maxsize);
  if(nwords > 0)
    uebufp += nwords;

  nwords = rocFile2Bank(APV_TEMP_CONFIG,
			(uint8_t *)uebufp,
			ROCID, inum++, maxsize);
  if(nwords > 0)
    uebufp += nwords;
  nwords = rocFile2Bank(COMMON_MODE_FILENAME,
			(uint8_t *)uebufp,
			ROCID, inum++, maxsize);
  if(nwords > 0)
    uebufp += nwords;

  nwords = rocFile2Bank(PEDESTAL_FILENAME,
			(uint8_t *)uebufp,
			ROCID, inum++, maxsize);
  if(nwords > 0)
    uebufp += nwords;

  /* Where we are - where we were */
  uint32_t buffersize = (uint32_t)((char *) uebufp - (char *) ueBuffer);

  /* Evio Event Header */
  ueBuffer[0] = (buffersize >> 2) - 1;         /* Event Length */
  ueBuffer[1] = (uetype << 16) | (0x10 << 8) ; /* User Event of Banks */

  /* Send it to the EMU */
  vtpRocEvioWriteUserEvent(ueBuffer);

  free(ueBuffer);
#endif


  printf("INFO: User Prestart 1 executed\n");

  /* from parser (do we need that in rol2 ???) */
  *(rol->nevents) = 0;
  rol->recNb = 0;

  return;
}



static void
__go()
{
  int i, id;
  char *env;
  int chmask = 0;
#ifndef USE_MPD
  int blklevel;
#endif

#if 1
  //vtpSerdesStatusAll(); called below

  block_level = vtpTiLinkGetBlockLevel(0);
  printf("Setting VTP block level to: %d\n", block_level);

  //???
  //vtpSetBlockLevel(block_level);
  //vtpV7SetResetSoft(1);
  //vtpV7SetResetSoft(0);
  //???

  //sergey vtpEbResetFifo(); //???

  for(i=0; i<fnMPD; i++)
  {
    id = mpdSlot(i);
    printf("Calling mpdSetBlocklevel(%d,%d)\n",id,block_level);
    mpdSetBlocklevel(id,block_level);
  }
#endif


#ifdef USE_MPD
  printf("INFO: Call vtpMpdGo");
  vtpMpdGo();
#endif

  /* Clear TI Link recieve FIFO */
  vtpTiLinkResetFifo(1);

#ifdef CHECKSERDES
  chmask = vtpSerdesCheckLinks();
  printf("VTP Serdes link up mask = 0x%05x\n",chmask);

  printf("Calling vtpSerdesStatusAll()\n");
  vtpSerdesStatusAll();
#endif

  /* Get the current Block Level from the TI */
  blklevel = vtpTiLinkGetBlockLevel(1);
  printf("\nBlock level read from TI Link = %d\n", blklevel);

printf("11\n");fflush(stdout);
//  printf("Calling vtpSetBlockLevel(%d)\n",blklevel);
//  vtpSetBlockLevel(blklevel);
printf("12\n");fflush(stdout);

  /* Update the ROC EB blocklevel in the EVIO banks */
  vtpRocEbSetBlockLevel(blklevel);
printf("13\n");fflush(stdout);

  /* Start the ROC Event Builder */
  vtpRocEbStart();
printf("14\n");fflush(stdout);

  /*Send Go Event*/
  vtpRocEvioWriteControl(EV_GO, 0, *(rol->nevents));
printf("15\n");fflush(stdout);

  rocStatus();





  printf("INFO: User Go 1 Enabling\n");fflush(stdout);
  //CDOENABLE(VTP,1,1);
  CDOENABLE(VTP,1,0);
  printf("INFO: User Go 1 Enabled\n");fflush(stdout);

  return;
}



static void
__end()
{
  int ii, total_count, rem_count;
  unsigned int ntrig;
  unsigned long long nlongs;

  CDODISABLE(VTP,1,0);

#ifdef USE_MPD
  printf("INFO: Call vtpMpdEnd");
  vtpMpdEnd();
#endif

  /* Get total event information and set the Software ROC counters */
  ntrig = vtpRocGetTriggerCnt();
  *(rol->nevents) = ntrig;
  /*sergey
  *(rol->last_event) = ntrig;
  */

  /*Send End Event*/
  vtpRocEvioWriteControl(EV_END, rol->runNumber, *(rol->nevents));


  /* Disable the ROC EB */
  vtpRocEbStop();


  rocStatus();

  /* Disconnect the socket */
  vtpRocTcpConnect(0,0,0);


  /* Print final Stats */
  nlongs = vtpRocGetNlongs();
  /*sergey
  *(rol->totalwds) = nlongs;
  */
  printf(" TOTAL Triggers = %d   Nlongs = %llu\n",ntrig, nlongs);




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
  printf("usrtrig: SHOULD NEVER BE HERE !\n");fflush(stdout);

/* Right now this is a dummy routine as the trigger and readout is
   running in the FPGAs. In principle however the ROC can poll on
   some parameter which will allow it to enter this routine and the
   User can insert an asynchonous event into the data stream.

   Also the ROC can reqire that every trigger is managed by this routine.
   Esentially, one can force the FPGA to get an acknowledge of the trigger
   by the software ROC.

*/

  return;
}

void
usrtrig_done()
{
  printf("usrtrig_done: SHOULD NEVER BE HERE !\n");fflush(stdout);
  return;
}

void
__done()
{
  printf("__done: SHOULD NEVER BE HERE !\n");fflush(stdout);
  /* from parser */
  poolEmpty = 0; /* global Done, Buffers have been freed */

  /* Acknowledge TI */
  //CDOACK(VTP,1,1);
  CDOACK(VTP,0,0);

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

      vtpDmaMemClose();

      ended=1;
    }

}

#else

void
vtp1mpd_dummy()
{
  return;
}

#endif



/* tipci1.c - first readout list for TIPCI, runs on regular Linux */

/*
enter : 6
 TI register read......
 Reg 00, Value: 71e448eb 
 Reg 04, Value: 010149ff 
 Reg 08, Value: 000005d0 
 Reg 0c, Value: 0f000f00 
 Reg 10, Value: 80003fe0 
 Reg 14, Value: 01013f0a 
 Reg 18, Value: 22223f03 
 Reg 1c, Value: 00000011 
 Reg 20, Value: 33230490 
 Reg 24, Value: 23fe8004 
 Reg 28, Value: fe800000 
 Reg 2c, Value: 3505b001 
 Reg 30, Value: 35050000 
 Reg 34, Value: 8000eb22 
 Reg 38, Value: 01010101 
 Reg 3c, Value: 00800001 
 Reg 40, Value: 00000000 
 Reg 44, Value: 00000000 
 Reg 48, Value: 000a0000 
 Reg 4c, Value: 00eb0000 
 Reg 50, Value: 77000000 
 Reg 54, Value: 0319da05 
 Reg 58, Value: cb2daf00 
 Reg 5c, Value: 00000000 
 Reg 60, Value: 16009a27 
 Reg 64, Value: 5a000081 
 Reg 68, Value: 84800101 
 Reg 6c, Value: 47f78a19 
 Reg 70, Value: 1f8acb46 
 Reg 74, Value: fefdfbfa 
 Reg 78, Value: fefdfb55 
 Reg 7c, Value: fefdfb54 
 Reg 80, Value: fefdfb20 
 Reg 84, Value: fefd0000 
 Reg 88, Value: fefd0000 
 Reg 8c, Value: 00000000 
 Reg 90, Value: 00000000 
 Reg 94, Value: 00000001 
 Reg 98, Value: 6e200260 
 Reg 9c, Value: 00000000 
 Reg a0, Value: f9640d06 
 Reg a4, Value: 480355aa 
 Reg a8, Value: 0243c5bd 
 Reg ac, Value: 00000000 
 Reg b0, Value: 00000000 
 Reg b4, Value: 0000bfd7 
 Reg b8, Value: e4000000 
 Reg bc, Value: 00000000 
 Reg c0, Value: 00be0000 
 Reg c4, Value: 00000000 
 Reg c8, Value: 0000663c 
 Reg cc, Value: 00000000 
 Reg d0, Value: 00be0000 
 Reg d4, Value: 00000000 
 Reg d8, Value: 00000033 
 Reg dc, Value: 00000acb 
 Reg e0, Value: 61be0c36 
 Reg e4, Value: 00000000 
 Reg e8, Value: badadd53 
 Reg ec, Value: a2280001 
 Reg f0, Value: 00000000 
 Reg f4, Value: 0800031f 
 Reg f8, Value: 02330043 
 Reg fc, Value: 00000000 
 */

#define NEW


#define NEW_TI /* new TI */


#undef SSIPC

static int nusertrig, ndone;





/*********************************/
/* FOLLOWING DEFINED IN Makefile */

//// another file is used (srs1.c)
/////*enable/disable SRS readout*/
//////#define USE_SRS

/*enable/disable SAMPA readout*/
//#define USE_SAMPA

/*enable/disable VMM readout*/
//#define USE_VMM

/*enable/disable MAROC readout*/
//#define USE_MAROC

/*enable/disable PETIROC readout*/
//#define USE_PETIROC

/*enable/disable NALU readout*/
//#define USE_NALU

/*enable/disable HPS readout*/
//#define USE_HPS

/*********************************/
/*********************************/



#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>

#include <sys/time.h>


//#define DEBUG


#ifdef USE_HPS
//RTH
#include <RogueCoda.h>
#endif

#ifdef SSIPC
#include <rtworks/ipc.h>
#include "epicsutil.h"
static char ssname[80];
#endif

#include "circbuf.h"

/* from fputil.h */
#define SYNC_FLAG 0x20000000



/* polling mode if needed */
#define POLLING_MODE


/* main TI board */
#define TI_ADDR   (21<<19)  /* if 0 - default will be used, assuming slot 21*/








/* name used by loader */

#ifdef USE_VMM

#define ROL_NAME__ "VMM1"
#ifdef TI_MASTER
#define INIT_NAME vmm1_master__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME vmm1_slave__init
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME vmm1__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#elif USE_MAROC

#define ROL_NAME__ "MAROC1"
#ifdef TI_MASTER
#define INIT_NAME maroc1_master__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME maroc1_slave__init
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME maroc1__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#elif USE_PETIROC

#define ROL_NAME__ "PETIROC1"
#ifdef TI_MASTER
#define INIT_NAME petiroc1_master__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME petiroc1_slave__init
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME petiroc1__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#elif USE_NALU

#define ROL_NAME__ "NALU1"
#ifdef TI_MASTER
#define INIT_NAME nalu1_master__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME nalu1_slave__init
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME nalu1__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#elif USE_SRS

#define ROL_NAME__ "SRS1"
#ifdef TI_MASTER
#define INIT_NAME srs1_master__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME srs1_slave__init
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME srs1__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#elif USE_URWELL

#define ROL_NAME__ "URWELL1"
#ifdef TI_MASTER
#define INIT_NAME urwell1_master__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME urwell1_slave__init
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME urwell1__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#elif USE_HPS

#define ROL_NAME__ "HPSSVT1"
#ifdef TI_MASTER
#define INIT_NAME hpssvt1_master__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME hpssvt1_slave__init
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME hpssvt1__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#elif USE_SAMPA

#define ROL_NAME__ "SAMPA1"
#ifdef TI_MASTER
#define INIT_NAME sampa1_master__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME sampa1_slave__init
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME sampa1__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#else

#define ROL_NAME__ "TIPCI1"
#ifdef TI_MASTER
#define INIT_NAME tipci1_master__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME tipci1_slave__init
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME tipci1__init
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#endif






/*for uRwell readout, enable both SRS and MAROC (and VMM if used)*/
#ifdef USE_URWELL

#ifdef Linux_x86_64_RHEL9 // use srs on rhel9 only for now
#define USE_SRS
#endif

#define USE_MAROC

//#define USE_VMM

#endif






#include "rol.h"

void usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE);
void usrtrig_done();


/* TRIG_MODE = TIP_READOUT_EXT_POLL for polling mode (External front panel input) */
/*           = TIP_READOUT_TS_POLL  for polling mode (Fiber input from TS) */
/* Fix from Bryan 17may16 */
#define TRIG_MODE TIP_READOUT



#ifdef USE_SRS

#include "srsLib.h"
int srsFD[MAX_FEC];
char FEC[MAX_FEC][100];
int nfec=1;

#endif


#ifdef USE_SAMPA

#include "libdam.h"
#include "regs_map.h"
#include "sampaLib.h"

static int fd = 0;
static int fee = 7;

#define NFEE 8
#define MAXDATA 30000

#endif




#ifdef USE_VMM

#include "vmmLib.h"
#define MAXDATA 30000

#endif


#ifdef USE_MAROC

#include "marocLib.h"
#include "marocConfig.h"

#define SCALERS_PRINT             1
#define EVENT_DATA_PRINT          1
#define EVENT_STAT_PRINT          1

#define EVENT_BUFFER_NWORDS       1024

static unsigned long long nwords_current = 0;
static unsigned long long nwords_total = 0;
static unsigned long long nevents_current = 0;
static unsigned long long nevents_total = 0;
static int nmaroc = 0;

#endif



#ifdef USE_PETIROC

#include "petirocLib.h"
#include "petirocConfig.h"

#define EVENT_BUFFER_NWORDS       65536

static int npetiroc = 0;

#endif

#ifdef USE_NALU

//#include "naluLib.h"
//#include "naluConfig.h"

#define EVENT_BUFFER_NWORDS       65536

static int nnalu = 0;

#endif


#define TIR_SOURCE 1
#include "GEN_source.h"


static char rcname[5];

#define NBOARDS 22    /* maximum number of VME boards: we have 21 boards, but numbering starts from 1 */
#define MY_MAX_EVENT_LENGTH 3000/*3200*/ /* max words per board */
static unsigned int tdcbuf[65536];

/*#ifdef DMA_TO_BIGBUF*/
/* must be 'rol' members, like dabufp */
extern unsigned int dabufp_usermembase;
extern unsigned int dabufp_physmembase;
/*#endif*/

extern int rocMask; /* defined in roc_component.c */

#define NTICKS 1000 /* the number of ticks per second */
/*temporary here: for time profiling */





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

#ifdef USE_HPS
//RTH
struct RogueCodaData *rcd;
#endif

/*dummy to resolve reference*/
int
getTdcTypes(int *xxx)
{
  return(0);
}
int
getTdcSlotNumbers(int *xxx)
{
  return(0);
}

/*sergey: ??? */
extern struct TI_A24RegStruct *TIp;
/*sergey: ??? */
static int ti_slave_fiber_port = 1;




#ifdef USE_SAMPA
int
sampaSetTest(int value)
{
  int chip;
  int chan;
  uint16_t val = value;


  for(chip=0; chip<=7; chip++)
  {
    for(chan=0; chan<=31; chan++)
    {
      sampaChannelZeroSuppressionThresholdWrite(fee, chip, chan, val);
    }
  }



  /*
  for(chip=0; chip<=7; chip++)
  {
    sampaSetBypass(fee, chip, val);
    usleep(1000);
  }
  */

  return(0);
}
#endif









static void
__download()
{
  int i1, i2, i3;
  char ver[100];
  char *ch, tmp[64];

#ifdef USE_HPS
  //RTH
  printf("\n\n===\n===\n===\n=== creating svt rogue interface ===========================================\n");fflush(stdout);
  if ( (rcd = rogueCodaInit()) == NULL ) exit(-1);
  printf("done creating svt rogue interface ==========================================\n\n\n");fflush(stdout);
#endif

#ifdef POLLING_MODE
  rol->poll = 1;
#else
  rol->poll = 0;
#endif

  printf("download 1\n");fflush(stdout);


  printf("\n>>>>>>>>>>>>>>> ROCID=%d, CLASSID=%d <<<<<<<<<<<<<<<<\n",rol->pid,rol->classid);

  printf("CONFFILE >%s<\n\n",rol->confFile);
  printf("LAST COMPILED: %s %s\n", __DATE__, __TIME__);

  printf("USRSTRING >%s<\n\n",rol->usrString);

  /* if slave, get fiber port number from user string */
#ifdef TI_SLAVE
  ti_slave_fiber_port = 1; /* default */

  ch = strstr(rol->usrString,"fp=");
  if(ch != NULL)
  {
    strcpy(tmp,ch+strlen("fp="));
    printf("FP >>>>>>>>>>>>>>>>>>>>>%s<<<<<<<<<<<<<<<<<<<<<\n",tmp);
    ti_slave_fiber_port = atoi(tmp);
    printf("ti_slave_fiber_port =%d\n",ti_slave_fiber_port);
    tipusSetFiberIn_preInit(ti_slave_fiber_port);
  }
#endif


#ifdef USE_VMM
  tipusClockOutput_preInit(2); /*2 - output clock 41.667MHz*/

  /*pre-settings befors calling vmmInit() in Prestart*/
  vmmSetWindowParameters(78, 4);
  vmmSetGain(4);
  vmmSetThreshold(250/*250*/);


#endif

  /**/
  //printf("STATUS0:\n");
  //tipusStatus(1);

  CTRIGINIT;

  printf("STATUS1:\n");
  tipusStatus(1);

  CDOINIT(GEN,TIR_SOURCE);

  /************/
  /* init daq */

  daqInit();
  DAQ_READ_CONF_FILE;



  /*************************************/
  /* redefine TI settings if neseccary */

  tipusSetUserSyncResetReceive(1);




  printf("STATUS2:\n");
  tipusStatus(1);






  /*************************************/
  /* redefine TI settings if neseccary */

#ifndef TI_SLAVE
  /* TS 1-6 create physics trigger, no sync event pin, no trigger 2 */
vmeBusLock();

  tipusLoadTriggerTable(0);

vmeBusUnlock();
#endif

  /*********************************************************/
  /*********************************************************/


  /******************/
  /* USER code here */



#ifdef USE_SRS

  /*************************************************************
   * Setup SRS 
   */

  printf("STEP1 ==============================================================\n");

  /* here we set our IP address(es), suppose to be 10.0.x.2 */

  nfec=0;

  /* testing before run
  strncpy(FEC[nfec++], "10.0.0.2",100);
  strncpy(FEC[nfec++], "10.0.3.2",100);
  strncpy(FEC[nfec++], "10.0.8.2",100);
  */

  /* from Bryan on May 13, 2016 */

  if (rol->pid == 7) /* clondaq6 */
  {
    strncpy(FEC[nfec++], "10.0.0.2",100);
    strncpy(FEC[nfec++], "10.0.2.2",100);
    strncpy(FEC[nfec++], "10.0.3.2",100);
    strncpy(FEC[nfec++], "10.0.8.2",100);
  }
  else if (rol->pid == 8) /* clondaq8 */
  {
    strncpy(FEC[nfec++], "10.0.4.2",100);
    strncpy(FEC[nfec++], "10.0.5.2",100); 
    strncpy(FEC[nfec++], "10.0.6.2",100); 
    strncpy(FEC[nfec++], "10.0.7.2",100); 
  }
  else if (rol->pid == 85) /* clondaq9 */
  {
    strncpy(FEC[nfec++], "10.0.0.2",100);  /*our IP address*/
  }
  else
  {
    printf("ERROR1: SRS HAS WRONG PID=%d\n",rol->pid);
    exit(0);
  }

  char hosts[MAX_FEC][100];
  int ifec=0;
  int starting_port = 7000;

  memset(&srsFD, 0, sizeof(srsFD));


  printf("STEP2 ==============================================================\n");

  /* hosts[] here contains receiving computer IP address(es), it have to correcpond to FEC IP address(es) set on previous step;
     for example if FEC IP is 10.0.0.2, then computer IP must be 10.0.0.3 - make sure you configure computer local network as 10.0.0.3 */

  /* was when both were in clondaq6
 for(ifec=0; ifec<4; ifec++)
  {
    sprintf(hosts[ifec],"10.0.0.%d",ifec+3);
    srsSetDAQIP(FEC[ifec], hosts[ifec], starting_port+ifec);
    srsConnect((int*)&srsFD[ifec], hosts[ifec], starting_port+ifec);
  }

  for(ifec=4; ifec<nfec; ifec++)
  {
    sprintf(hosts[ifec],"10.0.4.%d",ifec-1);
    srsSetDAQIP(FEC[ifec], hosts[ifec], starting_port+ifec);
    srsConnect((int*)&srsFD[ifec], hosts[ifec], starting_port+ifec);
  }
  */


  /* Associate FECs with Specific Host IPs and ports */
  if (rol->pid == 7) /* clondaq6 */
  {
    for(ifec=0; ifec<nfec; ifec++)
    {
      sprintf(hosts[ifec],"10.0.0.%d",ifec+3);
      srsSetDAQIP(FEC[ifec], hosts[ifec], starting_port+ifec);
      srsConnect((int*)&srsFD[ifec], hosts[ifec], starting_port+ifec);
    }
  }
  else if (rol->pid == 8) /* clondaq8 */
  {
    for(ifec=0; ifec<nfec; ifec++)
    {
      sprintf(hosts[ifec],"10.0.0.%d",ifec+3);
      srsSetDAQIP(FEC[ifec], hosts[ifec], starting_port+ifec);
      srsConnect((int*)&srsFD[ifec], hosts[ifec], starting_port+ifec);
    }
  }
  else if (rol->pid == 85) /* clondaq9 */
  {
    for(ifec=0; ifec<nfec; ifec++)
    {
      printf("========== ifec=%d\n",ifec);
      sprintf(hosts[ifec],"10.0.0.%d",ifec+3); /*IP address on computer we will send data to*/
      printf("========== our FEC[ifec] is >%s<, computer hosts[ifec] is >%s<\n",FEC[ifec],hosts[ifec]);
      printf("========== befor srsSetDAQIP (starting_port=%d, ifec=%d)\n",starting_port,ifec);
      srsSetDAQIP(FEC[ifec], hosts[ifec], starting_port+ifec);
      printf("========== after srsSetDAQIP\n");
      srsConnect((int*)&srsFD[ifec], hosts[ifec], starting_port+ifec);
      printf("========== after srsConnect\n");
    }
  }
  else
  {
    printf("ERROR2: SRS HAS WRONG PID=%d\n",rol->pid);
    exit(0);
  }

  printf("STEP3 ==============================================================\n");

  /* Configure FEC */
  for(ifec=0; ifec<nfec; ifec++)
  {
    /* Same as call to 
    srsExecConfigFile("config/set_IP10012.txt"); */
    srsSetDTCC(FEC[ifec], 
		 1, // int dataOverEth
		 0, // int noFlowCtrl 
		 2, // int paddingType
		 0, // int trgIDEnable
		 0, // int trgIDAll
		 4, // int trailerCnt
		 0xaa, // int paddingByte
		 0xdd  // int trailerByte
		 );

    /* Same as call to 
	srsExecConfigFile("config/adc_IP10012.txt"); */
    srsConfigADC(FEC[ifec],
		   0xffff, // int reset_mask
		   0, // int ch0_down_mask
		   0, // int ch1_down_mask
		   0, // int eq_level0_mask
		   0, // int eq_level1_mask
		   0, // int trgout_enable_mask
		   0xffff // int bclk_enable_mask
		   );

    /* Same as call to 
	srsExecConfigFile("config/fecCalPulse_IP10012.txt"); */
    srsSetApvTriggerControl(FEC[ifec],
			      4, // 3 - test pulse mode, 4 - run mode
			      4, // how many time slots the APV chip is reading from its memory
                                 // for each trigger = (n+1)*3  
			      4, // 40000 ???
                                   // in test pulse mode: period of the trigger sequenser
                                   // in run mode: deadtime
                                   // NOTE: must be more than the DAQ time (datalength Nchannels)
			    0x66 /*0x4c*//*0x56*//*0x60*/, // int trgdelay 61(tage total sum) 5f(scintillator trigger) 60(MASTER OR) // 69 with Hodoscope
                                    // Orig was 61 : Rafo, In EEL it was 6c
			      0x7f, // not used in run mode ??? int tpdelay: Orig Value 0x7f  // Can try 0x80
			      0x8A // rosync: delay between the FEC trigger and the start of data recording. Default was 9f
                                   // Adjusted to capture correctly the APV data frames
                                   // 0x9f=3.975us  (Can try 0x6e)
			      );
    if(rol->pid == 8)
      {
	srsSetEventBuild(FEC[ifec],
			 0x1ff, // int chEnable
			 550, // int dataLength
			 2, // int mode
			 0, // int eventInfoType
			 0xaa000bb8 | ((ifec+4)<<16) // unsigned int eventInfoData
			 );
      }
    else if(rol->pid == 7)
      {
	srsSetEventBuild(FEC[ifec],
			 0x1ff, // int chEnable
			 550, // int dataLength
			 2, // int mode
			 0, // int eventInfoType
			 0xaa000bb8 | ((ifec)<<16) // unsigned int eventInfoData
			 );
      }
    else /* rol->pid == 85 */
      {
	srsSetEventBuild(FEC[ifec],
			 0xfff, //0xfff, //0xfffc,//0xfff/*1ff*/, // int chEnable // sergey: mask for front end cards connected
			 2500, // int dataLength // the number of 16-bit samples, 12bits used (1 sample=128 - what ???). 3 ts = 550, 6ts = 1000, 15ts=2500, 9ts = 1400,12ts = 2000, 27ts 4000
			 2, // int mode
			 0, // int eventInfoType
			 0xaa000bb8 | ((ifec)<<16) // unsigned int eventInfoData
			 );
      }
	
    /* Same as call to 
	srsExecConfigFile("config/apv_IP10012.txt"); */
    srsAPVConfig(FEC[ifec], 
		   0xff, // int channel_mask, 
		   0x03, // int device_mask,
		   0x19, // int mode, 
		   0x80, // int latency, 
		   0x2, // int mux_gain, 
		   0x62, // int ipre, 
		   0x34, // int ipcasc, 
		   0x22, // int ipsf, 
		   0x22, // int isha, 
		   0x22, // int issf, 
		   0x37, // int ipsp, 
		   0x10, // int imuxin, 
		   0x64, // int ical, 
		   0x28, // int vsps,
		   0x3c, // int vfs, 
		   0x1e, // int vfp, 
		   0xef, // int cdrv, 
		   0xf7 // int csel
		   );

    /* Same as call to 
	srsExecConfigFile("/daqfs/home/moffit/work/SRS/test/config/fecAPVreset_IP10012.txt"); */
    srsAPVReset(FEC[ifec]);

    /* Same as call to 
	srsExecConfigFile("config/pll_IP10012.txt"); */
    srsPLLConfig(FEC[ifec], 
		   0xff, // int channel_mask,
		   0x10, // int fine_delay,  (was 0, must be 0x10)===> Update: put it back to 0 to check if the RMS issue on 1st two channels will be fixed. Mor Update Kondo mentioned it must be 0x10, because with new FEC it should be 0x10, unlike the old version of the crate
		   0 // int trg_delay
		   );
      
  }

  printf("STEP4 ==============================================================\n");

  for(ifec=0; ifec<nfec; ifec++)
  {
    srsStatus(FEC[ifec],0);
  }
  
#endif /*USE_SRS*/




#ifdef USE_SAMPA
  /*
  fd = sampaInit();
  if(fd <= 0)
  {
    printf("ERROR in sampaInit() - exit\n");
    exit(0);
  }
  */
#endif /*USE_SAMPA*/


  sprintf(rcname,"RC%02d",rol->pid);
  printf("rcname >%4.4s<\n",rcname);

#ifdef SSIPC
  sprintf(ssname,"%s_%s",getenv("HOST"),rcname);
  printf("Smartsockets unique name >%s<\n",ssname);
  epics_msg_sender_init(getenv("EXPID"), ssname); /* SECOND ARG MUST BE UNIQUE !!! */
#endif

#ifdef USE_HPS
  //RTH
  if ( rogueCodaDownload(rcd,rol->confFile,rol->usrString) != 0 ) exit(-1);
#endif



  /*
tipusSetTriggerSource: WARN:  Only valid trigger source for TI Slave is HFBR5 (trig = 10)  Ignoring specified trig (3)
tipusSetTriggerSource: INFO: tipusTriggerSource = 0x410
  */















#if 0 /*ALL THAT MUST BE IN CONFIG FILE !!! */


/*sergey: following will disable fiber trigger source and enable front panel ts#5 */

/*master only*/
#ifndef TI_SLAVE

#ifdef SOFTTRIG

  /*pulser*/
  tipusSetTriggerSourceMask(TIPUS_TRIGSRC_LOOPBACK | TIPUS_TRIGSRC_PULSER);
  tipusSetBusySource(TIPUS_BUSY_LOOPBACK | TIPUS_BUSY_FP, 1);
  tipusSetSyncEventInterval(0);

#else

#if 1
  /*front panel*/
  tipusLoadTriggerTable(0);
  tipusSetTriggerSourceMask(TIPUS_TRIGSRC_LOOPBACK | TIPUS_TRIGSRC_TSINPUTS);
  tipusSetBusySource(TIPUS_BUSY_LOOPBACK | TIPUS_BUSY_FP, 1);
  tipusDisableTSInput(0x3F);

  //tipusEnableTSInput(0x1);
 tipusEnableTSInput(0x10); /* enable ts#5 */

  //tipusSetInputPrescale(5, 15); /*second par is same meaning as in internal pulser*/
  tipusSetTriggerHoldoff(1,4,1);
#else
  /*fiber*/
  tipusLoadTriggerTable(0);

  if(tipusGetSlavePort()==1)
    tipusSetTriggerSourceMask(TIPUS_TRIGSRC_LOOPBACK | TIPUS_TRIGSRC_HFBR1);
  else
    tipusSetTriggerSourceMask(TIPUS_TRIGSRC_LOOPBACK | TIPUS_TRIGSRC_HFBR5);

  //tipusSetBusySource(TIPUS_BUSY_LOOPBACK | TIPUS_BUSY_FP, 1);
  tipusDisableTSInput(0x3F);
  //tipusEnableTSInput(0x10);
  tipusSetTriggerHoldoff(1,4,1);
#endif

#endif /*#ifdef SOFTTRIG*/


#endif /*ifndef TI_SLAVE*/



#endif /* #if 0*/


























  logMsg("INFO: User Download Executed\n",1,2,3,4,5,6);
}



static void
__prestart()
{
  int ii, i1, i2, i3;
  int ret, chip;

  /* Clear some global variables etc for a clean start */
  *(rol->nevents) = 0;
  event_number = 0;
  /* was tiEnableVXSSignals();*/

#ifdef POLLING_MODE
  CTRIGRSS(GEN, TIR_SOURCE, usrtrig, usrtrig_done);
#else
  CTRIGRSA(GEN, TIR_SOURCE, usrtrig, usrtrig_done);
#endif

  printf(">>>>>>>>>> next_block_level = %d, block_level = %d, use next_block_level\n",next_block_level,block_level);
  block_level = next_block_level;


  /**************************************************************************/
  /* setting TI busy conditions, based on boards found in Download          */
  /* tiInit() does nothing for busy, tiConfig() sets fiber, we set the rest */
  /* NOTE: if ti is busy, it will not send trigger enable over fiber, since */
  /*       it is the same fiber and busy has higher priority                */

vmeBusLock();
#ifndef TI_SLAVE
  tipusSetBusySource(TIPUS_BUSY_LOOPBACK | TIPUS_BUSY_FP, 1);
#else
  ////tipusSetBusySource(TIPUS_BUSY_LOOPBACK | TIPUS_BUSY_HFBR1, 1); felix does not issues busy yet
  //tipusSetBusySource(TIPUS_BUSY_LOOPBACK, 1);
  tipusSetBusySource(0, 1);
#endif
vmeBusUnlock();






#ifdef TI_SLAVE

vmeBusLock();

#if 0
#if 1
  /*??? why TIPUS_SYNC_LOOPBACK ???*/
  if(tipusGetSlavePort()==1)
  {
    printf("TIpcie: Setting SyncSrc to HFBR1 and Loopback\n");
    tipusSetSyncSource(TIPUS_SYNC_HFBR1 | TIPUS_SYNC_LOOPBACK);
  }
  else
  {
    printf("TIpcie: Setting SyncSrc to HFBR5 and Loopback\n");
    tipusSetSyncSource(TIPUS_SYNC_HFBR5 | TIPUS_SYNC_LOOPBACK);
  }
#endif
#endif


  tipusSetInstantBlockLevelChange(3); /*???*/

#if 0
  /*sergey: tipusSetSyncSource() resets register 0x24 (sync), so we'll loose UserSyncReset enabling, reinstall it*/
  tipusSetUserSyncResetReceive(1);
#endif

vmeBusUnlock();

#endif







#ifdef USE_SRS

 int ifec=0, nframes=0, dCnt=0;
 printf("SRS Prestarting ..\n");fflush(stdout);
 for(ifec=0; ifec<nfec; ifec++)
 {
   srsStatus(FEC[ifec],0);
 }
 
 /* Check SRS buffers and clear them */
 dCnt = srsCheckAndClearBuffers(srsFD, nfec,
				(volatile unsigned int *)tdcbuf,
				2*80*1024, 1, &nframes);
 if(dCnt>0)
 {
    printf("SYNC ERROR: SRS had extra data at SyncEvent.\n");
  }
 
#endif /* USE_SRS */




  /*
  if(nfadc>0)
  {
    printf("Set BUSY from SWB for FADCs\n");
vmeBusLock();
    tiSetBusySource(TI_BUSY_SWB,0);
vmeBusUnlock();
  }
  */



  /* USER code here */
  /******************/

#ifdef USE_HPS
  //RTH
  if ( rogueCodaPrestart(rcd) != 0 ) exit(-1);
#endif





#if 0 /* FOR NOW */


vmeBusLock();
  tipusIntDisable();
vmeBusUnlock();


  /* master and standalone crates, NOT slave */
#ifndef TI_SLAVE

  sleep(1);
vmeBusLock();
  tipusSyncReset(1);
vmeBusUnlock();
  sleep(1);
vmeBusLock();
  tipusSyncReset(1);
vmeBusUnlock();
  sleep(1);
vmeBusLock();
  ret = tipusGetSyncResetRequest();
vmeBusUnlock();
  if(ret)
  {
    printf("ERROR: syncrequest still ON after tiSyncReset(); trying again\n");
    sleep(1);
vmeBusLock();
    tipusSyncReset(1);
vmeBusUnlock();
    sleep(1);
  }
vmeBusLock();
  ret = tipusGetSyncResetRequest();
vmeBusUnlock();


  if(ret)
  {
    printf("ERROR: syncrequest still ON after tiSyncReset(); try 'tcpClient <rocname> tiSyncReset'\n");
  }
  else
  {
    printf("INFO: syncrequest is OFF now\n");
  }
  /*FOR NOW
  printf("holdoff rule 1 set to %d\n",tiGetTriggerHoldoff(1));
  printf("holdoff rule 2 set to %d\n",tiGetTriggerHoldoff(2));
  */

#endif

/* set block level in all boards where it is needed;
   it will overwrite any previous block level settings */

/* Fix from Bryan 17may16 */
#ifndef TI_SLAVE /* TI-slave gets its blocklevel from the TI-master (through fiber) */
vmeBusLock();
  tipusSetBlockLevel(block_level);
vmeBusUnlock();
#endif






#endif




vmeBusLock();
  tipusStatus(1);
vmeBusUnlock();




#ifdef USE_SAMPA

  fd = sampaInit();
  if(fd <= 0)
  {
    printf("ERROR in sampaInit() - exit\n");
    exit(0);
  }

  /* set time window width */
  /*56 samples + 8 other words = 64 words per channel; 64 x 8chips x 32channels = 16384 (0x4000) words which is FIFO size*/
  sampaSetTimeWindowWidth(30/*56*/); /*do NOT exceed 56 !!!*/
  sampaGetTimeWindowWidth();

  /*for MM test in EEL/125, if offset=30, peak around 15*/
  sampaSetTimeWindowOffset(/*30*/160); /*do NOT exceed 192 !!! 30 for EEL, ?? for the svt1&svt2 trigger in the hall*/
  sampaGetTimeWindowOffset();

  /*
  window | status
  6        55550e00
  7        55550f00
  8        55551000
  9        55551100
  .................
 16        55551800
 32        55552800 (run 25)
 48        55553800 (run 27)
 56        55554000 (run 28) <- maximum possible with current fifo size
 57        55554001 (missing 2 channels, run 29)
 80        55554001 (missing about 25% of channels, run 24)
160        55554001 (missing about half of channels, run 23)
  */


  //run 3452 - 20mv/fc, trim=0
  //run 3515 - 20mv/fc, trim=4
  //run 3558 - 20mv/fc, trim=7
  //run 3560 - 30mv/fc, trim=0
  //run 3563 - 30mv/fc, trim=4
  //run 3566 - 30mv/fc, trim=7
  //run 3567 - 30mv/fc, trim=7, 4 connectors
  //run 3572 - 30mv/fc, trim=0, 4 connectors

  for(chip=0; chip<=7; chip++)
  {
    int trim = 0; //0-smallest gain, 4-default, 7-biggest gain
    sampaSetADCVoltageReferenceTrim(fee, chip, trim);
  }
  sleep(1);
  for(chip=0; chip<=7; chip++)
  {
    int trim;
    trim = sampaGetADCVoltageReferenceTrim(fee, chip);
    printf("[chip=%d] Trim=%d\n",chip,trim);    
  }
  sleep(1);

  //sampaSetTest(75);

#endif /*USE_SAMPA*/


#ifdef USE_VMM

  ret = vmmInit();
  if(ret<0) exit(0);

#endif


#ifdef USE_MAROC

#ifdef Linux_x86_64_RHEL9 // start from ip=...10 for rhel9 only for now
  marocSetIPStart(10);
  marocGetIPStart();
#endif
  
  nmaroc = marocInit(0, MAROC_MAX_NUM/*2*/);
  if(nmaroc<0) exit(0);
  marocInitGlobals();
  marocConfig("");

#endif





#ifdef USE_PETIROC
#if 1
  npetiroc = petirocInit(0, PETIROC_MAX_NUM, PETIROC_INIT_REGSOCKET);
  if(npetiroc<0) exit(0);
  printf("npetiroc=%d\n",npetiroc);
  petirocInitGlobals();
  petirocConfig("");
vmeBusLock();
  tipusStatus(1);
vmeBusUnlock();
#endif
#endif

#ifdef USE_NALU
#if 0
  npetiroc = petirocInit(0, PETIROC_MAX_NUM, PETIROC_INIT_REGSOCKET);
  if(npetiroc<0) exit(0);
  printf("npetiroc=%d\n",npetiroc);
  petirocInitGlobals();
  petirocConfig("");
vmeBusLock();
  tipusStatus(1);
vmeBusUnlock();
#endif
#endif


  printf("INFO: Prestart1 Executed\n");fflush(stdout);

  *(rol->nevents) = 0;
  rol->recNb = 0;

  return;
}       




static void
__pause()
{
  CDODISABLE(GEN,TIR_SOURCE,0);
  logMsg("INFO: Pause Executed\n",1,2,3,4,5,6);
  
} /*end pause */




static void
__go()
{
  int ii, jj, id, slot;

  logMsg("INFO: Entering Go 1\n",1,2,3,4,5,6);


#ifndef TI_SLAVE
vmeBusLock();
  /* set sync event interval (in blocks) */
  tipusSetSyncEventInterval(0/*10000*//*block_level*/);
vmeBusUnlock();

/* Fix from Bryan 17may16 */
#else
/* Block level was broadcasted from TI-Master / TS */
  block_level = tipusGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",block_level);
  tipusSetBlockBufferLevel(0);
#endif


#ifdef USE_HPS
  //RTH
  printf("before rogueCodaGo\n");
  if ( rogueCodaGo(rcd,rol->runNumber) != 0 ) exit(-1);
#endif


#ifdef USE_SRS
  int ifec=0;

  for(ifec=0; ifec<nfec; ifec++)
  {
    srsTrigEnable(FEC[ifec]);
    srsStatus(FEC[ifec],0);
  }
#endif /*USE_SRS*/



#ifdef USE_SAMPA

  /*in 'Go' we clean fifo*/
  //dam_register_write(fd, PHY_RESET, 0x10);
  //usleep(1000);
  //dam_register_write(fd, PHY_RESET, 0x0);

#endif /*USE_SAMPA*/


#ifdef USE_VMM

  vmmEnable();

#endif /*USE_VMM*/


#ifdef USE_MAROC

  //set block_level and latency here: maroc_setup_readout(int devid, int lookback, int window)

  marocEnable();

#endif /*USE_MAROC*/

#ifdef USE_PETIROC
#if 1
  petiroc_set_blocksize_all(block_level);
      //faSetBlockLevel(slot, block_level);
  //set block_level and latency here: petiroc_setup_readout(int devid, int lookback, int window)
  petirocEnable();
#endif
#endif /*USE_PETIROC*/

#ifdef USE_NALU
#if 0
  petiroc_set_blocksize_all(block_level);
      //faSetBlockLevel(slot, block_level);
  //set block_level and latency here: petiroc_setup_readout(int devid, int lookback, int window)
  petirocEnable();
#endif
#endif /*USE_NALU*/


  tipusStatus(1);

  nusertrig = 0;
  ndone = 0;

  CDOENABLE(GEN,TIR_SOURCE,0); /* bryan has (,1,1) ... */

  logMsg("INFO: Go 1 Executed\n",1,2,3,4,5,6);
}

static void
__end()
{
  int iwait=0;
  int blocksLeft=0;
  int id;
  int ifec=0;

  printf("\n\nINFO: End1 Reached\n");fflush(stdout);

  CDODISABLE(GEN,TIR_SOURCE,0);

  /* Before disconnecting... wait for blocks to be emptied */
vmeBusLock();
  blocksLeft = tipusBReady();
vmeBusUnlock();


//printf("11-1\n");tipusStatus(1);printf("11-2\n");

  printf(">>>>>>>>>>>>>>>>>>>>>>> %d blocks left on the TI\n",blocksLeft);fflush(stdout);
  if(blocksLeft)
  {
    //printf("12-1\n");tipusStatus(1);printf("12-2\n");
    printf(">>>>>>>>>>>>>>>>>>>>>>> before while ... %d blocks left on the TI\n",blocksLeft);fflush(stdout);
    while(iwait < 10)
    {
      sleep(1);
      //printf("13-1\n");tipusStatus(1);printf("13-2\n");
      if(blocksLeft <= 0) break;
vmeBusLock();
      blocksLeft = tipusBReady();
      printf(">>>>>>>>>>>>>>>>>>>>>>> inside while ... %d blocks left on the TI\n",blocksLeft);fflush(stdout);
vmeBusUnlock();
//printf("14-1\n");tipusStatus(1);printf("14-2\n");
      iwait++;
    }
    printf(">>>>>>>>>>>>>>>>>>>>>>> after while ... %d blocks left on the TI\n",blocksLeft);fflush(stdout);
    //printf("15-1\n");tipusStatus(1);printf("15-2\n");
  }

  //printf("99-1\n");tipusStatus(1);printf("99-2\n");


#ifdef USE_HPS
  //RTH
  rogueCodaEnd(rcd);
#endif

#ifdef USE_SRS
  for(ifec=0; ifec<nfec; ifec++)
  {
    srsTrigDisable(FEC[ifec]);
    srsStatus(FEC[ifec],0);
  }
#endif /*USE_SRS*/


#ifdef USE_SAMPA

#endif /*USE_SAMPA*/


#ifdef USE_VMM

  vmmDisable();

#endif /*USE_VMM*/


#ifdef USE_MAROC

  marocEnd(); //close sockets, opened by marocInit() in Prestart

#endif /*USE_MAROC*/


#ifdef USE_PETIROC
#if 1
  petirocEnd(); //close sockets, opened by petirocInit() in Prestart
#endif
#endif /*USE_PETIROC*/

#ifdef USE_NALU
#if 0
  petirocEnd(); //close sockets, opened by petirocInit() in Prestart
#endif
#endif /*USE_NALU*/


#if 0
  printf("SWITCH TIPCI TO INTERNAL CLOCK IN 'END'\n");fflush(stdout);
vmeBusLock();
  tipusSetClockSource(0); /*switching TI to internal clock - until fixed in firmware*/
  tipusStatus(1);
vmeBusUnlock();
#endif

//printf("999\n");
//tipusStatus(1);

  printf("INFO: End1 Executed\n\n\n");fflush(stdout);

  return;
}






void
usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE)
{
  int *jw, ind, ind2, i, ii, iii, jj, jjj, blen, len, rlen, nbytes, ret, chip;
  int rcdRet, tmp;
  unsigned int *tdc, *blk_hdr, *evt_hdr;
  unsigned int *dabufp1, *dabufp2;
  int njjloops, slot;
  int nwords, dCnt;
  int nMultiSamples;
  TIMERL_VAR;
  char *chptr, *chptr0;
  int ifec=0;
  int nframes=0;
  int SRS_FEC_BASE=0;
  static unsigned int event_num = 0, delay = 0;  
  //printf("EVTYPE=%d syncFlag=%d\n",EVTYPE,syncFlag);

  // RTH For emulation
  if(syncFlag) printf("EVTYPE=%d syncFlag=%d\n",EVTYPE,syncFlag);

  rol->dabufp = (int *) 0;

  /*
usleep(100);
  */
  /*
  sleep(1);
  */

  CEOPEN(EVTYPE, BT_BANKS); /* reformatted on CODA_format.c !!! */

  if((syncFlag<0)||(syncFlag>1))         /* illegal */
  {
    printf("Illegal1: syncFlag=%d EVTYPE=%d\n",syncFlag,EVTYPE);
  }
  else if((syncFlag==0)&&(EVTYPE==0))    /* illegal */
  {
    printf("Illegal2: syncFlag=%d EVTYPE=%d\n",syncFlag,EVTYPE);
  }
  else if((syncFlag==1)&&(EVTYPE==0))    /* force_sync (scaler) events */
  {
    ;
/*
!!! we are geting here on End transition: syncFlag=1 EVTYPE=0 !!!
*/
  }
  else if((syncFlag==0)&&(EVTYPE==15)) /* helicity strob events */
  {
    ;
  }
  else           /* physics and physics_sync events */
  {



    /*************/
    /* TI stuff */

    /* Set high, the first output port 
    tiSetOutputPort(1,0,0,0);
    */

#if 0
TIMERL_START;
#endif



    /* Grab the data from the TI */
    len = tipusReadBlock(tdcbuf,1024,0);


#if 0
TIMERL_STOP(5000/block_level,1000+rol->pid);
#endif

    if(len<=0)
    {
      printf("ERROR in tipReadBlock : No data or error, len = %d\n",len);
      sleep(1);
      if(TRIG_MODE==TIPUS_READOUT_EXT_POLL) tipusSetBlockLimit(1); /* Effectively... stop triggers */
    }
    else
    {

#ifdef DEBUG
      printf("tip: len=%d\n",len);
      for(jj=0; jj<len; jj++) printf("tip[%2d] 0x%08x\n",jj,tdcbuf[jj]);
#endif

      BANKOPEN(0xe10A,1,rol->pid);
      for(jj=0; jj<len; jj++) *rol->dabufp++ = tdcbuf[jj];
      BANKCLOSE;

    }
    /* Turn off all output ports 
    tiSetOutputPort(0,0,0,0);
    */

    /* TI stuff */
    /*************/





    /***************/
    /* HPS READOUT */

#ifdef USE_HPS

    // RTH
    //printf("\n===> befor rogueCodaTrigger rcdRet=%d\n",rcdRet);
    rcdRet = rogueCodaTrigger(rcd,tdcbuf,len);
    //printf("\n===> after rogueCodaTrigger rcdRet=%d\n",rcdRet);
    //printf("\n===> rcdRet=%d\n",rcdRet);
    if(rcdRet<=0)
    {
      printf("ERROR: rogueCodaTrigger returned %d, skipping the rest of rogue\n",rcdRet);
      goto skip_hps;
    }

#ifdef DEBUG
    printf("\n===> rcdRet=%d\n",rcdRet);
    for(ii=0; ii<rcdRet; ii++) printf(" -svtdata[%6d] = 0x%08x\n",ii,tdcbuf[ii]);
#endif



    //RTH
    BANKOPEN(0xe130,1,rol->pid);


    /* block header */
    blk_hdr = rol->dabufp; /* remember block header location */
    *rol->dabufp ++ = (0x10<<27) + (block_level&0xFF);
#ifdef DEBUG
    printf("BLOCK HEADER = 0x%08x\n",*(rol->dabufp-1));
#endif

    /* read block of data event-by-event */
    for(ii=0; ii<rcdRet; ii++)
    {

      /* event header */
      evt_hdr = rol->dabufp; /* remember event header location */
      *rol->dabufp ++ = (0x12<<27);
#ifdef DEBUG
      printf("EVENT HEADER = 0x%08x\n",*(rol->dabufp-1));
#endif

#ifdef DEBUG
      printf("  befor rogueCodaEvent\n");
#endif

#if 0
TIMERL_START;
#endif

      //printf("  befor rogueCodaEvent\n");
      len = rogueCodaEvent(rcd,rol->dabufp,ii);
      //printf("  after rogueCodaEvent\n");

#if 0
TIMERL_STOP(100000/block_level,1000+rol->pid);
#endif

      if(len<3) printf("ERROR: rogueCodaEvent returns len=%d\n",len);

      /* add 'len' to event header */
      tmp = *evt_hdr;
      tmp |= (len&0x7FFFFFF);
      *evt_hdr = tmp;

#ifdef DEBUG
      printf("  after rogueCodaEvent\n");
      printf("  [%2d] len=%d\n",ii,len);
      for(jj=0; jj<len; jj++) printf("    data [%3d] = 0x%08x\n",jj,rol->dabufp[jj]);      
#endif

      /* extract the number of samples from tails and set them in data headers */
      iii=len-1;
      while(iii>=0)
      {
        jjj = iii; /* last tail word, set tail tag */
        tmp = *(rol->dabufp+jjj);
        tmp |= (0x15<<27);
        *(rol->dabufp+jjj) = tmp;

        jjj = iii - 3; /* index of the 'nMultiSamples' (first tail word) */
        nMultiSamples = (*(rol->dabufp+jjj)) & 0xFFF;

        jjj = jjj - nMultiSamples*4 - 1; /* data header index, add data tag and nMultiSamples */
        tmp = *(rol->dabufp+jjj);
        tmp |= (0x14<<27);
		
        if(nMultiSamples != ((( (tmp>>8)&0xFFFF )-32)/16) )
	{
          printf("ERROR: nMultiSamples=%d from tail does not consistent with the number of bytes = %d\n",nMultiSamples,tmp&0x7FFFF);
	  /*
          tmp = tmp & 0xFF;
          tmp |= (nMultiSamples<<8);
	  */
	}
        
        *(rol->dabufp+jjj) = tmp;

#ifdef DEBUG
        printf("nMultiSamples from [%d] is %d, data header index is %d, value=0x%08x\n",iii-3,nMultiSamples,jjj,*(rol->dabufp+jjj));
#endif

        jjj = jjj - 2; /* timestamp index, add timestamp tag */
        tmp = *(rol->dabufp+jjj);
        tmp |= (0x13<<27);
        *(rol->dabufp+jjj) = tmp;

        jjj = jjj - 1; /* builder header index, add builder header tag */
        tmp = *(rol->dabufp+jjj);
        tmp |= (0x16<<27);
        *(rol->dabufp+jjj) = tmp;

        iii = jjj - 1;
      }
#ifdef DEBUG
      for(jj=0; jj<len+1; jj++) printf("    corr data [%3d] = 0x%08x\n",jj,rol->dabufp[jj-1]);
#endif
      rol->dabufp += len;
    }
    

    nwords = ((long int)rol->dabufp-(long int)blk_hdr)/4+1;
#ifdef DEBUG
    printf("nwords=%d\n",nwords);
#endif

    /* filler(s) if needed */

    /* block trailer */
    *rol->dabufp ++ = ( (0x11<<27) + (nwords&0x3FFFFF) );
#ifdef DEBUG
    printf("    corr data [%3d] = 0x%08x\n",len+1,*(rol->dabufp-1));
#endif

    BANKCLOSE;


skip_hps:


#ifdef DEBUG
    printf("hpssvt1: end svt readout\n");fflush(stdout);
#endif

#endif /* ifdef USE_HPS */






/*usleep(1);*/ /*1-55us, 50-104us, 70-126us*/


#ifdef USE_MAROC
  len = marocReadBlock(tdcbuf, EVENT_BUFFER_NWORDS);
  if(len > 0)
  {
//    printf("maroc: len=%d\n",len);fflush(stdout);
    
    //marocPrintBlock((volatile unsigned int *)tdcbuf, len);
    BANKOPEN(0xe136,1,rol->pid);
    for(jj=0; jj<len; jj++) *rol->dabufp++ = tdcbuf[jj];
    BANKCLOSE;    
  }
  else
  {
    printf("ERROR: marocReadBlock() returned %d\n",len);fflush(stdout);
    //exit(-1);
  }
#endif /*USE_MAROC*/



#ifdef USE_PETIROC
  len = petirocReadBlock(tdcbuf, EVENT_BUFFER_NWORDS);
  if(len > 0)
  {
//    printf("petiroc: len=%d\n",len);fflush(stdout);
//    for(jj=0; jj<len; jj++) printf("%d: %08X\n", jj, tdcbuf[jj]);    

    BANKOPEN(0xe138,1,rol->pid);
    for(jj=0; jj<len; jj++) *rol->dabufp++ = tdcbuf[jj];
    BANKCLOSE;    
  }
  else
  {
    printf("\nERROR in tipci1: petirocReadBlock() returned %d\n\n",len);fflush(stdout);
    //exit(-1);
    sleep(1);
  }
#if 0
if(!(event_num % 10))
{
  printf("event %d\n", event_num);
  for(jj=0; jj<15; jj++)
    petiroc_get_idelay_status(jj);
}
event_num++;
#endif
#endif /*USE_PETIROC*/


#ifdef USE_NALU
//  len = petirocReadBlock(tdcbuf, EVENT_BUFFER_NWORDS);
//  if(len > 0)
//  {
////    printf("petiroc: len=%d\n",len);fflush(stdout);
////    for(jj=0; jj<len; jj++) printf("%d: %08X\n", jj, tdcbuf[jj]);    
//
//    BANKOPEN(0xe138,1,rol->pid);
//    for(jj=0; jj<len; jj++) *rol->dabufp++ = tdcbuf[jj];
//    BANKCLOSE;    
//  }
//  else
//  {
//    printf("\nERROR in tipci1: petirocReadBlock() returned %d\n\n",len);fflush(stdout);
//    //exit(-1);
//    sleep(1);
//  }
#endif /*USE_NALU*/



#ifdef USE_SRS

  dCnt=0;

  /************************************************************
   * SRS READOUT
   */


  if(rol->pid == 8) /* clondaq8 */
  {
    SRS_FEC_BASE = 4;
  }

  for(ifec=0; ifec<nfec; ifec++)
  {
    dCnt=0;
    BANKOPEN(0xe11f,1,5+ifec+SRS_FEC_BASE);

#if 1
TIMERL_START;
#endif
    dCnt = srsReadBlock(srsFD[ifec],
			   (volatile unsigned int *)rol->dabufp,
			   2*80*1024, block_level, &nframes);
#if 1
TIMERL_STOP(5000/block_level,1000+rol->pid);
#endif

    if(dCnt==-999)
    {
      printf("Timeout during readout from SRS - ignore this event\n");
      dCnt = srsCheckAndClearBuffers(srsFD, nfec,
   			     (volatile unsigned int *)tdcbuf,
		         2*80*1024, block_level, &nframes);
      printf("(dCnt from clear: %d)\n", dCnt);
    }
    else if(dCnt<=0)
    {
      printf("**************************************************\n");
      printf("No SRS data or error.  dCnt = %d\n",dCnt);
      printf("**************************************************\n");
      dCnt = srsCheckAndClearBuffers(srsFD, nfec,
			       (volatile unsigned int *)tdcbuf,
			       2*80*1024, block_level, &nframes);
      printf("(dCnt from clear: %d)\n", dCnt);
    }
    else
    {
//      printf("SRS data len=%d\n",dCnt);
      /* Bump data buffer by the amount of data received */
      rol->dabufp += dCnt;
    }

    BANKCLOSE;
  }

#endif /*USE_SRS*/






#ifdef USE_SAMPA

  /*
  usleep(1000);
  for(chip=0; chip<7; chip++)
  {
    ret = sampaGetErrors(fee, chip);
    printf("[chip=%d] Correctable header hamming errors = %d, Uncorrectable header hamming errors = %d\n",chip,ret&0x1F,(ret>>5)&0x7);
  }
  */

  usleep(1000); // have to check here if event in fifo is ready; until it is implemented, do sleep() 
  len = sampaReadBlock((/*volatile*/ uint32_t *)tdcbuf, MAXDATA);
  if(len > 0)
  {
    //sampaPrintBlock((volatile unsigned int *)tdcbuf, len);
    BANKOPEN(0xe134,1,rol->pid);
    for(jj=0; jj<len; jj++) *rol->dabufp++ = tdcbuf[jj];
    BANKCLOSE;
  }
  else
  {
    printf("ERROR: sampaReadBlock() returned %d\n",len);
  }

#endif /*USE_SAMPA*/








#ifdef USE_VMM

  usleep(1000); // have to check here if event in fifo is ready; until it is implemented, do sleep() 
  len = vmmReadBlock((volatile unsigned int *)tdcbuf, MAXDATA);
  if(len > 0)
  {
    //printf("vmm: len=%d\n",len);

    //vmmPrintBlock((volatile unsigned int *)tdcbuf, len);
    BANKOPEN(0xe135,1,0/*rol->pid*/);
    for(jj=0; jj<len; jj++) *rol->dabufp++ = tdcbuf[jj];
    BANKCLOSE;

  }
  else
  {
    printf("ERROR: vmmReadBlock() returned %d\n",len);
  }

#endif /*USE_VMM*/







#if 0
#ifndef TI_SLAVE

  /* create HEAD bank if master and standalone crates, NOT slave */

    event_number = (EVENT_NUMBER) * block_level - block_level;

    BANKOPEN(0xe112,1,0);

    dabufp1 = rol->dabufp;

    *rol->dabufp ++ = LSWAP((0x10<<27)+block_level); /*block header*/

    for(ii=0; ii<block_level; ii++)
    {
      event_number ++;
	  /*
	  printf(">>>>>>>>>>>>> %d %d\n",(EVENT_NUMBER),event_number);
      sleep(1);
	  */
      *rol->dabufp ++ = LSWAP((0x12<<27)+(event_number&0x7FFFFFF)); /*event header*/

      nwords = 5; /* UPDATE THAT IF THE NUMBER OF WORDS CHANGED BELOW !!! */
      *rol->dabufp ++ = LSWAP((0x14<<27)+nwords); /*head data*/
      *rol->dabufp ++ = 0; /*version  number */
      *rol->dabufp ++ = LSWAP(RUN_NUMBER); /*run  number */
      *rol->dabufp ++ = LSWAP(event_number); /*event number */
      if(ii==(block_level-1))
	  {
        *rol->dabufp ++ = LSWAP(time(0)); /*event unix time */
        *rol->dabufp ++ = LSWAP(EVTYPE); /*event type */
	  }
      else
	  {
        *rol->dabufp ++ = 0;
        *rol->dabufp ++ = 0;
	  }
	}

    nwords = ((long int)rol->dabufp-(long int)dabufp1)/4+1;
	/*printf("nwords=%d\n",nwords);*/

    *rol->dabufp ++ = LSWAP((0x11<<27)+nwords); /*block trailer*/

    BANKCLOSE;

#endif
#endif




#ifndef TI_SLAVE

  /* create HEAD bank if master and standalone crates, NOT slave */

    event_number = (EVENT_NUMBER) * block_level - block_level;

    BANKOPEN(0xe112,1,0);

    dabufp1 = rol->dabufp;

    *rol->dabufp ++ = ((0x10<<27)+block_level); /*block header*/

    for(ii=0; ii<block_level; ii++)
    {
      event_number ++;
      /*
      printf(">>>>>>>>>>>>> %d %d\n",(EVENT_NUMBER),event_number);
      sleep(1);
      */
      *rol->dabufp ++ = ((0x12<<27)+(event_number&0x7FFFFFF)); /*event header*/

      nwords = 6; /* UPDATE THAT IF THE NUMBER OF WORDS CHANGED BELOW !!! */
      *rol->dabufp ++ = ((0x14<<27)+nwords); /*head data*/

      /* COUNT DATA WORDS FROM HERE */
      *rol->dabufp ++ = 0; /*version  number */
      *rol->dabufp ++ = (RUN_NUMBER); /*run  number */
      *rol->dabufp ++ = (event_number); /*event number */
      if(ii==(block_level-1))
      {
        *rol->dabufp ++ = (time(0)); /*event unix time */
        *rol->dabufp ++ = (EVTYPE);  /*event type */
        *rol->dabufp ++ = 0;              /*reserved for L3 info*/
      }
      else
      {
        *rol->dabufp ++ = 0;
        *rol->dabufp ++ = 0;
        *rol->dabufp ++ = 0;
      }
      /* END OF DATA WORDS */

    }

    nwords = ((long int)rol->dabufp-(long int)dabufp1)/4 + 1;

    *rol->dabufp ++ = ((0x11<<27)+nwords); /*block trailer*/

    BANKCLOSE;

#endif





#if 1 /* enable/disable sync events processing */


    /* read boards configurations */
    if(syncFlag==1 || EVENT_NUMBER==1)
    {
      printf("SYNC: read boards configurations\n");

      BANKOPEN(0xe10E,3,rol->pid);
      chptr = chptr0 =(char *)rol->dabufp;
      nbytes = 0;

      /* add one 'return' to make evio2xml output nicer */
      *chptr++ = '\n';
      nbytes ++;



#ifdef TIP_UPLOADALL_NOTDEFINED      
vmeBusLock();
      len = tipusUploadAll(chptr, 10000);
vmeBusUnlock();

/*printf("\nTI len=%d\n",len);
      printf(">%s<\n",chptr);*/
      chptr += len;
      nbytes += len;
#endif




#ifdef USE_PETIROC
      if(npetiroc>0)
      {
vmeBusLock();
        len = petirocUploadAll(chptr, 60000);
vmeBusUnlock();
        printf("\nPETIROC len=%d\n",len);
        /*printf("%s\n",chptr);*/
        chptr += len;
        nbytes += len;
      }
#endif




#ifdef USE_SRS_HIDE
      if(nfec>0)
      {
vmeBusLock();
        len = srsUploadAll(chptr, 10000);
vmeBusUnlock();
        /*printf("\nFADC len=%d\n",len);
        printf("%s\n",chptr);*/
        chptr += len;
        nbytes += len;
      }
#endif




      /* 'nbytes' does not includes end_of_string ! */
      chptr[0] = '\n';
      chptr[1] = '\n';
      chptr[2] = '\n';
      chptr[3] = '\n';
      nbytes = (((nbytes+1)+3)/4)*4;
      chptr0[nbytes-1] = '\0';

      nwords = nbytes/4;
      rol->dabufp += nwords;

      BANKCLOSE;


      printf("SYNC: read boards configurations - done\n");
    }

    /* read scaler(s) */
    if(syncFlag==1 || EVENT_NUMBER==1)
    {
      printf("SYNC: read scalers\n");
	}


#ifndef TI_SLAVE
    /* print livetite */
    if(syncFlag==1)
	{
      printf("SYNC: livetime\n");

      int livetime, live_percent;
vmeBusLock();
      tipusLatchTimers();
      livetime = tipusLive(0);
vmeBusUnlock();
      live_percent = livetime/10;
	  printf("============= Livetime=%3d percent\n",live_percent);
#ifdef SSIPC
	  {
        int status;
        status = epics_msg_send("hallb_livetime","int",1,&live_percent);
	  }
#endif
      printf("SYNC: livetime - done\n");
	}
#endif



#ifdef USE_HPS
     // RTH Begin
    //printf("5\n");
     rcdRet = rogueCodaUpdate(rcd,syncFlag);
     //printf("6\n");

     // Error
     if ( rcdRet < 0 ) __end();

     // Dump config
     if ( rcdRet & 0x1 )
     {
         BANKOPEN(0xe131,3,rol->pid);
printf("7\n");
         rol->dabufp += rogueCodaConfig(rcd,rol->dabufp);
printf("8\n");
         BANKCLOSE;
     }

     // End run for calibration
     if ( rcdRet & 0x10 ) __end();

     // RTH END
#endif




    /* for physics sync event, make sure all board buffers are empty */
    if(syncFlag==1)
    {
      printf("SYNC: make sure all board buffers are empty\n");

      /* Check TIpcie */
      int nblocks;
      nblocks = tipusGetNumberOfBlocksInBuffer();
      /*printf(" Blocks ready for readout: %d\n\n",nblocks);*/
      
      if(nblocks > 1) /* TIpcie Blocks decrement on readout acknowledge */
	{
	  printf("SYNC ERROR: TI nblocks = %d\n",nblocks);fflush(stdout);
	  sleep(10);
	}

#ifdef USE_SRS
      /* Check SRS */
      dCnt = srsCheckAndClearBuffers(srsFD, nfec,
				     (volatile unsigned int *)tdcbuf,
				     2*80*1024, block_level, &nframes);
      if(dCnt>0)
      {
	printf("SYNC ERROR: SRS had extra data at SyncEvent.\n");
      }
	  
#endif /* USE_SRS */
      
      printf("SYNC: make sure all board buffers are empty - done\n");
    }


#endif /* if 0 */







  }

  /* close event */
  CECLOSE;

  /*
  nusertrig ++;
  printf("usrtrig called %d times\n",nusertrig);fflush(stdout);
  */
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
  
  ndone ++;
  //printf("_done called %d times\n",ndone);fflush(stdout);
  
  /* from parser */
  poolEmpty = 0; /* global Done, Buffers have been freed */

  /* Acknowledge tir register */
  CDOACK(GEN,TIR_SOURCE,0);

  return;
}


static void
__status()
{
  return;
}  



int 
rocClose()
{
  printf("--- Execute rocClose ---\n");


#ifdef USE_HPS
  // RTH
printf("9\n");
  rogueCodaClose(rcd);
printf("10\n");
#endif


#ifdef USE_SRS
  int ifec=0;

  for(ifec=0; ifec<nfec; ifec++)
    srsTrigDisable(FEC[ifec]);

  sleep(1);

  for(ifec=0; ifec<nfec; ifec++)
  {
    if(srsFD[ifec]) close(srsFD[ifec]);
  }
#endif

#if 0
  printf("SWITCH TIPCI TO INTERNAL CLOCK IN 'RESET'\n");fflush(stdout);
vmeBusLock();
  tipusSetClockSource(0); /*switching TI to internal clock - until fixed in firmware*/
  tipusStatus(1); /*switching TI to internal clock - until fixed in firmware*/
vmeBusUnlock();
#endif


 printf("rocClose() 1\n");
  tipusStatus(1);
 printf("rocClose() 2\n");

}


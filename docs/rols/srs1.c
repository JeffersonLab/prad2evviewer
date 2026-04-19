
/* srs1.c - first readout list for SRS, runs on regular Linux */


#define NEW


#define NEW_TI /* new TI */


#undef SSIPC

static int nusertrig, ndone;


#define USE_SRS


#undef DEBUG



#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>

#include <sys/time.h>

#if 0
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

/* readout list name */
#define ROL_NAME__ "SRS1"

/* polling mode if needed */
#define POLLING_MODE

/* main TI board */
#define TI_ADDR   (21<<19)  /* if 0 - default will be used, assuming slot 21*/



/* name used by loader */

#ifdef TI_MASTER
#define INIT_NAME srs1_master__init
#ifdef NEW_TI
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#define TIP_READOUT TIP_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#else
#ifdef TI_SLAVE
#define INIT_NAME srs1_slave__init
#ifdef NEW_TI
#define TIP_READOUT TIPUS_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define TIP_READOUT TIP_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#endif
#else
#define INIT_NAME srs1__init
#ifdef NEW_TI
#define TIP_READOUT TIPUS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#define TIP_READOUT TIP_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif
#endif

#include "rol.h"

void usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE);
void usrtrig_done();










#define BLOCKLEVEL 1
#define BUFFERLEVEL  8


/* TRIG_MODE = TIP_READOUT_EXT_POLL for polling mode (External front panel input) */
/*           = TIP_READOUT_TS_POLL  for polling mode (Fiber input from TS) */
/* Fix from Bryan 17may16 */
#define TRIG_MODE TIP_READOUT


/*  SOFTTRIG */
/*    is for pedestal and/or debugging. */
/*   undef to use Front Panel inputs or TS Fiber Connection (TRIG_MODE above)*/
#undef SOFTTRIG



#ifdef SOFTTRIG
/* PULSER_TYPE   0 - Fixed   1 - Random*/
#define PULSER_TYPE          1
/* PULSER_FIXED  */
#define PULSER_FIXED_NUMBER  BLOCKLEVEL
#define PULSER_FIXED_PERIOD  2
#define PULSER_FIXED_LONG    1

/* PULSER_RANDOM_FREQ where arg sets frequency to 500kHz/(2^(arg-1))*/
#define PULSER_RANDOM_FREQ   15 /*6 ~15kHz, 10-500Hz, 8-2kHz, 6-7kHz(92% TI only, drops) */
#endif







#ifdef USE_SRS

#include "srsLib.h"
int srsFD[MAX_FEC];
char FEC[MAX_FEC][100];
int nfec=1;

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

#if 0
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



static void
__download()
{
  int i1, i2, i3;
  char ver[100];

#if 0
  //RTH
  if ( (rcd = rogueCodaInit()) == NULL ) exit(-1);
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

  /**/
  CTRIGINIT;
  CDOINIT(GEN,TIR_SOURCE);

  /*************************************/
  /* redefine TI settings if neseccary */

#ifndef TI_SLAVE
  /* TS 1-6 create physics trigger, no sync event pin, no trigger 2 */
vmeBusLock();

#ifdef NEW_TI
  tipusLoadTriggerTable(0);
#else
  tipLoadTriggerTable(0/*was 3*/);
  /*was tiSetTriggerWindow(7);*/	/* (7+1)*4ns trigger it coincidence time to form trigger type */
#endif



/*in GEN_source.h ??????????
  tipusSetTriggerHoldoff(1,1,2);
  tipusSetTriggerHoldoff(2,4,0);

  tipusSetEventFormat(3);
  tipusSetFPInputReadout(1);

  tipusSetPrescale(0);

#ifdef SOFTTRIG
  tipusSetTriggerSource(TIPUS_TRIGGER_PULSER);
#else
  tipusSetTriggerSource(TIPUS_TRIGGER_TSINPUTS);
#endif
  tipusEnableTSInput(0xf);

  tipusSetBlockBufferLevel(10);

  tipusSetSyncEventInterval(1000);
  tipusSetBlockLevel(blockLevel);

  tipusTrigLinkReset();
  usleep(10000);
*/


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
  else if (rol->pid == 85) /* clondaq9, clondaq14 */
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
  else if (rol->pid == 85) /* clon10new */
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
			      2, // how many time slots the APV chip is reading from its memory
                                 // for each trigger = (n+1)*3  
			      4, // 40000 ???
                                   // in test pulse mode: period of the trigger sequenser
                                   // in run mode: deadtime
                                   // NOTE: must be more than the DAQ time (datalength Nchannels)
			    0x69 /*0x4c*//*0x56*//*0x60*/, // int trgdelay 61(tage total sum) 5f(scintillator trigger) 60(MASTER OR)
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
			 0xcfff, //0x3fff,//0xfff/*1ff*/, // int chEnable // sergey: mask for front end cards connected
			 1550, // int dataLength // the number of 16-bit samples, 12bits used (1 sample=128 - what ???). 3 ts = 550, 6ts = 1000, 15ts=2260, 9ts = 1400,12ts = 2000, 27ts 4000
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
		   0x80, // int latency, // 0x80=3.2us
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
  
#endif

#ifdef NEW_TI
  tipusStatus(0);
#else
  tipStatus(0);
#endif

  sprintf(rcname,"RC%02d",rol->pid);
  printf("rcname >%4.4s<\n",rcname);

#ifdef SSIPC
  sprintf(ssname,"%s_%s",getenv("HOST"),rcname);
  printf("Smartsockets unique name >%s<\n",ssname);
  epics_msg_sender_init(getenv("EXPID"), ssname); /* SECOND ARG MUST BE UNIQUE !!! */
#endif

#if 0
  //RTH
  if ( rogueCodaDownload(rcd,rol->confFile,rol->usrString) != 0 ) exit(-1);
#endif



#if 0
#ifdef SOFTTRIG
  /*pulser*/
  tipusSetTriggerSourceMask(TIPUS_TRIGSRC_LOOPBACK | TIPUS_TRIGSRC_PULSER);
  tipusSetBusySource(TIPUS_BUSY_LOOPBACK | TIPUS_BUSY_FP, 1);
  tipusSetSyncEventInterval(0);
#else
  /*front panel*/
  tipusLoadTriggerTable(0);
  tipusSetTriggerSourceMask(TIPUS_TRIGSRC_LOOPBACK | TIPUS_TRIGSRC_TSINPUTS);
  tipusSetBusySource(TIPUS_BUSY_LOOPBACK | TIPUS_BUSY_FP, 1);
  tipusDisableTSInput(0x3F);
  tipusEnableTSInput(0x10);
  //tipusSetInputPrescale(5, 15); /*second par is same meaning as in internal pulser*/
  tipusSetTriggerHoldoff(1,4,1);
#endif
#endif



  logMsg("INFO: User Download Executed\n",1,2,3,4,5,6);
}



static void
__prestart()
{
  int ii, i1, i2, i3;
  int ret;

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

#ifndef TI_SLAVE
vmeBusLock();

#ifdef NEW_TI
  tipusSetBusySource(TIPUS_BUSY_FP, 0);
#else
  tipSetBusySource(TIP_BUSY_FP, 0);
  /*FOR NOW !!!!!!!!!!!!!!
  tiSetBusySource(TIP_BUSY_LOOPBACK,0);
  */
#endif

vmeBusUnlock();
#endif

#ifdef TI_SLAVE
vmeBusLock();

#ifdef NEW_TI
  tipusSetBusySource(TIPUS_BUSY_FP_FADC, 0);
  /* tipusSetBusySource(0, 1); */
  printf("TIpcie: Setting SyncSrc to HFBR1 and Loopback\n");
  tipusSetSyncSource(TIPUS_SYNC_HFBR1 | TIPUS_SYNC_LOOPBACK);
  tipusSetInstantBlockLevelChange(1);
  /*sergey: tipusSetSyncSource() resets register 0x24 (sync), so we'll loose UserSyncReset enabling, reinstall it*/
  tipusSetUserSyncResetReceive(1);
#else
  tipSetBusySource(TIP_BUSY_FP_FADC, 0);
  /* tipSetBusySource(0, 1); */
  printf("TIpcie: Setting SyncSrc to HFBR1 and Loopback\n");
  tipSetSyncSource(TIP_SYNC_HFBR1 | TIP_SYNC_LOOPBACK);
  tipSetInstantBlockLevelChange(1);
  /*sergey: tipSetSyncSource() resets register 0x24 (sync), so we'll loose UserSyncReset enabling, reinstall it*/
  tipSetUserSyncResetReceive(1);
#endif

vmeBusUnlock();

#endif



#ifdef USE_SRS

 int ifec=0, nframes=0, dCnt=0;
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

#if 0
  //RTH
  if ( rogueCodaPrestart(rcd) != 0 ) exit(-1);
#endif

#if 0 /* FOR NOW */





vmeBusLock();


#ifdef NEW_TI
  tipusIntDisable();
#else
  tipIntDisable();
#endif

vmeBusUnlock();


  /* master and standalone crates, NOT slave */
#ifndef TI_SLAVE

#ifdef NEW_TI

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

#else

  sleep(1);
vmeBusLock();
  tipSyncReset(1);
vmeBusUnlock();
  sleep(1);
vmeBusLock();
  tipSyncReset(1);
vmeBusUnlock();
  sleep(1);

vmeBusLock();
  ret = tipGetSyncResetRequest();
vmeBusUnlock();
  if(ret)
  {
    printf("ERROR: syncrequest still ON after tiSyncReset(); trying again\n");
    sleep(1);
vmeBusLock();
    tipSyncReset(1);
vmeBusUnlock();
    sleep(1);
  }

vmeBusLock();
  ret = tipGetSyncResetRequest();
vmeBusUnlock();

#endif

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

#ifdef NEW_TI
  tipusSetBlockLevel(block_level);
#else
  tipSetBlockLevel(block_level);
#endif

vmeBusUnlock();
#endif


#endif


vmeBusLock();
#ifdef NEW_TI
  tipusStatus(1);
#else
  tipStatus(1);
#endif
vmeBusUnlock();

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


#ifdef TI_MASTER
#ifdef SOFTTRIG
  tipusSetRandomTrigger(1,0xf);
#endif
#endif


#ifndef TI_SLAVE
vmeBusLock();

  /* set sync event interval (in blocks) */
#ifdef NEW_TI
  tipusSetSyncEventInterval(0/*10000*//*block_level*/);
#else
  tipSetSyncEventInterval(0/*10000*//*block_level*/);
#endif

vmeBusUnlock();

/* Fix from Bryan 17may16 */
#else
/* Block level was broadcasted from TI-Master / TS */
#ifdef NEW_TI
  block_level = tipusGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",block_level);
  tipusSetBlockBufferLevel(0);
#else
  block_level = tipGetCurrentBlockLevel();
  printf("rocGo: Block Level set to %d\n",block_level);
  tipSetBlockBufferLevel(0);
#endif

#endif



#if 0
  //RTH
 if ( rogueCodaGo(rcd,rol->runNumber) != 0 ) exit(-1);
#endif

#ifdef USE_SRS
  int ifec=0;

  for(ifec=0; ifec<nfec; ifec++)
  {
    printf("SRS: Enabling SRS Triggers for FEC[ifec=%d]=%d ...\n",ifec,FEC[ifec]);
    srsTrigEnable(FEC[ifec]);
    printf(" ... Enabling SRS Triggers is done.\n",ifec);
    srsStatus(FEC[ifec],0);
  }

#endif

#ifdef NEW_TI
  tipusStatus(1);
#else
  tipStatus(1);
#endif

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

#ifdef TI_MASTER
#ifdef SOFTTRIG
  tipusDisableRandomTrigger();
#endif
#endif

  CDODISABLE(GEN,TIR_SOURCE,0);

  /* Before disconnecting... wait for blocks to be emptied */
vmeBusLock();
#ifdef NEW_TI
  blocksLeft = tipusBReady();
#else
  blocksLeft = tipBReady();
#endif
vmeBusUnlock();
  printf(">>>>>>>>>>>>>>>>>>>>>>> %d blocks left on the TI\n",blocksLeft);fflush(stdout);
  if(blocksLeft)
  {
    printf(">>>>>>>>>>>>>>>>>>>>>>> before while ... %d blocks left on the TI\n",blocksLeft);fflush(stdout);
    while(iwait < 10)
	{
      sleep(1);
	  if(blocksLeft <= 0) break;
vmeBusLock();
#ifdef NEW_TI
	  blocksLeft = tipusBReady();
#else
	  blocksLeft = tipBReady();
#endif
      printf(">>>>>>>>>>>>>>>>>>>>>>> inside while ... %d blocks left on the TI\n",blocksLeft);fflush(stdout);
vmeBusUnlock();
	  iwait++;
	}
    printf(">>>>>>>>>>>>>>>>>>>>>>> after while ... %d blocks left on the TI\n",blocksLeft);fflush(stdout);
  }

#if 0
  //RTH
  rogueCodaEnd(rcd);
#endif

#ifdef USE_SRS
  for(ifec=0; ifec<nfec; ifec++)
  {
    srsTrigDisable(FEC[ifec]);
    srsStatus(FEC[ifec],0);
  }
#endif


vmeBusLock();
#ifdef NEW_TI
  tipusStatus(1);
#else
  tipStatus(1);
#endif
vmeBusUnlock();

  printf("INFO: End1 Executed\n\n\n");fflush(stdout);

  return;
}



void
usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE)
{
  int *jw, ind, ind2, i, ii, iii, jj, jjj, blen, len, rlen, nbytes;
  int rcdRet, tmp;
  unsigned int *tdc, *blk_hdr, *evt_hdr;
  unsigned int *dabufp1, *dabufp2;
  int njjloops, slot;
  int nwords, dCnt;
  int nMultiSamples;
  TIMERL_VAR;
  char *chptr, *chptr0;

  /*printf("EVTYPE=%d syncFlag=%d\n",EVTYPE,syncFlag);*/

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
#ifdef NEW_TI
    //len = tipusReadTriggerBlock(tdcbuf);
    len = tipusReadBlock(tdcbuf,1024,0);
#else
    len = tipReadBlock(tdcbuf,1024,0);
#endif
#if 0
TIMERL_STOP(5000/block_level,1000+rol->pid);
#endif

    if(len<=0)
    {
      printf("ERROR in tipReadBlock : No data or error, len = %d\n",len);
      sleep(1);

#ifdef NEW_TI
      if(TRIG_MODE==TIPUS_READOUT_EXT_POLL) tipusSetBlockLimit(1); /* Effectively... stop triggers */
#else
      if(TRIG_MODE==TIP_READOUT_EXT_POLL) tipSetBlockLimit(1); /* Effectively... stop triggers */
#endif

    }
    else
    {
	  
#ifdef DEBUG
      printf("ti: len=%d\n",len);
      for(jj=0; jj<len; jj++) printf("ti[%2d] 0x%08x\n",jj,tdcbuf[jj]);
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




#if 0

    // RTH
    rcdRet = rogueCodaTrigger(rcd,tdcbuf,len);
    //printf("\n===> rcdRet=%d\n",rcdRet);

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
      len = rogueCodaEvent(rcd,rol->dabufp,ii);
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


#ifdef DEBUG
    printf("fadc1: start fadc processing\n");fflush(stdout);
#endif

#endif /* if 0 */



/*usleep(1);*/ /*1-55us, 50-104us, 70-126us*/




#ifdef USE_SRS

  dCnt=0;

  /************************************************************
   * SRS READOUT
   */

  int ifec=0;
  int nframes=0;
  int SRS_FEC_BASE=0;

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
    }
    else if(dCnt<=0)
    {
      printf("**************************************************\n");
      printf("No SRS data or error.  dCnt = %d\n",dCnt);
      printf("**************************************************\n");
    }
    else
    {
      //printf("SRS data len=%d\n",dCnt);
      /* Bump data buffer by the amount of data received */
      rol->dabufp += dCnt;
    }

    BANKCLOSE;
  }

#endif


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

#if 0

      BANKOPEN(0xe10E,3,rol->pid);
      chptr = chptr0 =(char *)rol->dabufp;
      nbytes = 0;

      /* add one 'return' to make evio2xml output nicer */
      *chptr++ = '\n';
      nbytes ++;

#ifdef TIP_UPLOADALL_NOTDEFINED      
vmeBusLock();
#ifdef NEW_TI
      len = tipusUploadAll(chptr, 10000);
#else
      len = tipUploadAll(chptr, 10000);
#endif
vmeBusUnlock();
#endif

/*printf("\nTI len=%d\n",len);
      printf(">%s<\n",chptr);*/
      chptr += len;
      nbytes += len;

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
#endif

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
#ifdef NEW_TI
      tipusLatchTimers();
      livetime = tipusLive(0);
#else
      tipLatchTimers();
      livetime = tipLive(0);
#endif
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


#if 0
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
#ifdef NEW_TI
      nblocks = tipusGetNumberOfBlocksInBuffer();
#else
      nblocks = tipGetNumberOfBlocksInBuffer();
#endif
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
  /*
  ndone ++;
  printf("_done called %d times\n",ndone);fflush(stdout);
  */
  /* from parser */
  poolEmpty = 0; /* global Done, Buffers have been freed */

  /* Acknowledge tir register */
  CDOACK(GEN,TIR_SOURCE,0);

#ifdef SOFTTRIG
  if(PULSER_TYPE==0)
  {
#ifdef NEW_TI
    tipusSoftTrig(1,PULSER_FIXED_NUMBER,PULSER_FIXED_PERIOD,PULSER_FIXED_LONG);
#else
    tipSoftTrig(1,PULSER_FIXED_NUMBER,PULSER_FIXED_PERIOD,PULSER_FIXED_LONG);
#endif
  }
#endif

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

#if 0
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
}


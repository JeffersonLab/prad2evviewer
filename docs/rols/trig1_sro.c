
/* trig1.c - trig1 crate (trigger supervisor) first readout list */

#if defined(VXWORKS) || defined(Linux_vme)


#undef SSIPC

static int nusertrig, ndone;

#define USE_DSC2


#undef DEBUG



#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>

#ifndef VXWORKS
#include <sys/time.h>
/*
typedef      long long       hrtime_t;
*/
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
#define ROL_NAME__ "TRIG1_SRO"

/* polling mode if needed */
#define POLLING_MODE

/* main TI board */
#define TS_ADDR   (21<<19)  /* if 0 - default will be used, assuming slot 21*/



/* name used by loader */
#define INIT_NAME trig1_sro__init
#define TS_READOUT TS_READOUT_EXT_POLL /* Poll for available data, front panel triggers */


#include "rol.h"

#include "daqLib.h"
#include "sdLib.h"
#include "tsLib.h"
#include "tsConfig.h"
#include "tdLib.h"

#ifdef USE_DSC2
#include "dsc2Lib.h"
#include "dsc2Config.h"
#endif

void usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE);
void usrtrig_done();

#include "TSPRIMARY_source.h"



/* user code */



static char rcname[5];

#define NBOARDS 22    /* maximum number of VME boards: we have 21 boards, but numbering starts from 1 */
#define MY_MAX_EVENT_LENGTH 3000/*3200*/ /* max words per board */
static unsigned int *tdcbuf;

/*#ifdef DMA_TO_BIGBUF*/
/* must be 'rol' members, like dabufp */
extern unsigned int dabufp_usermembase;
extern unsigned int dabufp_physmembase;
/*#endif*/

extern int rocMask; /* defined in roc_component.c */
unsigned int tdslotmask = 0;    /* bit=slot (starting from 0) */
static int ntd;

#define NTICKS 1000 /* the number of ticks per second */
/*temporary here: for time profiling */




#ifndef VXWORKS

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

#endif



void
tsleep(int n)
{
#ifdef VXWORKS
  taskDelay ((sysClkRateGet() / NTICKS) * n);
#else
#endif
}



static int ti_slave_fiber_port = 1;





/*
#ifdef USE_V1190
*/
static int tdctypebyslot[NBOARDS];
static int error_flag[NBOARDS];
static int ndsc2=0, ndsc2_daq=0;
static int ntdcs;


int
getTdcTypes(int *typebyslot)
{
  int jj;
  for(jj=0; jj<NBOARDS; jj++) typebyslot[jj] = tdctypebyslot[jj];
  return(ntdcs);
}

static int slotnums[NBOARDS];
int
getTdcSlotNumbers(int *slotnumbers)
{
  int jj;
  for(jj=0; jj<NBOARDS; jj++) slotnumbers[jj] = slotnums[jj];
  return(ntdcs);
}

/*
#endif
*/



/* GTP bits: 0-HTCC, */
/* FP bits: 0..5 - hit-based sec1..6, 6..11 - ltcc sec1..6, 12..15 - helicity t_settle,quartet,clock,helicity */
/*
static unsigned int tsTrigPatternData[8][256] - use 12 low bits from 32

                    [256]: bits 0-4
*/
#if 0
void
tsTriggerTable1()
{
  unsigned int imem=0, iword=0;



  /******************************* GTP *******************************/

  /* Fill in the single bit patterns with "single trigger" patterns */
  for (imem=0; imem<4; imem++)
  {
    /* Start by initializing all bit patterns to their numerical event types,
	and setting them all to be "multiple trigger" patterns */
    for (iword=0; iword<256; iword++)
	{
	  /* set bit(8) to 1 (hw trig1), and bit(11:10) to 3 for multi-bit trigger */
	  tsTrigPatternData[imem][iword] = 0xD00 + iword;
	}
      
    /* Zero inputs, No triggers */
    tsTrigPatternData[imem][0] = 0;

    for (iword=0; iword<8; iword++)
	{
	  /* set bit(8) to 1 (hw trig1), and bit(10) to 1 for single-bit trigger */
	  tsTrigPatternData[imem][((1<<iword)%0xff)] = 0x500 + iword + 1 + imem*8;
	}
  }


  /******************************* FP *******************************/

  /* Fill in the single bit patterns with "single trigger" patterns */
  for (imem=4; imem<8; imem++)
  {
    /* Start by initializing all bit patterns to their numerical event types,
	and setting them all to be "multiple trigger" patterns */
    for (iword=0; iword<256; iword++)
	{
	  /* set bit(8) to 1 (hw trig1), and bit(11:10) to 3 for multi-bit trigger */
	  tsTrigPatternData[imem][iword] = 0xD00 + iword;
	}
      
    /* Zero inputs, No triggers */
    tsTrigPatternData[imem][0] = 0;

    for (iword=0; iword<8; iword++)
	{
	  /* set bit(8) to 1 (hw trig1), and bit(10) to 1 for single-bit trigger */
	  tsTrigPatternData[imem][((1<<iword)%0xff)] = 0x500 + iword + 1 + imem*8;
	}
  }



}
#endif




static void
__download()
{
  int ii, i1, i2, i3, id, slot;
  char filename[1024];
#ifdef POLLING_MODE
  rol->poll = 1;
#else
  rol->poll = 0;
#endif

  printf("\n>>>>>>>>>>>>>>> ROCID=%d, CLASSID=%d <<<<<<<<<<<<<<<<\n",rol->pid,rol->classid);
  printf("CONFFILE >%s<\n\n",rol->confFile);
  printf("LAST COMPILED: %s %s\n", __DATE__, __TIME__);

  printf("USRSTRING >%s<\n\n",rol->usrString);

  /**/
  CTRIGINIT;

  /* initialize OS windows and TS board */
#ifdef VXWORKS
  CDOINIT(TSPRIMARY);
#else
  CDOINIT(TSPRIMARY,TIR_SOURCE);
#endif


  /************/
  /* init daq */

  daqInit();
  DAQ_READ_CONF_FILE;


  /*************************************/
  /* redefine TS settings if neseccary */

  tsSetUserSyncResetReceive(1);


  /* TS 1-6 create physics trigger, no sync event pin, no trigger 2 */
vmeBusLock();
/*tsLoadTriggerTable();*/
  /*tsSetTriggerWindow(7);TS*/	// (7+1)*4ns trigger it coincidence time to form trigger type
vmeBusUnlock();


  /*********************************************************/
  /*********************************************************/



  /* set wide pulse */
vmeBusLock();
/*sergey: WAS tsSetSyncDelayWidth(1,127,1);*/
/*worked for bit pattern latch tsSetSyncDelayWidth(0x54,127,1);*/
vmeBusUnlock();

  usrVmeDmaSetConfig(2,5,1); /*A32,2eSST,267MB/s*/
  /*usrVmeDmaSetConfig(2,5,0);*/ /*A32,2eSST,160MB/s*/
  /*usrVmeDmaSetConfig(2,3,0);*/ /*A32,MBLT*/

  tdcbuf = (unsigned int *)i2_from_rol1;



  /******************/
  /* USER code here */


  /* TD setup */

  ntd = 0;
  tdInit((3<<19),0x80000,20,0);
  ntd = tdGetNtds(); /* actual number of TD boards found  */

  for(id=0; id<ntd; id++) 
  {
    slot = tdSlot(id);
    tdResetMGTRx(id);
    tdResetSlaveConfig(id); /*sergey: remove busy's from all 8 fibers, may be left from previous runs*/
  }


  tdGSetBlockLevel(block_level);
  tdGSetBlockBufferLevel(buffer_level);

  //tdAddSlave(17,2); // TI Slave - Bottom Crate (payload)
  //tdAddSlave(17,5); // TI Slave - Bench (GTP)

  tdslotmask = 0;
  for(id=0; id<ntd; id++) 
  {
    slot = tdSlot(id);
    tdslotmask |= (1<<slot);
    printf("=======================> tdslotmask=0x%08x\n",tdslotmask);
  }
  printf("TDSLOTMASK: tdslotmask=0x%08x (from library 0x%08x)\n",tdslotmask,tdSlotMask());

  sprintf(filename,"%s/portnames_%s.txt",getenv("CLON_PARMS"),getenv("EXPID"));
  printf("loading portnames from file >%s<\n",filename);
  tdLoadPortNames(filename);

  /*
  tdGStatus(0);
  */

  /***************************************
   *   SD SETUP
   ***************************************/
  printf("SD init starts\n");
vmeBusLock();
  printf("SD init 1\n");
  sdInit(1);   /* Initialize the SD library */
  sdSetActiveVmeSlots(tdslotmask); /* Use the tdslotmask to configure the SD */
  sdStatus();
vmeBusUnlock();
  printf("SD init done\n");




  /* if TDs are present, set busy from SD board */
  if(ntd>0)
  {
    printf("Set BUSY from SWB for TDs\n");
vmeBusLock();
    tsSetBusySource(TS_BUSY_SWB,0);
vmeBusUnlock();
  }




  /*sergey: following piece from tsConfig.c, doing it there not always propagate correct block_level to slaves;
	doing it again here seems helps, have to investigate */
  tsSetInstantBlockLevelChange(1); /* enable immediate block level setting */
  printf("trig1: setting block_level = %d\n",block_level);
sleep(1);
  tsSetBlockLevel(block_level);
sleep(1);
  tsSetInstantBlockLevelChange(0); /* disable immediate block level setting */






#ifdef USE_DSC2
  printf("DSC2 Download() starts =========================\n");

#ifndef VXWORKS
  vmeSetQuietFlag(1); /* skip the errors associated with BUS Errors */
#endif
vmeBusLock();
  dsc2Init(0x100000,0x80000,20,0);
vmeBusUnlock();
#ifndef VXWORKS
  vmeSetQuietFlag(0); /* Turn the error statements back on */
#endif

  ndsc2 = dsc2GetNdsc();
  if(ndsc2>0)
  {
    DSC2_READ_CONF_FILE;
	/*do not need it here
    maxA32Address = dsc2GetA32MaxAddress();
    fadcA32Address = maxA32Address + FA_MAX_A32_MEM;
	*/
    ndsc2_daq = dsc2GetNdsc_daq();
  }
  else
  {
    ndsc2_daq = 0;
  }
  printf("dsc2: %d boards set to be readout by daq\n",ndsc2_daq);
  printf("DSC2 Download() ends =========================\n\n");
#endif



  /* send synreset here for HPS SVT, will send it again in Prestart in usual place */
  sleep(1);

#if 0
vmeBusLock();
  tsSyncReset(1); /* '1' will push 'next_block_level' to 'block_level' in slave TI's (not TD's !), we did it already in download */
vmeBusUnlock();
#endif



 
/* sro: master and standalone crates, NOT slave: Assert SYNC reset*/
vmeBusLock();

/*tiSyncReset(1); not sure if it is needed */

  printf("Assert SYNC\n");
  tsUserSyncReset(1);
vmeBusUnlock();


 sleep(1);







  

  sprintf(rcname,"RC%02d",rol->pid);
  printf("rcname >%4.4s<\n",rcname);

#ifdef SSIPC
  sprintf(ssname,"%s_%s",getenv("HOST"),rcname);
  printf("Smartsockets unique name >%s<\n",ssname);
  epics_msg_sender_init(getenv("EXPID"), ssname); /* SECOND ARG MUST BE UNIQUE !!! */
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

  /*tsEnableVXSSignals();TS*/

#ifdef POLLING_MODE
  CTRIGRSS(TSPRIMARY, TIR_SOURCE, usrtrig, usrtrig_done);
#else
  CTRIGRSA(TSPRIMARY, TIR_SOURCE, usrtrig, usrtrig_done);
#endif



  /**************************************************************************/
  /* setting TS busy conditions, based on boards found in Download          */
  /* tsInit() does nothing for busy, tsConfig() sets fiber, we set the rest */
  /* NOTE: if ts is busy, it will not send trigger enable over fiber, since */
  /*       it is the same fiber and busy has higher priority                */

vmeBusLock();
  tsSetBusySource(TS_BUSY_LOOPBACK,0);
  /*tsSetBusySource(TS_BUSY_FP,0);*/
vmeBusUnlock();












  /*****************************************************************/
  /*****************************************************************/





  /* USER code here */
  /******************/

vmeBusLock();
  tsIntDisable();
vmeBusUnlock();




#ifdef USE_DSC2
  printf("DSC2 Prestart() starts =========================\n");
  /* dsc2 configuration */
  if(ndsc2>0) DSC2_READ_CONF_FILE;
  printf("DSC2 Prestart() ends =========================\n\n");
#endif



  /* NOT USED !!!!!!!!!!!!!!!!!!!!
vmeBusLock();
  tsSyncReset(1);
vmeBusUnlock();
  sleep(1);

vmeBusLock();
  ret = tsGetSyncResetRequest();
vmeBusUnlock();
  if(ret)
  {
    printf("ERROR: syncrequest still ON after tsSyncReset(); trying again\n");
    sleep(1);
vmeBusLock();

    tsSyncReset(1);

vmeBusUnlock();
    sleep(1);
  }
  */









#if 0 /*sro*/

  /* SYNC RESET - reset event number (and clear FIFOs) in TIs */

  sleep(1);
vmeBusLock();
  tsSyncReset(1); /* '1' will push 'next_block_level' to 'block_level' in slave TI's (not TD's !), we did it already in download */
vmeBusUnlock();
  sleep(1);




  /* USER RESET - use it because 'SYNC RESET' produces too short pulse, still need 'SYNC RESET' above because 'USER RESET'
  does not do everything 'SYNC RESET' does (in paticular does not reset event number) */

vmeBusLock();
  tsUserSyncReset(1);
  tsUserSyncReset(0);
vmeBusUnlock();











vmeBusLock();
  ret = tsGetSyncResetRequest();
vmeBusUnlock();
  if(ret)
  {
    printf("ERROR: syncrequest still ON after tsSyncReset(); try 'tcpClient <rocname> tsSyncReset'\n");
  }
  else
  {
    printf("INFO: syncrequest is OFF now\n");
  }

  printf("holdoff rule 1 set to %d\n",tsGetTriggerHoldoff(1));
  printf("holdoff rule 2 set to %d\n",tsGetTriggerHoldoff(2));



#endif


  
/* set block level in all boards where it is needed;
   it will overwrite any previous block level settings */


/*
#ifdef USE_VSCM
  for(ii=0; ii<nvscm1; ii++)
  {
    slot = vscmSlot(ii);
vmeBusLock();
    vscmSetBlockLevel(slot, block_level);
vmeBusUnlock();
  }
#endif
*/



/*
  {
  char portfile[1024];
  sprintf(portfile,"%s/portnames_%s.txt",getenv("CLON_PARMS"),getenv("EXPID"));
  printf("Loading port names from file >%s<\n",portfile);
  tdLoadPortNames(portfile);
  }
*/


vmeBusLock();
  tsStatus(1);
vmeBusUnlock();


vmeBusLock();
  ret = tdGStatus(block_level);
vmeBusUnlock();
  if(ret)
  {
    logMsg("ERROR: Go 1: WRONG BLOCK_LEVEL, START NEW RUN FROM 'CONFIGURE !!!\n",1,2,3,4,5,6);
    logMsg("ERROR: Go 1: WRONG BLOCK_LEVEL, START NEW RUN FROM 'CONFIGURE !!!\n",1,2,3,4,5,6);
    logMsg("ERROR: Go 1: WRONG BLOCK_LEVEL, START NEW RUN FROM 'CONFIGURE !!!\n",1,2,3,4,5,6);
    UDP_user_request(MSGERR, "rol1", "WRONG BLOCK_LEVEL, START NEW RUN FROM 'CONFIGURE !!!");
  }
  else
  {
    UDP_user_request(0, "rol1", "BLOCK_LEVEL IS OK");
  }



  printf("INFO: Prestart1 Executed\n");fflush(stdout);

  *(rol->nevents) = 0;
  rol->recNb = 0;

  return;
}       

static void
__end()
{
  int iwait=0;
  int blocksLeft=0;
  int id;

  printf("\n\nINFO: End1 Reached\n");fflush(stdout);

  CDODISABLE(TSPRIMARY,TIR_SOURCE,0);

  /* Before disconnecting... wait for blocks to be emptied */
vmeBusLock();
  blocksLeft = tsBReady();
vmeBusUnlock();
  printf(">>>>>>>>>>>>>>>>>>>>>>> %d blocks left on the TS\n",blocksLeft);fflush(stdout);
  if(blocksLeft)
  {
    printf(">>>>>>>>>>>>>>>>>>>>>>> before while ... %d blocks left on the TS\n",blocksLeft);fflush(stdout);
    while(iwait < 10)
	{
      taskDelay(10);
	  if(blocksLeft <= 0) break;
vmeBusLock();
	  blocksLeft = tsBReady();
      printf(">>>>>>>>>>>>>>>>>>>>>>> inside while ... %d blocks left on the TS\n",blocksLeft);fflush(stdout);
vmeBusUnlock();
	  iwait++;
	}
    printf(">>>>>>>>>>>>>>>>>>>>>>> after while ... %d blocks left on the TS\n",blocksLeft);fflush(stdout);
  }



vmeBusLock();
  tsStatus(1);
vmeBusUnlock();

  printf("INFO: End1 Executed\n\n\n");fflush(stdout);

  return;
}


static void
__pause()
{
  CDODISABLE(TSPRIMARY,TIR_SOURCE,0);
  logMsg("INFO: Pause Executed\n",1,2,3,4,5,6);
  
} /*end pause */


static void
__go()
{
  int ii, jj, id, slot, ret;

  logMsg("INFO: Entering Go 1\n",1,2,3,4,5,6);


/* sro: master and standalone crates, NOT slave: Release SYNC reset*/
vmeBusLock();
  printf("Release SYNC\n");
  tsUserSyncReset(0);
vmeBusUnlock();



 
  /* set sync event interval (in blocks) */
vmeBusLock();
  tsSetSyncEventInterval(0/*10000*//*block_level*/);
vmeBusUnlock();

#ifdef USE_DSC2
  for(ii=0; ii<ndsc2_daq; ii++)
  {
    slot = dsc2Slot(ii);
vmeBusLock();
    dsc2ResetScalersGroupA(slot);
    dsc2ResetScalersGroupB(slot);
vmeBusUnlock();
  }
#endif

  /* always clear exceptions */
  //jlabgefClearException(1);
  vmeClearException(1);

  nusertrig = 0;
  ndone = 0;

#if 0
  CDOENABLE(TSPRIMARY,TIR_SOURCE,0);
#endif
  CDODISABLE(TSPRIMARY,TIR_SOURCE,0);

  logMsg("INFO: Go 1 Executed\n",1,2,3,4,5,6);
}



void
usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE)
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
  /*
  ndone ++;
  printf("_done called %d times\n",ndone);fflush(stdout);
  */
  /* from parser */
  poolEmpty = 0; /* global Done, Buffers have been freed */

  /* Acknowledge tir register */
  CDOACK(TSPRIMARY,TIR_SOURCE,0);

  return;
}

static void
__status()
{
  return;
}  

#else

void
trig1_sro_dummy()
{
  return;
}

#endif

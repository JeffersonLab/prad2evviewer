
/* scaler1.c - first readout list for scalers */

#if defined(VXWORKS) || defined(Linux_vme)

#define NEW

#undef SSIPC

static int nusertrig, ndone;

#undef DMA_TO_BIGBUF /*if want to dma directly to the big buffers*/


#define USE_SIS3801
#define USE_DSC2
#define USE_V1190
//#define USE_V851
#define USE_MO


/* if event rate goes higher then 10kHz, with random triggers we have wrong
slot number reported in GLOBAL HEADER and/or GLOBAL TRAILER words; to work
around that problem temporary patches were applied - until fixed (Sergey) */
#define SLOTWORKAROUND

#undef DEBUG

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
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

#include "daqLib.h"
#include "moLib.h"
#include "v851.h"
#include "tiLib.h"
#include "tiConfig.h"
#include "dsc2Lib.h"
#include "dsc2Config.h"

#include "circbuf.h"

/* from fputil.h */
#define SYNC_FLAG 0x20000000

/* readout list name */
#define ROL_NAME__ "SCALER1"

/* polling mode if needed */
#define POLLING_MODE

/* main TI board */
#define TI_ADDR   (21<<19)  /* if 0 - default will be used, assuming slot 21*/



/* name used by loader */

#ifdef TI_MASTER
#define INIT_NAME scaler1_master__init
#define TI_READOUT TI_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME scaler1_slave__init
#define TI_READOUT TI_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME scaler1__init
#define TI_READOUT TI_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#include "rol.h"

void usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE);
void usrtrig_done();

#include "TIPRIMARY_source.h"



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
titest2()
{
  printf("roc >%4.4s<, next_block_level=%d, current block level = %d\n",rcname,tiGetNextBlockLevel(),tiGetCurrentBlockLevel());
}



void
tsleep(int n)
{
#ifdef VXWORKS
  taskDelay ((sysClkRateGet() / NTICKS) * n);
#else
#endif
}



extern struct TI_A24RegStruct *TIp;
static int ti_slave_fiber_port = 1;

void
titest1()
{
  if(TIp==NULL) {printf("NULL\n");return;}
  printf("0x%08x(%d) 0x%08x(%d)\n",
		 vmeRead32(&TIp->fiberLatencyMeasurement),vmeRead32(&TIp->fiberLatencyMeasurement),
		 vmeRead32(&TIp->fiberAlignment),vmeRead32(&TIp->fiberAlignment));
}



/*
#ifdef USE_V1190
*/
static int tdctypebyslot[NBOARDS];
static int error_flag[NBOARDS];
static int ndsc2=0, ndsc2_daq=0;
static int ntdcs;
#ifdef USE_V1190
#include "tdc1190.h"
#endif

int
getTdcTypes(int *typebyslot)
{
  int jj;
  for(jj=0; jj<NBOARDS; jj++) typebyslot[jj] = tdctypebyslot[jj];
  return(ntdcs);
}



#ifdef SLOTWORKAROUND
static int slotnums[NBOARDS];
int
getTdcSlotNumbers(int *slotnumbers)
{
  int jj;
  for(jj=0; jj<NBOARDS; jj++) slotnumbers[jj] = slotnums[jj];
  return(ntdcs);
}
#endif

/*
#endif
*/

#ifdef USE_SIS3801
#include "sis3801.h"
static int nsis;
unsigned int addr;
#define MASK    0x00000000   /* unmask all 32 channels (0-enable,1-disable) */

/* general settings */
void
sis3801config(int id, int mode)
{
  sis3801control(id, DISABLE_EXT_NEXT);
  sis3801reset(id);
  sis3801clear(id);
  sis3801setinputmode(id,mode);
  sis3801enablenextlogic(id);
  sis3801control(id, ENABLE_EXT_DIS);
}

static int mode = 2;

#endif




static unsigned long run_trig_count = 0;

static void
__download()
{
  int i1, i2, i3, ii;
  int id, slot;
  char *ch, tmp[64];
  /*unsigned int maxA32Address;
  unsigned int fadcA32Address = 0x09000000;*/

#ifdef POLLING_MODE
  rol->poll = 1;
#else
  rol->poll = 0;
#endif

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
    tiSetFiberIn_preInit(ti_slave_fiber_port);
  }
#endif


  /**/
  CTRIGINIT;

  /* initialize OS windows and TI board */
#ifdef VXWORKS
  CDOINIT(TIPRIMARY);
#else
  CDOINIT(TIPRIMARY,TIR_SOURCE);
#endif


  /************/
  /* init daq */

  daqInit();
  DAQ_READ_CONF_FILE;


  /*************************************/
  /* redefine TI settings if neseccary */

  tiSetUserSyncResetReceive(1);

#ifndef TI_SLAVE
  /* TS 1-6 create physics trigger, no sync event pin, no trigger 2 */
vmeBusLock();
  tiLoadTriggerTable(3);
  tiSetTriggerWindow(7);	// (7+1)*4ns trigger it coincidence time to form trigger type
vmeBusUnlock();
#endif


  /*********************************************************/
  /*********************************************************/



  /* set wide pulse */
vmeBusLock();
/*sergey: WAS tiSetSyncDelayWidth(1,127,1);*/
/*worked for bit pattern latch tiSetSyncDelayWidth(0x54,127,1);*/
vmeBusUnlock();

//usrVmeDmaSetConfig(2,5,1); /*A32,2eSST,267MB/s*/
  /*usrVmeDmaSetConfig(2,5,0);*/ /*A32,2eSST,160MB/s*/
 usrVmeDmaSetConfig(2,3,0); /*A32,MBLT*/ /*use this for caem TDC1190s on new CONTROLLERS XVB603 */




  /*
  usrVmeDmaSetChannel(1);
  printf("===== Use DMA Channel %d\n\n\n",usrVmeDmaGetChannel());
  */

  tdcbuf = (unsigned int *)i2_from_rol1;







  /******************/
  /* USER code here */







#ifdef USE_DSC2
  printf("DSC2 Download() starts =========================\n");

#ifndef VXWORKS
  vmeSetQuietFlag(1); /* skip the errors associated with BUS Errors */
#endif
vmeBusLock();
 dsc2Init(0x400000,0x80000,1,0); /* initialize slot 8 only !!! */
vmeBusUnlock();
#ifndef VXWORKS
  vmeSetQuietFlag(0); /* Turn the error statements back on */
#endif

  ndsc2 = dsc2GetNdsc();
  if(ndsc2>0)
  {
    DSC2_READ_CONF_FILE;
    /*maxA32Address = dsc2GetA32MaxAddress();
    fadcA32Address = maxA32Address + FA_MAX_A32_MEM;*/
    ndsc2_daq = dsc2GetNdsc_daq();
  }
  else
  {
    ndsc2_daq = 0;
  }
  printf("dsc2: %d boards total, %d boards set to be readout by daq\n",ndsc2,ndsc2_daq);
  printf("DSC2 Download() ends =========================\n\n");
#endif




#ifdef USE_V1190
  printf("V1190 Download() starts =========================\n");

vmeBusLock();
  ntdcs = tdc1190Init(0x11100000,0x80000,20,0);
  if(ntdcs>0) TDC_READ_CONF_FILE;
vmeBusUnlock();

  for(ii=0; ii<ntdcs; ii++)
  {
    slot = tdc1190Slot(ii);
    tdctypebyslot[slot] = tdc1190Type(ii);
    printf(">>> id=%d slot=%d type=%d\n",ii,slot,tdctypebyslot[slot]);
  }


#ifdef SLOTWORKAROUND
  for(ii=0; ii<ntdcs; ii++)
  {
vmeBusLock();
    slot = tdc1190GetGeoAddress(ii);
vmeBusUnlock();
	slotnums[ii] = slot;
    printf("[%d] slot %d\n",ii,slotnums[ii]);
  }
#endif


  /* if TDCs are present, set busy from P2 */
  if(ntdcs>0)
  {
    printf("Set BUSY from P2 for TDCs\n");
vmeBusLock();
    tiSetBusySource(TI_BUSY_P2,0);
vmeBusUnlock();
  }

  for(ii=0; ii<ntdcs; ii++)
  {
vmeBusLock();
    tdc1190Clear(ii);
vmeBusUnlock();
    error_flag[ii] = 0;
  }

  printf("V1190 Download() ends =========================\n\n");
#endif




#ifdef USE_SIS3801
  printf("SIS3801 Download() starts =========================\n");

vmeBusLock();

  mode = 2; /* Control Inputs mode = 2  */
  nsis = sis3801Init(0x200000, 0x100000, 2, mode);
  /*if(nsis>0) TDC_READ_CONF_FILE;*/

  for(id = 0; id < nsis; id++)
  {
    sis3801config(id, mode);
    sis3801control(id, DISABLE_EXT_NEXT);

    printf("    Status = 0x%08x\n",sis3801status(id));
  }

#if 0
  /* Set up the 0th scaler as the interrupt source */
  /* 2nd arg: vector = 0 := use default */
  scalIntInit(0, 0);

  /* Connect service routine */
  scalIntConnect(myISR, 0);
#endif

vmeBusUnlock();

  printf("SIS3801 Download() ends =========================\n\n");
#endif


#ifdef USE_V851
vmeBusLock();

  /*10KHz pulser (slot 13)*/
  v851Init(0xc000,0);
  v851_start(100000,0);

  /*1MHz pulser (slot 14)*/
  v851Init(0xd000,1);
  v851_start(1000000,1);

vmeBusUnlock();
#endif

#ifdef USE_MO
vmeBusLock();
/*moInit(0xa00000,0); done once in DiagGuiServer*/
  moConfigPrint();
vmeBusUnlock();
#endif


  sprintf(rcname,"RC%02d",rol->pid);
  printf("rcname >%4.4s<\n",rcname);

#ifdef SSIPC
  sprintf(ssname,"%s_%s",getenv("HOST"),rcname);
  printf("Smartsockets unique name >%s<\n",ssname);
  epics_msg_sender_init(expid, ssname); /* SECOND ARG MUST BE UNIQUE !!! */
#endif

  logMsg("INFO: User Download Executed\n",1,2,3,4,5,6);
}



static void
__prestart()
{
  int ii, i1, i2, i3;
  int ret, id;

  /* Clear some global variables etc for a clean start */
  *(rol->nevents) = 0;
  event_number = 0;

  tiEnableVXSSignals();

#ifdef POLLING_MODE
  CTRIGRSS(TIPRIMARY, TIR_SOURCE, usrtrig, usrtrig_done);
#else
  CTRIGRSA(TIPRIMARY, TIR_SOURCE, usrtrig, usrtrig_done);
#endif

  printf(">>>>>>>>>> next_block_level = %d, block_level = %d, use %d\n",next_block_level,block_level,next_block_level);
  block_level = next_block_level;


  /**************************************************************************/
  /* setting TI busy conditions, based on boards found in Download          */
  /* tiInit() does nothing for busy, tiConfig() sets fiber, we set the rest */
  /* NOTE: if ti is busy, it will not send trigger enable over fiber, since */
  /*       it is the same fiber and busy has higher priority                */

#ifndef TI_SLAVE
vmeBusLock();
tiSetBusySource(TI_BUSY_LOOPBACK,0);
  /*tiSetBusySource(TI_BUSY_FP,0);*/
vmeBusUnlock();
#endif


  /* USER code here */


#ifdef USE_DSC2
  printf("DSC2 Prestart() starts =========================\n");
  /* dsc2 configuration */
  if(ndsc2>0) DSC2_READ_CONF_FILE;
  printf("DSC2 Prestart() ends =========================\n\n");
#endif


#ifdef USE_V1190
  for(ii=0; ii<ntdcs; ii++)
  {
vmeBusLock();
    tdc1190SetBLTEventNumber(ii, block_level);
vmeBusUnlock();
  }
#endif


#ifdef USE_SIS3801

vmeBusLock();
  for(id = 0; id < nsis; id++)
  {
    /*sis3801clear(id);*/
    sis3801config(id, mode);
    sis3801control(id, DISABLE_EXT_NEXT);
  }
vmeBusUnlock();

#endif

vmeBusLock();
  tiIntDisable();
vmeBusUnlock();

  /* master and standalone crates, NOT slave */
#ifndef TI_SLAVE

  sleep(1);
vmeBusLock();
  tiSyncReset(1);
vmeBusUnlock();
  sleep(1);
vmeBusLock();
  tiSyncReset(1);
vmeBusUnlock();
  sleep(1);

vmeBusLock();
  ret = tiGetSyncResetRequest();
vmeBusUnlock();
  if(ret)
  {
    printf("ERROR: syncrequest still ON after tiSyncReset(); trying again\n");
    sleep(1);
vmeBusLock();
    tiSyncReset(1);
vmeBusUnlock();
    sleep(1);
  }

vmeBusLock();
  ret = tiGetSyncResetRequest();
vmeBusUnlock();
  if(ret)
  {
    printf("ERROR: syncrequest still ON after tiSyncReset(); try 'tcpClient <rocname> tiSyncReset'\n");
  }
  else
  {
    printf("INFO: syncrequest is OFF now\n");
  }

  printf("holdoff rule 1 set to %d\n",tiGetTriggerHoldoff(1));
  printf("holdoff rule 2 set to %d\n",tiGetTriggerHoldoff(2));

#endif

/* set block level in all boards where it is needed;
   it will overwrite any previous block level settings */




#if 0
#ifdef TI_SLAVE /* assume that for master and standalone TIs block level is set from config file */
vmeBusLock();
  tiSetBlockLevel(block_level);
vmeBusUnlock();
#endif
#endif
 printf("tiCurrentBlockLevel = %d, block_level = %d\n",tiGetCurrentBlockLevel(),block_level);


vmeBusLock();
  tiStatus(1);
vmeBusUnlock();

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
  int id, ii;

  printf("\n\nINFO: End1 Reached\n");fflush(stdout);

#ifdef USE_SIS3801

  for(id = 0; id < nsis; id++)
  {
vmeBusLock();
    sis3801control(id, DISABLE_EXT_NEXT);
vmeBusUnlock();
    printf("    Status = 0x%08x\n",sis3801status(id));
  }
#if 0
  scalIntDisable();
#endif

#endif

  CDODISABLE(TIPRIMARY,TIR_SOURCE,0);

  /* Before disconnecting... wait for blocks to be emptied */
vmeBusLock();
  blocksLeft = tiBReady();
vmeBusUnlock();
  printf(">>>>>>>>>>>>>>>>>>>>>>> %d blocks left on the TI\n",blocksLeft);fflush(stdout);
  if(blocksLeft)
  {
    printf(">>>>>>>>>>>>>>>>>>>>>>> before while ... %d blocks left on the TI\n",blocksLeft);fflush(stdout);
    while(iwait < 10)
	{
      taskDelay(10);
	  if(blocksLeft <= 0) break;
vmeBusLock();
	  blocksLeft = tiBReady();
      printf(">>>>>>>>>>>>>>>>>>>>>>> inside while ... %d blocks left on the TI\n",blocksLeft);fflush(stdout);
vmeBusUnlock();
	  iwait++;
	}
    printf(">>>>>>>>>>>>>>>>>>>>>>> after while ... %d blocks left on the TI\n",blocksLeft);fflush(stdout);
  }

vmeBusLock();
  tiStatus(1);
vmeBusUnlock();

  printf("INFO: End1 Executed\n\n\n");fflush(stdout);

  return;
}


static void
__pause()
{
  int id;

#ifdef USE_SIS3801

  for(id=0; id<nsis; id++)
  {
vmeBusLock();
    sis3801clear(id);
    sis3801control(id, DISABLE_EXT_NEXT);
vmeBusUnlock();
  }

#endif

  CDODISABLE(TIPRIMARY,TIR_SOURCE,0);
  logMsg("INFO: Pause Executed\n",1,2,3,4,5,6);
  
} /*end pause */


static void
__go()
{
  int ii, jj, id, slot;

  logMsg("INFO: Entering Go 1\n",1,2,3,4,5,6);

#ifndef TI_SLAVE
  /* set sync event interval (in blocks) */
vmeBusLock();
 tiSetSyncEventInterval(0/*10000*//*block_level*/);
vmeBusUnlock();
#endif

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

#ifdef USE_V1190
  for(jj=0; jj<ntdcs; jj++)
  {
vmeBusLock();
    tdc1190Clear(jj);
vmeBusUnlock();
    error_flag[jj] = 0;
  }
  taskDelay(100);

#endif

#ifdef USE_SIS3801
  run_trig_count = 0;
  for(id=0; id<nsis; id++)
  {
vmeBusLock();
    sis3801control(id, DISABLE_EXT_NEXT);
    sis3801clear(id);
vmeBusUnlock();
  }
#if 0
  /* Enable interrupts */
  scalIntEnable(0x1);
#endif

#endif

  /* always clear exceptions */
  vmeClearException(1);

  nusertrig = 0;
  ndone = 0;

  CDOENABLE(TIPRIMARY,TIR_SOURCE,0); /* bryan has (,1,1) ... */

  logMsg("INFO: Go 1 Executed\n",1,2,3,4,5,6);
}



void
usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE)
{
  int *jw, ind, ind2, i, ii, jj, kk, blen, len, rlen, itdcbuf, nbytes;
  unsigned int *tdcbuf_save, *tdc, utmp;
  unsigned int *dabufp1, *dabufp2;
  int njjloops, slot, type;
  int nwords, id, status;
  int dready = 0, timeout = 0, siswasread = 0;
#ifndef VXWORKS
  TIMERL_VAR;
#endif
#ifdef USE_V1190
  int nev, rlenbuf[22];
  unsigned long tdcslot, tdcchan, tdcval, tdc14, tdcedge, tdceventcount;
  unsigned long tdceventid, tdcbunchid, tdcwordcount, tdcerrorflags;
  unsigned int *tdchead;
#ifdef SLOTWORKAROUND
  unsigned long tdcslot_h, tdcslot_t, remember_h;
#endif
#endif
#ifdef DMA_TO_BIGBUF
  unsigned int pMemBase, uMemBase, mSize;
#endif
  char *chptr, *chptr0;

  /*printf("EVTYPE=%d syncFlag=%d\n",EVTYPE,syncFlag);*/

  if(syncFlag) printf("EVTYPE=%d syncFlag=%d\n",EVTYPE,syncFlag);

  rol->dabufp = NULL;

  /*
usleep(100);
  */
  /*
  sleep(1);
  */

  run_trig_count++;

#ifdef USE_SIS3801
  if(run_trig_count==1)
  {
    printf("First event - sis3801: ENABLE_EXT_NEXT\n");
vmeBusLock();
    for(id = 0; id < nsis; id++)
    {
      sis3801control(id, ENABLE_EXT_NEXT);
    }
vmeBusUnlock();
  }
#endif


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

    /* for EVIO format, will dump raw data */
    tdcbuf_save = tdcbuf;



    /*************/
    /* TI stuff */

    /* Grab the data from the TI */
vmeBusLock();
    len = tiReadBlock(tdcbuf,900>>2,1);
vmeBusUnlock();
    if(len<=0)
    {
      printf("ERROR in tiReadBlock : No data or error, len = %d\n",len);
      sleep(1);
    }
    else
    {
      /*
      printf("ti: len=%d\n",len);
      for(jj=0; jj<len; jj++) printf("ti[%2d] 0x%08x\n",jj,LSWAP(tdcbuf[jj]));
      */
	  
      BANKOPEN(0xe10A,1,rol->pid);
      for(jj=0; jj<len; jj++) *rol->dabufp++ = tdcbuf[jj];
      BANKCLOSE;
	  
    }

    /* TI stuff */
    /*************/





#ifndef VXWORKS
TIMERL_START;
#endif



    /*************/
    /* TDC stuff */

#ifdef USE_V1190
    if(ntdcs>0)
    {
vmeBusLock();
      tdc1190ReadStart(tdcbuf, rlenbuf);
vmeBusUnlock();

      itdcbuf = 0;
      njjloops = ntdcs;

      BANKOPEN(0xe10B,1,rol->pid);
      for(ii=0; ii<njjloops; ii++)
      {
        rlen = rlenbuf[ii];
	/*printf("rol1(TDCs): ii=%d, rlen=%d\n",ii,rlen);*/

	/*	  
#ifdef DEBUG
        level = tdc1190GetAlmostFullLevel(ii);
        iii = tdc1190StatusAlmostFull(ii);
        logMsg("ii=%d, rlen=%d, almostfull=%d level=%d\n",ii,rlen,iii,level,5,6);
#endif
	*/	  

        if(rlen <= 0) continue;

        tdc = &tdcbuf[itdcbuf];
        itdcbuf += rlen;


#ifdef SLOTWORKAROUND
	/* go through current board and fix slot number */
        for(jj=0; jj<rlen; jj++)
	{
          utmp = LSWAP(tdc[jj]);

          if( ((utmp>>27)&0x1F) == 8 ) /* GLOBAL HEADER */
	  {
            slot = utmp&0x1f;
            if( slot != slotnums[ii] )
	    {
              /*printf("ERROR: old=0x%08x: WRONG slot=%d IN GLOBAL HEADER, must be %d - fixed\n",utmp,slot,slotnums[ii]);*/
              utmp = (utmp & 0xFFFFFFE0) | slotnums[ii];
              /*printf("new=0x%08x\n",utmp);*/
              tdc[jj] = LSWAP(utmp);
            }
	  }
          else if( ((utmp>>27)&0x1F) == 0x10 ) /* GLOBAL TRAILER */
	  {
            slot = utmp&0x1f;
            if( slot != slotnums[ii] )
	    {
              /*printf("ERROR: old=0x%08x: WRONG slot=%d IN GLOBAL TRAILER, must be %d - fixed\n",utmp,slot,slotnums[ii]);*/
              utmp = (utmp & 0xFFFFFFE0) | slotnums[ii];
              /*printf("new=0x%08x\n",utmp);*/
              tdc[jj] = LSWAP(utmp);
            }
	  }
        }
#endif

        for(jj=0; jj<rlen; jj++)
	{
	  *rol->dabufp ++ = tdc[jj];
	  printf("TDC[%3d]=0x%08x\n",jj,LSWAP(tdc[jj]));
	}
      }
      BANKCLOSE;

	}

#endif /* USE_V1190 */

    /* TDC stuff */
    /*************/






    /*****************/
    /* SCALERS stuff */

#ifdef USE_SIS3801

    if(nsis>0)
    {
      /* get status from all boards */
      status = 0;
      for(ii=0; ii<nsis; ii++)
      {
vmeBusLock();
        status |= sis3801status(ii);
vmeBusUnlock();
      }

      /* if at least one board is full, reset */
      if(status & FIFO_FULL)
      {
        printf("SIS3801 IS FULL - CLEAN IT UP AND START AGAIN\n");fflush(stdout);

        for(id=0; id<nsis; id++)
        {
vmeBusLock();
          sis3801config(id, mode);
          sis3801control(id, ENABLE_EXT_NEXT);
vmeBusUnlock();
          printf("    Status = 0x%08x\n",sis3801status(id));
        }
      }
      else
      {
        siswasread = 0;
        for(ii=0; ii<nsis; ii++)
        {
          timeout = 0;
          dready = 0;
          while((dready == 0) && (timeout++ < 10))
          {
vmeBusLock();
            dready = (sis3801status(ii) & FIFO_EMPTY) ? 0 : 1;
vmeBusUnlock();
          }

          if(dready == 0)
	  {
            /*printf("NOT READY\n");fflush(stdout)*/;
	  }
          else
          {
            /*printf("READY =======================================\n");fflush(stdout);*/
            tdcbuf[0] = 10000;
vmeBusLock();
            len = sis3801read(ii, tdcbuf);
vmeBusUnlock();
            if(len>=9900/*10000*/) printf("WARN: sis3801[%d] returned %d bytes\n",ii,len);fflush(stdout);
            len = len >> 2;
            /*printf("\nsis3801[%d]: read %d words\n",ii,len);fflush(stdout);
            for(jj = 0; jj <len; jj++)
	    {
	      if((jj%4) == 0) printf("\n%4d: ", jj);
	      printf(" 0x%08x ",tdcbuf[jj]);
	    }
            printf("\n");
	    */
            BANKOPEN(0xe125,1,/*rol->pid*/ii);
            for(jj=0; jj<len; jj++) *rol->dabufp++ = LSWAP(tdcbuf[jj]);
            BANKCLOSE;

            siswasread = 1;
	  }
        }


#ifdef USE_DSC2
        if(siswasread)
	{
	  if(ndsc2_daq>0)
	  {
            BANKOPEN(0xe115,1,rol->pid);
            for(jj=0; jj<ndsc2_daq; jj++)
            {
              slot = dsc2Slot_daq(jj);
vmeBusLock();
              /* in following argument 4 set to 0xFF means latch and read everything, 0x3F - do not latch and read everything */
              nwords = dsc2ReadScalers(slot, tdcbuf, 0x10000, 0xFF, 1);
              /*printf("nwords=%d, nwords = 0x%08x 0x%08x 0x%08x 0x%08x\n",nwords,tdcbuf[0],tdcbuf[1],tdcbuf[2],tdcbuf[3]);*/
vmeBusUnlock();
              /* unlike other boards, dcs2 scaler readout already swapped in 'dsc2ReadScalers', so swap it back, because
              rol2.c expects big-endian format*/
              for(kk=0; kk<nwords; kk++) *rol->dabufp ++ = LSWAP(tdcbuf[kk]);
            }
            BANKCLOSE;
	  }
	}
#endif


      }
    }
#endif

    /* SCALERS stuff */
    /*****************/







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

      /* COUNT DATA WORDS FROM HERE */
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
      /* END OF DATA WORDS */

    }

    nwords = ((long int)rol->dabufp-(long int)dabufp1)/4 + 1;

    *rol->dabufp ++ = LSWAP((0x11<<27)+nwords); /*block trailer*/

    BANKCLOSE;

#endif





#ifndef VXWORKS
TIMERL_STOP(100000/block_level,1000+rol->pid);
#endif







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

vmeBusLock();
      len = tiUploadAll(chptr, 10000);
vmeBusUnlock();
      /*printf("\nTI len=%d\n",len);
      printf(">%s<\n",chptr);*/
      chptr += len;
      nbytes += len;

#ifdef USE_V1190_HIDE
	  if(ntdcs>0)
	  {
vmeBusLock();
        len = tdc1190UploadAll(chptr, 10000);
vmeBusUnlock();
        /*printf("\nTDC len=%d\n",len);
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











#ifndef TI_SLAVE
    /* print livetime */
    if(syncFlag==1)
	{
      printf("SYNC: livetime\n");

      int livetime, live_percent;
vmeBusLock();
      tiLatchTimers();
      livetime = tiLive(0);
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




    /* for physics sync event, make sure all board buffers are empty */
    if(syncFlag==1)
    {
      printf("SYNC: make sure all board buffers are empty\n");

      int nblocks;
      nblocks = tiGetNumberOfBlocksInBuffer();
      /*printf(" Blocks ready for readout: %d\n\n",nblocks);*/

      if(nblocks)
	  {
        printf("SYNC ERROR: TI nblocks = %d\n",nblocks);fflush(stdout);
        sleep(10);
	  }
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
  CDOACK(TIPRIMARY,TIR_SOURCE,0);

  return;
}

static void
__status()
{
  return;
}  

#else

void
scaler1_dummy()
{
  return;
}

#endif

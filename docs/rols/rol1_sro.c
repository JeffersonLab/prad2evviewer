
/* rol1.c - 'standard' first readout list */

#if defined(VXWORKS) || defined(Linux_vme)

#define NEW

#undef SSIPC

static int nusertrig, ndone;

#undef DMA_TO_BIGBUF /*if want to dma directly to the big buffers*/

#define USE_FADC250
#define USE_DSC2
#define USE_V1190
#define USE_SSP
#define USE_SSP_RICH
#define USE_VSCM
#define USE_DCRB
#define USE_VETROC
//#define USE_FLP


//#define USE_ED

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
#include <sys/stat.h>


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
#include "sdLib.h"
#include "vscmLib.h"
#include "dcrbLib.h"
#include "sspLib.h"
#include "sspConfig.h"
#include "fadcLib.h"
#include "fadc250Config.h"
#include "vetrocLib.h"
#include "tiLib.h"
#include "tiConfig.h"
#include "dsc2Lib.h"
#include "dsc2Config.h"

#include "circbuf.h"

/* from fputil.h */
#define SYNC_FLAG 0x20000000


/* polling mode if needed */
#define POLLING_MODE

/* main TI board */
#define TI_ADDR   (21<<19)  /* if 0 - default will be used, assuming slot 21*/




/* readout list name, and name used by loader */

#ifdef TI_ASYNC

#define ROL_NAME__ "ROL1_ASYNC"
#ifdef TI_MASTER
#define INIT_NAME rol1_async_master__init
#define TI_READOUT TI_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME rol1_async_slave__init
#define TI_READOUT TI_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME rol1_async__init
#define TI_READOUT TI_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#else

#define ROL_NAME__ "ROL1_SRO"
#ifdef TI_MASTER
#define INIT_NAME rol1_sro_master__init
#define TI_READOUT TI_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#else
#ifdef TI_SLAVE
#define INIT_NAME rol1_sro_slave__init
#define TI_READOUT TI_READOUT_TS_POLL /* Poll for available data, triggers from master over fiber */
#else
#define INIT_NAME rol1_sro__init
#define TI_READOUT TI_READOUT_EXT_POLL /* Poll for available data, front panel triggers */
#endif
#endif

#endif



#include "rol.h"

void usrtrig(unsigned int EVTYPE, unsigned int EVSOURCE);
void usrtrig_done();

#include "TIPRIMARY_source.h"



/* user code */

/*
#include "uthbook.h"
#include "coda.h"
#include "tt.h"
*/
#include "scaler7201.h"
#include "c792Lib.h"
#include "tdc1190.h"
#include "vscmLib.h"


#ifdef USE_SSP
#include "sspLib.h"
static int nssp;   /* Number of SSPs found with sspInit(..) */
static int ssp_not_ready_errors[21];
#endif

static char rcname[5];

#define NBOARDS 22    /* maximum number of VME boards: we have 21 boards, but numbering starts from 1 */
#define MY_MAX_EVENT_LENGTH 3000/*3200*/ /* max words per board */
static unsigned int *tdcbuf;

/*#ifdef DMA_TO_BIGBUF*/
/* must be 'rol' members, like dabufp */
extern unsigned int dabufp_usermembase;
extern unsigned int dabufp_physmembase;
/*#endif*/

extern char configname[128]; /* coda_component.c */

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

#ifdef USE_VETROC
#include "vetrocLib.h"
extern int nvetroc;                   /* Number of VETROCs in Crate */
static int VETROC_SLOT;
static int VETROC_ROFLAG = 1;           /* 0-noDMA, 1-board-by-board DMA, 2-chainedDMA */

/* for the calculation of maximum data words in the block transfer */
static unsigned int MAXVETROCWORDS = 10000;
static unsigned int vetrocSlotMask; /* bit=slot (starting from 0) */
#endif

#ifdef USE_DCRB
#include "dcrbLib.h"
extern int ndcrb;                     /* Number of DCs in Crate */
static int DCRB_SLOT;
static int DCRB_ROFLAG = 1;           /* 0-noDMA, 1-board-by-board DMA, 2-chainedDMA */

/* for the calculation of maximum data words in the block transfer */
static unsigned int MAXDCRBWORDS = 10000;
static unsigned int dcrbSlotMask; /* bit=slot (starting from 0) */
#endif

#ifdef USE_DC
#include "dcLib.h"
extern int ndc;                     /* Number of DCs in Crate */
static int DC_SLOT;
static int DC_ROFLAG = 1;           /* 0-noDMA, 1-board-by-board DMA, 2-chainedDMA */

/* for the calculation of maximum data words in the block transfer */
static unsigned int MAXDCWORDS = 10000;
static unsigned int dcSlotMask; /* bit=slot (starting from 0) */
#endif

#ifdef USE_SSP
static unsigned int sspSlotMask = 0; /* bit=slot (starting from 0) */
static int SSP_SLOT;
#endif

#ifdef USE_FLP
#include "flpLib.h"
static int nflp;
#endif

#ifdef USE_VSCM
static unsigned int vscmSlotMask = 0; /* bit=slot (starting from 0) */
static int nvscm1 = 0;
static int VSCM_ROFLAG = 1;
#endif


static int sd_found = 0;


#ifdef USE_FADC250

#include "fadcLib.h"
extern int fadcBlockError; /* defined in fadcLib.c */

#define DIST_ADDR  0xEA00	  /*  base address of FADC signal distribution board  (A16)  */


unsigned int fadcSlotMask   = 0;    /* bit=slot (starting from 0) */
static int nfadc;                 /* Number of FADC250s verified with the library */
static int NFADC;                   /* The Maximum number of tries the library will
                                     * use before giving up finding FADC250s */
static int FA_SLOT;                 /* We'll use this over and over again to provide
				                     * us access to the current FADC slot number */ 

static int FADC_ROFLAG           = 2;  /* 0-noDMA, 1-board-by-board DMA, 2-chainedDMA */

/* for the calculation of maximum data words in the block transfer */
static unsigned int MAXFADCWORDS = 0;
static unsigned int MAXTIWORDS  = 0;
static unsigned int MAXVSCMWORDS  = 100000;


/* IC lookup tables */

/*tdc's in slots 19 and 20*/
int ic_tdc_high[2][128] = {
0*256
};

int ic_tdc_low[2][128] = {
0*256
};



/*adc's in slots 3-10 and 13-18*/

char *
getFadcPedsFilename(int rocid)
{
  char *dir = NULL;
  /*char *expid = NULL;*/
  static char fname[1024];

  if((dir=getenv("CLAS")) == NULL)
  {
    printf("ERROR: environment variable CLAS is not defined - exit\n");
    return(NULL);
  }
  /*
  if((expid=getenv("EXPID")) == NULL)
  {
    printf("ERROR: environment variable EXPID is not defined - exit\n");
    return(NULL);
  }
  */
  sprintf(fname,"%s/parms/peds/%s/fadc%02d.ped",dir,expid,rocid);

  return(fname);
}

#endif

static void
__download()
{
  int i1, i2, i3;
  char *ch, tmp[64];
  int ret;

#ifdef USE_FADC250
  int ii, id, isl, ichan, slot;
  unsigned short iflag;
  int fadc_mode = 1, iFlag = 0;
  int ich, NSA, NSB;
  unsigned int maxA32Address;
  unsigned int fadcA32Address = 0x09000000;
#endif
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

  /*
  printf("rol1: downloading DDL table ...\n");
  clonbanks_();
  printf("rol1: ... done.\n");
  */

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
 
// TESTING - BEN 
  tiEnableVXSSignals();


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

  usrVmeDmaSetConfig(2,5,1); /*A32,2eSST,267MB/s*/
  /*usrVmeDmaSetConfig(2,5,0);*/ /*A32,2eSST,160MB/s*/
  /*usrVmeDmaSetConfig(2,3,0);*/ /*A32,MBLT*/



  /*
if(rol->pid==18)
{
  usrVmeDmaSetConfig(2,3,0);
}
  */


  /*
  usrVmeDmaSetChannel(1);
  printf("===== Use DMA Channel %d\n\n\n",usrVmeDmaGetChannel());
  */

  tdcbuf = (unsigned int *)i2_from_rol1;







  /******************/
  /* USER code here */


vmeBusLock();
  ret = sdInit(1);  /* Initialize the SD library; will use 'sd_found' later */
  if(ret >= 0)
  {
    sd_found = 1;
  }
  else
  {
    printf("\n\nsdInit returns %d, probably SD does not installed in that crate ..\n\n\n",ret);
    sd_found = 0;
  }
vmeBusUnlock();



#ifdef USE_FADC250
  printf("FADC250 Download() starts =========================\n");

  /* Here, we assume that the addresses of each board were set according
   to their geographical address (slot number): on FADC250 it must be set by jumpers,
   while some other boards (TI, DSC2 etc) can set it automatically if jumpers are set to 0

   * Slot  2:  (2<<19) = 0x00100000

   * Slot  3:  (3<<19) = 0x00180000
   * Slot  4:  (4<<19) = 0x00200000
   * Slot  5:  (5<<19) = 0x00280000
   * Slot  6:  (6<<19) = 0x00300000
   * Slot  7:  (7<<19) = 0x00380000
   * Slot  8:  (8<<19) = 0x00400000
   * Slot  9:  (9<<19) = 0x00480000
   * Slot 10: (10<<19) = 0x00500000

   * Slot 11: (11<<19) = 0x00580000
   * Slot 12: (12<<19) = 0x00600000

   * Slot 13: (13<<19) = 0x00680000
   * Slot 14: (14<<19) = 0x00700000
   * Slot 15: (15<<19) = 0x00780000
   * Slot 16: (16<<19) = 0x00800000
   * Slot 17: (17<<19) = 0x00880000
   * Slot 18: (18<<19) = 0x00900000
   * Slot 19: (19<<19) = 0x00980000
   * Slot 20: (20<<19) = 0x00A00000

   * Slot 21: (21<<19) = 0x00A80000

   */

  NFADC = 16 + 2; /* 16 slots + 2 (for the switch slots) */

  /* NOTE: starting from 'fadcA32Base' address, memory chunks size=FA_MAX_A32_MEM(=0x800000)
							will be used for every board in A32Blk space:
adc      A32BLK address
1        0x09000000
2        0x09800000
3        0x0A000000
4        0x0A800000
5        0x0B000000
6        0x0B800000
7        0x0C000000
8        0x0C800000
9        0x0D000000
10       0x0D800000
11       0x0E000000
12       0x0E800000
13       0x0F000000
14       0x0F800000
15       0x10000000
16       0x10800000

DSC2: the same as FADCs

CAEN BOARDS IN A32 SPACE MUST BE USING ADDRESSES FROM 0x11000000 AND ABOVE !!!

v1495: 0x11xx0000, where xx follows the same scheme as FADCs
v1190: 0x11xx0000, where xx follows the same scheme as FADCs

*/

  /* Setup the iFlag.. flags for FADC initialization */
  iFlag = 0;
  /* base address */
  iFlag = (DIST_ADDR)<<10;
  /* Sync Source */
  iFlag |= (1<<0);    /* VXS */


  if(sd_found)
  {
    printf("Assume SD usage for FADCs\n");

    /* Trigger Source */
    iFlag |= (1<<2);    /* VXS */
    /* Clock Source */
    /*iFlag |= (1<<5);*/    /* VXS */
    iFlag |= (0<<5);  /* Internal Clock Source */
  }
  else
  {
    printf("Assume SDC usage for FADCs\n");

    /* Trigger Source - have to do it to make faInit() configure for SDC board */
    iFlag |= (1<<1);    /* Front Panel */

    /* Clock Source */
    iFlag |= (1<<4);    /* Front Panel */
    /*iFlag |= (0<<5);*/  /* Internal Clock Source */

    /* SDC address */
    iFlag |= (0xea<<8);
  }



#ifndef VXWORKS
  vmeSetQuietFlag(1); /* skip the errors associated with BUS Errors */
#endif

  faSetA32BaseAddress(fadcA32Address);
vmeBusLock();
  faInit((unsigned int)(3<<19),(1<<19),NFADC,iFlag); /* start from 0x00180000, increment 0x00080000 */
vmeBusUnlock();

  faGetMinA32MB(0);
  faGetMaxA32MB(0);

  nfadc = faGetNfadc(); /* acual number of FADC boards found  */
#ifndef VXWORKS
  vmeSetQuietFlag(0); /* Turn the error statements back on */
#endif

  if(nfadc>0)
  {
    if(nfadc==1) FADC_ROFLAG = 1; /*no chainedDMA if one board only*/
    if(!sd_found) FADC_ROFLAG = 1; /*no chainedDMA if no SD*/

    if(FADC_ROFLAG==2) faEnableMultiBlock(1);
    else faDisableMultiBlock();

    /* configure all modules based on config file */
    FADC_READ_CONF_FILE;

    /* Additional Configuration for each module */
    fadcSlotMask=0;
    for(id=0; id<nfadc; id++) 
    {
      FA_SLOT = faSlot(id);      /* Grab the current module's slot number */
      fadcSlotMask |= (1<<FA_SLOT); /* Add it to the mask */
      printf("=======================> fadcSlotMask=0x%08x",fadcSlotMask);

	  {
        unsigned int PL, PTW, NSB, NSA, NP;
vmeBusLock();
        faGetProcMode(FA_SLOT, &fadc_mode, &PL, &PTW, &NSB, &NSA, &NP);
vmeBusUnlock();
        printf(", slot %d, fadc_mode=%d\n",FA_SLOT,fadc_mode);
	  }

      /* Bus errors to terminate block transfers (preferred) */
vmeBusLock();
      faEnableBusError(FA_SLOT);
vmeBusUnlock();

#ifdef NEW
      /*****************/
      /*trigger-related*/
vmeBusLock();
      faResetMGT(FA_SLOT,1);
      faSetTrigOut(FA_SLOT, 7);
vmeBusUnlock();
#endif

	  /*****************/
	  /*****************/
    }


    /* 1) Load FADC pedestals from file for trigger path.
       2) Offset FADC threshold for each channel based on pedestal for both readout and trigger */
	if(rol->pid>36 && rol->pid!=46 && rol->pid!=37 && rol->pid!=39 && rol->pid!=58)
    {
vmeBusLock();
      faGLoadChannelPedestals(getFadcPedsFilename(rol->pid), 1);
vmeBusUnlock();
    }
    /* read back and print trigger pedestals */
/*
    printf("\n\nTrigger pedestals readback\n");
    for(id=0; id<nfadc; id++) 
    {
      FA_SLOT = faSlot(id);
      for(ichan=0; ichan<16; ichan++)
      {
        printf("  slot=%2d chan=%2d ped=%5d\n",FA_SLOT,ichan,faGetChannelPedestal(FA_SLOT, ichan));
      }
    }
    printf("\n\n");
*/
  }


/*
STATUS for FADC in slot 18 at VME (Local) base address 0x900000 (0xa16b1000)
---------------------------------------------------------------------- 
 Board Firmware Rev/ID = 0x020e : ADC Processing Rev = 0x0907
 Alternate VME Addressing: Multiblock Enabled
   A32 Enabled at VME (Local) base 0x0f800000 (0xa95b1000)
   Multiblock VME Address Range 0x10800000 - 0x11000000

 Signal Sources: 
   Ref Clock : Internal
   Trig Src  : VXS (Async)
   Sync Reset: VXS (Async)

 Configuration: 
   Internal Clock ON
   Bus Error ENABLED
   MultiBlock transfer ENABLED (Last Board  - token via VXS)

 ADC Processing Configuration: 
   Channel Disable Mask = 0x0000
   Mode = 1  (ENABLED)
   Lookback (PL)    = 1360 ns   Time Window (PTW) = 400 ns
   Time Before Peak = 12 ns   Time After Peak   = 24 ns
   Max Peak Count   = 1 
   Playback Mode    = 0 

  CSR       Register = 0x00001800
  Control 1 Register = 0x10b00338 
  Control 2 Register = 0x00000000 - Disabled
  Internal Triggers (Live) = 0
  Trigger   Scaler         = 0
  Events in FIFO           = 0  (Block level = 1)
  MGT Status Register      = 0x00000400 
  BERR count (from module) = 0
*/

  /***************************************
   *   SD SETUP
   ***************************************/
vmeBusLock();
  /*sd_found = sdInit(1); moved before anything else*/   /* Initialize the SD library */
  if(sd_found)
  {
    sdSetActiveVmeSlots(fadcSlotMask); /* Use the fadcSlotMask to configure the SD */
    sdStatus();
    sdSetTrigoutLogic(0, 2); /* Enable SD trigout as OR by default */
  }
vmeBusUnlock();




#ifdef USE_FADC250
  /* if FADCs are present, set busy from SD board */
  if(nfadc>0)
  {
    printf("Set BUSY from SWB for FADCs\n");
vmeBusLock();
    if(sd_found) tiSetBusySource(TI_BUSY_SWB,0);
    else tiSetBusySource(TI_BUSY_FP_FADC,0);
vmeBusUnlock();
  }
#endif


  /*****************************************************************/
  /*****************************************************************/

#ifdef USE_FADC250
  printf("FADC250 Prestart() starts =========================\n");

  /* Program/Init VME Modules Here */
  for(id=0; id<nfadc; id++)
  {
    FA_SLOT = faSlot(id);
vmeBusLock();
    if(sd_found) faSetClockSource(FA_SLOT,2);
    else         faSetClockSource(FA_SLOT,/*0*/1); /* 0-internal, 1-front panel*/
vmeBusUnlock();
  }

  sleep(1);

  for(id=0; id<nfadc; id++)
  {
    FA_SLOT = faSlot(id);
vmeBusLock();
    faSoftReset(FA_SLOT,0); /*0-soft reset, 1-soft clear*/
vmeBusUnlock();

#ifdef NEW
    if(!faGetMGTChannelStatus(FA_SLOT))
    {
vmeBusLock();
      faResetMGT(FA_SLOT,1);
      faResetMGT(FA_SLOT,0);
vmeBusUnlock();
    }
#endif

vmeBusLock();
    faResetToken(FA_SLOT);
    faResetTriggerCount(FA_SLOT);
    faStatus(FA_SLOT,0);
    faPrintThreshold(FA_SLOT);
vmeBusUnlock();
  }

  /*  Enable FADC */
  for(id=0; id<nfadc; id++) 
  {
    FA_SLOT = faSlot(id);
    /*faSetMGTTestMode(FA_SLOT,0);*/
    /*faChanDisable(FA_SLOT,0xffff);enabled in download*/
vmeBusLock();
    faEnable(FA_SLOT,0,0);
    /*faSetCompression(FA_SLOT,0);*/
vmeBusUnlock();
  }

  faGStatus(0);

  printf("FADC250 Prestart() ends =========================\n\n");
  sleep(2);
#endif


  printf("FADC250 Download() ends =========================\n\n");
#endif





  
  /*******************************/
  /*DCRB moved here from Prestart*/


#ifdef USE_DCRB
  printf("DCRB starts =========================\n");

#ifndef VXWORKS
  vmeSetQuietFlag(1); /* skip the errors associated with BUS Errors */
#endif

vmeBusLock();
  ndcrb = dcrbInit((3<<19), 0x80000, 20, 7); /* 7 boards from slot 3, 7 boards from slot 14 */
  dcrbSetDAC_Pulser(3,0x38,10000.0,0,0,1000,100); /*slot 3, front-end inj: 0x38, 10000.0Hz, offset=0, low=0, high=1000, width=100 */
  if(ndcrb>0)
  {
    DCRB_READ_CONF_FILE;

    /*dcrbGSetDAC(-30); MUST COME FROM CONFIG FILE */ /* threshold in mV */

    /* last param is double-hit resolution in ns */
    /*dcrbGSetProcMode(2000,2000,1000); MUST COME FROM CONFIG FILE */
  }
vmeBusUnlock();

#ifndef VXWORKS
  vmeSetQuietFlag(0); /* Turn the error statements back on */
#endif

  if(ndcrb==1) DCRB_ROFLAG = 1; /*no chainedDMA if one board only*/
  if((ndcrb>0) && (DCRB_ROFLAG==2)) dcrbEnableMultiBlock(1);
  else if(ndcrb>0) dcrbDisableMultiBlock();

  /* Additional Configuration for each module */
  dcrbSlotMask=0;
  for(ii=0; ii<ndcrb; ii++) 
  {
    DCRB_SLOT = dcrbSlot(ii);      /* Grab the current module's slot number */
    dcrbSlotMask |= (1<<DCRB_SLOT); /* Add it to the mask */
    printf("=======================> dcrbSlotMask=0x%08x\n",dcrbSlotMask);
  }

  /* DCRB stuff */

  for(id=0; id<ndcrb; id++) 
  {
    DCRB_SLOT = dcrbSlot(id);
vmeBusLock();
//    dcrbTriggerPulseWidth(DCRB_SLOT, 8000);
	dcrbLinkReset(DCRB_SLOT);
vmeBusUnlock();

    /* will try to reset 5 times
    for(ii=0; ii<5; ii++)
	{
      if(dcrbLinkStatus(DCRB_SLOT)) break;
	  printf("Reseting link at slot %d\n",DCRB_SLOT);
vmeBusLock();
      dcrbLinkReset(DCRB_SLOT);
vmeBusUnlock();
	}
    */

  }


  for(id=0; id<ndcrb; id++)
  {
    DCRB_SLOT = dcrbSlot(id);
    if(dcrbLinkStatus(DCRB_SLOT)) printf("Link at slot %d is UP\n",DCRB_SLOT);
    else printf("Link at slot %d is DOWN\n",DCRB_SLOT);
  }


  /* SD Trigout when DCRB multiplicity >= 1
  if(sd_found)
  {
    sdSetTrigoutLogic(0, 1);
  }
*/


  printf("DCRB ends =========================\n\n");




#endif



  /*******************************/
  /*******************************/







  /*write FADC/DCRB/etc masks to the temporary file so vtp can read it - until figure out better solution */
{
  FILE *fdout;
  char fileout[256], str[256];

  sprintf(fileout,"%s/sro/%s.txt",getenv("CLON_PARMS"),getenv("HOST"));
  if((fdout=fopen(fileout,"w")) == NULL)
  {
    printf("Cannot open output file >%s< - exit\n",fileout);
    return;
  }
  else
  {
    printf("Opened output file >%s< for writing\n",fileout);
  }

#ifdef USE_FADC250
  sprintf(str,"%d 0x%08x 0x1\n",rol->pid,fadcSlotMask);
  printf("rocid, fadcSlotMask is >%s<",str);
  fputs(str,fdout);
#endif

#ifdef USE_DCRB
  sprintf(str,"%d 0x%08x 0x2\n",rol->pid,dcrbSlotMask);
  printf("rocid, dcrbSlotMask is >%s<",str);
  fputs(str,fdout);
#endif
  
  if(chmod(fileout,S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH) != 0)
  {
    printf("ERROR: cannot change mode on output file\n");
  }
  fclose(fdout);
}


  




  
/* master and standalone crates, NOT slave: Assert SYNC reset*/
#ifndef TI_SLAVE
vmeBusLock();

/*tiSyncReset(1); not sure if it is needed */

  printf("Assert SYNC\n");
  tiUserSyncReset(1,1);
vmeBusUnlock();
#endif

  sprintf(rcname,"RC%02d",rol->pid);
  printf("rcname >%4.4s<\n",rcname);

  printf("configname >%s<\n",configname);
  printf("configname >%s<\n",configname);
  printf("configname >%s<\n",configname);
  printf("configname >%s<\n",configname);
  printf("configname >%s<\n",configname);

#ifdef TI_MASTER
  /* update tridas json by hosts/ports */
  /*tridasJsonUpdate(configname);*/
#endif

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
  int ret;
#ifdef USE_FADC250
  int id, isl, ichan, slot;
  unsigned short iflag;
  int iFlag = 0;
  int ich;
  unsigned short aa = 0;
  unsigned short bb;
  unsigned short thr = 400;
#endif

  /* Clear some global variables etc for a clean start */
  *(rol->nevents) = 0;
  event_number = 0;

//tiEnableVXSSignals();

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
















#ifdef USE_DCRB
  printf("DCRB starts =========================\n");

#ifndef VXWORKS
  vmeSetQuietFlag(1); /* skip the errors associated with BUS Errors */
#endif

vmeBusLock();
  ndcrb = dcrbInit((3<<19), 0x80000, 20, 7); /* 7 boards from slot 3, 7 boards from slot 14 */
  dcrbSetDAC_Pulser(3,0x38,10000.0,0,0,1000,100); /*slot 3, front-end inj: 0x38, 10000.0Hz, offset=0, low=0, high=1000, width=100 */
  if(ndcrb>0)
  {
    DCRB_READ_CONF_FILE;

    /*dcrbGSetDAC(-30); MUST COME FROM CONFIG FILE */ /* threshold in mV */

    /* last param is double-hit resolution in ns */
    /*dcrbGSetProcMode(2000,2000,1000); MUST COME FROM CONFIG FILE */
  }
vmeBusUnlock();

#ifndef VXWORKS
  vmeSetQuietFlag(0); /* Turn the error statements back on */
#endif

  if(ndcrb==1) DCRB_ROFLAG = 1; /*no chainedDMA if one board only*/
  if((ndcrb>0) && (DCRB_ROFLAG==2)) dcrbEnableMultiBlock(1);
  else if(ndcrb>0) dcrbDisableMultiBlock();

  /* Additional Configuration for each module */
  dcrbSlotMask=0;
  for(ii=0; ii<ndcrb; ii++) 
  {
    DCRB_SLOT = dcrbSlot(ii);      /* Grab the current module's slot number */
    dcrbSlotMask |= (1<<DCRB_SLOT); /* Add it to the mask */
    printf("=======================> dcrbSlotMask=0x%08x\n",dcrbSlotMask);
  }

  /* DCRB stuff */

  for(id=0; id<ndcrb; id++) 
  {
    DCRB_SLOT = dcrbSlot(id);
vmeBusLock();
//    dcrbTriggerPulseWidth(DCRB_SLOT, 8000);
	dcrbLinkReset(DCRB_SLOT);
vmeBusUnlock();

    /* will try to reset 5 times
    for(ii=0; ii<5; ii++)
	{
      if(dcrbLinkStatus(DCRB_SLOT)) break;
	  printf("Reseting link at slot %d\n",DCRB_SLOT);
vmeBusLock();
      dcrbLinkReset(DCRB_SLOT);
vmeBusUnlock();
	}
    */

  }


  for(id=0; id<ndcrb; id++)
  {
    DCRB_SLOT = dcrbSlot(id);
    if(dcrbLinkStatus(DCRB_SLOT)) printf("Link at slot %d is UP\n",DCRB_SLOT);
    else printf("Link at slot %d is DOWN\n",DCRB_SLOT);
  }


  /* SD Trigout when DCRB multiplicity >= 1
  if(sd_found)
  {
    sdSetTrigoutLogic(0, 1);
  }
*/


  /*random pulser if need to generate artificial data*/
  for(id=0; id<ndcrb; id++)
  {
    int grp_mask = 0x38;
    int offset_mV = 0;
    int low_mV = -100;
    int high_mV = 100;
    int width = 10; //?
    float freq = 1000000.0;

    DCRB_SLOT = dcrbSlot(id);
    printf("Calling dcrbSetDAC_Pulser(%d, %d, %f, %d, %d, %d, %d)\n",DCRB_SLOT, grp_mask, freq, offset_mV, low_mV, high_mV, width);
    dcrbSetDAC_Pulser(DCRB_SLOT, grp_mask, freq, offset_mV, low_mV, high_mV, width);
  }

  printf("DCRB ends =========================\n\n");




#endif

















 

  /* USER code here */
  /******************/
vmeBusLock();
  tiIntDisable();
vmeBusUnlock();

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
  printf("\n\nINFO: End1 Reached\n");fflush(stdout);

  CDODISABLE(TIPRIMARY,TIR_SOURCE,0);

  printf("INFO: End1 Executed\n\n\n");fflush(stdout);

  return;
}


static void
__pause()
{
  CDODISABLE(TIPRIMARY,TIR_SOURCE,0);
  logMsg("INFO: Pause Executed\n",1,2,3,4,5,6);
  
} /*end pause */


static void
__go()
{
  logMsg("INFO: Entering Go 1\n",1,2,3,4,5,6);

/* master and standalone crates, NOT slave: Release SYNC reset*/
#ifndef TI_SLAVE
vmeBusLock();
  printf("Release SYNC\n");
  tiUserSyncReset(0,1);
vmeBusUnlock();
#endif

  /* always clear exceptions */
  //jlabgefClearException(1);

  nusertrig = 0;
  ndone = 0;

#ifdef TI_ASYNC
  CDOENABLE(TIPRIMARY,TIR_SOURCE,0); /* bryan has (,1,1) ... */
#else
  CDODISABLE(TIPRIMARY,TIR_SOURCE,0);
#endif

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
fadc1_dummy()
{
  return;
}

#endif


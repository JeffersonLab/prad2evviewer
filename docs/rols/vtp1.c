
/* vtp1.c - first readout list for VTP boards (polling mode) */


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

#include "daqLib.h"
#include "vtpLib.h"
#include "vtpConfig.h"

#include "circbuf.h"

/*****************************/
/* former 'crl' control keys */

/* readout list VTP1 */
#define ROL_NAME__ "VTP1"

/* polling */
#define POLLING_MODE


/* name used by loader */
#define INIT_NAME vtp1__init

#include "rol.h"

void usrtrig(unsigned long, unsigned long);
void usrtrig_done();

/* vtp readout */
#include "VTP_source.h"

#define READOUT_TI
#define READOUT_VTP
#define USE_DMA

#ifdef USE_DMA
  #define MAXBUFSIZE 100000
  unsigned long gDmaBufPhys_TI;
  unsigned long gDmaBufPhys_VTP;
  unsigned int gFixedBuf[MAXBUFSIZE];
#else
  #define MAXBUFSIZE 4000
  unsigned int gFixedBuf[MAXBUFSIZE];
#endif

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

static void
__download()
{

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

  /* user code */

  printf("INFO: User Download 1 Executed\n");

  return;
}


static void
__prestart()
{
  int i, ret;
  unsigned long jj, adc_id, sl;
  char *env;
  char *myhost = getenv("HOST");
  char tmp[256];

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

  printf("calling VTP_READ_CONF_FILE ..\n");fflush(stdout);
  VTP_READ_CONF_FILE;
  
#ifdef USE_DMA
  vtpDmaMemOpen(2, MAXBUFSIZE*4);  //was in Download ...
  vtpDmaInit(VTP_DMA_TI);
  vtpDmaInit(VTP_DMA_VTP);
#endif

  ret = vtpSerdesCheckLinks();
  if(ret==0) /*at least on link is down*/
  {
    sprintf(tmp,"%s",myhost);
    printf("ERROR: Prestart 1: SERDES LINK(S) DOWN, REBOOT ROC AND START NEW RUN FROM 'CONFIGURE !!!\n");
    printf("ERROR: Prestart 1: SERDES LINK(S) DOWN, REBOOT ROC AND START NEW RUN FROM 'CONFIGURE !!!\n");
    printf("ERROR: Prestart 1: SERDES LINK(S) DOWN, REBOOT ROC AND START NEW RUN FROM 'CONFIGURE !!!\n");
    UDP_user_request(MSGERR, tmp, "SERDES LINK(S) DOWN, REBOOT ROC AND START NEW RUN FROM 'CONFIGURE !!!");
  }

  printf("INFO: User Prestart 1 executed\n");

  /* from parser (do we need that in rol2 ???) */
  *(rol->nevents) = 0;
  rol->recNb = 0;

  return;
}

static void
__end()
{
  int ii, total_count, rem_count;

  CDODISABLE(VTP,1,0);

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

static void
__go()
{
  int i;
  char *env;

#ifdef READOUT_TI
  /* Clear TI Link recieve FIFO */
  vtpTiLinkResetFifo(1);
  
  /* If there's an error in the status, re-initialize */
  if(vtpTiLinkStatus() == ERROR)
    {
      printf("%s: WARN: Error from TI Link status.  Resetting.\n",
	     __func__);
      vtpTiLinkInit();
    }
#endif

  vtpSerdesStatusAll();

#ifdef READOUT_VTP
  block_level = vtpTiLinkGetBlockLevel(0);
  printf("Setting VTP block level to: %d\n", block_level);
  vtpSetBlockLevel(block_level);

  vtpV7SetResetSoft(1);
  vtpV7SetResetSoft(0);

  vtpTiLinkResetFifo(0); // same as vtpEbResetFifo() on older z7
#endif

/* Do DMA readout before Go enabled to clear out any buffered data - hack fix until problem with extra TI block header from past run is found */
#ifdef READOUT_TI
  #ifdef USE_DMA
    vtpDmaStart(VTP_DMA_TI, vtpDmaMemGetPhysAddress(0), MAXBUFSIZE*4);
    vtpDmaWaitDone(VTP_DMA_TI);
    vtpDmaStatus(0);
  #else
    vtpEbTiReadEvent(gFixedBuf, MAXBUFSIZE);
  #endif
  vtpTiLinkStatus();
#endif

#ifdef READOUT_VTP
  #ifdef USE_DMA
    if(vtpGetFW_Type(0)!=VTP_FW_TYPE_PRAD)
    {
      vtpDmaStart(VTP_DMA_VTP, vtpDmaMemGetPhysAddress(1), MAXBUFSIZE*4);
      vtpDmaWaitDone(VTP_DMA_VTP);
    }
  #endif
#endif

  printf("INFO: User Go 1 Enabling\n");
  CDOENABLE(VTP,1,1);
  printf("INFO: User Go 1 Enabled\n");

  return;
}





void
usrtrig(unsigned long EVTYPE, unsigned long EVSOURCE)
{
  int len, ii, nbytes, nwords;
  char *chptr, *chptr0;
  volatile unsigned int *pBuf;
  TIMERL_VAR;

  if(syncFlag) printf("EVTYPE=%d syncFlag=%d\n",EVTYPE,syncFlag);

  rol->dabufp = NULL;

TIMERL_START;

  CEOPEN(EVTYPE, BT_BANKS);
  
#ifdef READOUT_TI
  #ifdef USE_DMA
    vtpDmaStart(VTP_DMA_TI, vtpDmaMemGetPhysAddress(0), MAXBUFSIZE*4);
  #endif
#endif

#ifdef READOUT_VTP
  #ifdef USE_DMA
    if(vtpGetFW_Type(0)==VTP_FW_TYPE_PRAD)
    {
    }
    else
      vtpDmaStart(VTP_DMA_VTP, vtpDmaMemGetPhysAddress(1), MAXBUFSIZE*4);
  #endif
#endif

#ifdef READOUT_TI
#ifdef USE_DMA
//    vtpDmaStart(VTP_DMA_TI, vtpDmaMemGetPhysAddress(0), MAXBUFSIZE*4);
  len = vtpDmaWaitDone(VTP_DMA_TI)>>2;
  if(len) len--;
  pBuf = (volatile unsigned int *)vtpDmaMemGetLocalAddress(0);
#else
  len = vtpEbTiReadEvent(gFixedBuf, MAXBUFSIZE);
  pBuf = (volatile unsigned int *)gFixedBuf;
#endif
  if(len>1000)
  {
    printf("LEN1=%d\n",len);
    for(ii=0; ii<len; ii++) printf("vtpti[%2d] = 0x%08x\n",ii,pBuf[ii]);
  }
  BANKOPEN(0xe10A,1,rol->pid);
  for(ii=0; ii<len; ii++)
  {
    //printf("vtpti[%2d] = 0x%08x\n",ii,pBuf[ii]);
    *rol->dabufp++ = pBuf[ii];
  }
  BANKCLOSE;
#endif

#ifdef READOUT_VTP
#ifdef USE_DMA
//    vtpDmaStart(VTP_DMA_VTP, vtpDmaMemGetPhysAddress(1), MAXBUFSIZE*4);

  if(vtpGetFW_Type(0)==VTP_FW_TYPE_PRAD)
  {
    len = vtpTestEbReadEvent(gFixedBuf, MAXBUFSIZE); // PRAD VTP
    pBuf = (volatile unsigned int *)gFixedBuf;
  }
  else
  {
    len = vtpDmaWaitDone(VTP_DMA_VTP)>>2;
    if(len) len--;
    pBuf = (volatile unsigned int *)vtpDmaMemGetLocalAddress(1);
  }
#else
  if(vtpGetFW_Type(0)==VTP_FW_TYPE_PRAD)
    len = vtpTestEbReadEvent(gFixedBuf, MAXBUFSIZE); // PRAD VTP
  else
    len = vtpEbReadEvent(gFixedBuf, MAXBUFSIZE); // regular VTP
  pBuf = (volatile unsigned int *)gFixedBuf;
#endif
  if(len>(MAXBUFSIZE/4)) /* if we are using more then 25% of the buffer, print message*/
  {
    printf("LEN2=%d\n",len);
    for(ii=0; ii<len; ii++) printf("vtp[%2d] = 0x%08x\n",ii,pBuf[ii]);
  }
  
  BANKOPEN(0xe122,1,rol->pid);
  for(ii=0; ii<len; ii++)
  {
    //printf("vtp[%2d] = 0x%08x\n",ii,pBuf[ii]);
    *rol->dabufp++ = pBuf[ii];
  }
  BANKCLOSE;
#endif

TIMERL_STOP(10000,1000+rol->pid);

#if 1
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

    len = vtpUploadAll(chptr, 30000);
    /*printf("len=%d\n",len);
    printf(">%s<\n",chptr);*/
    chptr += len;
    nbytes += len;

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
  }
#endif










  CECLOSE;


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

      vtpDmaMemClose();

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


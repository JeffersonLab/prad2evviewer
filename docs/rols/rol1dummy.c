
/* rol1dummy.c - UNIX first readout list (polling mode) */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <sys/types.h>
#ifndef VXWORKS
#include <sys/time.h>
#endif

#include "circbuf.h"


#define USE_GEM

/*****************************/
/* former 'crl' control keys */

/* readout list ROL1DUMMY */
#define ROL_NAME__ "ROL1DUMMY"

/* polling */
#define POLLING_MODE


/* name used by loader */
#define INIT_NAME rol1dummy__init

#include "rol.h"

void usrtrig(unsigned long, unsigned long);
void usrtrig_done();

/* test readout */
#include "TEST_source.h"

/************************/
/************************/
/*
#include "coda.h"
#include "tt.h"
*/
static char rcname[5];

/*
long decrement;
extern long nevent;
*/

/* for compatibility with hps1.c */
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

#ifdef USE_GEM
#include <evio.h>
#include <evioBankUtil.h>
#define MAXEVIOBUF 10000000
static int input_handle;
//static char *input_filename = "/tmp/pradfb_000350.evio.0";
//static char *input_filename = "/home/clasrun/PRad/GemView/db/physics_online1586.dat";
//static char *input_filename = "/work/boiarino/data/TCS_1_.evio.0";
static char *input_filename = "/tmp/clas_002229.evio.0";
static unsigned int buf[MAXEVIOBUF];
#endif

static void
__download()
{

#ifdef POLLING_MODE
  rol->poll = 1;
#else
  rol->poll = 0;
#endif

  printf("\n>>>>>>>>>>>>>>> ROCID=%d, CLASSID=%d <<<<<<<<<<<<<<<<\n\n",rol->pid,rol->classid);

  /* Clear some global variables etc for a clean start */
  CTRIGINIT;

  /* init trig source TEST */
  TEST_INIT;
  /*CDOINIT(TIPRIMARY,TIR_SOURCE);*/

  printf("INFO: User Download 1 Executed\n");

  /*
  UDP_user_request(MSGINF,"rol1","information");
  UDP_user_request(MSGWRN,"rol1","warnings");
  UDP_user_request(MSGERR,"rol1","errors");
  */

  return;
}


static void
__prestart()
{
  int status;
  unsigned long jj, adc_id, sl;
  char *env;

  *(rol->nevents) = 0;

  /* Register a sync trigger source */
#ifdef POLLING_MODE
  CTRIGRSS(TEST, 1, usrtrig, usrtrig_done);
#else
  CTRIGRSA(TEST, 1, usrtrig, usrtrig_done);
#endif

  /* not needed here ??? */
  rol->poll = 1;


  sprintf(rcname,"RC%02d",rol->pid);
  printf("rcname >%4.4s<\n",rcname);



  /* user code */

#ifdef USE_GEM
  /* open evio input file */
  if((status = evOpen(input_filename,"r",&input_handle))!=0)
  {
    printf("\n ?Unable to open input file >%s<, status=%d\n\n",input_filename,status);
    
    exit(1);
  }
  else
  {
    printf("\n Open data file >%s<\n",input_filename);
  }
#endif


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

  CDODISABLE(TEST,1,0);

#ifdef USE_GEM
  printf("\n  Read %d events\n\n",*(rol->nevents));
  evClose(input_handle);
#endif

  printf("INFO: User End 1 Executed\n");

  return;
}

static void
__pause()
{
  CDODISABLE(TEST,1,0);

  printf("INFO: User Pause 1 Executed\n");

  return;
}

static void
__go()
{
  char *env;

  printf("INFO: User Go 1 Enabling\n");
  CDOENABLE(TEST,1,1);
  printf("INFO: User Go 1 Enabled\n");

  printf("INFO: User Go 1 Executed\n");

  return;
}

void
usrtrig(unsigned long EVTYPE, unsigned long EVSOURCE)
{
  int len, ii, status;
  int ind, tag, num, fragtag, fragnum, nbytes, ind_data, timestamp_flag, type, *nhits;
  GET_PUT_INIT;

  rol->dabufp = NULL;

  CEOPEN(EVTYPE, BT_BANKS);

  /*usleep(500);*/
  /*sleep(1);*/
  

  /* do nothing 
  while(1) sleep(1);
  */




#ifdef USE_GEM

reopened:

  if( (status = evRead(input_handle,buf,MAXEVIOBUF)) == 0 )
  {
	//fragtag = 7;
	fragtag = 3;
    fragnum = -1;
    //tag = 57631;
    num = 0;
    
    for(tag=9; tag<=13; tag++) /* 5,6,7,8,9,10,11,12 */
	{
      ind = evLinkBank(buf, fragtag, fragnum, tag, num, &nbytes, &ind_data);
      
      if(ind > 0)
	  {
		
		/*PRINT_BUFFER(&buf[ind_data], &buf[ind_data]+(nbytes/4));*/

        BANKOPEN(tag,1,num);
        len = nbytes/4;
        for(ii=0; ii<len; ii++)
        {
          *rol->dabufp++ = buf[ind_data+ii];
          
        }
        BANKCLOSE;

	  }
	}

  }
  else if(status==-1)
  {
    printf("Reached end-of-file (status=%d), reopen\n",status);fflush(stdout);

    evClose(input_handle);

    /* open evio input file */
    if((status = evOpen(input_filename,"r",&input_handle))!=0)
    {
      printf("\n ?Unable to open input file >%s<, status=%d\n\n",input_filename,status);
      exit(1);
    }
    else
    {
      printf("\n Open data file >%s<\n",input_filename);
    }
    goto reopened;
  }
  else
  {
    printf("Error reading data file (status=%d)\n",status);fflush(stdout);
    exit(1);
  }



#endif

  
  BANKOPEN(0xe1FF,1,rol->pid);
  len = 20;
  for(ii=0; ii<len; ii++)
  {
    *rol->dabufp++ = ii;
  }
  BANKCLOSE;
  

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

  return;
}
  
static void
__status()
{
  return;
}  

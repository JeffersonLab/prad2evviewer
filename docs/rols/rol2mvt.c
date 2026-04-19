
/* rol2mvt.c - second readout list for VXS crates with MVT readout */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#ifndef VXWORKS
/*for fchmod*/
#include <sys/types.h>
#include <sys/stat.h>
#endif

#ifndef VXWORKS
#include <sys/types.h>
#include <time.h>
#endif



#ifndef Linux_armv7l
#define SSIPC
#endif
#undef SSIPC



#ifdef SSIPC_HIDE
#include <rtworks/ipc.h>
#include "epicsutil.h"
static char ssname[80];
#endif

#include "circbuf.h"

#include "daqLib.h"
int getTdcTypes(int *typebyslot);
int getTdcSlotNumbers(int *slotnumbers);


/****************************************************
 * USE_MVT
 ***************************************************/
#define USE_MVT
/****************************************************
 * MVT START : Includes
 ***************************************************/
#ifdef USE_MVT
#include "mvtLib.h"
#include "BecConfigParams.h"
#endif // USE_MVT
/****************************************************
 * MVT END : Includes
 ***************************************************/

/*#define DEBUG*/
/*#define DEBUG1*/              /*TI*/
/*#define DEBUG2*/              /*TDC*/
/*#define DEBUG3*/              /*SSP*/
/*#define DEBUG4*/              /*VSCM*/
/*#define DEBUG5*/              /*HEAD*/
//#define DEBUG6_MVT_1ST_PASS   /*MVT*/
//#define DEBUG6_MVT_2ND_PASS   /*MVT*/
/*#define DEBUG7*/              /*VTP*/
/*#define DEBUG8*/              /*DCRB*/
/*#define DEBUG9*/              /*SSP_RICH*/

#define ROL_NAME__ "ROL2MVT"
#define INIT_NAME rol2mvt__init

#define POLLING_MODE
#define EVENT_MODE

#include "rol.h"
#include "EVENT_source.h"
/************************/
/************************/


#define MYNEV 21000

int mynev; /*defined in tttrans.c */




#define SPLIT_BLOCKS


#define PASS_AS_IS


#define MODE7


#undef SLOTWORKAROUND

#define ABS(x)      ((x) < 0 ? -(x) : (x))

/* open composite bank */
#define CCOPEN(btag,fmt,bnum) \
  /*if it is first board, open bank*/ \
  if(a_slot_old==-1) \
  { \
    { \
      int len1, n1; \
      char *ch; \
      len1 = strlen(fmt); /* format length in bytes */ \
      n1 = (len1+5)/4; /* format length in words */ \
      dataout_save1 = dataout ++; /*remember '0xf' bank length location*/ \
      *dataout++ = (btag<<16) + (0xf<<8) + bnum; /*bank header*/ \
      /* tagsegment header following by format */ \
      *dataout++ = (len1<<20) + (0x6<<16) + n1; \
      ch = (char *)dataout; \
      strncpy(ch,fmt,len1); \
      ch[len1]='\0';ch[len1+1]='\4';ch[len1+2]='\4';ch[len1+3]='\4';ch[len1+4]='\4'; \
      dataout += n1; \
      /* 'internal' bank header */ \
      dataout_save2 = dataout ++;  /*remember 'internal' bank length location*/ \
      *dataout++ = (0<<16) + (0x0<<8) + 0; \
    } \
    b08 = (unsigned char *)dataout; \
  } \
  /*if new slot, write stuff*/ \
  if(a_slot != a_slot_old) \
  { \
    a_channel_old = -1; /*for new slot, reset a_channel_old to -1*/ \
    a_slot_old = a_slot; \
    *b08++ = a_slot; \
    b32 = (unsigned int *)b08; \
    *b32 = a_triggernumber; \
    b08 += 4; \
    b64 = (unsigned long long *)b08; \
    *b64 = (((unsigned long long)a_trigtime[0])<<24) | a_trigtime[1];	\
    b08 += 8; \
    /*set pointer for the number of channels*/ \
    Nchan = (unsigned int *)b08; \
    Nchan[0] = 0; \
    b08 += 4; \
  }

/* close composite bank */
#define CCCLOSE \
{ \
  unsigned int padding; \
  dataout = (unsigned int *) ( ( ((unsigned long int)b08+3)/4 ) * 4); \
  padding = (unsigned long int)dataout - (unsigned long int)b08; \
  /*dataout_save1[1] |= (padding&0x3)<<14;*/ \
  dataout_save2[1] |= (padding&0x3)<<14; \
  /*printf("CCCLOSE: 0x%08x %d --- 0x%08x %d --> padding %d\n",dataout,dataout,b08,b08,((dataout_save2[1])>>14)&0x3);*/ \
  *dataout_save1 = (dataout-dataout_save1-1); \
  *dataout_save2 = (dataout-dataout_save2-1); \
  lenout += (*dataout_save1+1); \
  lenev += (*dataout_save1+1); \
  b08 = NULL; \
}


#define DO_PEDS


#define MAXBLOCK 22  /* 22-max# of blocks=boards of certain type */
/****************************************************
 * IM change MAXEVENT to avoid huge translation tables
 ***************************************************/
//#define MAXEVENT 256 /* max number of events in one block */
#define MAXEVENT 40  /* max number of events in one block */


/* vscm boards data type defs */
#define VSCM_TYPE_BLKHDR    0x10
#define VSCM_TYPE_BLKTLR    0x11
#define VSCM_TYPE_EVTHDR    0x12
#define VSCM_TYPE_TRGTIME   0x13
#define VSCM_TYPE_BCOTIME   0x14
#define VSCM_TYPE_FSSREVT   0x18
#define VSCM_TYPE_DNV       0x1E
#define VSCM_TYPE_FILLER    0x1F

/* dcrb boards data type defs */
#define DCRB_TYPE_BLKHDR    0x10
#define DCRB_TYPE_BLKTLR    0x11
#define DCRB_TYPE_EVTHDR    0x12
#define DCRB_TYPE_TRGTIME   0x13
#define DCRB_TYPE_DCRBEVT   0x18
#define DCRB_TYPE_DNV       0x1E
#define DCRB_TYPE_FILLER    0x1F

/* ssp-rich boards data type defs */
#define RICH_TYPE_BLKHDR    0x10
#define RICH_TYPE_BLKTLR    0x11
#define RICH_TYPE_EVTHDR    0x12
#define RICH_TYPE_TRGTIME   0x13
#define RICH_TYPE_FIBER     0x17
#define RICH_TYPE_TDC       0x18
#define RICH_TYPE_DNV       0x1E
#define RICH_TYPE_FILLER    0x1F

/****************************************************
 * MVT START : Globals
 ***************************************************/
#ifdef USE_MVT

/* mvt board data type defs */
#define MVT_TYPE_BLKHDR    0xF3BB0000
#define MVT_TYPE_BLKTLR    0xFCCAFCAA
#define MVT_TYPE_EVTHDR    0xF3EE0000
#define MVT_TYPE_SAMPLE    0xF3550000
#define MVT_TYPE_FEUHDR    0xF3110000
#define MVT_TYPE_FILLER    0xFAAAFAAA

int MVT_ZS_MODE = -1;
int MVT_PRESCALE = 0;
int MVT_CMP_DATA_FMT = 0;
int MVT_NBR_OF_BEU = 0;
int MVT_NBR_EVENTS_PER_BLOCK = 0;
int MVT_NBR_SAMPLES_PER_EVENT = 0;
int MVT_NBR_OF_FEU[DEF_MAX_NB_OF_BEU] = {0}; 

int mvt_event_number = 0; 

/* open composite bank : long format */
#define MVT_CCOPEN_NOSMPPACK(btag,fmt,bnum) \
  /*if it is first board, open bank*/ \
  if(a_slot_old==-1) \
  { \
    { \
      int len1, n1; \
      char *ch; \
      len1 = strlen(fmt); /* format length in bytes */ \
      n1 = (len1+5)/4; /* format length in words */ \
      dataout_save1 = dataout ++; /*remember '0xf' bank length location*/ \
      *dataout++ = (btag<<16) + (0xf<<8) + bnum; /*bank header*/ \
      /* tagsegment header following by format */ \
      *dataout++ = (len1<<20) + (0x6<<16) + n1; \
      ch = (char *)dataout; \
      strncpy(ch,fmt,len1); \
      ch[len1]='\0';ch[len1+1]='\4';ch[len1+2]='\4';ch[len1+3]='\4';ch[len1+4]='\4'; \
      dataout += n1; \
      /* 'internal' bank header */ \
      dataout_save2 = dataout ++;  /*remember 'internal' bank length location*/ \
      *dataout++ = (0<<16) + (0x0<<8) + 0; \
    } \
    b08 = (unsigned char *)dataout; \
  } \
  /*if new slot, write stuff*/ \
  if(a_slot != a_slot_old) \
  { \
    a_channel_old = -1; /*for new slot, reset a_channel_old to -1*/ \
    a_slot_old = a_slot; \
    *b08++ = a_slot; \
    b32 = (unsigned int *)b08; \
    *b32++ = a_triggernumber; \
    b08 += 4; \
    *b32++ = a_trigtime[1]; \
    *b32   = a_trigtime[0]; \
    b08 += 8; \
    /*set pointer for the number of channels*/ \
    Nchan = (unsigned int *)b08; \
    Nchan[0] = 0; \
    b08 += 4; \
  }

/* open composite bank : short format */
#define MVT_CCOPEN_PACKSMP(btag,fmt,bnum) \
  /*if it is first board, open bank*/ \
  if(a_slot_old==-1) \
  { \
    { \
      int len1, n1; \
      char *ch; \
      len1 = strlen(fmt); /* format length in bytes */ \
      n1 = (len1+5)/4; /* format length in words */ \
      dataout_save1 = dataout ++; /*remember '0xf' bank length location*/ \
      *dataout++ = (btag<<16) + (0xf<<8) + bnum; /*bank header*/ \
      /* tagsegment header following by format */ \
      *dataout++ = (len1<<20) + (0x6<<16) + n1; \
      ch = (char *)dataout; \
      strncpy(ch,fmt,len1); \
      ch[len1]='\0';ch[len1+1]='\4';ch[len1+2]='\4';ch[len1+3]='\4';ch[len1+4]='\4'; \
      dataout += n1; \
      /* 'internal' bank header */ \
      dataout_save2 = dataout ++;  /*remember 'internal' bank length location*/ \
      *dataout++ = (0<<16) + (0x0<<8) + 0; \
    } \
    b08 = (unsigned char *)dataout; \
  } \
  /*if new slot, write stuff*/ \
  if(a_slot != a_slot_old) \
  { \
    a_channel_old = -1; /*for new slot, reset a_channel_old to -1*/ \
    a_slot_old = a_slot; \
    *b08++ = a_slot; \
    b32 = (unsigned int *)b08; \
    *b32++ = a_triggernumber; \
    b08 += 4; \
    *b32++ = a_trigtime[1]; \
    *b32   = a_trigtime[0]; \
    b08 += 8; \
  }

#define MVT_ERROR_NBR_OF_BEU            0x00000001
#define MVT_ERROR_NBR_EVENTS_PER_BLOCK  0x00000002
#define MVT_ERROR_NBR_SAMPLES_PER_EVENT 0x00000004
#define MVT_ERROR_NBR_OF_FEU            0x00000008
#define MVT_ERROR_EVENT_NUM             0x00000010
#define MVT_ERROR_BLOCK_NUM             0x00000020

#define DEBUG_MVT_DAT_ERR

#define MAXSAMPLES 128
#define MAXFEU     24

int          nBT[ MAXBANKS]={0};
int          iBT[ MAXBANKS][MAXBLOCK]={0};

int          nSMP[MAXBANKS][MAXBLOCK][MAXEVENT]={0};
int          iSMP[MAXBANKS][MAXBLOCK][MAXEVENT][MAXSAMPLES]={0};

int          nFEU[MAXBANKS][MAXBLOCK][MAXEVENT][MAXSAMPLES]={0};
int          iFEU[MAXBANKS][MAXBLOCK][MAXEVENT][MAXSAMPLES][MAXFEU]={0};
unsigned int mFEU[MAXBANKS][MAXBLOCK][MAXEVENT][MAXSAMPLES]={0};

int nbchannelsFEU[MAXBANKS][MAXBLOCK][MAXEVENT][MAXSAMPLES][MAXFEU]={0};
int       sizeFEU[MAXBANKS][MAXBLOCK][MAXEVENT][MAXSAMPLES][MAXFEU]={0};

/* table to reshaffle TPC data */  
#define MAX_FEU_EVT  40  /* max number of events in one block */
#define MAX_FEU_NUM  18  /* max number of FEUs per BEU */
#define MAX_FEU_CHN 512  /* max number of channels per FEU */
#define MAX_FEU_SMP 128  /* max number of samples per FEU */

unsigned short feu_act_msk[MAX_FEU_EVT] = {0};
         short feu_chn_cnt[MAX_FEU_EVT][MAX_FEU_NUM]={0};
          char feu_chn_ent_ind[MAX_FEU_EVT][MAX_FEU_NUM][MAX_FEU_CHN]={-1};
          char feu_chn_ent_cnt[MAX_FEU_EVT][MAX_FEU_NUM][MAX_FEU_CHN]={0};
unsigned short feu_chn_ent_val[MAX_FEU_EVT][MAX_FEU_NUM][MAX_FEU_CHN][2+MAX_FEU_SMP]={0xFFFF};

FILE *mvt_fptr_err_2 = (FILE *)NULL;

#endif // USE_MVT
/****************************************************
 * MVT END : Globals
 ***************************************************/


#define VTP_SLOT 11

/* MUST COME FROM CONFIG FILE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
#define VSCM_BCO_FREQ 16


#ifdef DO_PEDS
#define NPEDEVENTS 100
#define NSLOTS 22
#define NCHANS 16
static int npeds[NSLOTS][NCHANS];
static float pedval[NSLOTS][NCHANS];
static float pedrms[NSLOTS][NCHANS];
static FILE *fd = NULL;
static FILE *fc = NULL;
static char *dir = NULL;
static char *expid = NULL;
#endif

static char rcname[5];

#define NTRIGBITS 6
static unsigned int bitscalers[NTRIGBITS];
static int havebits = 0;

/* v1190 stuff */
static int tdctypebyslot[NSLOTS];
static int slotnums[NSLOTS];
static int error_flag[NSLOTS];
static int NBsubtract[NSLOTS];

/* daq stuff */
static int rol2_report_raw_data;
/* daq stuff */


/* user routines */

void rol2trig(int a, int b);
void rol2trig_done();

static void
__download()
{
/****************************************************
 * MVT START : __download
 ***************************************************/
#ifdef USE_MVT
/*
	char logfilename[128];
	// time variables
	time_t      cur_time;
	struct tm  *time_struct;
	char *env_home;
	char tmp_dir[512];
	char mkcmd[512];
	struct  stat log_stat;
*/
	int ret;
#endif // USE_MVT
/****************************************************
 * MVT START : __download
 ***************************************************/

  rol->poll = 1;

  sprintf(rcname,"RS%02d",rol->pid);
  printf("rcname >%4.4s<\n",rcname);

#ifdef SSIPC_HIDE
  sprintf(ssname,"%s_%s",getenv("HOST"),rcname);
  printf("Smartsockets unique name >%s<\n",ssname);
  epics_msg_sender_init(getenv("EXPID"), ssname); /* SECOND ARG MUST BE UNIQUE !!! */
#endif

  havebits = 0;

/****************************************************
 * MVT START : __download
 ***************************************************/
#ifdef USE_MVT
	if( (ret = mvtManageLogFile( &mvt_fptr_err_2, rol->pid, 2, "Coda" ) ) < 0 )
	{
		fprintf( stderr, "%s: mvtManageLogFile failed to open log file 2 for %s roc_id %d\n", __FUNCTION__, mvtRocId2SysName( rol->pid ), rol->pid );
	}
/*
	// Get current time
	cur_time = time(NULL);
	time_struct = localtime(&cur_time);
	if( mvt_fptr_err_2 == (FILE *)NULL )
	{
	  // $CLON_LOG
		//Check that tmp directory exists and if not create it
	  tmp_dir[0] = '\0';
    // First try to find official log directory
	  if( (env_home = getenv( "CLON_LOG" )) )
	  {
		  //Check that mvt/tmp directory exists and if not create it
		  sprintf( tmp_dir, "%s/mvt/", env_home );
	    fprintf( stdout, "%s: attemmpt to work with log dir %s\n", __FUNCTION__, tmp_dir );
		  if( stat( tmp_dir, &log_stat ) )
		  {
			  // create directory
			  sprintf( mkcmd, "mkdir -p %s", tmp_dir );
			  ret = system( mkcmd );
			  fprintf(stderr, "%s: system call returned with %d\n", __FUNCTION__, ret );
			  if( (ret<0) || (ret==127) || (ret==256) )
			  {
				  fprintf(stderr, "%s: failed to create dir %s with %d\n", __FUNCTION__, tmp_dir, ret );
          tmp_dir[0] = '\0';
			  }
		  }
		  else if( !(S_ISDIR(log_stat.st_mode)) )
		  {
			  fprintf(stderr, "%s: %s file exists but is not directory\n", __FUNCTION__, tmp_dir );
        tmp_dir[0] = '\0';
		  }
    }

    // If above failed
    if( tmp_dir[0] == '\0' )
    {
      fprintf(stderr, "%s: CLON_LOG failed; log file in . directory\n", __FUNCTION__ );
      sprintf( tmp_dir, "./" );
    }

    // Open log file
		sprintf(logfilename, "%s/mvt_roc_%d_rol_2.log", tmp_dir, rol->pid);
    if( (mvt_fptr_err_2 = fopen(logfilename, "w")) == (FILE *)NULL )
    {
      fprintf(stderr, "%s: fopen failed to open log file %s in write mode with %d %s\n",
      __FUNCTION__, logfilename, errno, strerror( errno ));
    }
  }
	if( mvt_fptr_err_2 != (FILE *)NULL )
	{
		fprintf( mvt_fptr_err_2, "**************************************************\n" );
		fprintf( mvt_fptr_err_2, "%s at %02d%02d%02d %02dH%02d\n", __FUNCTION__,
			time_struct->tm_year%100, time_struct->tm_mon+1, time_struct->tm_mday,
			time_struct->tm_hour, time_struct->tm_min );
		fflush(  mvt_fptr_err_2 );
	}
*/
#endif // USE_MVT
/****************************************************
 * MVT START : __download
 ***************************************************/

  return;
}

static void
__prestart()
{
  int ii, ntdcs, status;

/****************************************************
 * MVT START : __prestart
 ***************************************************/
#ifdef USE_MVT
  int ibeu;
	int i_evt;
	int i_feu;
	int i_chan;
  
/*
	char logfilename[128];
	// time variables
	time_t      cur_time;
	struct tm  *time_struct;
	char *env_home;
	char tmp_dir[512];
	char mkcmd[512];
	struct  stat log_stat;
*/
	int ret;
#endif // USE_MVT
/****************************************************
 * MVT START : __prestart
 ***************************************************/

  printf("INFO: Entering Prestart ROL2\n");

  /* Clear some global variables etc for a clean start */
  CTRIGINIT;

  /* init trig source EVENT */
  EVENT_INIT;

  /* Register a sync trigger source (up to 32 sources) */
  CTRIGRSS(EVENT, 1, rol2trig, rol2trig_done); /* second arg=1 - what is that ? */

  rol2_report_raw_data = daqGetReportRawData();
  printf("ROL2: rol2_report_raw_data set to %d\n",rol2_report_raw_data);

  rol->poll = 1;

  rol->recNb = 0;


#ifndef NIOS

  /* get some tdc info fron rol1 */
  ntdcs = getTdcTypes(tdctypebyslot);
  printf("ROL2: ntdcs_1=%d\n",ntdcs);
  for(ii=0; ii<NSLOTS; ii++)
  {
    if(tdctypebyslot[ii]==1)      NBsubtract[ii] = 9; /*v1190*/
    else if(tdctypebyslot[ii]==2) NBsubtract[ii] = 9; /*v1290*/
    else if(tdctypebyslot[ii]==3) NBsubtract[ii] = 5; /*v1290N*/
    else                          NBsubtract[ii] = 0; /*there is no board*/

    printf("ROL2: slot %d, type %d, NBsubtract=%d\n",ii,tdctypebyslot[ii],NBsubtract[ii]);
  }

  ntdcs = getTdcSlotNumbers(slotnums);
  printf("ROL2: ntdcs_2=%d\n",ntdcs);
  for(ii=0; ii<ntdcs; ii++) printf("ROL2:  tdc_id %d, slotnum %d\n",ii,slotnums[ii]);

#endif

  for(ii=0; ii<NTRIGBITS; ii++) bitscalers[ii] = 0;
#ifdef SSIPC
  printf("\nROL2: sending message (prestart)\n");
  status = epics_msg_send("hallb_trig_event_bits","uint",NTRIGBITS,bitscalers);
  printf("\nROL2: message send (prestart)\n");
#endif

/****************************************************
 * MVT START : __prestart
 ***************************************************/
#ifdef USE_MVT
	// Start by managing the log file
	if( (ret = mvtManageLogFile( &mvt_fptr_err_2, rol->pid, 2, "Coda" ) ) < 0 )
	{
		fprintf( stderr, "%s: mvtManageLogFile failed to open log file 2 for %s roc_id %d\n", __FUNCTION__, mvtRocId2SysName( rol->pid ), rol->pid );
	}
/*
	// Get current time
	cur_time = time(NULL);
	time_struct = localtime(&cur_time);
	if( mvt_fptr_err_2 == (FILE *)NULL )
	{
	  // $CLON_LOG
		//Check that tmp directory exists and if not create it
	  tmp_dir[0] = '\0';
    // First try to find official log directory
	  if( (env_home = getenv( "CLON_LOG" )) )
	  {
		  //Check that mvt/tmp directory exists and if not create it
		  sprintf( tmp_dir, "%s/mvt/", env_home );
	    fprintf( stdout, "%s: attemmpt to work with log dir %s\n", __FUNCTION__, tmp_dir );
		  if( stat( tmp_dir, &log_stat ) )
		  {
			  // create directory
			  sprintf( mkcmd, "mkdir -p %s", tmp_dir );
			  ret = system( mkcmd );
			  fprintf(stderr, "%s: system call returned with %d\n", __FUNCTION__, ret );
			  if( (ret<0) || (ret==127) || (ret==256) )
			  {
				  fprintf(stderr, "%s: failed to create dir %s with %d\n", __FUNCTION__, tmp_dir, ret );
          tmp_dir[0] = '\0';
			  }
		  }
		  else if( !(S_ISDIR(log_stat.st_mode)) )
		  {
			  fprintf(stderr, "%s: %s file exists but is not directory\n", __FUNCTION__, tmp_dir );
        tmp_dir[0] = '\0';
		  }
    }

    // If above failed
    if( tmp_dir[0] == '\0' )
    {
      fprintf(stderr, "%s: CLON_LOG failed; log file in . directory\n", __FUNCTION__ );
      sprintf( tmp_dir, "./" );
    }

    // Open log file
		sprintf(logfilename, "%s/mvt_roc_%d_rol_2.log", tmp_dir, rol->pid);
    if( (mvt_fptr_err_2 = fopen(logfilename, "w")) == (FILE *)NULL )
    {
      fprintf(stderr, "%s: fopen failed to open log file %s in write mode with %d %s\n",
      __FUNCTION__, logfilename, errno, strerror( errno ));
    }
	}
	if( mvt_fptr_err_2 != (FILE *)NULL )
	{
		fprintf( mvt_fptr_err_2, "**************************************************\n" );
		fprintf( mvt_fptr_err_2, "%s at %02d%02d%02d %02dH%02d\n", __FUNCTION__,
			time_struct->tm_year%100, time_struct->tm_mon+1, time_struct->tm_mday,
			time_struct->tm_hour, time_struct->tm_min );
		fflush(  mvt_fptr_err_2 );
	}
*/
	if( (MVT_ZS_MODE = mvtGetZSMode(rol->pid)) < 0 )
	{
		printf("ERROR: MVT ZS mode negative\n");
		if( mvt_fptr_err_2 != (FILE *)NULL )
		{
			fprintf(mvt_fptr_err_2,"%s: ERROR MVT_ZS_MODE=%d\n", __FUNCTION__,  MVT_ZS_MODE  );
			fflush( mvt_fptr_err_2);
		}
	}
	MVT_CMP_DATA_FMT          = mvtGetCmpDataFmt();
	MVT_PRESCALE              = mvtGetPrescale(rol->pid);
	MVT_NBR_OF_BEU            = mvtGetNbrOfBeu(rol->pid);
	MVT_NBR_EVENTS_PER_BLOCK  = mvtGetNbrOfEventsPerBlock(rol->pid);
	MVT_NBR_SAMPLES_PER_EVENT = mvtGetNbrOfSamplesPerEvent(rol->pid);
	for (ibeu = 0; ibeu <DEF_MAX_NB_OF_BEU; ibeu++)
	{
		MVT_NBR_OF_FEU[ibeu] = mvtGetNbrOfFeu(rol->pid, ibeu+1);
	}

	printf("INFO: MVT_ZS_MODE %d\n",                MVT_ZS_MODE              );
	printf("INFO: MVT_CMP_DATA_FMT %d\n",           MVT_CMP_DATA_FMT         );
	printf("INFO: MVT_PRESCALE %d\n",               MVT_PRESCALE             );
	printf("INFO: MVT_NBR_OF_BEU %d\n",             MVT_NBR_OF_BEU           );
	printf("INFO: MVT_NBR_EVENTS_PER_BLOCK %d\n",   MVT_NBR_EVENTS_PER_BLOCK );
	printf("INFO: MVT_NBR_SAMPLES_PER_EVENT %d\n",  MVT_NBR_SAMPLES_PER_EVENT);
	for (ibeu = 0; ibeu<DEF_MAX_NB_OF_BEU; ibeu++)
	{
		printf("INFO: MVT_NBR_OF_FEU %d %d\n", ibeu, MVT_NBR_OF_FEU[ibeu] );
	}
  // Initialize data rearangement structures
  for( i_evt=0; i_evt<MAX_FEU_EVT; i_evt++ )
  {
    feu_act_msk[i_evt]=0;
    for( i_feu=0; i_feu<MAX_FEU_NUM; i_feu++ )
    {
      feu_chn_cnt[i_evt][i_feu]=0;
      for( i_chan=0; i_chan<MAX_FEU_CHN; i_chan++ )
      {
        feu_chn_ent_ind[i_evt][i_feu][i_chan]=-1;
        feu_chn_ent_cnt[i_evt][i_feu][i_chan]=0;
        feu_chn_ent_val[i_evt][i_feu][i_chan][0]=0x0;
        feu_chn_ent_val[i_evt][i_feu][i_chan][1]=0x0;
        feu_chn_ent_val[i_evt][i_feu][i_chan][2]=0x0;
      } // for( i_chan=0; i_chan<MAX_FEU_CHN; i_chan++ )
    } // for( i_feu=0; i_feu<MAX_FEU_NUM; i_feu++ )
  } // for( i_evt=0; i_evt<MAX_FEU_EVT; i_evt++ )
#endif // USE_MVT
/****************************************************
 * MVT START : __prestart
 ***************************************************/

  printf("INFO: Prestart ROL22 executed\n");

  return;
}

static void
__end()
{
/****************************************************
 * MVT START : __end
 ***************************************************/
#ifdef USE_MVT
	if( mvt_fptr_err_2 != (FILE *)NULL )
	{
		fflush( mvt_fptr_err_2 );
		fclose( mvt_fptr_err_2 );
		mvt_fptr_err_2 = (FILE *)NULL;
	}
#endif // USE_MVT
/****************************************************
 * MVT START : __end
 ***************************************************/
  printf("INFO: User End 2 Executed\n");

  return;
}

static void
__pause()
{
  printf("INFO: User Pause 2 Executed\n");

  return;
}

static void
__go()
{
  printf("User Go 2 Reached\n");fflush(stdout);

  mynev = 0;

/****************************************************
 * MVT START : __end
 ***************************************************/
#ifdef USE_MVT
	MVT_NBR_EVENTS_PER_BLOCK  = mvtGetNbrOfEventsPerBlock(rol->pid);
	printf("INFO: %s MVT_NBR_EVENTS_PER_BLOCK %d\n", __FUNCTION__, MVT_NBR_EVENTS_PER_BLOCK);
#endif // USE_MVT
/****************************************************
 * MVT START : __end
 ***************************************************/

  printf("INFO: User Go 2 Executed\n");fflush(stdout);

  return;
}


#define NEV 50
#define NSL 22
#define NCH 16
#define NPL 4

#define REPORT_RAW_BANK_IF_REQUESTED if(rol2_report_raw_data) iASIS[nASIS++] = jj


void
rol2trig(int a, int b)
{
  CPINIT;

  BANKINIT;

  int nB[MAXBANKS], iB[MAXBANKS][MAXBLOCK], sB[MAXBANKS][MAXBLOCK], nE[MAXBANKS][MAXBLOCK];
  int iE[MAXBANKS][MAXBLOCK][MAXEVENT], lenE[MAXBANKS][MAXBLOCK][MAXEVENT];
#ifdef PASS_AS_IS
  int nASIS, iASIS[MAXBANKS];
#endif

  unsigned int *iw;
  int i, j, k, m, iii, ind, bank, nhits=0, mux_index, rlen, printing, nnE, iev, ibl;
  int nr = 0;
  int ncol = 2;
  int a_channel, a_chan1, a_chan2, a_nevents, a_blocknumber, a_triggernumber, a_module_id;
  int a_windowwidth, a_pulsenumber, a_firstsample, samplecount, a_fiber;
  int a_adc1, a_adc2, a_valid1, a_valid2, a_nwords, a_slot, a_slot2, a_slot3;
  int a_hfcb_id, a_chip_id, a_chan;
  unsigned int a_bco, a_bco1;
  int a_slot_prev, sync_flag;
  int a_qualityfactor, a_pulseintegral, a_pulsetime, a_vm, a_vp;
  int a_trigtime[4];
  int a_tdc, a_edge;
  int a_slot_old;
  int a_channel_old;
  int npedsamples, atleastoneslot, atleastonechannel[NSLOTS];
  time_t now;
  int error, status;
  int ndnv, nw;
  char errmsg[256];
  unsigned int *StartOfBank;
  char *ch;
  unsigned int *Nchan, *Npuls, *Nsamp;
  int islot, ichan, ii, jj, kk;
  int banknum = 0;
  int have_time_stamp, a_nevents2, a_event_type;
  int a_event_number_l, a_timestamp_l, a_event_number_h, a_timestamp_h, a_bitpattern;
  int a_clusterN, a_clusterE, a_clusterY, a_clusterX, a_clusterT, a_type, a_data, a_time;
  int a_instance, a_view, a_coord, a_energy, a_coordU, a_coordV, a_coordW, a_lane;
  int a_h1tag, a_h2tag, a_nhits, a_coordX, a_coordY;
  long long timestamp, latency, latency_offset;

  int nheaders, ntrailers, nbcount, nbsubtract, nwtdc;
  unsigned int *tdchead, tdcslot, tdcchan, tdcval, tdc14, tdcedge, tdceventcount;
  unsigned int tdceventid, tdcbunchid, tdcwordcount, tdcerrorflags;

#ifdef MODE7
  /* because of wierd FADC data format in mode 7 (all channels reports PULSE INTEGRAL,
  then all channels reports PULSE TIME, we'll remember those values in pass 1 in array,
  and use that array in pass 2 instead of actual values from buffer */
  static int npulse_integral[NEV][NSL][NCH];
  static int npulse_time[NEV][NSL][NCH];
  static int npulse_VmVp[NEV][NSL][NCH];
  static int pulse_integral[NEV][NSL][NCH][NPL];
  static int pulse_time[NEV][NSL][NCH][NPL];
  static int pulse_Vm[NEV][NSL][NCH][NPL];
  static int pulse_Vp[NEV][NSL][NCH][NPL];
#endif

/****************************************************
 * MVT START : rol2trig
 ***************************************************/
#ifdef USE_MVT
  int l, p;

  int mvt_error_counter;
  int mvt_error_type;
  int cur_chan_1st_smp_ptr;
  int cur_chan_smp_ptr;

  int local_event_number_high;
  int local_event_number_low;

  int current_event_number_high;
  int current_event_number_low;
  int current_block_number;
  int current_sample;
  int current_channel_id;
  int current_timestamp_low;
  int current_timestamp_high;
  int current_feu_tmstmp;

  int currentBeuId;
  int currentFeuId;

  int i_dream;
  int i_channel;
  int i_feu;
  int i_sample;
  int i_pul;
  unsigned char   c_number_of_bytes;
  unsigned short *pul_dat_ptr;
  unsigned char   nb_of_smp;
#endif
/****************************************************
 * MVT START : rol2trig
 ***************************************************/


#ifdef DO_PEDS
  char fname[1024]; /* old format */
  char cname[1024]; /* cnf format */
#endif

  mynev ++; /* needed by ttfa.c */

#ifdef MODE7
  /* clean up pulse arrays */
  for(iev=0; iev<NEV; iev++)
  {
    for(i=0; i<NSL; i++)
    {
	  for(j=0; j<NCH; j++)
	  {
        npulse_integral[iev][i][j] = 0;
        npulse_time[iev][i][j] = 0;
        npulse_VmVp[iev][i][j] = 0;
		/*
	    for(k=0; k<NPL; k++)
	    {
          pulse_integral[iev][i][j][k] = 0;
          pulse_time[iev][i][j][k] = 0;
          pulse_Vm[iev][i][j][k] = 0;
          pulse_Vp[iev][i][j][k] = 0;
	    }
		*/
	  }
    }
  }
#endif


  BANKSCAN;

#ifdef DEBUG
  printf("\n\n\n\n\n nbanks=%d\n",nbanks);
  for(jj=0; jj<nbanks; jj++) printf("bankscan[%d]: tag 0x%08x typ=%d nr=%d nw=%d dataptr=0x%08x\n",
      jj,banktag[jj],banktyp[jj],banknr[jj],banknw[jj],bankdata[jj]);
#endif



  /* first for() over banks from rol1 */
#ifdef PASS_AS_IS
  nASIS = 0; /*cleanup AS IS banks counter*/
#endif
  for(jj=0; jj<nbanks; jj++)
  {
    datain = bankdata[jj];
    lenin = banknw[jj];
#ifndef VXWORKS
#ifndef NIOS
#ifndef Linux_armv7l
    /* swap input buffer (assume that data from VME is big-endian, and we are on little-endian Intel) */
    if(banktyp[jj] != 3) for(ii=0; ii<lenin; ii++) datain[ii] = LSWAP(datain[ii]);
#endif
#endif
#endif

    if(banktag[jj] == 0xe109) /* FADC250 hardware format */
	{
      banknum = rol->pid;

      /**************/
      /* FIRST PASS:*/

	  /* check data and fill following:
nB[jj]               - incremented on every block header, effectively count boards of this type
iB[jj][nB]           - block start index
sB[jj][nB]           - slot number

nE[jj][nB]           - event counter in current block - same for all blocks = nnE
iE[jj][nB][nE[nB]]   - event start index
lenE[jj][nB][nE[nB]] - event length in words
*/

#ifdef DEBUG
      printf("\nFIRST PASS FADC250\n\n");
#endif

      error = 0;
      ii=0;
      printing = 1;
      a_slot_prev = -1;
      nB[jj] = 0; /*cleanup block counter*/
      while(ii<lenin)
      {
#ifdef DEBUG
        printf("[%3d] 0x%08x\n",ii,datain[ii]);
#endif
        if( ((datain[ii]>>27)&0x1F) == 0x10) /*block header*/
        {
          a_slot_prev = a_slot;
          a_slot = ((datain[ii]>>22)&0x1F);
          a_module_id = ((datain[ii]>>18)&0xF);
          a_blocknumber = ((datain[ii]>>8)&0x3FF);
          a_nevents = (datain[ii]&0xFF);
#ifdef DEBUG
	      printf("[%3d] BLOCK HEADER: slot %d, nevents %d, block number %d module id %d\n",ii,
				 a_slot,a_nevents,a_blocknumber,a_module_id);
          printf(">>> update iB and nB\n");
#endif
          nB[jj]++;            /*increment block counter*/
          iB[jj][nB[jj]-1] = ii;     /*remember block start index*/
          sB[jj][nB[jj]-1] = a_slot; /*remember slot number*/
          nE[jj][nB[jj]-1] = 0;      /*cleanup event counter in current block*/
#ifdef DEBUG
		  printf("0xe109: jj=%d nB[jj]=%d\n",jj,nB[jj]);
#endif
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x11) /*block trailer*/
        {
          a_slot2 = ((datain[ii]>>22)&0x1F);
          a_nwords = (datain[ii]&0x3FFFFF);
#ifdef DEBUG
	      printf("[%3d] BLOCK TRAILER: slot %d, nwords %d\n",ii,
				   a_slot2,a_nwords);
          printf(">>> data check\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

          if(a_slot2 != a_slot)
	      {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR1 in FADC data: blockheader slot %d != blocktrailer slot %d\n",mynev,
				 ii,a_slot,a_slot2);
              sprintf(errmsg,"[%3d][%3d] ERROR1 in FADC data: blockheader slot %d != blocktrailer slot %d\n",mynev,
				 ii,a_slot,a_slot2);
              printing=0;
	        }
	      }
          if(a_nwords != (ii-iB[jj][nB[jj]-1]+1))
          {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR2 in FADC data: trailer #words %d != actual #words %d\n",mynev,
				 ii,a_nwords,ii-iB[jj][nB[jj]-1]+1);
              sprintf(errmsg,"[%3d[%3d]] ERROR2 in FADC data: trailer #words %d != actual #words %d\n",mynev,
				 ii,a_nwords,ii-iB[jj][nB[jj]-1]+1);
              printing=0;
	        }
          }
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x12) /*event header*/
        {
          a_slot3 = ((datain[ii]>>22)&0x1F);
          a_triggernumber = (datain[ii]&0x3FFFFF);
#ifdef DEBUG
	      printf("[%3d] EVENT HEADER: slot number %d, trigger number %d\n",ii,
				 a_slot3, a_triggernumber);
          printf(">>> update iE and nE\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

          /*"open" next event*/
          nE[jj][k]++; /*increment event counter in current block*/
          m = nE[jj][k]-1; /*current event number*/
          iE[jj][k][m]=ii; /*remember event start index*/

	      ii++;

        }
        else if( ((datain[ii]>>27)&0x1F) == 0x13) /*trigger time: remember timestamp*/
        {
          a_trigtime[0] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG
	      printf("[%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[0]);
#endif
	      ii++;
          iii=1;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
            a_trigtime[iii] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG
            printf("   [%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[iii]);
            printf(">>> remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
            iii++;
            ii++;
	      }
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x14) /*window raw data: remember channel#*/
        {
          a_channel = ((datain[ii]>>23)&0xF);
          a_windowwidth = (datain[ii]&0xFFF);	  
#ifdef DEBUG
	      printf("[%3d] WINDOW RAW DATA: slot %d, channel %d, window width %d\n",ii,
			 a_slot,a_channel,a_windowwidth);
          printf(">>> remember channel %d\n",a_channel);
#endif
	      ii++;
          samplecount = 0;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*loop over all samples*/
	      {
            a_valid1 = ((datain[ii]>>29)&0x1);
            a_adc1 = ((datain[ii]>>16)&0xFFF);
            a_valid2 = ((datain[ii]>>13)&0x1);
            a_adc2 = (datain[ii]&0xFFF);
#ifdef DEBUG
            printf("   [%3d] WINDOW RAW DATA: valid1 %d, adc1 0x%04x, valid2 %d, adc2 0x%04x\n",ii,
              a_valid1,a_adc1,a_valid2,a_adc2);
#endif
            samplecount += 2;

            ii++;
	      }

          if(a_windowwidth != samplecount)
          {
            error ++;
            printf("[%3d][%3d] ERROR1: a_windowwidth=%d != samplecount=%d (slot %d (%d %d ?), channel %d)\n",mynev,ii,
              a_windowwidth,samplecount,a_slot,a_slot_prev,a_slot2,a_channel);
            sprintf(errmsg,"[%3d][%3d] ERROR1: a_windowwidth=%d != samplecount=%d (slot %d (%d %d ?), channel %d)\n",mynev,ii,
              a_windowwidth,samplecount,a_slot,a_slot_prev,a_slot2,a_channel);
            fflush(stdout);
          }

        }
        else if( ((datain[ii]>>27)&0x1F) == 0x15) /*window sum: obsolete*/
        {
#ifdef DEBUG
	      printf("[%3d] WINDOW SUM: must be obsolete\n",ii);
#endif
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x16) /*pulse raw data*/
        {
          a_channel = ((datain[ii]>>23)&0xF);
          a_pulsenumber = ((datain[ii]>>21)&0x3);
          a_firstsample = (datain[ii]&0x3FF);
#ifdef DEBUG
	      printf("[%3d] PULSE RAW DATA: channel %d, pulse number %d, first sample %d\n",ii,
			   a_channel,a_pulsenumber,a_firstsample);
#endif

	      ii++;

          while( ((datain[ii]>>31)&0x1) == 0)
	      {
            a_valid1 = ((datain[ii]>>29)&0x1);
            a_adc1 = ((datain[ii]>>16)&0xFFF);
            a_valid2 = ((datain[ii]>>13)&0x1);
            a_adc2 = (datain[ii]&0xFFF);
#ifdef DEBUG
            printf("   [%3d] PULSE RAW DATA: valid1 %d, adc1 0x%04x, valid2 %d, adc2 0x%04x\n",ii,
              a_valid1,a_adc1,a_valid2,a_adc2);
#endif

            ii++;
	      }

        }



















        else if( ((datain[ii]>>27)&0x1F) == 0x17) /* pulse integral */
        {
          a_channel = ((datain[ii]>>23)&0xF);
          a_pulsenumber = ((datain[ii]>>21)&0x3);
          a_qualityfactor = ((datain[ii]>>19)&0x3);
          a_pulseintegral = (datain[ii]&0x7FFFF);

#ifdef MODE7
          m = nE[jj][k]-1; /*current event number*/
          pulse_integral[m][a_slot][a_channel][a_pulsenumber] = a_pulseintegral;
          if(npulse_integral[m][a_slot][a_channel] != a_pulsenumber)
		  {
            printf("ERROR: slot=%d channel=%d npulse_integral=%d != a_pulsenumber=%d\n",a_slot,
				   a_channel,npulse_integral[m][a_slot][a_channel],a_pulsenumber);
		  }
          npulse_integral[m][a_slot][a_channel] ++;
#endif


#ifdef DEBUG
	      printf("[%3d] PULSE INTEGRAL: channel %d, pulse number %d, quality %d, integral %d\n",ii,
				 a_channel,a_pulsenumber,a_qualityfactor,a_pulseintegral);
#endif

	      ii++;
        }




        else if( ((datain[ii]>>27)&0x1F) == 0x18) /* pulse time */
        {
          a_channel = ((datain[ii]>>23)&0xF);
          a_pulsenumber = ((datain[ii]>>21)&0x3);
          a_qualityfactor = ((datain[ii]>>19)&0x3);
          a_pulsetime = (datain[ii]&0xFFFF);

#ifdef MODE7
          m = nE[jj][k]-1; /*current event number*/
          pulse_time[m][a_slot][a_channel][a_pulsenumber] = a_pulsetime;
          if(npulse_time[m][a_slot][a_channel] != a_pulsenumber)
		  {
            printf("ERROR: slot=%d channel=%d npulse_time=%d != a_pulsenumber=%d\n",a_slot,
				   a_channel,npulse_time[m][a_slot][a_channel],a_pulsenumber);
		  }
          npulse_time[m][a_slot][a_channel] ++;
#endif

#ifdef DEBUG
	      printf("[%3d] PULSE TIME: channel %d, pulse number %d, quality %d, time %d\n",ii,
				 a_channel,a_pulsenumber,a_qualityfactor,a_pulsetime);
#endif

	      ii++;
        }
















        else if( ((datain[ii]>>27)&0x1F) == 0x19)
        {
#ifdef DEBUG
	      printf("[%3d] STREAMING RAW DATA: \n",ii);
#endif
	      ii++;
        }









        else if( ((datain[ii]>>27)&0x1F) == 0x1a) /* Vm Vp */
        {
          a_channel = ((datain[ii]>>23)&0xF);
          a_pulsenumber = ((datain[ii]>>21)&0x3);
          a_vm = ((datain[ii]>>12)&0x1FF);
          a_vp = (datain[ii]&0xFFF);

#ifdef MODE7
          m = nE[jj][k]-1; /*current event number*/
          pulse_Vm[m][a_slot][a_channel][a_pulsenumber] = a_vm;
          pulse_Vp[m][a_slot][a_channel][a_pulsenumber] = a_vp;
          if(npulse_VmVp[m][a_slot][a_channel] != a_pulsenumber)
		  {
            printf("ERROR: slot=%d channel=%d npulse_VmVp=%d != a_pulsenumber=%d\n",a_slot,
				   a_channel,npulse_VmVp[m][a_slot][a_channel],a_pulsenumber);
		  }
          npulse_VmVp[m][a_slot][a_channel] ++;
#endif

#ifdef DEBUG
	      printf("[%3d] PULSE VmVp: channel %d, pulse number %d, Vm %d, Vp %d\n",ii,
				 a_channel,a_pulsenumber,a_vm,a_vp);
#endif
	      ii++;
        }






        else if( ((datain[ii]>>27)&0x1F) == 0x1D)
        {
#ifdef DEBUG
	      printf("[%3d] EVENT TRAILER: \n",ii);
#endif
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x1E)
        {
	      printf("[%3d] : DATA NOT VALID\n",ii);
          exit(0);
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x1F)
        {
#ifdef DEBUG
	      printf("[%3d] FILLER WORD: \n",ii);
          printf(">>> do nothing\n");
#endif
	      ii++;
        }
        else
        {
          error ++;
          if(printing) /* printing only once at every event */
          {
            printf("[%3d][%3d] pass1 ERROR: in FADC data format 0x%08x (bits31-27=0x%02x), slot=%d (%d %d ?)\n",mynev,
			   ii,(int)datain[ii],(datain[ii]>>27)&0x1F,a_slot,a_slot_prev,a_slot2);
            sprintf(errmsg,"[%3d][%3d] pass1 ERROR: in FADC data format 0x%08x (bits31-27=0x%02x), slot=%d (%d %d ?)\n",mynev,
			   ii,(int)datain[ii],(datain[ii]>>27)&0x1F,a_slot,a_slot_prev,a_slot2);
            printing=0;
	      }
          ii++;
        }

      } /* loop over 'lenin' words */










      /**********************************************************/
      /*check if the number of events in every block is the same*/

      nnE = nE[jj][0];
      for(k=1; k<nB[jj]; k++)
      {
        if(nE[jj][k]!=nnE)
	    {
          error ++;
          if(printing)
          {
            printf("[%3d] SEVERE ERROR: different event number in different blocks (nnE=%d != nE[%d]=%d)\n",mynev,
              nnE,k,nE[jj][k]);
            sprintf(errmsg,"[%3d] SEVERE ERROR: different event number in different blocks (nnE=%d != nE[%d]=%d)\n",mynev,
              nnE,k,nE[jj][k]);
            printing=0;
	      }      
	    }
      }


#ifdef DEBUG
      printf("\n=================\n");
      for(k=0; k<nB[jj]; k++)
      {
        printf("Block %2d, block index %2d\n",k,iB[jj][k]);
        for(m=0; m<nnE; m++)
	    {
          printf("Event %2d, event index %2d, event lenght %2d\n",m,iE[jj][k][m],lenE[jj][k][m]);
          sprintf(errmsg,"Event %2d, event index %2d, event lenght %2d\n",m,iE[jj][k][m],lenE[jj][k][m]);
        }
      }
      printf("\n=================\n");
#endif



      /***********************************************************/
      /* if any error was found, create raw data bank and return */
      if(error)
      {
        printf("\n\n\n!!!!!!!!!!!!! ERROR FOUND - RECORD RAW BANK\n\n\n");
	    {
          FILE *fd;
          char fname[256];
          sprintf(fname,"/tmp/rol2_error_event_%d.txt",mynev);
          fd = fopen(fname,"w");
          if(fd != NULL)
	      {
            fprintf(fd,"%s\n",errmsg);
            for(ii=0; ii<lenin; ii++) fprintf(fd,"[%5d] 0x%08x\n",ii,datain[ii]);
            fclose(fd);
	      }
	    }
        CPOPEN(rol->pid,1,0);
        for(ii=0; ii<lenin; ii++)
        {
          dataout[ii] = datain[ii];
          b08 += 4;
        }
        CPCLOSE;
        CPEXIT;
        return;
      }
      /***********************************************************/
      /***********************************************************/

	} /* if(0xe109) */








    else if(banktag[jj] == 0xe10A) /* TI/TS hardware format */
	{
      banknum = rol->pid;

#ifdef DEBUG1
      printf("\nFIRST PASS TI\n\n");
#endif

      error = 0;
      ii=0;
      printing=1;
      a_slot_prev = -1;
      have_time_stamp = 1;
      nB[jj]=0; /*cleanup block counter*/
	  /*
	  if(lenin != 8) printf("lenin=%d (index=%d)\n",lenin,ii);
	  */
      while(ii<lenin)
      {
#ifdef DEBUG1
        printf("[%5d] 0x%08x (lenin=%d)\n",ii,datain[ii],lenin);
#endif
        if( ((datain[ii]>>27)&0x1F) == 0x10) /*block header*/
        {
          a_slot_prev = a_slot;
          a_slot = ((datain[ii]>>22)&0x1F);
          a_module_id = ((datain[ii]>>18)&0xF);
          a_blocknumber = ((datain[ii]>>8)&0x3FF);
          a_nevents = (datain[ii]&0xFF);
#ifdef DEBUG1
	      printf("[%3d] BLOCK HEADER: slot %d, nevents %d, block number %d module id %d\n",ii,
				 a_slot,a_nevents,a_blocknumber,a_module_id);
          printf(">>> update iB and nB\n");
#endif
          nB[jj]++;                  /*increment block counter*/
          iB[jj][nB[jj]-1] = ii;     /*remember block start index*/
          sB[jj][nB[jj]-1] = a_slot; /*remember slot number*/
          nE[jj][nB[jj]-1] = 0;      /*cleanup event counter in current block*/

#ifdef DEBUG1
		  printf("0xe10A: jj=%d nB[jj]=%d\n",jj,nB[jj]);
#endif

	      ii++;

          /* second block header word */
          if(((datain[ii]>>17)&0x7FFF) != 0x7F88) printf("ERROR in TI second block header word\n");
          have_time_stamp = ((datain[ii]>>16)&0x1);
          if(((datain[ii]>>8)&0xFF) != 0x20) printf("ERROR in TI second block header word\n");
          a_nevents2 = (datain[ii]&0xFF);
          if(a_nevents != a_nevents2) printf("ERROR in TI: a_nevents=%d != a_nevents2=%d\n",a_nevents,a_nevents2);
          ii++;


          for(iii=0; iii<a_nevents; iii++)
		  {
            /* event header */
            a_event_type = ((datain[ii]>>24)&0xFF);
			if(a_event_type==0) printf("ERROR in TI event header word: event type = %d\n",a_event_type);
            if(((datain[ii]>>16)&0xFF)!=0x01) printf("ERROR in TI event header word (0x%02x)\n",((datain[ii]>>16)&0xFF));
            a_nwords = datain[ii]&0xFF;
#ifdef DEBUG1
		    printf("[%3d] EVENT HEADER, a_nwords = %d\n",ii,a_nwords);
#endif

            /*"close" previous event if any*/
            k = nB[jj]-1; /*current block index*/
#ifdef DEBUG1
			printf("0xe10A: k=%d\n",k);
#endif
            if(nE[jj][k] > 0)
	        {
              m = nE[jj][k]-1; /*current event number*/
              lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
#ifdef DEBUG1
			  printf("0xe10A: m=%d lenE=%d\n",m,lenE[jj][k][m]);
#endif
	        }
			
            /*"open" next event*/
            nE[jj][k] ++; /*increment event counter in current block*/
            m = nE[jj][k]-1; /*current event number*/
            iE[jj][k][m] = ii; /*remember event start index*/
#ifdef DEBUG1
			printf("0xe10A: nE=%d m=%d iE=%d\n",nE[jj][k],m,iE[jj][k][m]);
#endif

		    ii++;


		    if(a_nwords>0)
		    {
		      a_event_number_l = datain[ii];
#ifdef DEBUG1
		      printf("[%3d] a_event_number_1 = %d\n",ii,a_event_number_l);
#endif
		      ii++;
		    }

		    if(a_nwords>1)
		    {
		      a_timestamp_l = datain[ii];
#ifdef DEBUG1
		      printf("[%3d] a_timestamp_l = %d\n",ii,a_timestamp_l);
#endif
		      ii++;
		    }

		    if(a_nwords>2)
		    {
		      a_event_number_h = (datain[ii]>>16)&0xF;
		      a_timestamp_h = datain[ii]&0xFFFF;
#ifdef DEBUG1
		      printf("[%3d] a_event_number_h = %d a_timestamp_h = %d \n",ii,a_event_number_h,a_timestamp_h);
#endif
		      ii++;
		    }

		    if(a_nwords>3)
		    {
		      a_bitpattern = datain[ii]&0xFF;
#ifdef DEBUG1
		      printf("[%3d] a_bitpattern = 0x%08x\n",ii,a_bitpattern);
#endif
		      ii++;
		    }

		    if(a_nwords>4)
		    {
#ifdef DEBUG1
		      printf("[%3d] TS word4 = 0x%08x\n",ii,datain[ii]);
#endif
		      ii++;
		    }

		    if(a_nwords>5)
		    {
#ifdef DEBUG1
		      printf("[%3d] TS word5 = 0x%08x\n",ii,datain[ii]);
#endif
		      ii++;
		    }

		    if(a_nwords>6)
		    {
#ifdef DEBUG1
		      printf("[%3d] TS word6 = 0x%08x\n",ii,datain[ii]);
#endif
		      ii++;
		    }

		    if(a_nwords>7)
		    {
		      printf("ERROR: TS has more then 7 data words - exit\n");
		      exit(0);
		    }

		  }

        }
        else if( ((datain[ii]>>27)&0x1F) == 0x11) /*block trailer*/
        {
          a_slot2 = ((datain[ii]>>22)&0x1F);
          a_nwords = (datain[ii]&0x3FFFFF);
#ifdef DEBUG1
	      printf("[%3d] BLOCK TRAILER: slot %d, nwords %d\n",ii,
				   a_slot2,a_nwords);
          printf(">>> data check\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

          if(a_slot2 != a_slot)
	      {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR1 in TI data: blockheader slot %d != blocktrailer slot %d\n",mynev,
				 ii,a_slot,a_slot2);
              printing=0;
	        }
	      }
          if(a_nwords != (ii-iB[jj][nB[jj]-1]+1))
          {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR2 in TI data: trailer #words %d != actual #words %d\n",mynev,
				 ii,a_nwords,ii-iB[jj][nB[jj]-1]+1);
              printing=0;
	        }
          }
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x1F)
        {
#ifdef DEBUG1
	      printf("[%3d] FILLER WORD: \n",ii);
          printf(">>> do nothing\n");
#endif
	      ii++;
        }
        else
		{
          printf("TI UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
		  {
			int jjj;
		    printf("\n   Previous stuff\n");
            for(jjj=ii-20; jjj<ii; jjj++) printf("          [%3d][%3d] 0x%08x\n",jjj,ii,datain[jjj]);
            for(jjj=ii; jjj<ii+10; jjj++) printf("           [%3d][%3d] 0x%08x\n",jjj,ii,datain[jjj]);
		    printf("   End Of Previous stuff\n");fflush(stdout);
		    exit(0);
		  }
          ii++;
		}

	  } /* while() */

	} /* else if(0xe10A) */










    else if(banktag[jj] == 0xe10B) /* v1190/v1290 hardware format */
	{
      banknum = rol->pid;

#ifdef DEBUG2
      printf("\nFIRST PASS TDC\n\n");
#endif

      error = 0;
      ii=0;
      printing=1;
      a_slot_prev = -1;
      have_time_stamp = 1;
      nB[jj]=0; /*cleanup block counter*/
      while(ii<lenin)
      {
#ifdef DEBUG2
        printf("[%5d] 0x%08x (lenin=%d)\n",ii,datain[ii],lenin);
#endif



		/*
        a_slot    = (datain[ii]>>27)&0x1F;
        a_edge    = (datain[ii]>>26)&0x1;
        a_channel = (datain[ii]>>19)&0x7F;
        a_tdc     = datain[ii]&0x7FFFF;

#ifdef DEBUG2
	    printf("[%5d] TDC DATA: 0x%08x (slot=%d channel=%d tdc=%d edge=%d)\n",ii,datain[ii],a_slot,a_channel,a_tdc,a_edge);
#endif
		*/

		
        if( ((datain[ii]>>27)&0x1F) == 0)
        {
          nbcount ++;

          tdcedge = ((datain[ii]>>26)&0x1);
		  /*printf("tdcslot=%d tdctypebyslot=%d\n",tdcslot,tdctypebyslot[tdcslot]);*/
          if(tdctypebyslot[tdcslot] == 1) /*1190*/
          {
            tdcchan = ((datain[ii]>>19)&0x7F);
            tdcval = (datain[ii]&0x7FFFF);
			/*if(tdcchan==63) {reftdc=1; printf("[ev %d] befor: 0x%08x\n",*(rol->nevents),tdcval);}*/
            tdcval = tdcval<<2; /*report 1190 value with same resolution as 1290*/
			/*if(tdcchan==63) printf("[ev %d] after: 0x%08x (0x%08x)\n",*(rol->nevents),tdcval,tdcval&0x7FFFF);*/
          }
          else /*1290*/
          {
            tdcchan = ((datain[ii]>>21)&0x1F);
            tdcval = (datain[ii]&0x1FFFFF);
		  }

#ifdef DEBUG2
		  if(1/*tdcchan<2*/)
          {
            printf("   DATA: type=%1d slot=%02d tdc=%02d ch=%03d edge=%01d value=%08d\n",
				   tdctypebyslot[tdcslot],tdcslot,(int)tdc14,(int)tdcchan,(int)tdcedge,(int)tdcval);
		  }
#endif
        }
        else if( ((datain[ii]>>27)&0x1F) == 1)
        {
          tdc14 = ((datain[ii]>>24)&0x3);
          tdceventid = ((datain[ii]>12)&0xFFF);
          tdcbunchid = (datain[ii]&0xFFF);
#ifdef DEBUG2
          printf(" HEAD: tdc=%02d event_id=%05d bunch_id=%05d\n",
            (int)tdc14,(int)tdceventid,(int)tdcbunchid);
#endif
        }
        else if( ((datain[ii]>>27)&0x1F) == 3)
        {
          tdc14 = ((datain[ii]>>24)&0x3);
          tdceventid = ((datain[ii]>12)&0xFFF);
          tdcwordcount = (datain[ii]&0xFFF);
#ifdef DEBUG2
          printf(" EOB:  tdc=%02d event_id=%05d word_count=%05d\n",
            (int)tdc14,(int)tdceventid,(int)tdcwordcount);
#endif
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x18)
        {
#ifdef DEBUG2
          printf(" FILLER\n");
#endif
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x11)
        {
          nbsubtract ++;
#ifdef DEBUG2
          printf(" TIME TAG: %08x\n",(int)(datain[ii]&0x7ffffff));
#endif
        }
        else if( ((datain[ii]>>27)&0x1F) == 8)
        {
          nbcount = 1; /* counter for the number of output words from board */
          tdcslot = datain[ii]&0x1F;
		  /*printf("tdcslot=%d\n",tdcslot);*/
          tdceventcount = (datain[ii]>>5)&0x3FFFFF;
#ifdef SLOTWORKAROUND
          tdcslot_h = tdcslot;
	      remember_h = datain[ii];

          /* correct slot number - cannot do it that was for multi-event readout
          if(slotnums[nheaders] != tdcslot)
		  {
            if( !((*(rol->nevents))%10000) ) printf("WARN: [%2d] slotnums=%d, tdcslot=%d -> use slotnums\n",
				   nheaders,slotnums[nheaders],tdcslot);
            tdcslot = slotnums[nheaders];
		  }
		  */
#endif

#ifdef SLOTWORKAROUND_XXX
          if(tdcslot!=slotnums[nB[jj]])
		  {
            /*printf("PASS 1 ERROR: WRONG TDC SLOT: nB[jj]=%d slot=%d(%d)\n",nB[jj],tdcslot,slotnums[nB[jj]]);*/
			tdcslot = slotnums[nB[jj]];
		  }
#endif

          nbsubtract = NBsubtract[tdcslot]; /* # words to subtract including errors (5 for v1290N, 9 for others) */
          nheaders++;

		  
#ifdef DEBUG2
		  
          printf("GLOBAL HEADER: 0x%08x (slot=%d nevent=%d)\n",
            datain[ii],datain[ii]&0x1F,(datain[ii]>>5)&0xFFFFFF);
		  
#endif





          a_slot = tdcslot;

          /* there is no block header for TDCs, only global header for every event; we'll
             assume that if slot shows up for the first time, we are in the beginning of block */
          if(a_slot_prev != a_slot)
		  {
#ifdef DEBUG2
            printf(">>> a_slot_prev=%d, a_slot=%d -> update iB and nB\n",a_slot_prev,a_slot);
#endif
            a_slot_prev = a_slot;

            nB[jj]++;                  /*increment block counter*/
            iB[jj][nB[jj]-1] = ii;     /*remember block start index*/
            sB[jj][nB[jj]-1] = a_slot; /*remember slot number*/
            nE[jj][nB[jj]-1] = 0;      /*cleanup event counter in current block*/

#ifdef DEBUG2
		    printf("0xe10B: jj=%d nB[jj]=%d\n",jj,nB[jj]);
#endif
		  }



          k = nB[jj]-1; /*current block index*/

          /*"open" next event*/
          nE[jj][k]++; /*increment event counter in current block*/
          m = nE[jj][k]-1; /*current event number*/
          iE[jj][k][m]=ii; /*remember event start index*/
#ifdef DEBUG2
		  printf("0xe10B: open event jj=%d event# %d index %d\n",jj,m,iE[jj][k][m]);
#endif



		  
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x10)
        {
#ifdef SLOTWORKAROUND
          /* double check for slot number */
          tdcslot_t = datain[ii]&0x1F;

		  /*
          if(slotnums[ntrailers] != tdcslot_t)
		  {
            if( !((*(rol->nevents))%10000) ) printf("WARN: [%2d] slotnums=%d, tdcslot=%d\n",
				   ntrailers,slotnums[ntrailers],tdcslot_t);
		  }
		  */



          if(tdcslot_h>21||tdcslot_t>21||tdcslot_h!=tdcslot_t)
          {
            /*printf("WARN: slot from header=%d (0x%08x), from trailer=%d (0x%08x), must be %d\n",
			  tdcslot_h,remember_h,tdcslot_t,datain[ii],tdcslot)*/;
          }
#endif
          /* put nwords in header; substract 4 TDC headers, 4 TDC EOBs,
          global trailer and error words if any */
          if((((datain[ii]>>5)&0x3FFF) - nbsubtract) != nbcount)
          {			
			
            printf("ERROR: a_slot=%d (tdcslot=%d): word counters: %d != %d (nbsubtract=%d, NBsubtract=%d)\n",a_slot,tdcslot,
				   (((datain[ii]>>5)&0x3FFF) - nbsubtract),nbcount,nbsubtract,NBsubtract[tdcslot]);
			
			/*
for(i=0; i<200; i++) tmpbad[i] = tdcbuf[i];
printf("tmpgood=0x%08x tmpbad=0x%08x\n",tmpgood,tmpbad);
			*/
          }
          ntrailers ++;

          nwtdc = (datain[ii]>>5)&0x3FFF;
#ifdef DEBUG2
          printf("GLOBAL TRAILER: 0x%08x (slot=%d nw=%d stat=%d)\n",
            datain[ii],datain[ii]&0x1F,(datain[ii]>>5)&0x3FFF,
            (datain[ii]>>23)&0xF);
#endif








          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

#ifdef DEBUG2
		  printf("0xe10B: close event jj=%d event# %d length %d\n",jj,m,lenE[jj][k][m]);
#endif








        }
        else if( ((datain[ii]>>27)&0x1F) == 4)
        {
          nbsubtract ++;
          tdc14 = ((datain[ii]>>24)&0x3);
          tdcerrorflags = (datain[ii]&0x7FFF);
		  /*
#ifdef DEBUG2
		  */
		  if( !(error_flag[tdcslot]%100) ) /* do not print for every event */
          {
            unsigned int ddd, lock, ntdc;
			
            printf(" ERR: event# %7d, slot# %2d, tdc# %02d, error_flags=0x%08x, err=0x%04x, lock=0x%04x\n",
              tdceventcount,tdcslot,(int)tdc14,(int)tdcerrorflags,ddd,lock);
			
		  }
          error_flag[tdcslot] ++;
		  /*
#endif
		  */
        }
        else
        {
#ifdef DEBUG2
          printf("ERROR: in TDC data format 0x%08x\n",(int)datain[ii]);
#endif
        }

        ii ++;

	  } /* while() */

	} /* else if(0xe10B) */

















    else if(banktag[jj] == 0xe10C) /* SSP hardware format */
	{
      banknum = rol->pid;

#ifdef DEBUG3
      printf("\nFIRST PASS SSP\n\n");
#endif

      error = 0;
      ndnv = 0;
      ii=0;
      printing=1;
      a_slot_prev = -1;
      have_time_stamp = 1;
      nB[jj]=0; /*cleanup block counter*/
      while(ii<lenin)
      {
#ifdef DEBUG3
        printf("[%5d] 0x%08x (lenin=%d)\n",ii,datain[ii],lenin);
#endif
        if( ((datain[ii]>>27)&0x1F) == 0x10) /*block header*/
        {
          a_slot_prev = a_slot;
          a_slot = ((datain[ii]>>22)&0x1F);
          a_module_id = ((datain[ii]>>18)&0xF);
          a_blocknumber = ((datain[ii]>>8)&0x3FF);
          a_nevents = (datain[ii]&0xFF);
#ifdef DEBUG3
	      printf("[%3d] BLOCK HEADER: slot %d, nevents %d, block number %d module id %d\n",ii,
				 a_slot,a_nevents,a_blocknumber,a_module_id);
          printf(">>> update iB and nB\n");
#endif
          nB[jj]++;                  /*increment block counter*/
          iB[jj][nB[jj]-1] = ii;     /*remember block start index*/
          sB[jj][nB[jj]-1] = a_slot; /*remember slot number*/
          nE[jj][nB[jj]-1] = 0;      /*cleanup event counter in current block*/

#ifdef DEBUG3
		  printf("0xe10c: jj=%d nB[jj]=%d\n",jj,nB[jj]);
#endif

	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x11) /*block trailer*/
        {
          a_slot2 = ((datain[ii]>>22)&0x1F);
          a_nwords = (datain[ii]&0x3FFFFF);
#ifdef DEBUG3
	      printf("[%3d] BLOCK TRAILER: slot %d, nwords %d\n",ii,
				   a_slot2,a_nwords);
          printf(">>> data check\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

          if(a_slot2 != a_slot)
	      {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR1 in SSP data: blockheader slot %d != blocktrailer slot %d\n",mynev,
				 ii,a_slot,a_slot2);
              printing=0;
	        }
	      }
          if(a_nwords != ( (ii-iB[jj][nB[jj]-1]+1) - ndnv ) )
          {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR2 in SSP data: trailer #words=%d != actual #words=%d - ndnv=%d)\n",
					 mynev,ii,a_nwords,ii-iB[jj][nB[jj]-1]+1,ndnv);
              printing=0;
	        }
          }
          if(ndnv>0)
          {
            if(printing)
            {
              printf("[%3d] WARN in SSP data: ndnv=%d)\n",mynev,ndnv);
              printing=0;
	        }
          }

	      ii++;
        }

        else if( ((datain[ii]>>27)&0x1F) == 0x12) /*event header*/
        {
          a_triggernumber = (datain[ii]&0x7FFFFFF);
#ifdef DEBUG3
	      printf("[%3d] EVENT HEADER: trigger number %d\n",ii,
				 a_triggernumber);
          printf(">>> update iE and nE\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

          /*"open" next event*/
          nE[jj][k]++; /*increment event counter in current block*/
          m = nE[jj][k]-1; /*current event number*/
          iE[jj][k][m]=ii; /*remember event start index*/

	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x13) /*trigger time: remember timestamp*/
        {
          a_trigtime[0] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG3
	      printf("[%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[0]);
#endif
	      ii++;
          iii=1;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
            a_trigtime[iii] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG3
            printf("   [%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[iii]);
            printf(">>> remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
            iii++;
            if(iii>2) printf("ERROR1 in SSP: iii=%d\n",iii); 
            ii++;
	      }
        }

        else if( ((datain[ii]>>27)&0x1F) == 0x14) /*hps cluster*/
        {
          a_clusterN = ((datain[ii]>>23)&0xF);
          a_clusterE = ((datain[ii]>>10)&0x1FFF);
		  a_clusterY = ((datain[ii]>>6)&0xF);
          a_clusterX = (datain[ii]&0x3F);
#ifdef DEBUG3
	      printf("[%3d] CLUSTER_(N,E,Y,X): %d %d %d %d\n",ii,a_clusterN,a_clusterE,a_clusterY,a_clusterX);
#endif
	      ii++;
          iii=0;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
            a_clusterT = (datain[ii]&0x3FF);
#ifdef DEBUG3
            printf("   [%3d] CLUSTER_T: %d\n",ii,a_clusterT);
            printf(">>> remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
            iii++;
            if(iii>1) printf("ERROR2 in SSP: iii=%d\n",iii); 

            ii++;
	      }
        }

        else if( ((datain[ii]>>27)&0x1F) == 0x15) /*hps trigger*/
        {
          a_type = ((datain[ii]>>23)&0xF);
          a_data = ((datain[ii]>>16)&0x7F);
		  a_time = (datain[ii]&0x3FF);
#ifdef DEBUG3
	      printf("[%3d] TYPE %d, DATA %d, TIME %d\n",ii,a_type,a_data,a_time);
#endif
	      ii++;
		}

        else if( ((datain[ii]>>27)&0x1F) == 0x1F)
        {
#ifdef DEBUG3
	      printf("[%3d] FILLER WORD\n",ii);
          printf(">>> do nothing\n");
#endif
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x1E)
        {
          ndnv ++;
#ifdef DEBUG3
	      printf("[%3d] DNV\n",ii);
          printf(">>> do nothing\n");
#endif
	      ii++;
        }
        else
		{
          printf("SSP UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
          ii++;
		}

	  } /* while() */

	} /* else if(0xe10C) */










    else if(banktag[jj] == 0xe122) /* VTP hardware format */
	{
//Ben Nov 22, 2017
      REPORT_RAW_BANK_IF_REQUESTED;
      banknum = rol->pid;

#ifdef DEBUG7
      printf("\nFIRST PASS VTP\n\n");
#endif

      error = 0;
      ndnv = 0;
      ii=0;
      printing=1;
      a_slot_prev = -1;
      have_time_stamp = 1;
      nB[jj]=0; /*cleanup block counter*/
      while(ii<lenin)
      {
#ifdef DEBUG7
        printf("[%5d] 0x%08x (lenin=%d)\n",ii,datain[ii],lenin);
#endif
        if( ((datain[ii]>>27)&0x1F) == 0x10) /*block header*/
        {
          a_slot_prev = a_slot;
          a_slot = /*((datain[ii]>>22)&0x1F)*/VTP_SLOT;
          a_module_id = ((datain[ii]>>18)&0xF);
          a_blocknumber = ((datain[ii]>>8)&0x3FF);
          a_nevents = (datain[ii]&0xFF);
#ifdef DEBUG7
	      printf("[%3d] BLOCK HEADER: slot %d, nevents %d, block number %d module id %d\n",ii,
				 a_slot,a_nevents,a_blocknumber,a_module_id);
          printf(">>> update iB and nB\n");
#endif
          nB[jj]++;                  /*increment block counter*/
          iB[jj][nB[jj]-1] = ii;     /*remember block start index*/
          sB[jj][nB[jj]-1] = a_slot; /*remember slot number*/
          nE[jj][nB[jj]-1] = 0;      /*cleanup event counter in current block*/

#ifdef DEBUG7
		  printf("0xe122: jj=%d nB[jj]=%d\n",jj,nB[jj]);
#endif

	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x11) /*block trailer*/
        {
          a_slot2 = /*((datain[ii]>>22)&0x1F)*/VTP_SLOT;
          a_nwords = (datain[ii]&0x3FFFFF);
#ifdef DEBUG7
	      printf("[%3d] BLOCK TRAILER: slot %d, nwords %d\n",ii,
				   a_slot2,a_nwords);
          printf(">>> data check\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

          if(a_slot2 != a_slot)
	      {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR1 in VTP data: blockheader slot %d != blocktrailer slot %d\n",mynev,
				 ii,a_slot,a_slot2);
              printing=0;
	        }
	      }
          if(a_nwords != ( (ii-iB[jj][nB[jj]-1]+1) - ndnv ) )
          {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR2 in VTP data: trailer #words=%d != actual #words=%d - ndnv=%d)\n",
					 mynev,ii,a_nwords,ii-iB[jj][nB[jj]-1]+1,ndnv);
              printing=0;
	        }
          }
          if(ndnv>0)
          {
            if(printing)
            {
              printf("[%3d] WARN in VTP data: ndnv=%d)\n",mynev,ndnv);
              printing=0;
	        }
          }

	      ii++;
        }

        else if( ((datain[ii]>>27)&0x1F) == 0x12) /*event header*/
        {
          a_triggernumber = (datain[ii]&/*0x7FFFFFF*/0x3FFFFF);
#ifdef DEBUG7
	      printf("[%3d] EVENT HEADER: trigger number %d\n",ii,
				 a_triggernumber);
          printf(">>> update iE and nE\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

          /*"open" next event*/
          nE[jj][k]++; /*increment event counter in current block*/
          m = nE[jj][k]-1; /*current event number*/
          iE[jj][k][m]=ii; /*remember event start index*/

	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x13) /*trigger time: remember timestamp*/
        {
          a_trigtime[0] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG7
	      printf("[%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[0]);
#endif
	      ii++;
          iii=1;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
            a_trigtime[iii] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG7
            printf("   [%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[iii]);
            printf(">>> remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
            iii++;
            if(iii>2) printf("ERROR1 in VTP: iii=%d\n",iii); 
            ii++;
	      }
        }





        else if( ((datain[ii]>>27)&0x1F) == 0x14) /*ECAL peak*/
        {
          a_instance = ((datain[ii]>>26)&0x1);
          a_view =     ((datain[ii]>>24)&0x3);
		  a_coord =    ((datain[ii]>>15)&0x1FF);
          a_energy =   ((datain[ii]>>2)&0x1FFF);
#ifdef DEBUG7
	      printf("[%3d] PEAK_(INST,VIEW,COORD,ENERGY): %d %d %d %d\n",ii,a_instance,a_view,a_coord,a_energy);
#endif
	      ii++;
          iii=0;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
            a_time = (datain[ii]&0x7FF);
#ifdef DEBUG7
            printf("   [%3d] PEAK_(TIME): %d\n",ii,a_time);
#endif
            iii++;
            if(iii>1) printf("ERROR2 in VTP: iii=%d\n",iii); 

            ii++;
	      }
        }

        else if( ((datain[ii]>>27)&0x1F) == 0x15) /*ECAL cluster*/
        {
          uint32_t last_val;
          a_instance = ((datain[ii]>>26)&0x1);
		  a_coordW =   ((datain[ii]>>17)&0x1FF);
		  a_coordV =   ((datain[ii]>>8)&0x1FF);
          last_val = datain[ii];
#ifdef DEBUG7
	      printf("[%3d] CLUSTER_(INST,COORDW,COORDV): %d %d %d\n",ii,a_instance,a_coordW,a_coordV);
#endif

	      ii++;
          iii=0;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
		    a_coordU = ((last_val<<1)&0x1FE) | ((datain[ii]>>30)&0x1);
		    a_energy = ((datain[ii]>>17)&0x1FFF);
            a_time =   (datain[ii]&0x7FF);
#ifdef DEBUG7
            printf("   [%3d] CLUSTER_(COORDU,ENERGY,TIME): %d\n",ii,a_coordU,a_energy,a_time);
#endif
            iii++;
            if(iii>1) printf("ERROR2 in VTP: iii=%d\n",iii); 

            ii++;
	      }
        }

        else if( ((datain[ii]>>27)&0x1F) == 0x16) /*ECAL trigger bits*/
        {
          a_instance = ((datain[ii]>>16)&0x1);
          a_lane = ((datain[ii]>>11)&0x1F);
		  a_time = (datain[ii]&0x7FF);
#ifdef DEBUG7
	      printf("[%3d] TRIG_(INST,LANE,TIME) %d %d %d\n",ii,a_instance,a_lane,a_time);
#endif
	      ii++;
		}


        else if( ((datain[ii]>>27)&0x1F) == 0x17) /*FT cluster*/
        {
          uint32_t last_val;
		  /*
          a_instance = ((datain[ii]>>26)&0x1);
		  a_coordW =   ((datain[ii]>>17)&0x1FF);
		  a_coordV =   ((datain[ii]>>8)&0x1FF);
          last_val = datain[ii];
#ifdef DEBUG7
	      printf("[%3d] CLUSTER_(INST,COORDW,COORDV): %d %d %d\n",ii,a_instance,a_coordW,a_coordV);
#endif
		  */
	      ii++;
          iii=0;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
			/*
		    a_coordU = ((last_val<<1)&0x1FE) | ((datain[ii]>>30)&0x1);
		    a_energy = ((datain[ii]>>17)&0x1FFF);
            a_time =   (datain[ii]&0x7FF);
#ifdef DEBUG7
            printf("   [%3d] FT CLUSTER_(COORDU,ENERGY,TIME): %d\n",ii,a_coordU,a_energy,a_time);
#endif
			*/
            iii++;
            if(iii>1) printf("ERROR2 in VTP: iii=%d\n",iii); 

            ii++;
	      }
        }







        else if( ((datain[ii]>>27)&0x1F) == 0x1F)
        {
#ifdef DEBUG7
	      printf("[%3d] FILLER WORD\n",ii);
          printf(">>> do nothing\n");
#endif
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x1E)
        {
          ndnv ++;
#ifdef DEBUG7
	      printf("[%3d] DNV\n",ii);
          printf(">>> do nothing\n");
#endif
	      ii++;
        }
        else
		{
          /*printf("VTP UNKNOWN data1: [%3d] 0x%08x\n",ii,datain[ii]);*/
          ii++;
		}

	  } /* while() */

	} /* else if(0xe122) */











    else if(banktag[jj] == 0xe123) /* SSP-RICH hardware format */
	{
      banknum = rol->pid;

#ifdef DEBUG9
      printf("\nFIRST PASS SSP-RICH\n\n");
#endif

      error = 0;
      ndnv = 0;
      ii=0;
      printing=1;
      a_slot_prev = -1;
      have_time_stamp = 1;
      nB[jj]=0; /*cleanup block counter*/
      while(ii<lenin)
      {
#ifdef DEBUG9
        printf("[%5d] 0x%08x (lenin=%d)\n",ii,datain[ii],lenin);
#endif
        if( ((datain[ii]>>27)&0x1F) == 0x10) /*block header*/
        {
          a_slot_prev = a_slot;
          a_slot = ((datain[ii]>>22)&0x1F);
          a_module_id = ((datain[ii]>>18)&0xF);
          a_blocknumber = ((datain[ii]>>8)&0x3FF);
          a_nevents = (datain[ii]&0xFF);
#ifdef DEBUG9
	      printf("[%3d] BLOCK HEADER: slot %d, nevents %d, block number %d module id %d\n",ii,
				 a_slot,a_nevents,a_blocknumber,a_module_id);
          printf(">>> update iB and nB\n");
#endif
          nB[jj]++;                  /*increment block counter*/
          iB[jj][nB[jj]-1] = ii;     /*remember block start index*/
          sB[jj][nB[jj]-1] = a_slot; /*remember slot number*/
          nE[jj][nB[jj]-1] = 0;      /*cleanup event counter in current block*/

#ifdef DEBUG9
		  printf("0xe123: jj=%d nB[jj]=%d\n",jj,nB[jj]);
#endif

	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x11) /*block trailer*/
        {
          a_slot2 = ((datain[ii]>>22)&0x1F);
          a_nwords = (datain[ii]&0x3FFFFF);
#ifdef DEBUG9
	      printf("[%3d] BLOCK TRAILER: slot %d, nwords %d\n",ii,
				   a_slot2,a_nwords);
          printf(">>> data check\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

          if(a_slot2 != a_slot)
	      {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR1 in SSP-RICH data: blockheader slot %d != blocktrailer slot %d\n",mynev,
				 ii,a_slot,a_slot2);
              printing=0;
	        }
	      }
          if(a_nwords != ( (ii-iB[jj][nB[jj]-1]+1) - ndnv ) )
          {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR2 in SSP-RICH data: trailer #words=%d != actual #words=%d - ndnv=%d)\n",
					 mynev,ii,a_nwords,ii-iB[jj][nB[jj]-1]+1,ndnv);
              printing=0;
	        }
          }
          if(ndnv>0)
          {
            if(printing)
            {
              printf("[%3d] WARN in SSP-RICH data: ndnv=%d)\n",mynev,ndnv);
              printing=0;
	        }
          }

	      ii++;
        }

        else if( ((datain[ii]>>27)&0x1F) == 0x12) /*event header*/
        {
          /*a_slot = ((datain[ii]>>22)&0x1F);*/
          a_triggernumber = (datain[ii]&0x1FFFFF);
#ifdef DEBUG9
	      printf("[%3d] EVENT HEADER: trigger number %d\n",ii,
				 a_triggernumber);
          printf(">>> update iE and nE\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
	      }

          /*"open" next event*/
          nE[jj][k]++; /*increment event counter in current block*/
          m = nE[jj][k]-1; /*current event number*/
          iE[jj][k][m]=ii; /*remember event start index*/

	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x13) /*trigger time: remember timestamp*/
        {
          a_trigtime[0] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG9
	      printf("[%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[0]);
#endif
	      ii++;
          iii=1;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
            a_trigtime[iii] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG9
            printf("   [%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[iii]);
            printf(">>> remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
            iii++;
            if(iii>2) printf("ERROR1 in SSP-RICH: iii=%d\n",iii); 
            ii++;
	      }
        }




        else if( ((datain[ii]>>27)&0x1F) == RICH_TYPE_FIBER) /*fiber number*/
        {
          a_fiber = ((datain[ii]>>22)&0x1F);
		  /*a_triggernumber = (datain[ii]&0x1FFFFF);*/
#ifdef DEBUG9
	      printf("[%3d] FIBER %d\n",ii,a_fiber);
#endif
	      ii++;
		}

        else if( ((datain[ii]>>27)&0x1F) == RICH_TYPE_TDC) /*tdc*/
        {
          a_edge = ((datain[ii]>>26)&0x1);
          a_chan = ((datain[ii]>>16)&0xFF);
		  a_time = (datain[ii]&0x7FFF);
#ifdef DEBUG9
	      printf("[%3d] EDGE %d, CHAN %d, TIME %d\n",ii,a_edge,a_chan,a_time);
#endif
	      ii++;
		}




        else if( ((datain[ii]>>27)&0x1F) == 0x1F)
        {
#ifdef DEBUG9
	      printf("[%3d] FILLER WORD\n",ii);
          printf(">>> do nothing\n");
#endif
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x1E)
        {
          ndnv ++;
#ifdef DEBUG9
	      printf("[%3d] DNV\n",ii);
          printf(">>> do nothing\n");
#endif
	      ii++;
        }
        else
		{
          printf("SSP-RICH UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
          ii++;
		}

	  } /* while() */

	} /* else if(0xe123) */


/****************************************************
 * MVT START : rol2trig first pass
 ***************************************************/
/* Hardware format
F3BB BEUIDBlkNum
F3EE Fine/TSTP4 TSTP3 TSTP2 TSTP1 EvtNum1 EvtNum2 EvtNum3 SmpNum AcceptEvtNum
F311 LnkNumSize FEUdata...
F311 LnkNumSize FEUdata...
F355 FineTSTP TSTP1 TSTP2 TSTP3 EvtNum1 EvtNum2 EvtNum3 SmpNum AcceptEvtNum
F311 LnkNumSize FEUdata...
F311 LnkNumSize FEUdata...
Filler...
FCCA FCAA
*/
#ifdef USE_MVT
  else if(banktag[jj] == 0xe118) /* MVT hardware format */
  {
    if( ((mvt_event_number%MVT_PRESCALE)==0) && (MVT_PRESCALE!=1000000) )
      REPORT_RAW_BANK_IF_REQUESTED;
    banknum = rol->pid;

#ifdef DEBUG6_MVT_1ST_PASS
    printf("\nFIRST PASS MVT\n\n");
    if( mvt_fptr_err_2 != (FILE *)NULL )
    {
      fprintf(mvt_fptr_err_2,"%s: FIRST PASS MVT STARTS lenin=%d ii=%d datain[ii]=0x%08x address=0x%08x\n",
        __FUNCTION__, lenin, ii, datain[ii], datain);
      fflush(mvt_fptr_err_2);
    }
#endif

    error    = 0;
    ii       = 0;
    printing = 1;
    nB[jj]   = 0; /* cleanup block counter */
		nBT[jj]  = 0; /* cleanup block trailer counter */
    while(ii<lenin)
    {
#ifdef DEBUG6_MVT_1ST_PASS
      printf("[%5d] 0x%08x (lenin=%d)\n",ii,datain[ii],lenin);
      if( mvt_fptr_err_2 != (FILE *)NULL )
      {
        fprintf(mvt_fptr_err_2,"%s: FIRST PASS MVT Go through lenin=%d ii=%d datain[ii]=0x%08x\n",
          __FUNCTION__, lenin, ii, datain[ii]);
        fflush(mvt_fptr_err_2);
      }
#endif
      if( (datain[ii]&0xFFFF0000) == MVT_TYPE_BLKHDR) /*block header*/
      {
				nB[jj]++;                 /* increment block counter */
				iB[jj][nB[jj]-1] = ii;    /* remember block start index */
				nE[jj][nB[jj]-1] = 0;     /* cleanup event counter in current block */
				ii++;
#ifdef DEBUG6_MVT_1ST_PASS
        if( mvt_fptr_err_2 != (FILE *)NULL )
        {
          fprintf(mvt_fptr_err_2,"%s: FIRST PASS MVT MVT_TYPE_BLKHDR=0x%08x jj=%d ii=%d datain[ii]=0x%08x nB[jj]=%d\n", 
            __FUNCTION__, MVT_TYPE_BLKHDR, jj, ii, datain[ii], nB[jj]);
          fflush(mvt_fptr_err_2);
        }
#endif
      }
      else if( (datain[ii]&0xFFFFFFFF) == MVT_TYPE_BLKTLR) /*block trailer*/
      {
				/* "close" previous event if any */
				k = nB[jj]-1; /* current block index*/
				if(nE[jj][k] > 0)
				{
					m = nE[jj][k]-1;                  /* current event number*/
					lenE[jj][k][m] = ii-iE[jj][k][m]; /* #words in current event */
				}	      
				nBT[jj]++; 
				ii++; 
      }
      else if( (datain[ii]&0xFFFF0000) == MVT_TYPE_EVTHDR) /*event header*/
      {
				k = nB[jj]-1; /*current block index*/		

				/*"close" previous event if any*/
				if(nE[jj][k] > 0)
				{
					m = nE[jj][k]-1;                  /* current event number */
					lenE[jj][k][m] = ii-iE[jj][k][m]; /* #words in current event */
				}
				/*"open" next event*/
				nE[jj][k]++;           /* increment event counter in current block*/
				m = nE[jj][k]-1;       /* current event number*/
				iE[jj][k][m]      =ii; /* remember event start index*/	
				nSMP[jj][k][m]    = 1; /* initialize sample number in current event */
				iSMP[jj][k][m][0] = ii;
				nFEU[jj][k][m][0] = 0; /* cleanup FEU counter in current sample*/
				mFEU[jj][k][m][0] = 0; /* cleanup FEU map in current sample*/
				ii++;	      
      }
      else if( (datain[ii]&0xFFFF0000) == MVT_TYPE_SAMPLE) /*mvt sample*/
      {
				k = nB[jj]-1;            /* current block index */
				m = nE[jj][k]-1;         /* current event number */
				nSMP[jj][k][m]++;        /* initialize sample number in current event */
				l = nSMP[jj][k][m]-1;
				iSMP[jj][k][m][l] = ii;
				nFEU[jj][k][m][l] = 0;   /* cleanup FEU counter in current sample */
				mFEU[jj][k][m][l] = 0;   /* cleanup FEU map in current sample */
				ii++;	  
      }
			else if( (datain[ii]&0xFFFF0000) == MVT_TYPE_FEUHDR) /* mvt sample */
			{
				k = nB[jj]-1;             /* current block index*/
				m = nE[jj][k]-1;          /* current event number*/
				l = nSMP[jj][k][m]-1;     /* current sample number*/
				nFEU[jj][k][m][l]++;      /* incrment FEU counter in current sample*/
				p = nFEU[jj][k][m][l]-1;  /* current feu number*/
				iFEU[jj][k][m][l][p]=ii;  /* remember event start index*/
				nbchannelsFEU[jj][k][m][l][p]=0;
				sizeFEU[jj][k][m][l][p] = (datain[ii]&0x000003FF);
        currentFeuId = ((( datain[ ii] ) & 0x00007C00 ) >> 10 );
        if( (currentFeuId < 0) || (currentFeuId >23) )
          fprintf(mvt_fptr_err_2,"%s: FIRST PASS strange feuid %d in datain[ii]=0x%08x \n",
            __FUNCTION__, currentFeuId, ii, datain[ii]);
        else
          mFEU[jj][k][m][l] |= (1<<currentFeuId);
#ifdef DEBUG6_MVT_1ST_PASS
        if( mvt_fptr_err_2 != (FILE *)NULL )
        {
          fprintf(mvt_fptr_err_2,"%s: MVT_ZS_MODE : %d ii=%d datain[ii]=0x%08x\n",
            __FUNCTION__, MVT_ZS_MODE, ii, datain[ii] );
          fflush(mvt_fptr_err_2);
        }
#endif
				if (!MVT_ZS_MODE)
        { 
					nbchannelsFEU[jj][k][m][l][p] =  ( sizeFEU[jj][k][m][l][p]  - 2 - 6  ) / 74 * 64; // ATT : this size has an offset
				}
				else
        {
					nbchannelsFEU[jj][k][m][l][p] =  ( sizeFEU[jj][k][m][l][p]  - 2 - 6  ) / 2;
				}
#ifdef DEBUG6_MVT_1ST_PASS					
        if( mvt_fptr_err_2 != (FILE *)NULL )
        {
          fprintf(mvt_fptr_err_2,"%s: FIRST PASS 0xe118 : bnk=%d blk=%d evt=%d smp=%d feu=%d size %d (0x%08x) nbchannelsFEU %d\n",
          __FUNCTION__, jj, k, m, l, p, sizeFEU[jj][k][m][l][p] , sizeFEU[jj][k][m][l][p], nbchannelsFEU[jj][k][m][l][p]    );
          fflush(mvt_fptr_err_2);
        }
#endif
				if( nbchannelsFEU[jj][k][m][l][p] > 0 )
        {
          if( MVT_ZS_MODE != 2 ) // Traet No ZS and Tracker ZS cases
					  ii+= ((sizeFEU[jj][k][m][l][p]  -2) >> 1 ) ;   //big jump over the data
          else // Traet TPC ZS case
          {
            // skip FEU packet header F311 LnkNumSize (2 16-bit words) and FEU data header 4 16-bit words
            ii+=3;
            do
            {
              current_channel_id = ((( datain[ ii ] ) & 0x01FF0000) >> 16 );
#ifdef DEBUG_FP_MVT					
				      if( mvt_fptr_err_2 != (FILE *)NULL )
				      {
					      fprintf(mvt_fptr_err_2,
                  "%s: FIRST PASS 0xe118: ZS TPC: smp=%d feu=%d size %d nbchannelsFEU %d currentFeuId=%d current_channel_id=%d\n",
						      __FUNCTION__, l, p, sizeFEU[jj][k][m][l][p], nbchannelsFEU[jj][k][m][l][p], currentFeuId, current_channel_id);
					      fflush(mvt_fptr_err_2);
				      }
#endif
              cur_chan_1st_smp_ptr = feu_chn_ent_ind[m][p][current_channel_id];
#ifdef DEBUG_FP_MVT					
			        if( mvt_fptr_err_2 != (FILE *)NULL )
			        {
				        fprintf(mvt_fptr_err_2,
                  "%s: FIRST PASS 0xe118: ZS TPC: currentFeuId=%d current_channel_id=%d cur_chan_1st_smp_ptr=%d\n",
					        __FUNCTION__, currentFeuId, current_channel_id, cur_chan_1st_smp_ptr);
				        fflush(mvt_fptr_err_2);
			        }
#endif
              if( cur_chan_1st_smp_ptr == -1 )
              {
                cur_chan_1st_smp_ptr = 0;
                feu_chn_ent_ind[m][p][current_channel_id]=cur_chan_1st_smp_ptr;
                feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr  ]=l;
                feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr+1]=0;
                feu_chn_ent_cnt[m][p][current_channel_id]=1;
                feu_chn_cnt[m][p]++;
#ifdef DEBUG_FP_MVT					
				        if( mvt_fptr_err_2 != (FILE *)NULL )
				        {
					        fprintf(mvt_fptr_err_2,
                    "%s: FIRST PASS 0xe118: ZS TPC: First: currentFeuId=%d current_channel_id=%d cur_chan_1st_smp_ptr=%d feu_chn_cnt=%d feu_chn_ent_cnt=%d\n",
						        __FUNCTION__, currentFeuId, current_channel_id, cur_chan_1st_smp_ptr, feu_chn_cnt[m][p], feu_chn_ent_cnt[m][p][current_channel_id]);
					        fflush(mvt_fptr_err_2);
				        }
#endif
              }
              else if( (feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr] + feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr+1] + 1) < l )
              {
                cur_chan_1st_smp_ptr += (feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr+1]+2);
                feu_chn_ent_ind[m][p][current_channel_id]=cur_chan_1st_smp_ptr;
                feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr  ]=l;
                feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr+1]=0;
                feu_chn_ent_cnt[m][p][current_channel_id]++;
#ifdef DEBUG_FP_MVT					
				        if( mvt_fptr_err_2 != (FILE *)NULL )
				        {
					        fprintf(mvt_fptr_err_2,
                    "%s: FIRST PASS 0xe118: ZS TPC: Next: currentFeuId=%d current_channel_id=%d cur_chan_1st_smp_ptr=%d feu_chn_cnt=%d feu_chn_ent_cnt=%d\n",
						        __FUNCTION__, currentFeuId, current_channel_id, cur_chan_1st_smp_ptr, feu_chn_cnt[m][p], feu_chn_ent_cnt[m][p][current_channel_id]);
					        fflush(mvt_fptr_err_2);
				        }
#endif
              }
              cur_chan_smp_ptr = feu_chn_ent_ind[m][p][current_channel_id] + feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr+1] + 2;
              feu_chn_ent_val[m][p][current_channel_id][cur_chan_smp_ptr] =  datain[ii]&0x00000FFF; // store data
              feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr+1]++;
#ifdef DEBUG_FP_MVT					
			        if( mvt_fptr_err_2 != (FILE *)NULL )
			        {
				        fprintf(mvt_fptr_err_2,
                  "%s: FIRST PASS 0xe118: ZS TPC: currentFeuId=%d current_channel_id=%d cur_pul=%d cur_pul_1st_smp_ptr=%d cur_pul_smp_ptr=%d cur_pul_smp_cnt=%d\n",
					        __FUNCTION__, currentFeuId, current_channel_id, feu_chn_ent_cnt[m][p][current_channel_id],
                  cur_chan_1st_smp_ptr, cur_chan_smp_ptr, feu_chn_ent_val[m][p][current_channel_id][cur_chan_1st_smp_ptr+1]);
				        fflush(mvt_fptr_err_2);
			        }
#endif
              ii++;
            } while( (datain[ii]&0x70000000) != 0x70000000 ); // Check for FEU trailer
          } // else of if( MVT_ZS_MODE != 2 ): trating TPC ZS case
        }
				else
					ii++;
#ifdef DEBUG6_MVT_1ST_PASS								
        if( mvt_fptr_err_2 != (FILE *)NULL )
        {
          fprintf(mvt_fptr_err_2,"%s: FIRST PASS 0xe118 after big jump over data: ii=%d datain[ii]=0x%08x \n",
          __FUNCTION__, ii, datain[ii]);
          fflush(mvt_fptr_err_2);
        }
#endif
			}
      else if( (datain[ii]&0xFFFFFFFF) == MVT_TYPE_FILLER) /*filler*/
      {
        ii++;
      }
      else
      {
#ifdef DEBUG6_MVT_1ST_PASS
//        printf("MVT other: [%3d] 0x%08x\n",ii,datain[ii]);
#endif
        ii++;
      }
    } /* while(ii<lenin) */

		//check data structure  		
		mvt_error_counter = 0;
		mvt_error_type    = 0;
		if (nB[jj] != MVT_NBR_OF_BEU )
		{
			mvt_error_counter ++;
			mvt_error_type |= MVT_ERROR_NBR_OF_BEU;
#ifdef DEBUG6_MVT_1ST_PASS
      if( mvt_fptr_err_2 != (FILE *)NULL )
      {
        fprintf(mvt_fptr_err_2,"%s: MVT FIRST PASS 0xe118 ERROR: jj=%d nB[jj]=%d != MVT_NBR_OF_BEU=%d\n",
          __FUNCTION__, jj, nB[jj], MVT_NBR_OF_BEU);
        fflush(mvt_fptr_err_2);
      }
#endif
		}
    else
    {
			for(ibl=0; ibl < MVT_NBR_OF_BEU ; ibl++)
			{ 		
				currentBeuId = ((( datain[ iB[jj][ibl] ] ) & 0x0000F000 ) >> 12 )  - 1;		
				if( nE[jj][ibl] != MVT_NBR_EVENTS_PER_BLOCK )
				{
					mvt_error_counter ++;
					mvt_error_type |= MVT_ERROR_NBR_EVENTS_PER_BLOCK;
				} 
				else
				{
          for (iev=0; iev < MVT_NBR_EVENTS_PER_BLOCK ; iev ++)
          { 
            if( nSMP[jj][ibl][iev] != MVT_NBR_SAMPLES_PER_EVENT )
            {
				      mvt_error_counter ++;
				      mvt_error_type |= MVT_ERROR_NBR_SAMPLES_PER_EVENT;
				      printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
				      printf("0xe118 : mvt error currentBeuId =%d jj =%d ibl=%d iev=%d nsmp =%d expected=%d\n",
					      currentBeuId, jj, ibl, iev, nSMP[jj][ibl][iev], MVT_NBR_SAMPLES_PER_EVENT );
            } 
				    else
				    {
				      for (i_sample=0; i_sample < MVT_NBR_SAMPLES_PER_EVENT ; i_sample ++)
				      { 
                if( nFEU[jj][ibl][iev][i_sample] != MVT_NBR_OF_FEU[currentBeuId] )
                {
                  printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
    //            printf("0xe118 : mvt error currentBeuId =%d jj =%d ibl=%d iev=%d i_sample=%d nfeu=%d expected=%d\n",
    //              currentBeuId, jj, ibl, iev, i_sample, nFEU[jj][ibl][iev][i_sample],
    //              MVT_NBR_OF_FEU[currentBeuId] );
                  printf("0xe118 : mvt error currentBeuId =%d jj =%d ibl=%d iev=%d i_smp=%d nfeu=%d exp=%d rcvd from 0x%08x\n",
                    currentBeuId, jj, ibl, iev, i_sample, nFEU[jj][ibl][iev][i_sample],
                    MVT_NBR_OF_FEU[currentBeuId], mFEU[jj][ibl][iev][i_sample]);
                  printf("0xe118 : nfeu for beu 0 %d 1 %d 2 %d\n", MVT_NBR_OF_FEU[0], MVT_NBR_OF_FEU[1], MVT_NBR_OF_FEU[2] );
                  mvt_error_counter ++;
                  mvt_error_type |= MVT_ERROR_NBR_OF_FEU;
                  if( mvt_fptr_err_2 != (FILE *)NULL )
                  {
                    fprintf(mvt_fptr_err_2,"%s: MVT FIRST PASS 0xe118 ERROR: currentBeuId =%d jj =%d ibl=%d iev=%d i_sample=%d nfeu=%d expected=%d received from 0x%08x\n",
                      __FUNCTION__, currentBeuId, jj, ibl, iev, i_sample, nFEU[jj][ibl][iev][i_sample],
                    MVT_NBR_OF_FEU[currentBeuId], mFEU[jj][ibl][iev][i_sample]);
                    fflush(mvt_fptr_err_2);
                  }
						    } // if( nFEU[jj][ibl][iev][i_sample] != MVT_NBR_OF_FEU[currentBeuId] )
				      } // for (i_sample=0; i_sample < MVT_NBR_SAMPLES_PER_EVENT ; i_sample ++)
			      } // else of if( nSMP[jj][ibl][iev] != MVT_NBR_SAMPLES_PER_EVENT )
			    } // for (iev=0; iev < MVT_NBR_EVENTS_PER_BLOCK ; iev ++)
		    } // else of if (nB[jj] != MVT_NBR_OF_BEU )
		  } // for(ibl=0; ibl < MVT_NBR_OF_BEU ; ibl++)
	  } // else of if (nB[jj] != MVT_NBR_OF_BEU )

		//check block numbers BEUSSP block headers
		// - test that all blocks have the same number
		// - test that block number increments correctly
		current_block_number = (( datain[  iB[jj][0]  ]  )& 0x00000FFF ) ; 
		for(ibl=0; ibl < MVT_NBR_OF_BEU ; ibl++)
		{ 
			if (   ( ( datain[  iB[jj][ibl]  ]  )& 0x00000FFF ) != current_block_number )
			{
				mvt_error_counter ++;
				mvt_error_type |= MVT_ERROR_BLOCK_NUM;
			}
		}

		//check event numbers and timestamps from BEUSSP event headers
		for (iev=0; iev < MVT_NBR_EVENTS_PER_BLOCK ; iev ++)
		{
			current_event_number_high = (( datain[ ( iSMP[jj][0][iev][0] + 2 ) ] ) & 0x00007FFC) >> 2;
			current_event_number_low  =
				((( datain[ ( iSMP[jj][0][iev][0] + 2 ) ] ) & 0x00000003) << 30) + 
				((( datain[ ( iSMP[jj][0][iev][0] + 3 ) ] ) & 0x7FFF0000) >>  1) + 
				((  datain[ ( iSMP[jj][0][iev][0] + 3 ) ] ) & 0x00007FFF);		
			for(ibl=0; ibl < MVT_NBR_OF_BEU ; ibl++)
			{ 
				for (i_sample=0; i_sample < MVT_NBR_SAMPLES_PER_EVENT ; i_sample ++)
				{ 
					local_event_number_high = (( datain[ ( iSMP[jj][ibl][iev][i_sample] + 2 ) ] ) & 0x00007FFC) >> 2;
					local_event_number_low  =
						((( datain[ ( iSMP[jj][ibl][iev][i_sample] + 2 ) ] ) & 0x00000003) << 30) + 
						((( datain[ ( iSMP[jj][ibl][iev][i_sample] + 3 ) ] ) & 0x7FFF0000) >>  1) + 
						((  datain[ ( iSMP[jj][ibl][iev][i_sample] + 3 ) ] ) & 0x00007FFF);		
					if (( local_event_number_high != current_event_number_high) || ( local_event_number_low != current_event_number_low))
					{
						mvt_error_counter ++;
						mvt_error_type |= MVT_ERROR_EVENT_NUM;
            printf("0xe118 : mvt error MVT_ERROR_EVENT_NUM counter 0x%8x %d mvt error type 0x%8x for buflen %d\n", 
              mvt_error_counter, mvt_error_counter, mvt_error_type, lenin );
            printf("iev=%d   current_event_number_high=%d low=%d\n", iev, current_event_number_high, current_event_number_low );
            printf("smp=%d   event_number_high=%d low=%d\n", i_sample, local_event_number_high, local_event_number_low );
            printf("0x%08x 0x%08x 0x%08x 0x%08x \n", i_sample, local_event_number_high, local_event_number_low );
//exit(1);
					}      		
				} // for (i_sample=0; i_sample < MVT_NBR_SAMPLES_PER_EVENT ; i_sample ++)
			} // for(ibl=0; ibl < MVT_NBR_OF_BEU ; ibl++)
		} // for (iev=0; iev < MVT_NBR_EVENTS_PER_BLOCK ; iev ++)

		if( mvt_error_counter !=0 )
		{
			printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
			printf("0xe118 : mvt error counter 0x%8x %d mvt error type 0x%8x for buflen %d\n",
        mvt_error_counter, mvt_error_counter, mvt_error_type, lenin );
			if( mvt_fptr_err_2 != (FILE *)NULL )
			{
				fprintf(mvt_fptr_err_2,"%s: 0xe118 : mvt error counter 0x%8x %d mvt error type 0x%8x for buflen %d\n",
          __FUNCTION__, mvt_error_counter, mvt_error_counter, mvt_error_type, lenin );
#ifdef DEBUG_MVT_DAT_ERR
				for(ii=0; ii<lenin; ii++)
				{
					if( (ii%8) == 0 )
						fprintf(mvt_fptr_err_2,"%4d:", ii );
					fprintf(mvt_fptr_err_2," 0x%08x", datain[ii] );
					if( (ii%8) == 7 )
						fprintf(mvt_fptr_err_2,"\n" );
				}
				fprintf(mvt_fptr_err_2,"\n" );
				fflush(mvt_fptr_err_2);
//fclose(mvt_fptr_err_2);
//exit(1);
#endif
			}
			else
			{
				printf("PPPPPPPPPPPPPPPAAAAAAAAAAAAAAAAAAANNNNNNNNNNNNNNNNNNNIIIIIIIIIIIIIIIIIIIIIICCCCCCCCCCCCC\n");
			}
		} // if (mvt_error_counter !=0)

		/*
		//check event numbers and timestamps from FEUs 
		for (iev=0; iev < MVT_NBR_EVENTS_PER_BLOCK ; iev ++)
		{ 
			for(ibl=0; ibl < MVT_NBR_OF_BEU ; ibl++)
			{ 				
				for (ismp=0; ismp < MVT_NBR_SAMPLES_PER_EVENT ; ismp ++)
				{ 
					for (ifeu = 0; i_feu < MVT_NBR_OF_FEU[ibl]; ifeu++)
					{	
					}
				}
			}
		}
		*/
	} /* else if(banktag[jj] == 0xe118) /* MVT hardware format */
#endif // #ifdef USE_MVT
/****************************************************
 * MVT END : rol2trig first pass
 ***************************************************/



    else if(banktag[jj] == 0xe112) /* HEAD bank raw format */
	{
      banknum = rol->pid;

#ifdef DEBUG5
      printf("\nFIRST PASS HEAD bank\n\n");
#endif

      error = 0;
      ii=0;
      printing=1;
      nB[jj]=0; /*cleanup block counter*/
      while(ii<lenin)
      {
#ifdef DEBUG5
        printf("[%5d] 0x%08x (lenin=%d)\n",ii,datain[ii],lenin);
#endif
        if( ((datain[ii]>>27)&0x1F) == 0x10) /*block header*/
        {
          a_nevents = (datain[ii]&0xFF);
#ifdef DEBUG5
	      printf("[%3d] BLOCK HEADER: nevents %d\n",ii,a_nevents);
          printf(">>> update iB and nB\n");
#endif
          nB[jj]++;                  /*increment block counter*/
          iB[jj][nB[jj]-1] = ii;     /*remember block start index*/
          sB[jj][nB[jj]-1] = 0;      /*slot number not used here*/
          nE[jj][nB[jj]-1] = 0;      /*cleanup event counter in current block*/

#ifdef DEBUG5
		  printf("0xe112: jj=%d nB[jj]=%d\n",jj,nB[jj]);
#endif

	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == 0x11) /*block trailer*/
        {
          a_nwords = (datain[ii]&0x3FFFFF);
#ifdef DEBUG5
	      printf("[%3d] BLOCK TRAILER: nwords %d\n",ii,a_nwords);
          printf(">>> data check\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
#ifdef DEBUG5
			printf("lenE(1)[%1d][%1d][%1d]=%d\n",jj,k,m,lenE[jj][k][m]);
#endif
	      }

          if(a_nwords != ( (ii-iB[jj][nB[jj]-1]+1)) )
          {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR2 in HEAD raw data: trailer #words=%d != actual #words=%d)\n",
					 mynev,ii,a_nwords,ii-iB[jj][nB[jj]-1]+1);
              printing=0;
	        }
          }

	      ii++;
        }

        else if( ((datain[ii]>>27)&0x1F) == 0x12) /*event header*/
        {
          a_triggernumber = (datain[ii]&0x7FFFFFF);
#ifdef DEBUG5
	      printf("[%3d] EVENT HEADER: trigger number %d\n",ii,
				 a_triggernumber);
          printf(">>> update iE and nE\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
#ifdef DEBUG5
			printf("lenE(2)[%1d][%1d][%1d]=%d\n",jj,k,m,lenE[jj][k][m]);
#endif
	      }

          /*"open" next event*/
          nE[jj][k]++; /*increment event counter in current block*/
          m = nE[jj][k]-1; /*current event number*/
          iE[jj][k][m]=ii; /*remember event start index*/

	      ii++;
        }

        else if( ((datain[ii]>>27)&0x1F) == 0x14) /*head data*/
        {
          nw = (datain[ii]&0xFF);
#ifdef DEBUG5
	      printf("[%3d] head bank has %d data words\n",ii,nw);
#endif
	      ii++;
          iii=0;
          while( iii<nw && ii<lenin ) /*get all data words*/
	      {
#ifdef DEBUG5
            printf("   [%3d][%3d]: %d\n",ii,iii,datain[ii]);
#endif
            iii++;
            ii++;
	      }
        }

        else
		{
          printf("PASS1 HEAD UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
          ii++;
		}

	  } /* while() */

	} /* else if(0xe112) */












    else if(banktag[jj] == 0xe104) /* VSCM hardware format */
	{
      REPORT_RAW_BANK_IF_REQUESTED;
      banknum = rol->pid;

#ifdef DEBUG4
      printf("\nFIRST PASS VSCM\n\n");
#endif

      error = 0;
      ndnv = 0;
      ii=0;
      printing=1;
      a_slot_prev = -1;
      have_time_stamp = 1;
      nB[jj]=0; /*cleanup block counter*/
      while(ii<lenin)
      {
#ifdef DEBUG4
        printf("[%5d] VSCM 0x%08x (lenin=%d)\n",ii,datain[ii],lenin);
#endif
        if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_BLKHDR) /*block header*/
        {
          a_slot_prev = a_slot;
          a_slot = ((datain[ii]>>22)&0x1F);
          /*a_module_id = ((datain[ii]>>18)&0xF);*/
          a_blocknumber = (datain[ii]&0x7FF);
          a_nevents = ((datain[ii]>>11)&0x7FF);
#ifdef DEBUG4
	      printf("[%3d] VSCM BLOCK HEADER: slot %d, nevents %d, block number %d module id %d\n",ii,
				 a_slot,a_nevents,a_blocknumber,a_module_id);
          printf(">>> VSCM update iB and nB\n");
#endif
          nB[jj]++;                  /*increment block counter*/
          iB[jj][nB[jj]-1] = ii;     /*remember block start index*/
          sB[jj][nB[jj]-1] = a_slot; /*remember slot number*/
          nE[jj][nB[jj]-1] = 0;      /*cleanup event counter in current block*/

#ifdef DEBUG4
		  printf("VSCM: jj=%d nB[jj]=%d\n",jj,nB[jj]);
#endif

	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_BLKTLR) /*block trailer*/
        {
          a_slot2 = ((datain[ii]>>22)&0x1F);
          a_nwords = (datain[ii]&0x3FFFFF);
#ifdef DEBUG4
	      printf("[%3d] VSCM BLOCK TRAILER: slot %d, nwords %d\n",ii,
				   a_slot2,a_nwords);
          printf(">>> VSCM data check\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
#ifdef DEBUG4
            printf(">>> VSCM lenE(block trailer)=%d\n",lenE[jj][k][m]);
#endif
	      }

          if(a_slot2 != a_slot)
	      {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR1 in VSCM data: blockheader slot %d != blocktrailer slot %d\n",mynev,
				 ii,a_slot,a_slot2);
              printing=0;
	        }
	      }
          if(a_nwords != ( (ii-iB[jj][nB[jj]-1]+1) - ndnv ) )
          {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR2 in VSCM data: trailer #words=%d != actual #words=%d - ndnv=%d)\n",
					 mynev,ii,a_nwords,ii-iB[jj][nB[jj]-1]+1,ndnv);
              printing=0;
	        }
          }
          if(ndnv>0)
          {
            if(printing)
            {
              printf("[%3d] WARN in VSCM data: ndnv=%d)\n",mynev,ndnv);
              printing=0;
	        }
          }

	      ii++;
        }

        else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_EVTHDR) /*event header*/
        {
          a_triggernumber = (datain[ii]&0x7FFFFFF);

#ifdef DEBUG4
	      printf("[%3d] VSCM EVENT HEADER: trigger number %d\n",ii,
				 a_triggernumber);
          printf(">>> VSCM update iE and nE\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
#ifdef DEBUG4
            printf(">>> VSCM lenE(event header)=%d\n",lenE[jj][k][m]);
#endif
	      }

          /*"open" next event*/
          nE[jj][k]++; /*increment event counter in current block*/
          m = nE[jj][k]-1; /*current event number*/
          iE[jj][k][m]=ii; /*remember event start index*/


	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_TRGTIME) /*trigger time: remember timestamp*/
        {
          a_trigtime[0] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG4
	      printf("[%3d] VSCM TRIGGER TIME: 0x%06x\n",ii,a_trigtime[0]);
#endif
	      ii++;
          iii=1;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
            a_trigtime[iii] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG4
            printf("   [%3d] VSCM TRIGGER TIME: 0x%06x\n",ii,a_trigtime[iii]);
            printf(">>> VSCM remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
            iii++;
            if(iii>2) printf("ERROR1 in VSCM: iii=%d\n",iii); 
            ii++;
	      }
        }

        else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_BCOTIME) /* bco start and stop */
		{
#ifdef DEBUG4
          printf("[%3d] VSCM  BCOTIME START: %u STOP: %u\n",ii, (datain[ii]>>0) & 0xFF, (datain[ii]>>16) & 0xFF);
#endif
          ii++;
		}

        else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_FSSREVT) /*FSSR event*/
        {
          a_hfcb_id = ((datain[ii]>>22)&0x1);
          a_chip_id = ((datain[ii]>>19)&0x7);
          a_chan = ((datain[ii]>>12)&0x7F);
          a_bco = ((datain[ii]>>4)&0xFF);
          a_adc1 = (datain[ii]&0x7);

#ifdef DEBUG4
	      printf("[%3d] VSCM FSSREVT: HFCBID=%1u CHIPID=%1u CHAN=%3u BCO=%3u ADC=%1u \n",ii,a_hfcb_id,a_chip_id,a_chan,a_bco,a_adc1);
#endif
	      ii++;
        }

        else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_FILLER)
        {
#ifdef DEBUG4
	      printf("[%3d] VSCM FILLER WORD\n",ii);
          printf(">>> VSCM do nothing\n");
#endif
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_DNV)
        {
          ndnv ++;
#ifdef DEBUG4
	      printf("[%3d] VSCM DNV\n",ii);
          printf(">>> VSCM do nothing\n");
#endif
	      ii++;
        }
        else
		{
          printf("VSCM UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
          ii++;
		}

	  } /* while() */

	} /* else if(0xe104) */












    else if(banktag[jj] == 0xe105) /* DCRB hardware format */
	{
      REPORT_RAW_BANK_IF_REQUESTED;
      banknum = rol->pid;

#ifdef DEBUG8
      printf("\nFIRST PASS DCRB\n\n");
#endif

      error = 0;
      ndnv = 0;
      ii=0;
      printing=1;
      a_slot_prev = -1;
      have_time_stamp = 1;
      nB[jj]=0; /*cleanup block counter*/
      while(ii<lenin)
      {
#ifdef DEBUG8
        printf("[%5d] DCRB 0x%08x (lenin=%d)\n",ii,datain[ii],lenin);
#endif
        if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_BLKHDR) /*block header*/
        {
          a_slot_prev = a_slot;
          a_slot = ((datain[ii]>>22)&0x1F);
          /*a_module_id = ((datain[ii]>>18)&0xF);*/
          a_blocknumber = (datain[ii]&0x7FF);
          a_nevents = ((datain[ii]>>11)&0x7FF);
#ifdef DEBUG8
	      printf("[%3d] DCRB BLOCK HEADER: slot %d, nevents %d, block number %d module id %d\n",ii,
				 a_slot,a_nevents,a_blocknumber,a_module_id);
          printf(">>> DCRB update iB and nB\n");
#endif
          nB[jj]++;                  /*increment block counter*/
          iB[jj][nB[jj]-1] = ii;     /*remember block start index*/
          sB[jj][nB[jj]-1] = a_slot; /*remember slot number*/
          nE[jj][nB[jj]-1] = 0;      /*cleanup event counter in current block*/

#ifdef DEBUG8
		  printf("DCRB: jj=%d nB[jj]=%d\n",jj,nB[jj]);
#endif

	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_BLKTLR) /*block trailer*/
        {
          a_slot2 = ((datain[ii]>>22)&0x1F);
          a_nwords = (datain[ii]&0x3FFFFF);
#ifdef DEBUG8
	      printf("[%3d] DCRB BLOCK TRAILER: slot %d, nwords %d\n",ii,
				   a_slot2,a_nwords);
          printf(">>> DCRB data check\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
#ifdef DEBUG8
            printf(">>> DCRB lenE(block trailer)=%d\n",lenE[jj][k][m]);
#endif
	      }

          if(a_slot2 != a_slot)
	      {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR1 in DCRB data: blockheader slot %d != blocktrailer slot %d\n",mynev,
				 ii,a_slot,a_slot2);
              printing=0;
	        }
	      }
          if(a_nwords != ( (ii-iB[jj][nB[jj]-1]+1) - ndnv ) )
          {
            error ++;
            if(printing)
            {
              printf("[%3d][%3d] ERROR2 in DCRB data: trailer #words=%d != actual #words=%d - ndnv=%d)\n",
					 mynev,ii,a_nwords,ii-iB[jj][nB[jj]-1]+1,ndnv);
              printing=0;
	        }
          }
          if(ndnv>0)
          {
            if(printing)
            {
              printf("[%3d] WARN in DCRB data: ndnv=%d)\n",mynev,ndnv);
              printing=0;
	        }
          }

	      ii++;
        }

        else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_EVTHDR) /*event header*/
        {
          a_triggernumber = (datain[ii]&0x7FFFFFF);

#ifdef DEBUG8
	      printf("[%3d] DCRB EVENT HEADER: trigger number %d\n",ii,
				 a_triggernumber);
          printf(">>> DCRB update iE and nE\n");
#endif

          /*"close" previous event if any*/
          k = nB[jj]-1; /*current block index*/
          if(nE[jj][k] > 0)
	      {
            m = nE[jj][k]-1; /*current event number*/
            lenE[jj][k][m] = ii-iE[jj][k][m]; /*#words in current event*/
#ifdef DEBUG8
            printf(">>> DCRB lenE(event header)=%d\n",lenE[jj][k][m]);
#endif
	      }

          /*"open" next event*/
          nE[jj][k]++; /*increment event counter in current block*/
          m = nE[jj][k]-1; /*current event number*/
          iE[jj][k][m]=ii; /*remember event start index*/


	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_TRGTIME) /*trigger time: remember timestamp*/
        {
          a_trigtime[0] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG8
	      printf("[%3d]  TRIGGER TIME: 0x%06x\n",ii,a_trigtime[0]);
#endif
	      ii++;
          iii=1;
          while( ((datain[ii]>>31)&0x1) == 0 && ii<lenin ) /*must be one more word*/
	      {
            a_trigtime[iii] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG8
            printf("   [%3d] DCRB TRIGGER TIME: 0x%06x\n",ii,a_trigtime[iii]);
            printf(">>> DCRB remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
            iii++;
            if(iii>2) printf("ERROR1 in DCRB: iii=%d\n",iii); 
            ii++;
	      }
        }

        else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_DCRBEVT) /*FSSR event*/
        {
          a_chan = ((datain[ii]>>16)&0x7F);
          a_tdc = (datain[ii]&0xFFFF);

#ifdef DEBUG8
	      printf("[%3d] DCRB DCRBEVT: CHAN=%3u TDC=%6u \n",ii,a_chan,a_tdc);
#endif
	      ii++;
        }

        else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_FILLER)
        {
#ifdef DEBUG8
	      printf("[%3d] DCRB FILLER WORD\n",ii);
          printf(">>> DCRB do nothing\n");
#endif
	      ii++;
        }
        else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_DNV)
        {
          ndnv ++;
#ifdef DEBUG8
	      printf("[%3d] DCRB DNV\n",ii);
          printf(">>> DCRB do nothing\n");
#endif
	      ii++;
        }
        else
		{
          printf("DCRB UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
          ii++;
		}

	  } /* while() */

	} /* else if(0xe105) */















#ifdef PASS_AS_IS
    else /*any other bank will be passed 'as is' */
	{
      iASIS[nASIS++] = jj; /* remember bank number as it reported by BANKSCAN */
      /*printf("mynev=%d: remember bank number %d\n",mynev,jj);*/
	}

#endif




  } /* first for() over banks from rol1 */
















  /********************************************************/
  /********************************************************/
  /********************************************************/
  /* SECOND PASS: disantangling and filling output buffer */


#ifdef DEBUG
  printf("\n\n\nSECOND PASS\n\n");
#endif

  lenout = 2; /* already done in CPINIT !!?? */
  b08 = NULL;
  printing=1;

  nnE = nE[0][0]; // ALL BLOCKS in ALL BANKS ARE EXPECTED TO HAVE THE SAME NUMBER OF EVENTS
  if(nnE<=0)
  {
    printf("rol2: ERROR: nnE=%d\n",nnE);
    exit(0);
  }
#ifdef DEBUG
  printf("nnE=%d\n",nnE);
#endif


  /*loop over events*/
  for(iev=0; iev<nnE; iev++)
  {
    lenev = 2;

    banknum = iev; /* using event number inside block as bank number - for now */

    for(jj=0; jj<nbanks; jj++) /* loop over evio banks */
    {

      datain = bankdata[jj];

#ifdef DEBUG
      printf("iev=%d jj=%d nB=%d\n",iev,jj,nB[jj]);
#endif








      if(banktag[jj] == 0xe109) /* FADC250 hardware format */
	  {
#ifdef DEBUG
        printf("SECOND PASS FADC250\n");
#endif

        a_slot_old = -1;

        for(ibl=0; ibl<nB[jj]; ibl++) /*loop over blocks*/
        {
          a_channel_old = -1;

#ifdef DEBUG
          printf("0xe109: Block %d, Event %2d, event index %2d, event lenght %2d\n",
            ibl,iev,iE[jj][ibl][iev],lenE[jj][ibl][iev]);
#endif
          a_slot = sB[jj][ibl];
          ii = iE[jj][ibl][iev];
          rlen = ii + lenE[jj][ibl][iev];
          while(ii<rlen)
          {
            if( ((datain[ii]>>27)&0x1F) == 0x12) /*event header: remember trigger#*/
            {
              a_slot3 = ((datain[ii]>>22)&0x1F);
              a_triggernumber = (datain[ii]&0x3FFFFF);
#ifdef DEBUG
	          printf("[%3d] EVENT HEADER: slot number %d, trigger number %d\n",ii,
				 a_slot3, a_triggernumber);
              printf(">>> remember trigger %d\n",a_triggernumber);
#endif
	          ii++;
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x13) /*trigger time: remember timestamp*/
            {
              a_trigtime[0] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG
	          printf("[%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[0]);
#endif
	          ii++;
              iii=1;
              while( ((datain[ii]>>31)&0x1) == 0 && ii<rlen ) /*must be one more word*/
	          {
                a_trigtime[iii] = (datain[ii]&0xFFFFFF);
#ifdef DEBUG
                printf("   [%3d] TRIGGER TIME: 0x%06x\n",ii,a_trigtime[iii]);
                printf(">>> remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
                iii++;
                ii++;
	          }
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x14) /*window raw data: remember channel#*/
            {
              a_channel = ((datain[ii]>>23)&0xF);
              a_windowwidth = (datain[ii]&0xFFF);
#ifdef DEBUG
	          printf("[%3d] WINDOW RAW DATA: slot %d, channel %d, window width %d\n",ii,
			    a_slot,a_channel,a_windowwidth);
              printf(">>> remember channel %d\n",a_channel);
#endif

#ifdef DEBUG
              printf("0x%08x: befor CCOPEN(1)\n",b08);
              printf("0x%08x: befor CCOPEN(1), dataout=0x%08x\n",b08,dataout);
#endif

	          CCOPEN(0xe101,"c,i,l,N(c,Ns)",banknum);

#ifdef DEBUG
              printf("0x%08x: after CCOPEN(1)\n",b08);
              printf("0x%08x: after CCOPEN(1), dataout=0x%08x\n",b08,dataout);
#endif

#ifdef DO_PEDS
              if(mynev==10 && fd==NULL)
	          {
                printf("mynev=%d - opening pedestal file\n",mynev);
                if((dir=getenv("CLAS")) == NULL)
	            {
                  printf("ERROR: environment variable CLAS is not defined - exit\n");
                  exit(0);
		        }
                if((expid=getenv("EXPID")) == NULL)
	            {
                  printf("ERROR: environment variable EXPID is not defined - exit\n");
                  exit(0);
		        }

                sprintf(fname,"%s/parms/peds/%s/roc%02d.ped",dir,expid,rol->pid);
                fd = fopen(fname,"w");
                if(fd==NULL)
                {
                  printf("ttfa: ERROR: cannot open pedestal file >%s<\n",fname);
		        }
                else
	            {
                  printf("ttfa: pedestal file >%s< is opened for writing\n",fname);
                }

                /*sprintf(cname,"%s/parms/peds/%s/roc%02d.cnf",dir,expid,rol->pid);*/
                sprintf(cname,"%s/parms/peds/%s/%s_ped.cnf",dir,expid,getenv("HOST"));
                /*sprintf(cname,"%s/parms/fadc250/%s_ped.cnf",dir,getenv("HOST"));*/
                fc = fopen(cname,"w");
                if(fc==NULL)
                {
                  printf("ttfa: ERROR: cannot open pedestal-cnf file >%s<\n",cname);
		        }
                else
	            {
                  printf("ttfa: pedestal-cnf file >%s< is opened for writing\n",cname);
                  now = time(NULL);
                  fprintf(fc,"#\n# File generated automatically by DAQ running in RAW mode at %s#\n",ctime(&now));
                }

                for(i=0; i<NSLOTS; i++)
	            {
                  for(j=0; j<NCHANS; j++)
  	              {
                    npeds[i][j] = 0;
                    pedval[i][j] = 0.0;
                    pedrms[i][j] = 0.0;
		          }
	            }
	          }
              else if(mynev==110 && fd!=NULL && fc!=NULL)
	          {
                printf("mynev=%d - closing pedestal file (Nmeasures=%d (%d %d ..))\n",
                  mynev,npeds[3][0],npeds[3][1],npeds[4][0]);

                /* check if we have pedestal for at least one channel in current slot */
                atleastoneslot = 0;
                for(i=0; i<NSLOTS; i++)
				{
                  atleastonechannel[i] = 0;
                  for(j=0; j<NCHANS; j++)
				  {
                    if(npeds[i][j]>0)
					{
                      atleastoneslot = 1;
                      atleastonechannel[i] = 1;
                      break;
					}
				  }
				}

                if(atleastoneslot)
				{
                  fprintf(fc,"FADC250_CRATE %s\n",getenv("HOST"));
				}
                for(i=0; i<NSLOTS; i++)
	            {
                  if(atleastonechannel[i])
				  {
                    fprintf(fc,"FADC250_SLOT %d\n",i);
				    fprintf(fc,"FADC250_ALLCH_PED ");
				  }
                  for(j=0; j<NCHANS; j++)
                  {
                    if(npeds[i][j]>0)
			        {
                      pedval[i][j] = pedval[i][j] / ((float)npeds[i][j]);
#ifdef VXWORKS
                      pedrms[i][j] = ( pedrms[i][j]/((float)npeds[i][j]) - (pedval[i][j]*pedval[i][j]) );
#else
                      pedrms[i][j] = sqrtf( pedrms[i][j]/((float)npeds[i][j]) - (pedval[i][j]*pedval[i][j]) );
#endif
                      fprintf(fd,"%2d %2d %5d %6.3f %2d\n",i,j,(int)pedval[i][j],pedrms[i][j],0);
			        }
                    /* always print, zero or not */
                    if(atleastonechannel[i]) fprintf(fc," %8.3f",pedval[i][j]);
		          }
                  if(atleastonechannel[i])
				  {
                    fprintf(fc,"\n");
				  }
	            }
                if(atleastoneslot)
				{
                  fprintf(fc,"FADC250_CRATE end\n");
				}

#ifndef VXWORKS
                /*open files for everybody*/
                if(chmod(fname,S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH) != 0)
                {
                  printf("ERROR: cannot change mode on output run config file\n");
                }
                if(chmod(cname,S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH) != 0)
                {
                  printf("ERROR: cannot change mode on output run config file\n");
                }
#endif

                fclose(fd);
                printf("ttfa: pedestal file >%s< is closed\n",fname);
                fd = NULL;

                fclose(fc);
                printf("ttfa: pedestal-cnf file >%s< is closed\n",fname);
                fc = NULL;
	          }
#endif

	          Nchan[0] ++; /* increment channel counter */
#ifdef DEBUG
              printf("0x%08x: increment Nchan[0]=%d\n",b08,Nchan[0]);
#endif

              *b08++ = a_channel; /* channel number */

#ifdef DEBUG
              printf("0x%08x: #samples %d\n",b08,a_windowwidth);
#endif
              b32 = (unsigned int *)b08;
              *b32 = a_windowwidth; /* the number of samples (same as window width) */
              b08 += 4;

	          ii++;

              /* use first 10% of the window to measure pedestal, or 2 if less then 2 */
              npedsamples = a_windowwidth / 10;
              if(npedsamples<2) npedsamples = 2;

              while( ((datain[ii]>>31)&0x1) == 0 && ii<rlen ) /*loop over all samples*/
	          {
                a_valid1 = ((datain[ii]>>29)&0x1);
                a_adc1 = ((datain[ii]>>16)&0xFFF);
                a_valid2 = ((datain[ii]>>13)&0x1);
                a_adc2 = (datain[ii]&0xFFF);

#ifdef DEBUG
                printf("   [%3d] WINDOW RAW DATA: valid1 %d, adc1 0x%04x, valid2 %d, adc2 0x%04x\n",ii,
                  a_valid1,a_adc1,a_valid2,a_adc2);
                printf("0x%08x: samples: %d %d\n",b08,a_adc1,a_adc2);
#endif

                b16 = (unsigned short *)b08;
                *b16 ++ = a_adc1;
                *b16 ++ = a_adc2;
                b08 += 4;

                ii++;

#ifdef DO_PEDS
                if(mynev>=10 && mynev<110 && npedsamples>0)
		        {
		          /*printf("[%2d][%2d] npedsamples=%d\n",a_slot,a_channel,npedsamples);*/
                  npedsamples -= 2;
                  npeds[a_slot][a_channel] += 2;
                  pedval[a_slot][a_channel] += *(b16-2);
                  pedval[a_slot][a_channel] += *(b16-1);
                  pedrms[a_slot][a_channel] += (*(b16-2))*(*(b16-2));
                  pedrms[a_slot][a_channel] += (*(b16-1))*(*(b16-1));
		        }
#endif
	          }
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x15) /*window sum: obsolete*/
            {
#ifdef DEBUG
	          printf("[%3d] WINDOW SUM: must be obsolete\n",ii);
#endif
	          ii++;
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x16) /*pulse raw data*/
            {
              a_channel = ((datain[ii]>>23)&0xF);
              a_pulsenumber = ((datain[ii]>>21)&0x3);
              a_firstsample = (datain[ii]&0x3FF);
#ifdef DEBUG
	          printf("[%3d] PULSE RAW DATA: channel %d, pulse number %d, first sample %d\n",ii,
			    a_channel,a_pulsenumber,a_firstsample);
#endif

              CCOPEN(0xe110,"c,i,l,N(c,N(c,Ns))",banknum);
#ifdef DEBUG
              printf("0x%08x: CCOPEN(2)\n",b08);
#endif

              if(a_channel != a_channel_old)
              {
                a_channel_old = a_channel;

                Nchan[0] ++; /* increment channel counter */
#ifdef DEBUG
                printf("0x%08x: increment Nchan[0]=%d\n",b08,Nchan[0]);
#endif

                *b08++ = a_channel; /* channel number */

                Npuls = (unsigned int *)b08; /* remember the place to put the number of pulses */
                Npuls[0] = 0;
                b08 += 4;
              }


              Npuls[0] ++;
              *b08++ = a_firstsample; /* first sample */

              Nsamp = (unsigned int *)b08; /* remember the place to put the number of samples */
              Nsamp[0] = 0;
              b08 += 4;

	          ii++;

              while( ((datain[ii]>>31)&0x1) == 0)
	          {
                a_valid1 = ((datain[ii]>>29)&0x1);
                a_adc1 = ((datain[ii]>>16)&0xFFF);
                a_valid2 = ((datain[ii]>>13)&0x1);
                a_adc2 = (datain[ii]&0xFFF);
#ifdef DEBUG
                printf("   [%3d] PULSE RAW DATA: valid1 %d, adc1 0x%04x, valid2 %d, adc2 0x%04x\n",ii,
                  a_valid1,a_adc1,a_valid2,a_adc2);
#endif

                b16 = (unsigned short *)b08;
                *b16 ++ = a_adc1;
                *b16 ++ = a_adc2;
                b08 += 4;

                Nsamp[0] += 2;
                ii++;
	          }

            }












            else if( ((datain[ii]>>27)&0x1F) == 0x17) /* pulse integral */
            {
              a_channel = ((datain[ii]>>23)&0xF);
              a_pulsenumber = ((datain[ii]>>21)&0x3);
              a_qualityfactor = ((datain[ii]>>19)&0x3);
              a_pulseintegral = (datain[ii]&0x7FFFF);
#ifdef DEBUG
	          printf("[%3d] PULSE INTEGRAL: channel %d, pulse number %d, quality %d, integral %d\n",ii,
			    a_channel,a_pulsenumber,a_qualityfactor,a_pulseintegral);
#endif


#ifdef DEBUG
              printf("0x%08x: pulseintegral = %d\n",b08,a_pulseintegral);
#endif


#ifdef MODE7
              if(npulse_VmVp[iev][a_slot][a_channel] == 0) /* it means mode 3 - fill output, otherwise do nothing */
			  {
#endif
                b32 = (unsigned int *)b08;
                *b32 = a_pulseintegral;
                b08 += 4;
#ifdef MODE7
			  }
#endif


	          ii++;
            }








            else if( ((datain[ii]>>27)&0x1F) == 0x18) /* pulse time */
            {
              a_channel = ((datain[ii]>>23)&0xF);
              a_pulsenumber = ((datain[ii]>>21)&0x3);
              a_qualityfactor = ((datain[ii]>>19)&0x3);
              a_pulsetime = (datain[ii]&0xFFFF);
#ifdef DEBUG
	          printf("[%3d] PULSE TIME: channel %d, pulse number %d, quality %d, time %d\n",ii,
			    a_channel,a_pulsenumber,a_qualityfactor,a_pulsetime);
#endif


#ifdef MODE7
              if(npulse_VmVp[iev][a_slot][a_channel] > 0) /* it means mode 7 */
			  {
if(a_pulsenumber == 0)
{
                CCOPEN(0xe102,"c,i,l,N(c,N(s,i,s,s))",banknum);
#ifdef DEBUG
                printf("0x%08x: CCOPEN(2), dataout=0x%08x\n",b08,dataout);
#endif
}
			  }
			  else
			  {
#endif
                CCOPEN(0xe103,"c,i,l,N(c,N(s,i))",banknum);
#ifdef DEBUG
                printf("0x%08x: CCOPEN(3), dataout=0x%08x\n",b08,dataout);
#endif
#ifdef MODE7
			  }
#endif

              if(a_channel != a_channel_old)
              {
                a_channel_old = a_channel;

                Nchan[0] ++; /* increment channel counter */
#ifdef DEBUG
                printf("0x%08x: increment Nchan[0]=%d\n",b08,Nchan[0]);
#endif

#ifdef DEBUG
		        printf("0x%08x: channel %d\n",b08,a_channel);
#endif
                *b08++ = a_channel; /* channel number */

                Npuls = (unsigned int *)b08; /* remember the place to put the number of pulses */
                Npuls[0] = 0;
#ifdef DEBUG
		        printf("0x%08x: Npuls[0]\n",b08);
#endif
                b08 += 4;
              }


#ifdef MODE7
              if(npulse_VmVp[iev][a_slot][a_channel] > 0) /* it means mode 7 */
			  {
if(a_pulsenumber == 0)
{
			    Npuls[0] = npulse_integral[iev][a_slot][a_channel];
				if(Npuls[0]>4) printf("ERROR: Npuls=%d\n",Npuls[0]);
                for(i=0; i<Npuls[0]; i++)
			    {                
                  b16 = (unsigned short *)b08;
                  *b16 = pulse_time[iev][a_slot][a_channel][i];
                  b08 += 2;

                  b32 = (unsigned int *)b08;
                  *b32 = pulse_integral[iev][a_slot][a_channel][i];
                  b08 += 4;

                  b16 = (unsigned short *)b08;
                  *b16 = pulse_Vm[iev][a_slot][a_channel][i];
                  b08 += 2;

                  b16 = (unsigned short *)b08;
                  *b16 = pulse_Vp[iev][a_slot][a_channel][i];
                  b08 += 2;
			    }
}
			  }
			  else
			  {
                Npuls[0] ++;
                b16 = (unsigned short *)b08;
                *b16 = a_pulsetime;
                b08 += 2;
			  }
#else

              Npuls[0] ++;
#ifdef DEBUG
              printf("0x%08x: pulsetime = %d\n",b08,a_pulsetime);
#endif
              b16 = (unsigned short *)b08;
              *b16 = a_pulsetime;
              b08 += 2;
#endif




	          ii++;
            }
















            else if( ((datain[ii]>>27)&0x1F) == 0x19)
            {
#ifdef DEBUG
	          printf("[%3d] STREAMING RAW DATA: \n",ii);
#endif
	          ii++;
            }




















            else if( ((datain[ii]>>27)&0x1F) == 0x1a) /* Vm Vp */
            {
              a_channel = ((datain[ii]>>23)&0xF);
              a_pulsenumber = ((datain[ii]>>21)&0x3);
              a_vm = ((datain[ii]>>12)&0x1FF);
              a_vp = (datain[ii]&0xFFF);
#ifdef DEBUG
	          printf("[%3d] PULSE VmVp: channel %d, pulse number %d, Vm %d, Vp %d\n",ii,
			    a_channel,a_pulsenumber,a_vm,a_vp);
#endif

              /* do not output anything, will be done above in 'pulse time' processing */

	          ii++;
            }
















            else if( ((datain[ii]>>27)&0x1F) == 0x1D)
            {
#ifdef DEBUG
	          printf("[%3d] EVENT TRAILER: \n",ii);
#endif
	          ii++;
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x1E)
            {
	          printf("[%3d] : DATA NOT VALID\n",ii);
              exit(0);
	          ii++;
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x1F)
            {
#ifdef DEBUG
	          printf("[%3d] FILLER WORD: \n",ii);
              printf(">>> do nothing\n");
#endif
	          ii++;
            }
            else
            {
              if(printing) /* printing only once at every event */
              {
                printf("[%3d] pass2 ERROR: in FADC data format 0x%08x (bits31-27=0x%02x)\n",
			      ii,(int)datain[ii],(datain[ii]>>27)&0x1F);
                printing=0;
	          }
              ii++;
            }

          } /*while*/

        } /* loop over blocks */


        if(b08 != NULL) CCCLOSE; /*call CCCLOSE only if CCOPEN was called*/
#ifdef DEBUG
          printf("0x%08x: CCCLOSE1, dataout=0x%08x\n",b08,dataout);
          printf("0x%08x: CCCLOSE2, dataout=0x%08x\n",b08,(unsigned int *) ( ( ((unsigned int)b08+3)/4 ) * 4));
          printf("-> 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
		    (unsigned int)b08+3,((unsigned int)b08+3),
            ((unsigned int)b08+3) / 4,(((unsigned int)b08+3) / 4)*4, 
		    (unsigned int *)((((unsigned int)b08+3) / 4)*4) );
#endif


	  } /* FADC250 hardware format */





      else if(banktag[jj] == 0xe10A) /* TI/TS hardware format */
	  {

        banknum = iev; /*rol->pid;*/

#ifdef DEBUG1
		printf("SECOND PASS TI\n");
#endif
        for(ibl=0; ibl<nB[jj]; ibl++) /*loop over blocks*/
        {
#ifdef DEBUG1
          printf("\n\n\n0xe10A: Block %d, Event %2d, event index %2d, event lenght %2d\n",
            ibl,iev,iE[jj][ibl][iev],lenE[jj][ibl][iev]);
#endif
          a_slot = sB[jj][ibl];
          ii = iE[jj][ibl][iev];
          rlen = ii + lenE[jj][ibl][iev];
          while(ii<rlen)
          {
#ifdef DEBUG1
            printf("[%3d] 0x%08x (rlen=%d)\n",ii,datain[ii],rlen);
#endif

            CPOPEN(0xe10A,1,banknum);

#ifdef DEBUG1
		    printf("[%3d] CPOPEN(0xe10A,,) b08=0x%08x\n",ii,b08);
#endif
			
            /* event header */
            a_event_type = ((datain[ii]>>24)&0xFF);
			if(a_event_type==0) printf("ERROR in TI event header word: event type = %d\n",a_event_type);
            if(((datain[ii]>>16)&0xFF)!=0x01) printf("ERROR in TI event header word (0x%02x)\n",((datain[ii]>>16)&0xFF));
            a_nwords = datain[ii]&0xFF;
#ifdef DEBUG1
		    printf("[%3d] EVENT HEADER, a_nwords = %d\n",ii,a_nwords);
#endif
			
            dataout[0] = datain[ii];
            b08 += 4;

		    ii++;

		    if(a_nwords>0)
		    {
		      a_event_number_l = datain[ii];
#ifdef DEBUG1
		      printf("[%3d] a_event_number_1 = %d\n",ii,a_event_number_l);
#endif
			  
              dataout[1] = datain[ii];
              b08 += 4;
			  
		      ii++;
		    }

		    if(a_nwords>1)
		    {
		      a_timestamp_l = datain[ii];
#ifdef DEBUG1
		      printf("[%3d] a_timestamp_l = %d\n",ii,a_timestamp_l);
#endif
			  
              dataout[2] = datain[ii];
              b08 += 4;
			  
		      ii++;
		    }

		    if(a_nwords>2)
		    {
		      a_event_number_h = (datain[ii]>>16)&0xF;
		      a_timestamp_h = datain[ii]&0xFFFF;
#ifdef DEBUG1
		      printf("[%3d] a_event_number_h = %d a_timestamp_h = %d \n",ii,a_event_number_h,a_timestamp_h);
#endif
			  
              dataout[3] = datain[ii];
              b08 += 4;
			  
		      ii++;
		    }

		    if(a_nwords>3)
		    {
		      a_bitpattern = datain[ii]&0xFF;
#ifdef DEBUG1
		      printf("[%3d] a_bitpattern = 0x%08x\n",ii,a_bitpattern);
#endif
              for(i=0; i<NTRIGBITS; i++) bitscalers[i] += ((a_bitpattern>>i)&0x1);
              havebits = 1;

              dataout[4] = datain[ii];
              b08 += 4;
			  
		      ii++;
		    }
			
		    if(a_nwords>4)
		    {
		      a_bitpattern = datain[ii]&0xFF;
#ifdef DEBUG1
		      printf("[%3d] TS word4 = 0x%08x\n",ii,datain[ii]);
#endif
              dataout[5] = datain[ii];
              b08 += 4;

		      ii++;
		    }

		    if(a_nwords>5)
		    {
#ifdef DEBUG1
		      printf("[%3d] TS word5 = 0x%08x\n",ii,datain[ii]);
#endif
              dataout[6] = datain[ii];
              b08 += 4;

		      ii++;
		    }

		    if(a_nwords>6)
		    {
#ifdef DEBUG1
		      printf("[%3d] TS word6 = 0x%08x\n",ii,datain[ii]);
#endif
              dataout[7] = datain[ii];
              b08 += 4;

		      ii++;
		    }

		    if(a_nwords>7)
		    {
		      printf("ERROR: TS has more then 7 data words - exit\n");
		      exit(0);
		    }


#ifdef DEBUG1
		    printf("[%3d] CPCLOSE b08=0x%08x\n",ii,b08);
#endif
            CPCLOSE;


	      } /* while() */

        } /* loop over blocks */

	  } /* TI hardware format */












      else if(banktag[jj] == 0xe10B) /* TDC hardware format */
	  {

        banknum = iev; /*rol->pid;*/
#ifdef DEBUG2
        printf("CPOPEN 0xe107\n");
#endif
        CPOPEN(0xe107,1,banknum);

        for(ibl=0; ibl<nB[jj]; ibl++) /*loop over blocks*/
        {
#ifdef DEBUG2
		  printf("\nSECOND PASS TDC (ibl=%d, nB=%d)\n",ibl,nB[jj]);
          printf("\n\n\n0xe10B: Block %d, Event %2d, event index %2d, event lenght %2d\n",
            ibl,iev,iE[jj][ibl][iev],lenE[jj][ibl][iev]);
#endif




          a_slot = sB[jj][ibl];
          ii = iE[jj][ibl][iev];
          rlen = ii + lenE[jj][ibl][iev];
          while(ii<rlen)
          {
#ifdef DEBUG2
            printf("[%5d] 0x%08x (rlen=%d)\n",ii,datain[ii],rlen);
#endif
		    if( ((datain[ii]>>27)&0x1F) == 0) /* data */
            {
              nbcount ++;

              tdcedge = ((datain[ii]>>26)&0x1);
		      /*printf("tdcslot=%d tdctypebyslot=%d\n",tdcslot,tdctypebyslot[tdcslot]);*/
              if( tdctypebyslot[tdcslot] == 1 ) /*1190*/
              {
                tdcchan = ((datain[ii]>>19)&0x7F);
                tdcval = (datain[ii]&0x7FFFF);
			    /*if(tdcchan==63) {reftdc=1; printf("[ev %d] befor: 0x%08x\n",*(rol->nevents),tdcval);}*/
                tdcval = tdcval<<2; /*report 1190 value with same resolution as 1290*/
			    /*if(tdcchan==63) printf("[ev %d] after: 0x%08x (0x%08x)\n",*(rol->nevents),tdcval,tdcval&0x7FFFF);*/
              }
              else /*1290*/
              {
                tdcchan = ((datain[ii]>>21)&0x1F);
                tdcval = (datain[ii]&0x1FFFFF);
		      }

              /* output data in 'standard' format: slot[27-31]  edge[26]  chan[19-25]  tdc[0-18] */
              *dataout ++ = (tdcslot<<27) + (tdcedge<<26) + (tdcchan<<19) + (tdcval&0x7FFFF);
              b08 += 4;

#ifdef DEBUG2
		      if(1/*tdcchan<2*/)
              {
                printf("   DATA: type=%1d slot=%02d tdc=%02d ch=%03d edge=%01d value=%08d\n",
				   tdctypebyslot[tdcslot],tdcslot,(int)tdc14,(int)tdcchan,(int)tdcedge,(int)tdcval);
		      }
#endif
            }
            else if( ((datain[ii]>>27)&0x1F) == 1)
            {
              tdc14 = ((datain[ii]>>24)&0x3);
              tdceventid = ((datain[ii]>12)&0xFFF);
              tdcbunchid = (datain[ii]&0xFFF);
#ifdef DEBUG2
              printf(" HEAD: tdc=%02d event_id=%05d bunch_id=%05d\n",
                (int)tdc14,(int)tdceventid,(int)tdcbunchid);
#endif
            }
            else if( ((datain[ii]>>27)&0x1F) == 3)
            {
              tdc14 = ((datain[ii]>>24)&0x3);
              tdceventid = ((datain[ii]>12)&0xFFF);
              tdcwordcount = (datain[ii]&0xFFF);
#ifdef DEBUG2
              printf(" EOB:  tdc=%02d event_id=%05d word_count=%05d\n",
                (int)tdc14,(int)tdceventid,(int)tdcwordcount);
#endif
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x18)
            {
#ifdef DEBUG2
              printf(" FILLER\n");
#endif
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x11)
            {
              nbsubtract ++;
#ifdef DEBUG2
              printf(" TIME TAG: %08x\n",(int)(datain[ii]&0x7ffffff));
#endif
            }
            else if( ((datain[ii]>>27)&0x1F) == 8)
            {
              nbcount = 1; /* counter for the number of output words from board */
              tdcslot = datain[ii]&0x1F;
              tdceventcount = (datain[ii]>>5)&0x3FFFFF;
              tdchead = (unsigned int *) dataout; /* remember pointer */
#ifdef SLOTWORKAROUND
              tdcslot_h = tdcslot;
	          remember_h = datain[ii];

              /* correct slot number - cannot do it that was for multi-event readout
              if(slotnums[nheaders] != tdcslot)
		      {
                if( !((*(rol->nevents))%10000) ) printf("WARN: [%2d] slotnums=%d, tdcslot=%d -> use slotnums\n",
				   nheaders,slotnums[nheaders],tdcslot);
                tdcslot = slotnums[nheaders];
		      }
		      */
#endif

#ifdef SLOTWORKAROUND_XXX
              if(tdcslot!=slotnums[ibl])
			  {
                /*printf("pass2 ERROR: WRONG TDC SLOT: ibl=%d slot=%d(%d)\n",ibl,tdcslot,slotnums[ibl]);*/
				tdcslot = slotnums[ibl];
			  }
#endif

              nbsubtract = NBsubtract[tdcslot]; /* # words to subtract including errors (5 for v1290N, 9 for others) */

			  /*sergey: do not output global header
              *dataout ++ = tdcslot;
              b08 += 4;
			  */

              nheaders++;

		  
#ifdef DEBUG2
		  
              printf("GLOBAL HEADER: 0x%08x (slot=%d nevent=%d) -> header=0x%08x\n",
                datain[ii],datain[ii]&0x1F,(datain[ii]>>5)&0xFFFFFF,*tdchead);
		  
#endif
		  
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x10)
            {
#ifdef SLOTWORKAROUND
              /* double check for slot number */
              tdcslot_t = datain[ii]&0x1F;

		      /*
              if(slotnums[ntrailers] != tdcslot_t)
		      {
                if( !((*(rol->nevents))%10000) ) printf("WARN: [%2d] slotnums=%d, tdcslot=%d\n",
				   ntrailers,slotnums[ntrailers],tdcslot_t);
		      }
		      */


              if(tdcslot_h>21||tdcslot_t>21||tdcslot_h!=tdcslot_t)
              {
                /*printf("WARN: slot from header=%d (0x%08x), from trailer=%d (0x%08x), must be %d\n",
			      tdcslot_h,remember_h,tdcslot_t,datain[ii],tdcslot)*/;
              }
#endif
              /* put nwords in header; substract 4 TDC headers, 4 TDC EOBs,
              global trailer and error words if any */
              /**tdchead = (*tdchead) | (((datain[ii]>>5)&0x3FFF) - nbsubtract);*/
              *tdchead |= ((((datain[ii]>>5)&0x3FFF) - nbsubtract)<<5);
              if((((datain[ii]>>5)&0x3FFF) - nbsubtract) != nbcount)
              {			
                printf("ERROR: word counters: %d != %d (subtract=%d)\n",
			      (((datain[ii]>>5)&0x3FFF) - nbsubtract),nbcount,nbsubtract);
              }
              ntrailers ++;

              nwtdc = (datain[ii]>>5)&0x3FFF;
#ifdef DEBUG2
              printf("GLOBAL TRAILER: 0x%08x (slot=%d nw=%d stat=%d) -> header=0x%08x\n",
                datain[ii],datain[ii]&0x1F,(datain[ii]>>5)&0x3FFF,
                (datain[ii]>>23)&0xF,*tdchead);
#endif

#ifdef DEBUG2
		      printf("0xe10B: close event jj=%d event# %d length %d\n",jj,m,lenE[jj][k][m]);
#endif
            }
            else if( ((datain[ii]>>27)&0x1F) == 4)
            {
              nbsubtract ++;
              tdc14 = ((datain[ii]>>24)&0x3);
              tdcerrorflags = (datain[ii]&0x7FFF);
		  /*
#ifdef DEBUG2
		  */
		      if( !(error_flag[tdcslot]%100) ) /* do not print for every event */
              {
                unsigned int ddd, lock, ntdc;
			
                printf(" ERR: event# %7d, slot# %2d, tdc# %02d, error_flags=0x%08x, err=0x%04x, lock=0x%04x\n",
                  tdceventcount,tdcslot,(int)tdc14,(int)tdcerrorflags,ddd,lock);
		      }
              error_flag[tdcslot] ++;
		  /*
#endif
		  */
            }
            else
            {
#ifdef DEBUG2
              printf("ERROR: in TDC data format 0x%08x\n",(int)datain[ii]);
#endif
            }

		    ii ++;

	      } /* while() */


        } /* loop over blocks */

        CPCLOSE;
#ifdef DEBUG2
        printf("CPCLOSE 0xe107\n");
#endif

	  } /* TDC hardware format */







      else if(banktag[jj] == 0xe10C) /* SSP hardware format */
	  {

        banknum = iev; /*rol->pid;*/

#ifdef DEBUG3
		printf("SECOND PASS SSP\n");
#endif
        for(ibl=0; ibl<nB[jj]; ibl++) /*loop over blocks*/
        {
#ifdef DEBUG3
          printf("\n\n\n0xe10C: Block %d, Event %2d, event index %2d, event lenght %2d\n",
            ibl,iev,iE[jj][ibl][iev],lenE[jj][ibl][iev]);
#endif

          CPOPEN(0xe10C,1,banknum);

          a_slot = sB[jj][ibl];
          ii = iE[jj][ibl][iev];
          rlen = ii + lenE[jj][ibl][iev];
          while(ii<rlen)
          {
#ifdef DEBUG3
            printf("[%5d] 0x%08x (rlen=%d)\n",ii,datain[ii],rlen);
#endif
			
            if( ((datain[ii]>>27)&0x1F) == 0x12) /*event header*/
			{
              a_triggernumber = (datain[ii]&0x7FFFFFF);
#ifdef DEBUG3
		      printf("[%3d] EVENT HEADER, a_triggernumber = %d\n",ii,a_triggernumber);
#endif
			
              *dataout ++ = datain[ii];
              b08 += 4;

		      ii++;
			}

            else if( ((datain[ii]>>27)&0x1F) == 0x13) /*trigger time: remember timestamp*/
            {
              a_trigtime[0] = (datain[ii]&0xFFFFFF);
              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;

              a_trigtime[1] = (datain[ii]&0xFFFFFF);
              *dataout ++ = datain[ii];
              b08 += 4;
              ii++;

#ifdef DEBUG3
              printf(">>> remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
	        }

            else if( ((datain[ii]>>27)&0x1F) == 0x14) /*hps cluster*/
            {
              a_clusterN = ((datain[ii]>>23)&0xF);
              a_clusterE = ((datain[ii]>>10)&0x1FFF);
		      a_clusterY = ((datain[ii]>>6)&0xF);
              a_clusterX = (datain[ii]&0x3F);
              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;

              a_clusterT = (datain[ii]&0x3FF);
              *dataout ++ = datain[ii];
              b08 += 4;
              ii++;

#ifdef DEBUG3
		      printf("[%3d] HPS CLUSTER_(N,E,Y,X,T): %d %d %d %d\n",ii,a_clusterN,a_clusterE,a_clusterY,a_clusterX,a_clusterT);
#endif
            }

            else if( ((datain[ii]>>27)&0x1F) == 0x15) /*hps trigger*/
            {
              a_type = ((datain[ii]>>23)&0xF);
              a_data = ((datain[ii]>>16)&0x7F);
		      a_time = (datain[ii]&0x3FF);
              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;
#ifdef DEBUG3
	          printf("[%3d] HPS TRIGGER: TYPE %d, DATA %d, TIME %d\n",ii,a_type,a_data,a_time);
#endif
		    }

            else if( ((datain[ii]>>27)&0x1F) == 0x1F)
            {
#ifdef DEBUG3
	          printf("[%3d] FILLER WORD\n",ii);
              printf(">>> do nothing\n");
#endif
	          ii++;
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x1E)
            {
#ifdef DEBUG3
	          printf("[%3d] DNV\n",ii);
              printf(">>> do nothing\n");
#endif
	          ii++;
            }
            else
		    {
              printf("SSP UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
		    }

	      } /* while() */

          CPCLOSE;

        } /* loop over blocks */

	  } /* SSP hardware format */





















      else if(banktag[jj] == 0xe122) /* VTP hardware format */
	  {

        banknum = iev; /*rol->pid;*/

#ifdef DEBUG7
		printf("\nSECOND PASS VTP\n");
#endif
        for(ibl=0; ibl<nB[jj]; ibl++) /*loop over blocks*/
        {
#ifdef DEBUG7
          printf("\n0xe122: Block %d, Event %2d, event index %2d, event lenght %2d\n",
            ibl,iev,iE[jj][ibl][iev],lenE[jj][ibl][iev]);
#endif

          CPOPEN(0xe122,1,banknum);

          a_slot = sB[jj][ibl];
          ii = iE[jj][ibl][iev];
          rlen = ii + lenE[jj][ibl][iev];
          while(ii<rlen)
          {
#ifdef DEBUG7
            printf("[%5d] 0x%08x (rlen=%d)\n",ii,datain[ii],rlen);
#endif
			
            if( ((datain[ii]>>27)&0x1F) == 0x12) /*event header*/
			{
              a_triggernumber = (datain[ii]&/*0x7FFFFFF*/0x3FFFFF);
#ifdef DEBUG7
		      printf("[%3d] EVENT HEADER, a_triggernumber = %d\n",ii,a_triggernumber);
#endif
			
              *dataout ++ = datain[ii];
              b08 += 4;

		      ii++;
			}

            else if( ((datain[ii]>>27)&0x1F) == 0x13) /*trigger time: remember timestamp*/
            {
              a_trigtime[0] = (datain[ii]&0xFFFFFF);
              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;

              a_trigtime[1] = (datain[ii]&0xFFFFFF);
              *dataout ++ = datain[ii];
              b08 += 4;
              ii++;

#ifdef DEBUG7
              printf(">>> remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
#endif
	        }





            else if( ((datain[ii]>>27)&0x1F) == 0x14) /*ECAL peak*/
            {
              a_instance = ((datain[ii]>>26)&0x1);
              a_view =     ((datain[ii]>>24)&0x3);
              a_time =     ((datain[ii]>>16)&0xFF);

              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;


		      a_coord =    ((datain[ii]>>16)&0x3FF);
              a_energy =   ((datain[ii]>>0)&0xFFFF);

              *dataout ++ = datain[ii];
              b08 += 4;
              ii++;

#ifdef DEBUG7
	          printf("[%3d] PEAK_(INST,VIEW,COORD,ENERGY,TIME): %d %d %d %d %d\n",ii,a_instance,a_view,a_coord,a_energy,a_time);
#endif
            }

            else if( ((datain[ii]>>27)&0x1F) == 0x15) /*ECAL cluster*/
            {
              a_instance = ((datain[ii]>>26)&0x1);
              a_time =     ((datain[ii]>>16)&0xFF);
		      a_energy =   ((datain[ii]>>0)&0xFFFF);

              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;


		      a_coordW = ((datain[ii]>>20)&0x3FF);
		      a_coordV = ((datain[ii]>>10)&0x3FF);
		      a_coordU = ((datain[ii]>>0)&0x3FF);

              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;
#ifdef DEBUG7
              printf("   [%3d] CLUSTER_(INST,COORDU,COORDV,COORDW,ENERGY,TIME): %d %d %d %d %d %d\n",ii,
                a_instance,a_coordU,a_coordV,a_coordW,a_energy,a_time);
#endif
		    }

            else if( ((datain[ii]>>27)&0x1F) == 0x16) /*HTCC trigger bits*/
            {
			  /*
              a_instance = ((datain[ii]>>16)&0x1);
              a_lane = ((datain[ii]>>11)&0x1F);
		      a_time = (datain[ii]&0x7FF);
			  */

              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;

	          printf("HTCC mask 47-31: 0x%08x\n",datain[ii]);
              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;

	          printf("HTCC mask 30-00: 0x%08x\n",datain[ii]);
              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;

			  /*
#ifdef DEBUG7
	          printf("[%3d] TRIG_(INST,LANE,TIME) %d %d %d\n",ii,a_instance,a_lane,a_time);
#endif
			  */
		    }




            else if( ((datain[ii]>>27)&0x1F) == 0x17) /*FT cluster*/
            {
              a_h2tag = ((datain[ii]>>15)&0x1);
              a_h1tag =     ((datain[ii]>>14)&0x1);
		      a_nhits =   ((datain[ii]>>10)&0xF);
		      a_coordY = ((datain[ii]>>5)&0x1F);
		      a_coordX = ((datain[ii]>>0)&0x1F);

              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;


		      a_energy = ((datain[ii]>>16)&0x3FFF);
		      a_time = ((datain[ii]>>0)&0x7FF);

              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;
#ifdef DEBUG7
              printf("   [%3d] FT CLUSTER_(COORDX,COORDY,NHITS,ENERGY,TIME): %d %d %d %d %d %d\n",ii,
                a_coordX,a_coordY,a_nhits,a_energy,a_time);
#endif
		    }



            else if( ((datain[ii]>>27)&0x1F) == 0x18) /*???*/
            {
              printf("VTP UNKNOWN data18: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
			}
            else if( ((datain[ii]>>27)&0x1F) == 0x19) /*???*/
            {
              printf("VTP UNKNOWN data19: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
			}
            else if( ((datain[ii]>>27)&0x1F) == 0x1A) /*???*/
            {
              printf("VTP UNKNOWN data1A: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
			}
            else if( ((datain[ii]>>27)&0x1F) == 0x1B) /*???*/
            {
              printf("VTP UNKNOWN data1B: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
			}
            else if( ((datain[ii]>>27)&0x1F) == 0x1C) /*???*/
            {
              printf("VTP UNKNOWN data1C: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
			}
            else if( ((datain[ii]>>27)&0x1F) == 0x1D) /*???trig2?????*/
            {
			  /*
			  a_h2tag = ((datain[ii]>>15)&0x1);
              a_h1tag =     ((datain[ii]>>14)&0x1);
		      a_nhits =   ((datain[ii]>>10)&0xF);
		      a_coordY = ((datain[ii]>>5)&0x1F);
		      a_coordX = ((datain[ii]>>0)&0x1F);
			  */

              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;

			  /*
		      a_energy = ((datain[ii]>>16)&0x3FFF);
		      a_time = ((datain[ii]>>0)&0x7FF);
			  */

              *dataout ++ = datain[ii];
              b08 += 4;
	          ii++;
			  /*
#ifdef DEBUG7
              printf("   [%3d] FT CLUSTER_(COORDX,COORDY,NHITS,ENERGY,TIME): %d %d %d %d %d %d\n",ii,
                a_coordX,a_coordY,a_nhits,a_energy,a_time);
#endif
			  */
		    }



            else if( ((datain[ii]>>27)&0x1F) == 0x1F)
            {
#ifdef DEBUG7
	          printf("[%3d] FILLER WORD\n",ii);
              printf(">>> do nothing\n");
#endif
	          ii++;
            }
            else if( ((datain[ii]>>27)&0x1F) == 0x1E)
            {
#ifdef DEBUG7
	          printf("[%3d] DNV\n",ii);
              printf(">>> do nothing\n");
#endif
	          ii++;
            }
            else
		    {
              printf("VTP UNKNOWN data2: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
		    }

	      } /* while() */

          CPCLOSE;

        } /* loop over blocks */

	  } /* VTP hardware format */























/****************************************************
 * MVT START : rol2trig second pass
 ***************************************************/
/* Hardware format
F3BB BEUIDBlkNum
F3EE FineTSTP TSTP1 TSTP2 TSTP3 EvtNum1 EvtNum2 EvtNum3 SmpNum AcceptEvtNum
F311 LnkNumSize FEUdata...
F311 LnkNumSize FEUdata...
F355 FineTSTP TSTP1 TSTP2 TSTP3 EvtNum1 EvtNum2 EvtNum3 SmpNum AcceptEvtNum
F311 LnkNumSize FEUdata...
F311 LnkNumSize FEUdata...
Filler...
FCCA FCAA
*/
#ifdef USE_MVT
    else if(banktag[jj] == 0xe118) /* MVT hardware format */
    {
#ifdef DEBUG6_MVT_2ND_PASS
        if( mvt_fptr_err_2 != (FILE *)NULL )
        {
            fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118 \n", __FUNCTION__);
            fflush(mvt_fptr_err_2);
        }
        printf("SECOND PASS MVT\n");
#endif
        banknum = iev; /*rol->pid;*/
        a_slot_old = -1;
        if ( (mvt_event_number % MVT_PRESCALE ) ||( MVT_PRESCALE == 1000000 ) )
        {
/*
            //CPOPEN(0xefef,1,banknum);
            //printf("0xe118 : event number %8x %8x %8x\n", mvt_event_number,mvt_event_number % MVT_PRESCALE,(mvt_event_number % MVT_PRESCALE ) ||( MVT_PRESCALE == 1000000 ) );
            *dataout ++ = 0xCAFEFADE;
            b08 += 4;
            for(ibl=0; ibl < MVT_NBR_OF_BEU; ibl++)
            { 
                *dataout ++ = lenE[jj][ibl][iev];
                b08 += 4;
            }
            *dataout ++ = 0xDEADBEEF;
            b08 += 4;
            //CPCLOSE;
*/
        }
        else // Real Data
        {		
            if( mvt_error_counter ==0 )
            {	
                //output the FEU data samples 				
                for(ibl=0; ibl < MVT_NBR_OF_BEU; ibl++)
                {   							
                    currentBeuId = ((( datain[ iB[jj][ibl] ] ) & 0x0000F000 ) >> 12 ) - 1;
#ifdef DEBUG6_MVT_2ND_PASS
                    if( mvt_fptr_err_2 != (FILE *)NULL )
                    {
                        fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, currentBeuId = %d \n", __FUNCTION__, currentBeuId);
                        fflush(mvt_fptr_err_2);
                    }
#endif
                    for (i_feu = 0; i_feu < MVT_NBR_OF_FEU[currentBeuId]; i_feu++)
                    {
                        // Treat the FEU if it has fired channels
                        if
                        (
                          ((MVT_ZS_MODE != 2) && (nbchannelsFEU[jj][ibl][iev][0][i_feu]>0))
                          ||
                          ((MVT_ZS_MODE == 2) && (feu_chn_cnt[iev][i_feu]>0))
                        )
                        {
                            a_channel_old = -1;

                            // Find pointer to feu data
                            if( MVT_ZS_MODE != 2 )
									            ii = iFEU[jj][ibl][iev][0][i_feu];
                            else
                            {
                              for( i_channel=0; i_channel<MAX_FEU_CHN; i_channel++ ) 
                              {
                                // Check that the channel has data 
                                if( feu_chn_ent_cnt[iev][i_feu][i_channel] > 0 )
                                  break;
                              }
                              ii = iFEU[jj][ibl][iev][ feu_chn_ent_val[iev][i_feu][i_channel][0] ][i_feu];
                            }
 
                            // CURRENT SLOT NUMBER built from beu id and feu link id
                            currentFeuId = ((( datain[ ii] ) & 0x00007C00 ) >> 10 );		
                            a_slot= ((currentBeuId << 5) + currentFeuId  + 1 )& 0x000000FF ;
#ifdef DEBUG6_MVT_2ND_PASS								
                            if( mvt_fptr_err_2 != (FILE *)NULL )
                            {
                                fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, data[%d]=0x%08x currentFeuId = %d \n",
                                    __FUNCTION__, ii, datain[ii], currentFeuId);
                                fflush(mvt_fptr_err_2);
                            }
#endif
                            // OUTPUT CURRENT TRIGGER/EVNET NUMBER
                            //current_event_number_high =(( datain[ ( iSMP[jj][ibl][iev][0] + 2 ) ] ) & 0x00007FFC) >>  2;
                            current_event_number_low  = ((( datain[ ( iSMP[jj][ibl][iev][0] + 2 ) ] ) & 0x00000003) << 30) + 
                                                        ((( datain[ ( iSMP[jj][ibl][iev][0] + 3 ) ] ) & 0x7FFF0000) >>  1) +
                                                         (( datain[ ( iSMP[jj][ibl][iev][0] + 3 ) ] ) & 0x00007FFF);
                            a_triggernumber = current_event_number_low;
#ifdef DEBUG6_MVT_2ND_PASS								
                            if( mvt_fptr_err_2 != (FILE *)NULL )
                            {
                                fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, current_event_number_low = %d \n",
                                    __FUNCTION__, current_event_number_low);
                                fflush(mvt_fptr_err_2);
                            }
#endif
                            //OUTPUT CURRENT TIMESTAMP 
                            current_timestamp_low  = (((   datain[ ( iSMP[jj][ibl][iev][0] + 2 ) ] )  & 0x7FFF0000) << 1) +
                                                     (((~( datain[ ( iSMP[jj][ibl][iev][0]     ) ] )) & 0x00000800) << 5) ;
                            current_timestamp_high = (((   datain[ ( iSMP[jj][ibl][iev][0] + 1 ) ] )  & 0x7FFF0000) >> 1) + 
                                                     (((   datain[ ( iSMP[jj][ibl][iev][0] + 1 ) ] )  & 0x00007FFF));
                            // Feu Time stamp
                            current_feu_tmstmp   = current_timestamp_low +
                                                   ((( datain[ ii + 2] ) & 0x0FFF0000) >>  13 ) + (( datain[ ii + 2] ) & 0x00000007);
                            if( MVT_ZS_MODE )
									            current_feu_tmstmp |= (1<<15);
                            a_trigtime[0] = current_timestamp_high;
                            a_trigtime[1] = current_feu_tmstmp;
#ifdef DEBUG6_MVT_2ND_PASS								
                            if( mvt_fptr_err_2 != (FILE *)NULL )
                            {
                                fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, current_timestamp_high = %08x current_feu_tmstmp = %08x \n",
                                    __FUNCTION__, current_event_number_low, current_feu_tmstmp);
                                fflush(mvt_fptr_err_2);
                            }
#endif
                            if( MVT_ZS_MODE != 2 ) // Treat No ZS and Tracket ZS cases
                            {
                              if( MVT_CMP_DATA_FMT == 0 ) // Unpacked data format
                              {
                                MVT_CCOPEN_NOSMPPACK(0xe11b,"c,i,l,N(s,Ns)",banknum);
                                Nchan[0] = nbchannelsFEU[jj][ibl][iev][0][i_feu];
#ifdef DEBUG6_MVT_2ND_PASS								
                                if( mvt_fptr_err_2 != (FILE *)NULL )
                                {
                                    fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, nbchannelsFEU = %d\n",
                                        __FUNCTION__, nbchannelsFEU[jj][ibl][iev][0][i_feu]);
                                    fflush(mvt_fptr_err_2);
                                }
#endif
                                if( MVT_ZS_MODE )
                                {
                                    for( i_channel=0; i_channel<nbchannelsFEU[jj][ibl][iev][0][i_feu]; i_channel++ ) 
                                    {
                                        //OUTPUT CHANNEL NUMBER
                                        ii = iFEU[jj][ibl][iev][0][i_feu] + 1 + 2 + i_channel;
                                        current_channel_id = ((( datain[ ii ] ) & 0x01FF0000) >> 16 ) + 1;  
                                        b16 = (unsigned short *)( b08 );
                                        *b16++ = current_channel_id;
                                        b08 += 2;

                                        //OUTPUT  NUMBER OF SAMPLES			
                                        b32 = (unsigned int *)( b08 );
                                        * b32 ++ = MVT_NBR_SAMPLES_PER_EVENT;
                                        b16+=2;
                                        b08+=4;

                                        for( i_sample=0; i_sample<MVT_NBR_SAMPLES_PER_EVENT; i_sample++)
                                        {		
                                            ii = iFEU[jj][ibl][iev][i_sample][i_feu] + 1 + 2 + i_channel;
                                            current_sample = ((( datain[ ii ] ) & 0x00000FFF) );
                                            *b16++ = current_sample;
                                            b08 += 2;
                                        } /* loop over samples */
#ifdef DEBUG6_MVT_2ND_PASS								
                                        if( mvt_fptr_err_2 != (FILE *)NULL )
                                        {
                                            fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, LOOP OVER SAMPLES DONE \n", __FUNCTION__);
                                            fflush( mvt_fptr_err_2);
                                        }
#endif
				                            } /* loop over channels */
                                }	
                                else
                                {
                                    for ( i_channel = 0; i_channel < nbchannelsFEU[jj][ibl][iev][0][i_feu] ; i_channel ++ ) 
                                    {
                                        //GET CURRENT DREAM ID
                                        i_dream = i_channel >> 6 ;
                                        // point to dream header for the current channel
                                        ii = iFEU[jj][ibl][iev][0][i_feu] + 1 + 2 + 1 + (32 + 5)*i_dream ;
                                        current_channel_id = ((( datain[ ii ] ) & 0x00000E00) >> 3 ) + (i_channel%64) + 1;

                                        //OUTPUT CHANNEL NUMBER
                                        b16 = (unsigned short *)( b08 );
                                        *b16++ = current_channel_id;
                                        b08 += 2;

                                        //OUTPUT  NUMBER OF SAMPLES 							
                                        b32 = (unsigned int *)( b08 );
                                        *b32 ++ = MVT_NBR_SAMPLES_PER_EVENT;
                                        b16+=2;
                                        b08+=4;
#ifdef DEBUG6_MVT_2ND_PASS								
                                        if( mvt_fptr_err_2 != (FILE *)NULL )
                                        {
                                            fprintf
                                            (
                                                mvt_fptr_err_2,
                                                "%s: SECOND PASS 0xe118, i_dream = %d i_channel = %d , data[%d]=0x%08x current_channel_id =%d MVT_NBR_SAMPLES_PER_EVENT = %d \n",
                                                    __FUNCTION__, i_dream, i_channel, ii, datain[ ii ], current_channel_id, MVT_NBR_SAMPLES_PER_EVENT
                                            );
                                            fflush(mvt_fptr_err_2);
                                        }
#endif
                                        for ( i_sample = 0; i_sample < MVT_NBR_SAMPLES_PER_EVENT; i_sample++)
                                        {
                                            // Point to the first channel of the Dream
                                            // 1 for BEU extra word, 2 for FEU header + 2 for Dream header
                                            ii = iFEU[jj][ibl][iev][i_sample][i_feu] + 1 + 2 + 2 + (32+5)*i_dream;
                                            // jump to the required channel
                                            ii += ((i_channel%64) >> 1);
                                            if (i_channel%2)
                                            {
                                                current_sample = ((( datain[ ii ] ) & 0x00000FFF) );
                                            }
                                            else
                                            {
                                                current_sample = ((( datain[ ii ] ) & 0x0FFF0000)>> 16 );
                                            }
                                            *b16++ = current_sample;
                                            b08 += 2;
                                        } /* loop over samples */
#ifdef DEBUG6_MVT_2ND_PASS								
                                        if( mvt_fptr_err_2 != (FILE *)NULL )
                                        {
                                            fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, LOOP OVER SAMPLES DONE \n", __FUNCTION__);
                                            fflush( mvt_fptr_err_2);
                                        }
#endif
                                    } //loop over channels
                                } // else of if( MVT_ZS_MODE )
                              }
                              else // of if( MVT_CMP_DATA_FMT == 0 ) -> packed data format
                              {
                                MVT_CCOPEN_PACKSMP(0xe128,"c,i,l,n(s,mc)",banknum);
                                // set number of channels
                                b16 = (unsigned short *)( b08 );
                                *b16++ = ((short)nbchannelsFEU[jj][ibl][iev][0][i_feu]);
                                b08 += 2;

#ifdef DEBUG6_MVT_2ND_PASS								
                                if( mvt_fptr_err_2 != (FILE *)NULL )
                                {
                                    fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, nbchannelsFEU = %d\n",
                                        __FUNCTION__, nbchannelsFEU[jj][ibl][iev][0][i_feu]);
                                    fflush(mvt_fptr_err_2);
                                }
#endif
                                if( MVT_ZS_MODE )
                                {
                                    for( i_channel=0; i_channel<nbchannelsFEU[jj][ibl][iev][0][i_feu]; i_channel++ ) 
                                    {
                                        //OUTPUT CHANNEL NUMBER
                                        ii = iFEU[jj][ibl][iev][0][i_feu] + 1 + 2 + i_channel;
                                        current_channel_id = ((( datain[ ii ] ) & 0x01FF0000) >> 16 ) + 1;  
                                        b16 = (unsigned short *)( b08 );
                                        *b16++ = current_channel_id;
                                        b08 += 2;
                                        // set number of bytes in packed byte stream
                                        c_number_of_bytes = (((MVT_NBR_SAMPLES_PER_EVENT - 1)/2)*3+1)+1;
                                        if( (MVT_NBR_SAMPLES_PER_EVENT % 2) == 0 )
                                            c_number_of_bytes++;
                                        *b08 ++ = c_number_of_bytes;
#ifdef DEBUG6_MVT_2ND_PASS								
                                        if( mvt_fptr_err_2 != (FILE *)NULL )
                                        {
                                            fprintf
                                            (
                                                mvt_fptr_err_2,
                                                "%s: SECOND PASS 0xe128, MVT_ZS_MODE=1 current_channel_id =%d MVT_NBR_SAMPLES_PER_EVENT = %d c_number_of_bytes=%d\n",
                                                    __FUNCTION__, current_channel_id, MVT_NBR_SAMPLES_PER_EVENT, c_number_of_bytes
                                            );
                                            fflush(mvt_fptr_err_2);
                                        }
#endif
                                        // Pack samples in unsigned bytes
                                        for( i_sample=0; i_sample<MVT_NBR_SAMPLES_PER_EVENT; i_sample++)
                                        {		
                                            ii = iFEU[jj][ibl][iev][i_sample][i_feu] + 1 + 2 + i_channel;
                                            current_sample = ((( datain[ ii ] ) & 0x00000FFF) );
                                            if( (i_sample % 2) == 0 )
                                            {
                                                *b08++ = ((char)( current_sample    &0xFF)); // 8 LSB-s
                                                *b08   = ((char)((current_sample>>8)&0x0F)); // 4 MSB-s
                                            }
                                            else
                                            {
                                                *b08  |= ((char)((current_sample>>4)&0xF0)); // 4 MSB-s
                                                 b08++;
                                                *b08++ = ((char)( current_sample    &0xFF)); // 8 LSB-s
                                            }
                                        } /* loop over samples */
                                        if( MVT_NBR_SAMPLES_PER_EVENT % 2 )
                                        {
                                            *b08 &= 0x0F; // force high nibble to 0
                                            b08++;
                                        }
#ifdef DEBUG6_MVT_2ND_PASS								
                                        if( mvt_fptr_err_2 != (FILE *)NULL )
                                        {
                                            fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, LOOP OVER SAMPLES DONE \n", __FUNCTION__);
                                            fflush( mvt_fptr_err_2);
                                        }
#endif
				                            } /* loop over channels */
                                }
                                else
                                {
                                    for ( i_channel = 0; i_channel < nbchannelsFEU[jj][ibl][iev][0][i_feu] ; i_channel ++ ) 
                                    {
                                        //GET CURRENT DREAM ID
                                        i_dream = i_channel >> 6 ;
                                        // point to dream header for the current channel
                                        ii = iFEU[jj][ibl][iev][0][i_feu] + 1 + 2 + 1 + (32 + 5)*i_dream ;
                                        current_channel_id = ((( datain[ ii ] ) & 0x00000E00) >> 3 ) + (i_channel%64) + 1;

                                        //OUTPUT CHANNEL NUMBER
                                        b16 = (unsigned short *)( b08 );
                                        *b16++ = current_channel_id;
                                        b08 += 2;

                                        // set number of bytes in packed byte stream
                                        c_number_of_bytes = (((MVT_NBR_SAMPLES_PER_EVENT - 1)/2)*3+1)+1;
                                        if( (MVT_NBR_SAMPLES_PER_EVENT % 2) == 0 )
                                            c_number_of_bytes++;
                                        *b08 ++ = c_number_of_bytes;
#ifdef DEBUG6_MVT_2ND_PASS								
                                        if( mvt_fptr_err_2 != (FILE *)NULL )
                                        {
                                            fprintf
                                            (
                                                mvt_fptr_err_2,
                                                "%s: SECOND PASS 0xe128, MVT_ZS_MODE=0 i_dream = %d i_channel = %d , data[%d]=0x%08x current_channel_id =%d MVT_NBR_SAMPLES_PER_EVENT = %d c_number_of_bytes=%d\n",
                                                    __FUNCTION__, i_dream, i_channel, ii, datain[ ii ], current_channel_id, MVT_NBR_SAMPLES_PER_EVENT, c_number_of_bytes
                                            );
                                            fflush(mvt_fptr_err_2);
                                        }
#endif
                                        for ( i_sample = 0; i_sample < MVT_NBR_SAMPLES_PER_EVENT; i_sample++)
                                        {
                                            // Point to the first channel of the Dream
                                            // 1 for BEU extra word, 2 for FEU header + 2 for Dream header
                                            ii = iFEU[jj][ibl][iev][i_sample][i_feu] + 1 + 2 + 2 + (32+5)*i_dream;
                                            // jump to the required channel
                                            ii += ((i_channel%64) >> 1);
                                            if (i_channel%2)
                                            {
                                                current_sample = ((( datain[ ii ] ) & 0x00000FFF) );
                                            }
                                            else
                                            {
                                                current_sample = ((( datain[ ii ] ) & 0x0FFF0000)>> 16 );
                                            }
                                            if( (i_sample % 2) == 0 )
                                            {
                                                *b08++ = ((char)( current_sample    &0xFF)); // 8 LSB-s
                                                *b08   = ((char)((current_sample>>8)&0x0F)); // 4 MSB-s
                                            }
                                            else
                                            {
                                                *b08  |= ((char)((current_sample>>4)&0xF0)); // 4 MSB-s
                                                 b08++;
                                                *b08++ = ((char)( current_sample    &0xFF)); // 8 LSB-s
                                            }
                                        } /* loop over samples */
                                        if( MVT_NBR_SAMPLES_PER_EVENT % 2 )
                                        {
                                            *b08 &= 0x0F; // force high nibble to 0
                                            b08++;
                                        }
#ifdef DEBUG6_MVT_2ND_PASS								
                                        if( mvt_fptr_err_2 != (FILE *)NULL )
                                        {
                                            fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, LOOP OVER SAMPLES DONE \n", __FUNCTION__);
                                            fflush( mvt_fptr_err_2);
                                        }
#endif
                                    } // loop over channels
                                } // else of if( MVT_ZS_MODE )
                              } // else of if( MVT_CMP_DATA_FMT == 0 )
                            }
                            else // Treat TPC ZS mode
                            {
                              // packed data format only
                              MVT_CCOPEN_PACKSMP(0xe129,"c,i,l,n(s,m(c,mc))",banknum);
                              // set number of channels
                              b16 = (unsigned short *)( b08 );
                              *b16++ = ((short)feu_chn_cnt[iev][i_feu]);
                              b08 += 2;
#ifdef DEBUG6_MVT_2ND_PASS								
                              if( mvt_fptr_err_2 != (FILE *)NULL )
                              {
                                fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118/0xe129, nbchannelsFEU = %d\n",
                                  __FUNCTION__, feu_chn_cnt[iev][i_feu]);
                                fflush(mvt_fptr_err_2);
                              }
#endif
                              /* loop over channels */
                              for( i_channel=0; i_channel<MAX_FEU_CHN; i_channel++ ) 
                              {
                                // Check that the channel has data 
                                if( feu_chn_ent_cnt[iev][i_feu][i_channel] > 0 )
                                {
                                  // OUTPUT CHANNEL NUMBER
                                  b16 =    (unsigned short *)( b08 );
                                  *b16++ = (unsigned short  )(i_channel+1);
                                  b08+= 2;

                                  // Output number of pulses
                                  *b08 ++ = feu_chn_ent_cnt[iev][i_feu][i_channel];
#ifdef DEBUG6_MVT_2ND_PASS								
                                  if( mvt_fptr_err_2 != (FILE *)NULL )
                                  {
                                    fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118/0xe129, chan=%d npul=%d\n",
                                      __FUNCTION__, i_channel, feu_chn_ent_cnt[iev][i_feu][i_channel]);
                                    fflush(mvt_fptr_err_2);
                                  }
#endif
                                  // loop over pulses
                                  pul_dat_ptr = &(feu_chn_ent_val[iev][i_feu][i_channel][0]);
                                  for( i_pul=0; i_pul<feu_chn_ent_cnt[iev][i_feu][i_channel]; i_pul++ )
                                  {
#ifdef DEBUG6_MVT_2ND_PASS								
                                    if( mvt_fptr_err_2 != (FILE *)NULL )
                                    {
                                      fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118/0xe129,   pul=%d fsmp=%d",
                                        __FUNCTION__, i_pul, *pul_dat_ptr);
                                      fflush(mvt_fptr_err_2);
                                    }
#endif
                                    // Output first sample of the pulse
                                    *b08 ++ = (unsigned char)(*pul_dat_ptr++);
                                    // set number of bytes in packed byte stream
                                    // according to number of samples
                                    nb_of_smp = (unsigned char)(*pul_dat_ptr++);
                                    c_number_of_bytes = (((nb_of_smp - 1)/2)*3+1)+1;
                                    if( (nb_of_smp % 2) == 0 )
                                      c_number_of_bytes++;
#ifdef DEBUG6_MVT_2ND_PASS								
                                    if( mvt_fptr_err_2 != (FILE *)NULL )
                                    {
                                      fprintf(mvt_fptr_err_2," nsmp=%d nbytes=%d\n",
                                        nb_of_smp, c_number_of_bytes);
                                      fflush(mvt_fptr_err_2);
                                    }
#endif
                                    *b08 ++ = c_number_of_bytes;
                                    for( i_sample=0; i_sample<nb_of_smp; i_sample++)
                                    {		
                                      current_sample = *pul_dat_ptr++;
                                      if( (i_sample % 2) == 0 )
                                      {
                                        *b08++ = ((char)( current_sample    &0xFF)); // 8 LSB-s
                                        *b08   = ((char)((current_sample>>8)&0x0F)); // 4 MSB-s
                                      }
                                      else
                                      {
                                        *b08  |= ((char)((current_sample>>4)&0xF0)); // 4 MSB-s
                                        b08++;
                                        *b08++ = ((char)( current_sample    &0xFF)); // 8 LSB-s
                                      }
                                    } /* loop over samples */
                                    if( nb_of_smp % 2 )
                                    {
                                      *b08 &= 0x0F; // force high nibble to 0
                                      b08++;
                                    }
#ifdef DEBUG6_MVT_2ND_PASS								
                                    if( mvt_fptr_err_2 != (FILE *)NULL )
                                    {
                                      fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118/0xe129, LOOP OVER SAMPLES DONE \n", __FUNCTION__);
                                      fflush( mvt_fptr_err_2);
                                    }
#endif
                                  } // loop over pulses 
                                  // Clear channel for posterity
                                  feu_chn_ent_cnt[iev][i_feu][i_channel]=0;
                                  feu_chn_ent_ind[iev][i_feu][i_channel]=-1;
                                  feu_chn_ent_val[iev][i_feu][i_channel][0]=0;
                                  feu_chn_ent_val[iev][i_feu][i_channel][1]=0;
                                } // Check that the channel has data
                              } /* loop over channels */
                            } // End of TCP ZS mode
                        } // if( nbchannelsFEU[jj][ibl][iev][0][i_feu] > 0 ) do it if the feu has fired channels
                        // Clear FEU channel count for posterity
                        feu_chn_cnt[iev][i_feu] = 0;
                    } // for (i_feu = 0; i_feu < MVT_NBR_OF_FEU[currentBeuId]; i_feu++) loop over feus
                } // for(ibl=0; ibl < MVT_NBR_OF_BEU; ibl++) loop over blocks */

                if( b08 != NULL ) CCCLOSE;
#ifdef DEBUG6_MVT_2ND_PASS								
                if( mvt_fptr_err_2 != (FILE *)NULL )
                {
                    fprintf(mvt_fptr_err_2,"%s: SECOND PASS 0xe118, CCCLOSE DONE \n", __FUNCTION__);
                    fflush( mvt_fptr_err_2);
                }
#endif
            }
            else // of if (mvt_error_counter ==0)
            {
                printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                printf("0xe118 : mvt error counter 0x%8x %d mvt error type 0x%8x \n",
                    mvt_error_counter, mvt_error_counter, mvt_error_type );
                //CPOPEN(0xeffe,1,banknum);
                    //b32 = (unsigned int *)( b08 );
                    //*dataout ++ = mvt_error_counter;
                    //b08 += 4;
                    //*dataout ++ = mvt_error_type;
                    //b08 += 4;
                //CPCLOSE;
            } // else of if (mvt_error_counter ==0)
        } // else of if ( (mvt_event_number % MVT_PRESCALE ) ||( MVT_PRESCALE == 1000000 ) )
        //CPCLOSE;
        mvt_event_number++;
    } /* else if(banktag[jj] == 0xe118) /* MVT hardware format */
#endif // #define USE_MVT
/****************************************************
 * MVT END: rol2trig second pass
 ***************************************************/







      else if(banktag[jj] == 0xe112) /* HEAD bank raw format */
	  {

        banknum = iev; /*rol->pid;*/

#ifdef DEBUG5
		printf("\n\nSECOND PASS HEAD\n");
#endif
        for(ibl=0; ibl<nB[jj]; ibl++) /*loop over blocks*/
        {
#ifdef DEBUG5
          printf("\n\n\n0xe112: Block %d, Event %2d, event index %2d, event lenght %2d\n",
            ibl,iev,iE[jj][ibl][iev],lenE[jj][ibl][iev]);
#endif

#ifdef DEBUG5
          printf("CPOPEN(0xe10F,,)\n");
#endif
          CPOPEN(0xe10F,1,banknum);

          ii = iE[jj][ibl][iev];
          rlen = ii + lenE[jj][ibl][iev];
          while(ii<rlen)
          {
#ifdef DEBUG5
            printf("[%5d] 0x%08x (rlen=%d)\n",ii,datain[ii],rlen);
#endif
			
            if( ((datain[ii]>>27)&0x1F) == 0x12) /*event header*/
			{
              a_triggernumber = (datain[ii]&0x7FFFFFF);
#ifdef DEBUG5
		      printf("[%3d] EVENT HEADER, a_triggernumber = %d\n",ii,a_triggernumber);
#endif
		      ii++;
			}

            else if( ((datain[ii]>>27)&0x1F) == 0x14) /*head bank data*/
            {
              nw = (datain[ii]&0xFF);
#ifdef DEBUG5
	          printf("[%3d] head bank has %d data words\n",ii,nw);
#endif
	          ii++;
              iii=0;
              while( iii<nw ) /*get all data words*/
	          {
#ifdef DEBUG5
                printf("   [%3d][%3d]: %d\n",ii,iii,datain[ii]);
#endif
                *dataout ++ = datain[ii];
                b08 += 4;
                iii++;
                ii++;
	          }
            }

            else
		    {
              printf("PASS2 HEAD UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
		    }

	      } /* while() */

          CPCLOSE;

        } /* loop over blocks */

	  } /* HEAD bank raw format */










      else if(banktag[jj] == 0xe104) /* VSCM hardware format */
	  {

        banknum = iev; /*rol->pid;*/

#ifdef DEBUG4
		printf("\n\nSECOND PASS VSCM\n");
#endif
        a_slot_old = -1;
        for(ibl=0; ibl<nB[jj]; ibl++) /*loop over blocks*/
        {
          a_channel_old = -1;
#ifdef DEBUG4
          printf("\n\n\nVSCM: Block %d, Event %2d, event index %2d, event lenght %2d\n",
            ibl,iev,iE[jj][ibl][iev],lenE[jj][ibl][iev]);
#endif

          a_slot = sB[jj][ibl];
          ii = iE[jj][ibl][iev];
          rlen = ii + lenE[jj][ibl][iev];
          while(ii<rlen)
          {
#ifdef DEBUG4
            printf("[%5d] VSCM 0x%08x (rlen=%d)\n",ii,datain[ii],rlen);
#endif
			
            if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_EVTHDR) /*event header*/
			{
              a_triggernumber = (datain[ii]&0x7FFFFFF);
#ifdef DEBUG4
		      printf("[%3d] VSCM EVENT HEADER, a_triggernumber = %d\n",ii,a_triggernumber);
#endif
		      ii++;
			}

            else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_TRGTIME) /*trigger time: remember timestamp*/
            {
              a_trigtime[0] = (datain[ii]&0xFFFFFF);
	          ii++;

              a_trigtime[1] = (datain[ii]&0xFFFFFF);
              ii++;

		      timestamp = (((unsigned long long)a_trigtime[0])<<24) | (a_trigtime[1]);

#ifdef DEBUG4
              printf(">>> VSCM remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
		      printf(">>> VSCM timestamp=%lld (bco style = %lld)\n",timestamp,((timestamp / (long long)(16)) % 256)); /*16-from par file*/
#endif

	        }

            else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_BCOTIME) /* bco start and stop */
		    {
#ifdef DEBUG4
              printf("[%3d] VSCM  BCOTIME START: %u STOP: %u\n",ii, (datain[ii]>>0) & 0xFF, (datain[ii]>>16) & 0xFF);
#endif
              ii++;
		    }




            else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_FSSREVT) /*FSSR event*/
            {
              a_hfcb_id = ((datain[ii]>>22)&0x1);
              a_chip_id = ((datain[ii]>>19)&0x7);
              a_chan = ((datain[ii]>>12)&0x7F);
              a_bco = ((datain[ii]>>4)&0xFF);
              a_adc1 = (datain[ii]&0x7);

              a_channel = a_chan + (a_chip_id<<7) + (a_hfcb_id<<10);

              a_chan1 = a_chip_id + (a_hfcb_id<<3);
              a_chan2 = a_chan;

#ifdef DEBUG4
	          printf("[%3d] VSCM FSSREVT: HFCBID=%1u CHIPID=%1u CHAN=%3u (%4u)  BCO=%3u ADC=%1u \n",
					 ii,a_hfcb_id,a_chip_id,a_chan,a_channel,a_bco,a_adc1);
#endif



              CCOPEN(0xe111,"c,i,l,N(c,c,c,c)",banknum);
#ifdef DEBUG4
              printf("0x%08x: CCOPEN(VSCM), dataout=0x%08x\n",b08,dataout);
#endif


              if(a_channel != a_channel_old)
              {
                a_channel_old = a_channel;

	            Nchan[0] ++; /* increment channel counter */
#ifdef DEBUG4
                printf("0x%08x: VSCM increment Nchan[0]=%d\n",b08,Nchan[0]);
#endif

				/*
                b16 = (unsigned short *)b08;
                *b16++ = a_channel;
                b08 += 2;
				*/
                *b08++ = a_chan1;
                *b08++ = a_chan2;

                /* remember the place to put the number of pulses 
                Npuls = (unsigned int *)b08;
                Npuls[0] = 0;
                b08 += 4;
				*/
			  }

              /* calculate 'latency' and put it in a data instead of 'bco' (16*8 is bco period (ns)) */
              latency_offset = timestamp % ((long long)VSCM_BCO_FREQ);
              latency = ABS( ((timestamp / (long long)VSCM_BCO_FREQ) % 256) - (long long)(a_bco) );
              latency = latency*((long long)VSCM_BCO_FREQ) + latency_offset; /* in clocks=8ns */
#ifdef DEBUG4
              printf("VSCM>>> bco=%u timestamp=%lld latency=%lld\n",a_bco,timestamp,latency);
              printf("VSCM>>> latency=%d BCO's (%d ns)",latency,latency*8);
#endif

              /*Npuls[0] ++*/;
              *b08++ = a_bco/*latency*/;
              *b08++ = a_adc1;

	          ii++;
            }









            else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_FILLER)
            {
#ifdef DEBUG4
	          printf("[%3d] VSCM FILLER WORD\n",ii);
              printf(">>> VSCM do nothing\n");
#endif
	          ii++;
            }
            else if( ((datain[ii]>>27)&0x1F) == VSCM_TYPE_DNV)
            {
#ifdef DEBUG4
	          printf("[%3d] VSCM DNV\n",ii);
              printf(">>> VSCM do nothing\n");
#endif
	          ii++;
            }
            else
		    {
              printf("VSCM UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
		    }

	      } /* while() */


        } /* loop over blocks */


        if(b08 != NULL) CCCLOSE; /*call CCCLOSE only if CCOPEN was called*/
#ifdef DEBUG4
        printf("0x%08x: CCCLOSE1, dataout=0x%08x\n",b08,dataout);
        printf("0x%08x: CCCLOSE2, dataout=0x%08x\n",b08,(unsigned int *) ( ( ((unsigned int)b08+3)/4 ) * 4));
        printf("-> 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
		    (unsigned int)b08+3,((unsigned int)b08+3),
            ((unsigned int)b08+3) / 4,(((unsigned int)b08+3) / 4)*4, 
		    (unsigned int *)((((unsigned int)b08+3) / 4)*4) );
#endif


	  } /* VSCM hardware format */











      else if(banktag[jj] == 0xe105) /* DCRB hardware format */
	  {

        banknum = iev; /*rol->pid;*/

#ifdef DEBUG8
		printf("\n\nSECOND PASS DCRB\n");
#endif
        a_slot_old = -1;
        for(ibl=0; ibl<nB[jj]; ibl++) /*loop over blocks (actually over dcrb boards)*/
        {
          a_channel_old = -1;
#ifdef DEBUG8
          printf("\n\n\nDCRB: Block %d, Event %2d, event index %2d, event lenght %2d\n",
            ibl,iev,iE[jj][ibl][iev],lenE[jj][ibl][iev]);
#endif

          a_slot = sB[jj][ibl];
          ii = iE[jj][ibl][iev];
          rlen = ii + lenE[jj][ibl][iev];
          while(ii<rlen)
          {
#ifdef DEBUG8
            printf("[%5d] DCRB 0x%08x (rlen=%d)\n",ii,datain[ii],rlen);
#endif
			
            if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_EVTHDR) /*event header*/
			{
              a_triggernumber = (datain[ii]&0x7FFFFFF);
#ifdef DEBUG8
		      printf("[%3d] DCRB EVENT HEADER, a_triggernumber = %d\n",ii,a_triggernumber);
#endif
		      ii++;
			}

            else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_TRGTIME) /*trigger time: remember timestamp*/
            {
              a_trigtime[0] = (datain[ii]&0xFFFFFF);
	          ii++;

              a_trigtime[1] = (datain[ii]&0xFFFFFF);
              ii++;

		      timestamp = (((unsigned long long)a_trigtime[0])<<24) | (a_trigtime[1]);

#ifdef DEBUG8
              printf(">>> DCRB remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
		      printf(">>> DCRB timestamp=%lld\n",timestamp);
#endif

	        }

            else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_DCRBEVT) /*DCRB event*/
            {
              a_channel = ((datain[ii]>>16)&0x7F);
              a_tdc = (datain[ii]&0xFFFF);

#ifdef DEBUG8
	          printf("[%3d] DCRB DCRBEVT: CHAN=%3u TDC=%6u\n",ii,a_channel,a_tdc);
#endif



              CCOPEN(0xe116,"c,i,l,N(c,s)",banknum);
#ifdef DEBUG8
              printf("0x%08x: CCOPEN(DCRB), dataout=0x%08x\n",b08,dataout);
#endif


			  /* ??? 
              if(a_channel != a_channel_old)
              {
                a_channel_old = a_channel;
			  }
			  */

	          Nchan[0] ++; /* increment channel counter */
#ifdef DEBUG8
              printf("0x%08x: DCRB increment Nchan[0]=%d\n",b08,Nchan[0]);
#endif

              *b08++ = a_channel;

              b16 = (unsigned short *)b08;
              *b16++ = a_tdc;
              b08 += 2;

	          ii++;
            }



            else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_FILLER)
            {
#ifdef DEBUG8
	          printf("[%3d] DCRB FILLER WORD\n",ii);
              printf(">>> DCRB do nothing\n");
#endif
	          ii++;
            }
            else if( ((datain[ii]>>27)&0x1F) == DCRB_TYPE_DNV)
            {
#ifdef DEBUG8
	          printf("[%3d] DCRB DNV\n",ii);
              printf(">>> DCRB do nothing\n");
#endif
	          ii++;
            }
            else
		    {
              printf("DCRB UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
		    }

	      } /* while() */


        } /* loop over blocks */


        if(b08 != NULL) CCCLOSE; /*call CCCLOSE only if CCOPEN was called*/
#ifdef DEBUG8
        printf("0x%08x: CCCLOSE1, dataout=0x%08x\n",b08,dataout);
        printf("0x%08x: CCCLOSE2, dataout=0x%08x\n",b08,(unsigned int *) ( ( ((unsigned int)b08+3)/4 ) * 4));
        printf("-> 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
		    (unsigned int)b08+3,((unsigned int)b08+3),
            ((unsigned int)b08+3) / 4,(((unsigned int)b08+3) / 4)*4, 
		    (unsigned int *)((((unsigned int)b08+3) / 4)*4) );
#endif


	  } /* DCRB hardware format */








      else if(banktag[jj] == 0xe123) /* RICH composite format */
	  {

        banknum = iev; /*rol->pid;*/

#ifdef DEBUG9
		printf("\n\nSECOND PASS RICH\n");
#endif
        a_slot_old = -1;
        for(ibl=0; ibl<nB[jj]; ibl++) /*loop over blocks (actually over rich boards)*/
        {
          a_channel_old = -1;
#ifdef DEBUG9
          printf("\n\n\nRICH: Block %d, Event %2d, event index %2d, event lenght %2d\n",
            ibl,iev,iE[jj][ibl][iev],lenE[jj][ibl][iev]);
#endif

          a_slot = sB[jj][ibl];
          ii = iE[jj][ibl][iev];
          rlen = ii + lenE[jj][ibl][iev];
          while(ii<rlen)
          {
#ifdef DEBUG9
            printf("[%5d] RICH 0x%08x (rlen=%d)\n",ii,datain[ii],rlen);
#endif
			
            if( ((datain[ii]>>27)&0x1F) == RICH_TYPE_EVTHDR) /*event header*/
			{
              /*a_slot = ((datain[ii]>>22)&0x1F);*/
              a_triggernumber = (datain[ii]&0x1FFFFF);
#ifdef DEBUG9
		      printf("[%3d] RICH EVENT HEADER, a_triggernumber = %d\n",ii,a_triggernumber);
#endif
		      ii++;
			}

            else if( ((datain[ii]>>27)&0x1F) == RICH_TYPE_TRGTIME) /*trigger time: remember timestamp*/
            {
              a_trigtime[0] = (datain[ii]&0xFFFFFF);
	          ii++;

              a_trigtime[1] = (datain[ii]&0xFFFFFF);
              ii++;

		      timestamp = (((unsigned long long)a_trigtime[0])<<24) | (a_trigtime[1]);

#ifdef DEBUG9
              printf(">>> RICH remember timestamp 0x%06x 0x%06x\n",a_trigtime[0],a_trigtime[1]);
		      printf(">>> RICH timestamp=%lld\n",timestamp);
#endif

	        }

            else if( ((datain[ii]>>27)&0x1F) == RICH_TYPE_FIBER) /*fiber number*/
            {
              a_fiber = ((datain[ii]>>22)&0x1F);
		      /*a_triggernumber = (datain[ii]&0x1FFFFF);*/
#ifdef DEBUG9
	          printf("[%3d] FIBER %d\n",ii,a_fiber);
#endif
	          ii++;
		    }

            else if( ((datain[ii]>>27)&0x1F) == RICH_TYPE_TDC) /*tdc*/
            {
              a_edge = ((datain[ii]>>26)&0x1);
              a_chan = ((datain[ii]>>16)&0xFF);
		      a_time = (datain[ii]&0x7FFF);
#ifdef DEBUG9
	          printf("[%3d] EDGE %d, CHAN %d, TIME %d\n",ii,a_edge,a_chan,a_time);
#endif


              CCOPEN(0xe124,"c,i,l,N(c,c,s)",banknum);
#ifdef DEBUG9
              printf("0x%08x: CCOPEN(RICH), dataout=0x%08x\n",b08,dataout);
#endif


	          Nchan[0] ++; /* increment channel counter */
#ifdef DEBUG9
              printf("0x%08x: RICH increment Nchan[0]=%d\n",b08,Nchan[0]);
#endif

              *b08++ = a_fiber;

              *b08++ = a_chan;

              b16 = (unsigned short *)b08;
              *b16++ = ((a_edge<<15)&0x8000) | (a_time&0x7FFF);
              b08 += 2;


	          ii++;
		    }

            else if( ((datain[ii]>>27)&0x1F) == RICH_TYPE_FILLER)
            {
#ifdef DEBUG9
	          printf("[%3d] RICH FILLER WORD\n",ii);
              printf(">>> RICH do nothing\n");
#endif
	          ii++;
            }
            else if( ((datain[ii]>>27)&0x1F) == RICH_TYPE_DNV)
            {
#ifdef DEBUG9
	          printf("[%3d] RICH DNV\n",ii);
              printf(">>> RICH do nothing\n");
#endif
	          ii++;
            }
            else
		    {
              printf("RICH UNKNOWN data: [%3d] 0x%08x\n",ii,datain[ii]);
              ii++;
		    }

	      } /* while() */


        } /* loop over blocks */


        if(b08 != NULL) CCCLOSE; /*call CCCLOSE only if CCOPEN was called*/
#ifdef DEBUG9
        printf("0x%08x: CCCLOSE1, dataout=0x%08x\n",b08,dataout);
        printf("0x%08x: CCCLOSE2, dataout=0x%08x\n",b08,(unsigned int *) ( ( ((unsigned int)b08+3)/4 ) * 4));
        printf("-> 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
		    (unsigned int)b08+3,((unsigned int)b08+3),
            ((unsigned int)b08+3) / 4,(((unsigned int)b08+3) / 4)*4, 
		    (unsigned int *)((((unsigned int)b08+3) / 4)*4) );
#endif


	  } /* RICH composite format */














    } /* loop over banks  */



#ifdef PASS_AS_IS
    /* if last event, loop over banks to be passed 'as is' and attach them to that last event */
    if(iev==(nnE-1))
	{
      for(ii=0; ii<nASIS; ii++)
      {
        jj = iASIS[ii]; /* bank number as it reported by BANKSCAN */
        datain = bankdata[jj];
        lenin = banknw[jj];
        /*printf("mynev=%d: coping bank number %d (header %d 0x%08x))\n",mynev,jj,*(datain-2),*(datain-1));*/
	    
        CPOPEN(banktag[jj],banktyp[jj],banknr[jj]);
        for(kk=0; kk<lenin; kk++)
        {
          dataout[kk] = datain[kk];
          b08 += 4;
        }        
        CPCLOSE;
	  }
    }
#endif


#ifdef SPLIT_BLOCKS

	/*
      at that point we want to close previout bank-of-banks and create header for new one;
      if iev==0, header already exist (came from ROL1); if iev==(nnE-1), do not create next header
    */

    header[0] = lenev - 1;

    /* rol->dabufpi[1] comes from ROL1 and contains information for block of events; we'll replace it
	   with correct info for every event using data from TI */
    header[1] = rol->dabufpi[1];

    /* header created by CEOPEN macros (see rol.h) */

    /* event type obtained from TI board have to be recorded into fragment header - event builder need it */
/*TEMP while(a_event_type>15) a_event_type --; TEMP*/
    header[1] = (header[1]&0xFF00FFFF) + (a_event_type<<16);


    /* time stamp obtained from TI board have to be recorded into fragment header - event builder need it */
    header[1] = (header[1]&0xFFFF00FF) + ((a_timestamp_l&0xFF)<<8);


    /* event number obtained from TI board have to be recorded into fragment header - event builder need it */
    header[1] = (header[1]&0xFFFFFF00) + (a_event_number_l&0xFF);

	sync_flag =  (header[1]>>24)&0xFF;

    /* if NOT the last event, clear syncflag; should do it only for blocks with sync event in the end,
    but do not bother checking, will do it for all blocks */
    if(iev<(nnE-1))
	{
      header[1] = header[1]&0x00FFFFFF;
	}

/*
	printf("HEADER: sync_flag=%d event_type=%d bank_type=%d event_number=%d\n",
      (header[1]>>24)&0xFF,(header[1]>>16)&0xFF,(header[1]>>8)&0xFF,header[1]&0xFF);
*/

  /*printf("(%d) header[0]=0x%08x (0x%08x)\n",iev,header[0],lenev - 1);*/
  /*printf("(%d) header[1]=0x%08x (0x%08x)\n",iev,header[1],rol->dabufpi[1]);*/

    /* if NOT the last event, create header for the next event */
    if(iev<(nnE-1))
	{
  /*printf("bump header pointer, iev=%d\n",iev);*/
      header = dataout;
      dataout += 2;
      lenout += 2;
	}
    else /* for last event in block */
	{
      /* for sync event, send scalers */
      if(sync_flag && havebits)
	  {
        printf("\nROL2: SYNC EVENT !!!\n");
        for(i=0; i<NTRIGBITS; i++) printf("trigbit[%d] = %7d\n",i,bitscalers[i]);
        printf("\n");
#ifdef SSIPC
        printf("\nROL2: sending message\n");
        status = epics_msg_send("hallb_trig_event_bits","uint",NTRIGBITS,bitscalers);
        printf("\nROL2: message send\n");
#endif
      }
	}

#endif



  } /* loop over events */





  /* returns full fragment length (long words) */  

#ifdef DEBUG 
  printf("return lenout=%d\n**********************\n\n",lenout);
#endif



/*
if(lenout>2) printf("return lenout=%d\n**********************\n\n",lenout);
*/

  rol->user_storage[0] = lenout;

#ifdef SPLIT_BLOCKS
  rol->user_storage[1] = nnE; /* report the number of events */
#else
  CPEXIT;
  rol->user_storage[1] = 1; /* report the number of events */
#endif

  /*printf("return lenout=%d\n**********************\n\n",lenout);*/

/*printf("--> %d %d\n",rol->user_storage[0],rol->user_storage[1]);*/


  /* print output buffer
  dataout = (unsigned int *)(rol->dabufp);
  printf("lenout=%d\n",lenout);
  for(ii=0; ii<lenout; ii++) printf("  DATA [%3d] 0x%08x (%d)\n",ii,dataout[ii],dataout[ii]);
  */

  return;
}

void
rol2trig_done()
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

/*hps12 during crash:
top - 17:36:34 up 5 days,  3:47,  1 user,  load average: 3.48, 1.20, 0
Tasks: 185 total,   3 running, 182 sleeping,   0 stopped,   0 zombie
Cpu(s):  3.3%us,  1.1%sy,  0.0%ni, 95.5%id,  0.0%wa,  0.0%hi,  0.0%si,
Mem:   8221276k total,   363480k used,  7857796k free,        0k buffe
Swap:        0k total,        0k used,        0k free,   182572k cache

  PID USER      PR  NI  VIRT  RES  SHR S %CPU %MEM    TIME+  COMMAND  
 8497 clasrun   19   0  5700 1428  516 R 100.0  0.0   0:04.84 csh     
 8496 clasrun   18   0  5708 1440  516 S 98.1  0.0   0:03.24 csh      
 8584 clasrun   18   0  2448 1000  732 R 80.2  0.0   0:05.15 top      
 3597 clasrun   18   0 1826m 2232  764 S 62.3  0.0   1:14.17 DiagGuiSe
 3211 root      17   0 12848 1300  560 S 25.4  0.0   0:31.02 pcscd    
*/

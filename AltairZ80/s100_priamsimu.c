/*  s00_priamsimu.c: Priam Smart Disk Controller (disk image)
  
    
    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    PETER SCHORN BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
    IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    Except as contained in this notice, the name of Patrick Linstruth shall not
    be used in advertising or otherwise to promote the sale, use or other dealings
    in this Software without prior written authorization from Patrick Linstruth.
*/

#include "altairz80_defs.h"
#include "sim_imd.h"

#ifdef DBG_MSG
#define DBG_PRINT(args) sim_printf args
#else
#define DBG_PRINT(args)
#endif

extern uint32 PCX;
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);


#define PRIAM_MAX_ADAPTERS       1
#define PRIAM_MAX_DRIVES         1
#define PRIAM_SECTOR_SIZE        512
#define PRIAM_SPT               22
#define PRIAM_HEADS             4
#define PRIAM_CYLINDERS         185
#define PRIAM_TRACKS            (PRIAM_HEADS * PRIAM_CYLINDERS) 
#define PRIAM_CAPACITY           (PRIAM_TRACKS*PRIAM_SPT)*PRIAM_SECTOR_SIZE      /* Default Jade Disk Capacity */

#define PRIAM_IO_BASE          0xF0
#define PRIAM_IO_SIZE          8

#define PRIAM_BUFFER_SIZE          1024

#define PRIAM_REGNO_FROMADDRESS(x) (x - PRIAM_IO_BASE)

#define PRIAMREGIN_PAR0 2
#define PRIAMREGIN_PAR1 3
#define PRIAMREGIN_PAR2 4
#define PRIAMREGIN_PAR3 5
#define PRIAMREGIN_PAR4 6
#define PRIAMREGIN_PAR5 7

#define PRIAMREGIN_CMD 0

#define PRIAMREGIN_DATA 1

#define PRIAMREGIN_DRIVENO PRIAMREGIN_PAR0
#define PRIAMREGIN_HEAD_CYLUPPER PRIAMREGIN_PAR1
#define PRIAMREGIN_CYLLOWER PRIAMREGIN_PAR2
#define PRIAMREGIN_SECTOR PRIAMREGIN_PAR3
#define PRIAMREGIN_NUMSECTORS PRIAMREGIN_PAR4

#define PRIAMREGOUT_RES0 2
#define PRIAMREGOUT_RES1 3
#define PRIAMREGOUT_RES2 4
#define PRIAMREGOUT_RES3 5
#define PRIAMREGOUT_RES4 6
#define PRIAMREGOUT_RES5 7

#define PRIAMREGOUT_STAT 0
#define PRIAMREGOUT_DATA 1

#define PRIAMREGOUT_TRANSTATUS PRIAMREGOUT_RES0
#define PRIAMREGOUT_HEAD_CYLUPPER PRIAMREGOUT_RES1
#define PRIAMREGOUT_CYLLOWER PRIAMREGOUT_RES2
#define PRIAMREGOUT_SECTOR PRIAMREGOUT_RES3
#define PRIAMREGOUT_NUMSECTORS PRIAMREGOUT_RES4

#define PRIAMSTATUS_CMDREJECT (1 << 7)
#define PRIAMSTATUS_COMPREQ (1 << 6)
#define PRIAMSTATUS_BUSY (1 << 3)
#define PRIAMSTATUS_DATAXFERREQ (1 << 2)
#define PRIAMSTATUS_ISREADREQ (1 << 1)
#define PRIAMSTATUS_DBUSENABLE (1 << 0)


#define PRIAMTRANSTATE_DRIVE(x) ((x & 3) << 6)
#define PRIAMTRANSTATE_COMPTYPE(x) ((x & 3) << 4)
#define PRIAMTRANSTATE_COMPCODE(x) ((x & 7) << 0)

typedef struct {
	uint32 readptr;
	uint32 writeptr;
    uint8 hasData;
	uint8 data[PRIAM_BUFFER_SIZE];
} PRIAM_INTERNALBUFFER;


typedef struct {
	uint32 io_base;     /* I/O Base Address                 */
	uint32 io_size;     /* I/O Address Space requirement    */
    uint8 curdrv;
    PRIAM_INTERNALBUFFER *buffer;
    uint8 inregs[PRIAM_IO_SIZE];
    uint8 outregs[PRIAM_IO_SIZE];
	UNIT* uptr[PRIAM_MAX_DRIVES];
} PRIAM_INFO;


static PRIAM_INTERNALBUFFER PriamBuffer;

static PRIAM_INFO priam_info_data = { PRIAM_IO_BASE, PRIAM_IO_SIZE,0, 0, {0}, {0}, 0 };

static PRIAM_INFO *priam_info = &priam_info_data;


/* Local function prototypes */
static t_stat priam_reset(DEVICE *priam_dev);
static t_stat priam_svc(UNIT *uptr);
static t_stat priam_attach(UNIT *uptr, CONST char *cptr);
static t_stat priam_detach(UNIT *uptr);

static void PriamCmdHandle();
static void PriamReadDo();
static void PriamWriteStart();
static void PriamWriteDo();

static uint8 PRIAM_In(uint32 Addr);
static uint8 PRIAM_Out(uint32 Addr, int32 data);
static const char* priam_description(DEVICE *dptr);

static int32 priamdev(int32 Addr, int32 rw, int32 data);

static UNIT priam_unit[PRIAM_MAX_DRIVES] = {
    { UDATA (priam_svc, UNIT_FIX + UNIT_ATTABLE + UNIT_DISABLE + UNIT_ROABLE, PRIAM_CAPACITY), 10000 }
   
};

static REG priam_reg[] = {
    { NULL }
};

#define PRIAM_NAME  "Priam Smart Controller (simulated disk)"
#define PRIAM_SNAME "PRIAMSIMU"

static const char* priam_description(DEVICE *dptr) {
    return PRIAM_NAME;
}

/*
** These definitions should probably be in s100_jade.h
** so they can be included in other modules.
*/
#define UNIT_V_PRIAM_VERBOSE      (UNIT_V_UF + 0)                      /* VERBOSE / QUIET */
#define UNIT_PRIAM_VERBOSE        (1 << UNIT_V_PRIAM_VERBOSE)
#define UNIT_V_PRIAM_WPROTECT     (UNIT_V_UF + 1)                      /* WRTENB / WRTPROT */
#define UNIT_PRIAM_WPROTECT       (1 << UNIT_V_PRIAM_WPROTECT)

/*
** These definitions should probably be in altairz80_sio.h
** so they can be included in other modules, like this one.
*/
#define UNIT_V_SIO_SLEEP    (UNIT_V_UF + 7)     /* sleep after keyboard status check            */
#define UNIT_SIO_SLEEP      (1 << UNIT_V_SIO_SLEEP)

static MTAB priam_mod[] = {
    { UNIT_PRIAM_VERBOSE,   0,                    "QUIET",    "QUIET",
        NULL, NULL, NULL, "No verbose messages for unit " PRIAM_NAME "n"                 },
    { UNIT_PRIAM_VERBOSE,   UNIT_PRIAM_VERBOSE, "VERBOSE",  "VERBOSE",
        NULL, NULL, NULL, "Verbose messages for unit " PRIAM_NAME "n"                    },
    { UNIT_PRIAM_WPROTECT,  0,                      "WRTENB",    "WRTENB",  NULL, NULL, NULL,
        "Enables " PRIAM_NAME "n for writing"                 },
    { UNIT_PRIAM_WPROTECT,  UNIT_PRIAM_WPROTECT,  "WRTPROT",    "WRTPROT",  NULL, NULL, NULL,
        "Protects " PRIAM_NAME "n from writing"                },
    { 0 }
};

/* Debug flags */
#define ERROR_MSG           (1 << 0)
#define SEEK_MSG            (1 << 1)
#define CMD_MSG             (1 << 2)
#define RD_DATA_MSG         (1 << 3)
#define WR_DATA_MSG         (1 << 4)
#define STATUS_MSG          (1 << 5)
#define RD_DATA_DETAIL_MSG  (1 << 6)
#define WR_DATA_DETAIL_MSG  (1 << 7)

/* Debug Flags */
static DEBTAB priam_dt[] = {
    { "ERROR",      ERROR_MSG,          "Error messages"        },
    { "SEEK",       SEEK_MSG,           "Seek messages"         },
    { "CMD",        CMD_MSG,            "Command messages"      },
    { "READ",       RD_DATA_MSG,        "Read messages"         },
    { "WRITE",      WR_DATA_MSG,        "Write messages"        },
    { "STATUS",     STATUS_MSG,         "Status messages"       },
    { "RDDETAIL",   RD_DATA_DETAIL_MSG, "Read detail messages"  },
    { "WRDETAIL",   WR_DATA_DETAIL_MSG, "Write detail messags"  },
    { NULL,         0                                           }
};

DEVICE priam_dev = {
    PRIAM_SNAME,                           /* name */
    priam_unit,                            /* unit */
    priam_reg,                             /* registers */
    priam_mod,                             /* modifiers */
    PRIAM_MAX_DRIVES,                      /* # units */
    10,                                   /* address radix */
    31,                                   /* address width */
    1,                                    /* addr increment */
    PRIAM_MAX_DRIVES,                      /* data radix */
    PRIAM_MAX_DRIVES,                      /* data width */
    NULL,                                 /* examine routine */
    NULL,                                 /* deposit routine */
    &priam_reset,                          /* reset routine */
    NULL,                           /* boot routine */
    &priam_attach,                         /* attach routine */
    &priam_detach,                         /* detach routine */
    &priam_info_data,                      /* context */
    (DEV_DISABLE | DEV_DIS | DEV_DEBUG),  /* flags */
    ERROR_MSG,                            /* debug control */
    priam_dt,                              /* debug flags */
    NULL,                                 /* mem size routine */
    NULL,                                 /* logical name */
    NULL,                                 /* help */
    NULL,                                 /* attach help */
    NULL,                                 /* context for help */
    &priam_description                     /* description */
};

/* Reset routine */
t_stat priam_reset(DEVICE *dptr)
{
    uint8 i;
    PRIAM_INFO *pInfo = (PRIAM_INFO *)dptr->ctxt;

    if(dptr->flags & DEV_DIS) { /* Disconnect I/O Ports */
        sim_map_resource(pInfo->io_base, pInfo->io_size, RESOURCE_TYPE_IO, &priamdev, "priamdev", TRUE);
    } else {
        
        /* Connect I/O Ports at base address */
        if(sim_map_resource(pInfo->io_base, pInfo->io_size, RESOURCE_TYPE_IO, &priamdev, "priamdev", FALSE) != 0) {
            sim_debug(ERROR_MSG, &priam_dev, PRIAM_SNAME ": Error mapping I/O resource at 0x%02x\n", pInfo->io_base);
            return SCPE_ARG;
        }
    }

    pInfo->curdrv = 0;

    /* Reset Registers and Interface Controls */
    for (i=0; i < PRIAM_MAX_DRIVES; i++) {
        if (priam_info->uptr[i] == NULL) {
            priam_info->uptr[i] = &priam_dev.units[i];
        }
    }

    for (int i = 0; i < PRIAM_IO_SIZE; i++)
        pInfo->inregs[i] = 0;

    //After reset, bus is enabled and have completion request
    pInfo->outregs[PRIAMREGOUT_STAT] = PRIAMSTATUS_COMPREQ | PRIAMSTATUS_DBUSENABLE;
    pInfo->outregs[PRIAMREGOUT_DATA] = 0;
    pInfo->outregs[PRIAMREGOUT_TRANSTATUS] = PRIAMTRANSTATE_DRIVE(0) | PRIAMTRANSTATE_COMPTYPE(1) | PRIAMTRANSTATE_COMPCODE(6);
    pInfo->outregs[PRIAMREGOUT_RES1] = 0xAA;
    pInfo->outregs[PRIAMREGOUT_RES2] = 0x55;
    pInfo->outregs[PRIAMREGOUT_RES3] = 0xF0;
    pInfo->outregs[PRIAMREGOUT_RES4] = 0x0F;
    pInfo->outregs[PRIAMREGOUT_RES5] = 0x00;

    PriamBuffer.readptr = 0;
    PriamBuffer.writeptr = 0;
    PriamBuffer.hasData = 0;
    pInfo->buffer = &PriamBuffer;

    sim_debug(STATUS_MSG, &priam_dev, PRIAM_SNAME ": reset controller.\n");

    return SCPE_OK;
}

/* Attach routine */
t_stat priam_attach(UNIT *uptr, CONST char *cptr)
{
    char header[4];
    t_stat r;
    unsigned int i = 0;

    r = attach_unit(uptr, cptr);    /* attach unit  */

    if(r != SCPE_OK) {              /* error?       */
        sim_debug(ERROR_MSG, &priam_dev, PRIAM_SNAME ": ATTACH error=%d\n", r);
        return r;
    }

    /* Determine length of this disk */
    if(sim_fsize(uptr->fileref) != 0) {
        uptr->capac = sim_fsize(uptr->fileref);
    } else {
        uptr->capac = PRIAM_CAPACITY;
    }

    DBG_PRINT(("JADE: ATTACH uptr->capac=%d\n", uptr->capac));

    for (i = 0; i < PRIAM_MAX_DRIVES; i++) {
        if(priam_dev.units[i].fileref == uptr->fileref) {
            break;
        }
    }

    if (i >= PRIAM_MAX_DRIVES) {
        return SCPE_ARG;
    }

    /* Default for new file is DSK */
    uptr->u3 = IMAGE_TYPE_DSK;

    if(uptr->capac > 0) {
        char *rtn = fgets(header, 4, uptr->fileref);
        if((rtn != NULL) && (strncmp(header, "CPT", 3) == 0)) {
            sim_printf(PRIAM_SNAME ": CPT images not yet supported\n");
            uptr->u3 = IMAGE_TYPE_CPT;
            priam_detach(uptr);
            return SCPE_OPENERR;
        } else {
            uptr->u3 = IMAGE_TYPE_DSK;
        }
    }

    if (uptr->flags & UNIT_PRIAM_VERBOSE) {
        sim_printf(PRIAM_SNAME "%d: attached to '%s', type=%s, len=%d\n", i, cptr,
            uptr->u3 == IMAGE_TYPE_CPT ? "CPT" : "DSK",
            uptr->capac);
    }

    return SCPE_OK;
}


/* Detach routine */
t_stat priam_detach(UNIT *uptr)
{
    t_stat r;
    int8 i;

    for (i = 0; i < PRIAM_MAX_DRIVES; i++) {
        if(priam_dev.units[i].fileref == uptr->fileref) {
            break;
        }
    }

    if (i >= PRIAM_MAX_DRIVES) {
        return SCPE_ARG;
    }

    DBG_PRINT(("Detach PRIAM%d\n", i));

    r = detach_unit(uptr);  /* detach unit */

    if (r != SCPE_OK) {
        return r;
    }

    priam_dev.units[i].fileref = NULL;

    if (uptr->flags & UNIT_PRIAM_VERBOSE) {
        sim_printf(PRIAM_SNAME "%d: detached.\n", i);
    }

    return SCPE_OK;
}





static t_stat priam_svc(UNIT *uptr)
{
    return SCPE_OK;
}


static int32 priamdev(int32 Addr, int32 rw, int32 data)
{
    if (rw == 0) { /* Read */
        return(PRIAM_In(Addr));
    } else {       /* Write */
        return(PRIAM_Out(Addr, data));
    }
}


//IN instruction, our OUTPUT
static uint8 PRIAM_In(uint32 Addr)
{
    
    
    uint8 regno = PRIAM_REGNO_FROMADDRESS(Addr);

    if (regno == PRIAMREGIN_DATA)
    {
        //read data from buffer
        if (!priam_info->buffer->hasData)
        {
            sim_printf("PRIAM data read: no data in buffer!\n");
            return 0;
        }

        if (!(priam_info->outregs[PRIAMREGOUT_STAT] & PRIAMSTATUS_DATAXFERREQ) || !(priam_info->outregs[PRIAMREGOUT_STAT] & PRIAMSTATUS_ISREADREQ))
        {
            sim_printf("PRIAM data read: no read request pending!\n");
            return 0;
        }

        uint8 res = priam_info->buffer->data[priam_info->buffer->readptr];
        priam_info->buffer->readptr = (priam_info->buffer->readptr + 1) % PRIAM_BUFFER_SIZE;

        if (priam_info->buffer->readptr == priam_info->buffer->writeptr)
        {
            //Read is now complete, signal transaction end
            //We do not handle transactions of more than 1024
            priam_info->buffer->hasData = 0;
            priam_info->outregs[PRIAMREGOUT_STAT] = PRIAMSTATUS_COMPREQ | PRIAMSTATUS_DBUSENABLE;
            priam_info->outregs[PRIAMREGOUT_TRANSTATUS] = PRIAMTRANSTATE_DRIVE(0) | PRIAMTRANSTATE_COMPTYPE(0) | PRIAMTRANSTATE_COMPCODE(0);
        }
        //sim_printf("PRIAM READ return 0x%02X\n", res);
        return res;

    }

    //sim_printf("PRIAM IN address 0x%02X: return 0x%02X\n", Addr, priam_info->outregs[regno]);

    
    uint8 res = priam_info->outregs[regno];
    
    return res;
}

// OUT instruction, our INPUT
static uint8 PRIAM_Out(uint32 Addr, int32 Data)
{
    //sim_printf("PRIAM OUT address 0x%02X data  0x%02X\n", Addr, Data);
    uint8 regno = PRIAM_REGNO_FROMADDRESS(Addr);
    priam_info->inregs[regno] = Data;
    
	if (regno == PRIAMREGOUT_DATA)
	{
		//Write data to buffer
		if (!(priam_info->outregs[PRIAMREGOUT_STAT] & PRIAMSTATUS_DATAXFERREQ) || (priam_info->outregs[PRIAMREGOUT_STAT] & PRIAMSTATUS_ISREADREQ))
		{
			sim_printf("PRIAM data read: no write request pending!\n");
			return 0;
		}

        PriamWriteDo();

	}
    
    if (regno == PRIAMREGIN_CMD)
    {
        PriamCmdHandle();
    }

    return(Data);
}

#define PRIAMCMD_COMPACK 0
#define PRIAMCMD_READNORETRY 0x43
#define PRIAMCMD_READWITHRETRY 0x53
#define PRIAMCMD_WRITENORETRY 0x42
#define PRIAMCMD_WRITEWITHRETRY 0x52


uint8 PriamHeadFromParams(PRIAM_INFO* info)
{
    return ((info->inregs[PRIAMREGIN_HEAD_CYLUPPER] >> 4) & 7);
}

uint16 PriamCylFromParams(PRIAM_INFO* info)
{
	return ((info->inregs[PRIAMREGIN_HEAD_CYLUPPER] & 0xF) | (info->inregs[PRIAMREGIN_CYLLOWER]));
}

uint8 PriamSectorFromParams(PRIAM_INFO* info)
{
	return (info->inregs[PRIAMREGIN_SECTOR] & 0x7F);
}

uint8 PriamTotalSectorsFromParams(PRIAM_INFO* info)
{
	return (info->inregs[PRIAMREGIN_NUMSECTORS] & 0x7F);
}


static void PriamCmdHandle()
{
    uint8 cmd = priam_info->inregs[PRIAMREGIN_CMD];
    switch (cmd)
    {
    case PRIAMCMD_COMPACK:
        //sim_printf("PRIAM Completion Acknowledge received\n");
        priam_info->outregs[PRIAMREGOUT_STAT] = PRIAMSTATUS_DBUSENABLE;
        break;
    case PRIAMCMD_READNORETRY:
    case PRIAMCMD_READWITHRETRY:
        PriamReadDo();
        break;
    case PRIAMCMD_WRITENORETRY:
    case PRIAMCMD_WRITEWITHRETRY:
        PriamWriteStart();
        break;
    default:
        sim_printf("PRIAM Unknown command 0x%02X\n", cmd);
        priam_info->outregs[PRIAMREGOUT_STAT] = PRIAMSTATUS_CMDREJECT | PRIAMSTATUS_DBUSENABLE;
    }
}

static void PriamWriteDo()
{
    uint8 Data = priam_info->inregs[PRIAMREGIN_DATA];
	priam_info->buffer->data[priam_info->buffer->writeptr] = Data;
	priam_info->buffer->writeptr = (priam_info->buffer->writeptr + 1) % PRIAM_BUFFER_SIZE;

	uint32 numinbuf = priam_info->buffer->writeptr - priam_info->buffer->readptr;
	if (!numinbuf)
		numinbuf = PRIAM_BUFFER_SIZE;

	uint8 head = PriamHeadFromParams(priam_info);
	uint16 cyl = PriamCylFromParams(priam_info);
	uint8 sector = PriamSectorFromParams(priam_info);
	uint8 numsector = PriamTotalSectorsFromParams(priam_info);

	if (numsector > 2)
	{
		sim_printf("PRIAM too many sectors write requested\n");
		return;
	}

	if (numinbuf == PRIAM_SECTOR_SIZE * numsector)
	{
		//Have all data for write now			
		

		//sim_printf("PRIAM Write %d sectors from head %d cylinder %d sector %d\n", numsector, head, cyl, sector);

	    uint32 offset = ((cyl * PRIAM_HEADS * PRIAM_SPT) + (head * PRIAM_SPT) + sector) * PRIAM_SECTOR_SIZE;

		//sim_printf("PRIAM seek to %ld\n", offset);

		if (sim_fseek(priam_info->uptr[0]->fileref, offset, SEEK_SET) != 0) {
			sim_printf("PRIAM seek error\n");
			return;
		}

		while (priam_info->inregs[PRIAMREGIN_NUMSECTORS] & 0x7F)
		{
			if (sim_fwrite(&priam_info->buffer->data[priam_info->buffer->readptr], 1, PRIAM_SECTOR_SIZE, priam_info->uptr[0]->fileref) != PRIAM_SECTOR_SIZE) {
				sim_printf("PRIAM write error\n");
				return;
			}
			priam_info->buffer->readptr = (priam_info->buffer->readptr + PRIAM_SECTOR_SIZE) % PRIAM_BUFFER_SIZE;
			priam_info->inregs[PRIAMREGIN_NUMSECTORS]--;
		}

        //Complete, signal transaction end
		priam_info->buffer->hasData = 0;
		priam_info->outregs[PRIAMREGOUT_STAT] = PRIAMSTATUS_COMPREQ | PRIAMSTATUS_DBUSENABLE;
		priam_info->outregs[PRIAMREGOUT_TRANSTATUS] = PRIAMTRANSTATE_DRIVE(0) | PRIAMTRANSTATE_COMPTYPE(0) | PRIAMTRANSTATE_COMPCODE(0);
		
	}

}

static void PriamWriteStart()
{
    priam_info->buffer->readptr = 0;
    priam_info->buffer->writeptr = 0;
    priam_info->buffer->hasData = 0;

    priam_info->outregs[PRIAMREGOUT_STAT] = PRIAMSTATUS_DATAXFERREQ | PRIAMSTATUS_DBUSENABLE;

}

static void PriamReadDo()
{

	if (priam_info->uptr[0]->fileref == NULL) {
		sim_printf(PRIAM_SNAME "Drive Not Ready\n");
        return;
	}

    uint8 head = PriamHeadFromParams(priam_info);
    uint16 cyl = PriamCylFromParams(priam_info);
    uint8 sector = PriamSectorFromParams(priam_info);
    uint8 numsector = PriamTotalSectorsFromParams(priam_info);

    //sim_printf("PRIAM Read %d sectors from head %d cylinder %d sector %d\n", numsector, head, cyl, sector);

    if (numsector > 2)
    {
        sim_printf("PRIAM too many sectors requested\n");
        return;
    }
	
    uint32 offset = ((cyl * PRIAM_HEADS * PRIAM_SPT) + (head * PRIAM_SPT) + sector) * PRIAM_SECTOR_SIZE;

    //sim_printf("PRIAM seek to %ld\n", offset);

	if (sim_fseek(priam_info->uptr[0]->fileref, offset, SEEK_SET) != 0) {
        sim_printf("PRIAM seek error\n");
		return;
	}

	priam_info->buffer->readptr = 0;
	priam_info->buffer->writeptr = 0;
	priam_info->buffer->hasData = 0;

    while (priam_info->inregs[PRIAMREGIN_NUMSECTORS] & 0x7F)
    {
        if (sim_fread(&priam_info->buffer->data[priam_info->buffer->writeptr], 1, PRIAM_SECTOR_SIZE, priam_info->uptr[0]->fileref) != PRIAM_SECTOR_SIZE) {
            sim_printf("PRIAM read error\n");
            return;
        }
        priam_info->buffer->writeptr = (priam_info->buffer->writeptr + PRIAM_SECTOR_SIZE) % PRIAM_BUFFER_SIZE;
        priam_info->inregs[PRIAMREGIN_NUMSECTORS]--;
    }

    
    priam_info->buffer->hasData = 1;
    
    priam_info->outregs[PRIAMREGOUT_STAT] = PRIAMSTATUS_DATAXFERREQ | PRIAMSTATUS_DBUSENABLE | PRIAMSTATUS_ISREADREQ;

	
}
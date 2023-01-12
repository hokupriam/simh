/*  s00_priamserial.c: Priam Smart Disk Controller (physical disk over serial port)
  
    
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
#include "sim_serial.h"
#include "sim_timer.h"

#ifdef DBG_MSG
#define DBG_PRINT(args) sim_printf args
#else
#define DBG_PRINT(args)
#endif

extern uint32 PCX;
extern uint32 sim_map_resource(uint32 baseaddr, uint32 size, uint32 resource_type,
                               int32 (*routine)(const int32, const int32, const int32), const char* name, uint8 unmap);



#define PRIAMSERIAL_IO_BASE          0xF0
#define PRIAMSERIAL_IO_SIZE          8

#define MAXPORTNAME 256

#define READMAXTRIES 0


typedef struct {
	uint32 io_base;     /* I/O Base Address                 */
	uint32 io_size;     /* I/O Address Space requirement    */
    SERHANDLE serPort;	
    char serPortName[MAXPORTNAME];
} PRIAMSERIAL_INFO;


static REG priamserial_reg[] = {
	{ NULL }
};



static PRIAMSERIAL_INFO priamserial_info_data = { PRIAMSERIAL_IO_BASE, PRIAMSERIAL_IO_SIZE,INVALID_HANDLE, ""};

static PRIAMSERIAL_INFO *priamserial_info = &priamserial_info_data;


/* Local function prototypes */
static t_stat priamserial_reset(DEVICE *priam_dev);
static t_stat priamserial_svc(UNIT *uptr);


static uint8 PRIAMSerial_In(uint32 Addr);
static uint8 PRIAMSerial_Out(uint32 Addr, int32 data);
static const char* priamserial_description(DEVICE *dptr);

static int32 priamserialdev(int32 Addr, int32 rw, int32 data);

static t_stat Priamserial_set_port(UNIT* uptr, int32 val, CONST char* cptr, void* desc);
t_stat Priamserial_show_port(FILE* st, UNIT* uptr, int32 val, CONST void* desc);

static int32 Priamserial_Readserial(uint8* result, int count, int maxtries);


#define PRIAMSERIAL_NAME  "Priam Smart Controller over serial port"
#define PRIAMSERIAL_SNAME "PRIAMSERIAL"

static const char* priamserial_description(DEVICE *dptr) {
    return PRIAMSERIAL_NAME;
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

static MTAB priamserial_mod[] = {
    { UNIT_PRIAM_VERBOSE,   0,                    "QUIET",    "QUIET",
        NULL, NULL, NULL, "No verbose messages for unit " PRIAMSERIAL_NAME "n"                 },
    { UNIT_PRIAM_VERBOSE,   UNIT_PRIAM_VERBOSE, "VERBOSE",  "VERBOSE",
        NULL, NULL, NULL, "Verbose messages for unit " PRIAMSERIAL_NAME "n"                    },
	{ MTAB_XTD | MTAB_VDV,    0,                      "PORT",  "PORT",
		&Priamserial_set_port, & Priamserial_show_port, NULL, "Sets Priam serial port"   },

    
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
static DEBTAB priamserial_dt[] = {
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

DEVICE priamserial_dev = {
    PRIAMSERIAL_SNAME,                           /* name */
    NULL,                            /* unit */
    priamserial_reg,                             /* registers */
    priamserial_mod,                             /* modifiers */
    0,                      /* # units */
    10,                                   /* address radix */
    31,                                   /* address width */
    1,                                    /* addr increment */
    10,                      /* data radix */
    8,                      /* data width */
    NULL,                                 /* examine routine */
    NULL,                                 /* deposit routine */
    &priamserial_reset,                          /* reset routine */
    NULL,                           /* boot routine */
    NULL,                         /* attach routine */
    NULL,                         /* detach routine */
    &priamserial_info_data,                      /* context */
    (DEV_DISABLE | DEV_DIS | DEV_DEBUG),  /* flags */
    ERROR_MSG,                            /* debug control */
    priamserial_dt,                              /* debug flags */
    NULL,                                 /* mem size routine */
    NULL,                                 /* logical name */
    NULL,                                 /* help */
    NULL,                                 /* attach help */
    NULL,                                 /* context for help */
    &priamserial_description                     /* description */
};

/* Reset routine */
t_stat priamserial_reset(DEVICE *dptr)
{
    
    PRIAMSERIAL_INFO *pInfo = (PRIAMSERIAL_INFO *)dptr->ctxt;

    if(dptr->flags & DEV_DIS) { /* Disconnect I/O Ports */
        sim_map_resource(pInfo->io_base, pInfo->io_size, RESOURCE_TYPE_IO, &priamserialdev, "priamserialdev", TRUE);
    } else {
        
        /* Connect I/O Ports at base address */
        if(sim_map_resource(pInfo->io_base, pInfo->io_size, RESOURCE_TYPE_IO, &priamserialdev, "priamserialdev", FALSE) != 0) {
            sim_debug(ERROR_MSG, &priamserial_dev, PRIAMSERIAL_SNAME ": Error mapping I/O resource at 0x%02x\n", pInfo->io_base);
            return SCPE_ARG;
        }
    }

    sim_debug(STATUS_MSG, &priamserial_dev, PRIAMSERIAL_SNAME ": reset controller.\n");

    return SCPE_OK;
}

static void Priamserial_closeport()
{
	if (priamserial_info->serPort != INVALID_HANDLE)
	{
		sim_close_serial(priamserial_info->serPort);
		priamserial_info->serPort = INVALID_HANDLE;
		priamserial_info->serPortName[0] = 0;
	}
}

static t_stat Priamserial_set_port(UNIT* uptr, int32 val, CONST char* cptr, void* desc)
{
    
    char config[] = ";115200-8N1";
	if (cptr == NULL)
		return SCPE_ARG;

   
    Priamserial_closeport();

    t_stat r;
    
    strcpy_s(priamserial_info->serPortName, MAXPORTNAME, cptr);
    strcat_s(priamserial_info->serPortName, MAXPORTNAME, config);
    SERHANDLE port = sim_open_serial(priamserial_info->serPortName, NULL, &r);

    if (port == INVALID_HANDLE)
    {
        sim_printf(PRIAMSERIAL_SNAME " Error opening port %s.\n", priamserial_info->serPortName);
        priamserial_info->serPortName[0] = 0;
        return SCPE_OPENERR;
    }
    else
    {
        sim_printf(PRIAMSERIAL_SNAME " Opened port %s.\n", priamserial_info->serPortName);
        priamserial_info->serPort = port;

        uint8 buf[1] = { 0x55 };

		if (sim_write_serial(priamserial_info->serPort, buf, 1) != 1)
		{
			sim_printf(PRIAMSERIAL_SNAME " Port open: serial write error.\n");
            Priamserial_closeport();
			return SCPE_OPENERR;
		}

        uint32 nr = Priamserial_Readserial(buf, 1, 10);
        if (nr < 0)
		{
			sim_printf(PRIAMSERIAL_SNAME " Port open: serial read error.\n");
            Priamserial_closeport();
			return SCPE_OPENERR;
		}

        if (nr != 1 || buf[0] != 0xAA)
        {
			sim_printf(PRIAMSERIAL_SNAME " Port open: could not sync with Arduino.\n");
            Priamserial_closeport();
			return SCPE_OPENERR;
        }
        
        sim_printf(PRIAMSERIAL_SNAME " Synced with Arduino.\n");
    }
	
	return SCPE_OK;
}

t_stat Priamserial_show_port(FILE* st, UNIT* uptr, int32 val, CONST void* desc)
{
    if (priamserial_info->serPort == INVALID_HANDLE)
    {
        fprintf(st, "Port not open");
    }
    else
    {
        fprintf(st, "%s", priamserial_info->serPortName);
    }

	
	return SCPE_OK;
}



static t_stat priamserial_svc(UNIT *uptr)
{
    return SCPE_OK;
}


static int32 priamserialdev(int32 Addr, int32 rw, int32 data)
{
    if (rw == 0) { /* Read */
        return(PRIAMSerial_In(Addr));
    } else {       /* Write */
        return(PRIAMSerial_Out(Addr, data));
    }
}


static int32 Priamserial_Readserial(uint8* result, int count, int maxtries)
{
    char buf[1];
    int32 r;
    uint8 brk;

    for (int i = 0; i < count; i++)
    {
        int triesremain = maxtries;
        do 
        {
            if (maxtries)
            {
                if (!triesremain)
                {
                    break;
                }
                else
                {
                    if (triesremain != maxtries)
                        sim_os_ms_sleep(10);
                    triesremain--;
                    
                }
            }
            r = sim_read_serial(priamserial_info->serPort, buf, 1, &brk);

        } while (r == 0);
     
		if (r < 0)
			return r;

        if (maxtries && !triesremain)
        {
            sim_printf(PRIAMSERIAL_SNAME " Serial read timeout.\n", priamserial_info->serPortName);
            return i;
        }

        

        result[i] = buf[0];
    }

    return count;
}

//IN instruction, our OUTPUT
static uint8 PRIAMSerial_In(uint32 Addr)
{
    
    uint8 regno = Addr - PRIAMSERIAL_IO_BASE;
    char buf[1];

    if (priamserial_info->serPort == INVALID_HANDLE)
    {
        sim_printf(PRIAMSERIAL_SNAME " IN instruction: port not open.\n");
        return 0;
    }

    buf[0] = regno;

    
    if (sim_write_serial(priamserial_info->serPort, buf, 1) != 1)
    {
        sim_printf(PRIAMSERIAL_SNAME " IN instruction: Serial write error.\n");
        return 0;
    }

        
        
    if (Priamserial_Readserial(buf, 1, READMAXTRIES) != 1)
    {
        sim_printf(PRIAMSERIAL_SNAME " IN instruction: Serial read error.\n");
        return 0;
    }
    
    return buf[0];
}

// OUT instruction, our INPUT
static uint8 PRIAMSerial_Out(uint32 Addr, int32 Data)
{
	uint8 regno = Addr - PRIAMSERIAL_IO_BASE;
	char buf[2];

	if (priamserial_info->serPort == INVALID_HANDLE)
	{
		sim_printf(PRIAMSERIAL_SNAME " OUT instruction: port not open.\n");
		return 0;
	}

	buf[0] = regno | 0x80; //Signal write
    buf[1] = (char)Data;
	if (sim_write_serial(priamserial_info->serPort, buf, 2) != 2)	
	{
		sim_printf(PRIAMSERIAL_SNAME " OUT instruction: Serial write error.\n", priamserial_info->serPortName);
		return 0;
	}

    return(Data);
}


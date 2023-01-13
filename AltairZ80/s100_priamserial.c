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

#define PRIAMCMD_COMPACK 0
#define PRIAMCMD_READNORETRY 0x43
#define PRIAMCMD_READWITHRETRY 0x53
#define PRIAMCMD_WRITENORETRY 0x42
#define PRIAMCMD_WRITEWITHRETRY 0x52
#define PRIAMCMD_SEQUENCEUPANDRETURN 0x83
#define PRIAMCMD_SEQUENCEDOWN 0x81


typedef enum 
{
    NOREADWRITE,
    INPROGRESS,
    LASTBYTEDONE
} readwritestate_t;



typedef struct {
	uint32 readptr;
	uint32 writeptr;
	uint8 data[PRIAM_BUFFER_SIZE];
    uint32 havenumbytesfromdrive;
    uint32 havenumbytesfromuser;
	uint32 neednumbytesfromdrive;
	uint32 neednumbytesfromuser;
    readwritestate_t readstate;
    readwritestate_t writestate;
} PRIAM_INTERNALBUFFER;





typedef struct {
	uint32 io_base;     /* I/O Base Address                 */
	uint32 io_size;     /* I/O Address Space requirement    */
    SERHANDLE serPort;	
    char serPortName[MAXPORTNAME];

	PRIAM_INTERNALBUFFER* buffer;
	uint8 inregs[PRIAM_IO_SIZE];
	uint8 outregs[PRIAM_IO_SIZE];
} PRIAMSERIAL_INFO;



static REG priamserial_reg[] = {
	{ NULL }
};

static PRIAM_INTERNALBUFFER PriamBuffer;


static PRIAMSERIAL_INFO priamserial_info_data = { PRIAMSERIAL_IO_BASE, PRIAMSERIAL_IO_SIZE,INVALID_HANDLE, "", 0, {0}, {0} };

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
static t_stat priamserial_spin(UNIT* uptr, int32 value, CONST char* cptr, void* desc);


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
	{ MTAB_XTD | MTAB_VDV,  1,     NULL,           "SPINUP",         &priamserial_spin,
		NULL, NULL, "Priam drive spin up"},
	{ MTAB_XTD | MTAB_VDV,  0,      NULL,           "SPINDOWN",      &priamserial_spin,
		NULL, NULL, "Priam drive spin down"   },
    
    { 0 }
};

/* Debug flags */
#define ERROR_MSG           (1 << 0)
#define SERIAL_MSG            (1 << 1)
#define STATUS_MSG          (1 << 5)

/* Debug Flags */
static DEBTAB priamserial_dt[] = {
    { "ERROR",      ERROR_MSG,          "Error messages"        },
    { "SERIAL",     SERIAL_MSG,         "Serial comms messages" },
    { "STATUS",     STATUS_MSG,         "Status messages"       },
    
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

	PriamBuffer.readptr = 0;
	PriamBuffer.writeptr = 0;
	
    PriamBuffer.havenumbytesfromdrive = 0;
    PriamBuffer.havenumbytesfromuser = 0;
    PriamBuffer.neednumbytesfromdrive = 0;
    PriamBuffer.neednumbytesfromuser = 0;
    PriamBuffer.readstate = NOREADWRITE;
    PriamBuffer.writestate = NOREADWRITE;

	priamserial_info->buffer = &PriamBuffer;

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


static t_stat priamserial_spin(UNIT* uptr, int32 value, CONST char* cptr, void* desc)
{
    
    PRIAMSerial_Out(PRIAMREGIN_DRIVENO + PRIAMSERIAL_IO_BASE, 0);

    if (value)
    {
        PRIAMSerial_Out(PRIAMREGIN_CMD + PRIAMSERIAL_IO_BASE, PRIAMCMD_SEQUENCEUPANDRETURN);
    }
    else
    {
        PRIAMSerial_Out(PRIAMREGIN_CMD + PRIAMSERIAL_IO_BASE, PRIAMCMD_SEQUENCEDOWN);
    }
    
    
    while (!(PRIAMSerial_In(PRIAMREGOUT_STAT + PRIAMSERIAL_IO_BASE) & PRIAMSTATUS_COMPREQ));

    PRIAMSerial_Out(PRIAMREGIN_CMD + PRIAMSERIAL_IO_BASE, PRIAMCMD_COMPACK);
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


    sim_debug(SERIAL_MSG, &priamserial_dev, "\nIN reg %ld\n", regno);

    buf[0] = regno;

    //Check if data read command
    if (regno == PRIAMREGOUT_DATA)
    {
        //Check if read in progress
        if (priamserial_info->buffer->readstate == INPROGRESS)
        {
            //Read
            uint8 b = priamserial_info->buffer->data[priamserial_info->buffer->readptr];
            priamserial_info->buffer->readptr = (priamserial_info->buffer->readptr + 1) % PRIAM_BUFFER_SIZE;
            if (priamserial_info->buffer->readptr == priamserial_info->buffer->writeptr)
                priamserial_info->buffer->readstate = LASTBYTEDONE;

            sim_debug(SERIAL_MSG, &priamserial_dev, "\tRet from buffer: %02X\n", b);
            

            return b;
        }
        else
        {
            sim_printf("Error! Read not in progress on data register read\n");
        }
    }
	else if (regno == PRIAMREGOUT_STAT)
	{
		//Check if read or write in progress
		if (priamserial_info->buffer->readstate == INPROGRESS)
		{

            sim_debug(SERIAL_MSG, &priamserial_dev, "\tRead Ret xfer req\n");
			//Signal more bytes areavailable;
			return PRIAMSTATUS_DATAXFERREQ | PRIAMSTATUS_ISREADREQ | PRIAMSTATUS_DBUSENABLE;
		}
        else if (priamserial_info->buffer->writestate == INPROGRESS)
        {
			sim_debug(SERIAL_MSG, &priamserial_dev, "\tWrite Ret xfer req\n");
			//Signal more bytes areavailable;
			return PRIAMSTATUS_DATAXFERREQ | PRIAMSTATUS_DBUSENABLE;
        }
        
	}


    sim_debug(SERIAL_MSG, &priamserial_dev, "\tSerwrite % 02X\n", buf[0]);
    

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

    sim_debug(SERIAL_MSG, &priamserial_dev, "\tRet from serread: %02X\n", (uint8_t)(buf[0]));
    

    return buf[0];
}

// OUT instruction, our INPUT
static uint8 PRIAMSerial_Out(uint32 Addr, int32 Data)
{
	uint8 regno = Addr - PRIAMSERIAL_IO_BASE;
	char buf[2];


    sim_debug(SERIAL_MSG, &priamserial_dev, "\nOUT reg %ld value %02lx\n", regno, (uint8_t)(Data));
    
	
	priamserial_info->inregs[regno] = Data;

	if (priamserial_info->serPort == INVALID_HANDLE)
	{
		sim_printf(PRIAMSERIAL_SNAME " OUT instruction: port not open.\n");
		return 0;
	}

	buf[0] = regno | 0x80; //Signal write
    buf[1] = (char)Data;


    

	if (regno == PRIAMREGIN_DATA)
	{
		//Check if write in progress, if so buffer this write
		if (priamserial_info->buffer->writestate == INPROGRESS)
		{
            if (!priamserial_info->buffer->neednumbytesfromuser || priamserial_info->buffer->neednumbytesfromuser == priamserial_info->buffer->havenumbytesfromuser)
            {
                sim_printf("Error! Not expecting bytes on data register write\n");
                return Data;
            }
            else
            {
                //Write
                priamserial_info->buffer->data[priamserial_info->buffer->writeptr] = Data;

                priamserial_info->buffer->writeptr = (priamserial_info->buffer->writeptr + 1) % PRIAM_BUFFER_SIZE;

                priamserial_info->buffer->havenumbytesfromuser++;
                

                if (priamserial_info->buffer->neednumbytesfromuser == priamserial_info->buffer->havenumbytesfromuser)
                {
                    sim_debug(SERIAL_MSG, &priamserial_dev, "\tWrite complete, send buffer\n");

#if 0
                    for (uint32_t k = 0; k < priamserial_info->buffer->havenumbytesfromuser; k++)
                    {
                        if (k && !(k%16))
                            sim_debug(SERIAL_MSG, &priamserial_dev, "\n");
                        sim_debug(SERIAL_MSG, &priamserial_dev, "%02X", priamserial_info->buffer->data[priamserial_info->buffer->readptr + k]);

                    }
#endif

                    sim_write_serial(priamserial_info->serPort, 
                                    &priamserial_info->buffer->data[priamserial_info->buffer->readptr], 
                                    priamserial_info->buffer->havenumbytesfromuser);
                    priamserial_info->buffer->writestate = LASTBYTEDONE;
                }
                else
                {
                    sim_debug(SERIAL_MSG, &priamserial_dev, "\tByte write stored in buffer\n");
                }
                

                return Data;
            }
		}
		else
		{
			sim_printf("Error! Write not in progress on data register write\n");
		}
	}


	sim_debug(SERIAL_MSG, &priamserial_dev, "\tSerwrite %02X %02X\n", (uint8_t)(buf[0]), (uint8_t)(buf[1]));

	if (sim_write_serial(priamserial_info->serPort, buf, 2) != 2)
	{
		sim_printf(PRIAMSERIAL_SNAME " OUT instruction: Serial write error.\n", priamserial_info->serPortName);
		return 0;
	}

    //If read or write command, setup for buffering
    if (regno == PRIAMREGIN_CMD && (Data == PRIAMCMD_READNORETRY || Data == PRIAMCMD_READWITHRETRY))
    {
        uint32 numToRead = priamserial_info->inregs[PRIAMREGIN_NUMSECTORS] * PRIAM_SECTOR_SIZE;
        priamserial_info->buffer->readptr = 0;
        priamserial_info->buffer->writeptr = 0;
        //char brk;
        
        //sim_read_serial(priamserial_info->serPort, &priamserial_info->buffer->data[priamserial_info->buffer->writeptr], 
        //    numToRead,
        //    &brk);
        if (Priamserial_Readserial(&priamserial_info->buffer->data[priamserial_info->buffer->writeptr], numToRead, 0) != numToRead)
            sim_printf("Data read error\n");


        sim_debug(SERIAL_MSG, &priamserial_dev, "\tSerread got %04lX bytes\n", numToRead);
        

        priamserial_info->buffer->writeptr = (priamserial_info->buffer->writeptr + numToRead) % PRIAM_BUFFER_SIZE;

        priamserial_info->buffer->writestate = NOREADWRITE;
        priamserial_info->buffer->readstate = INPROGRESS;

        

    }
	else if (regno == PRIAMREGIN_CMD && (Data == PRIAMCMD_WRITENORETRY || Data == PRIAMCMD_WRITEWITHRETRY))
	{
		uint32 numToWrite = priamserial_info->inregs[PRIAMREGIN_NUMSECTORS] * PRIAM_SECTOR_SIZE;
		priamserial_info->buffer->readptr = 0;
		priamserial_info->buffer->writeptr = 0;
        priamserial_info->buffer->neednumbytesfromuser = numToWrite;
        priamserial_info->buffer->havenumbytesfromuser = 0;

		
		priamserial_info->buffer->writestate = INPROGRESS;
		priamserial_info->buffer->readstate = NOREADWRITE;



	}

    
    return(Data);
}


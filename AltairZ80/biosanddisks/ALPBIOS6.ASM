;******************************************************
;						      *
;	PROGRAM ID:	DOUBLE D BIOS  (DDBIOS)       *
;						      *
;******************************************************
;						      *
;	VERSION:	CP/M 2.2 8"  RELEASE 2A       *
;						      *
;******************************************************
;						      *
;	PRESENTED BY:	JADE COMPUTER PRODUCTS	      *
;			4901 W. ROSECRANS BLVD.	      *
;			HAWTHORNE, CALIFORNIA	      *
;			90250,   U.S.A.		      *
;						      *
;*********************************************** SK ***

;******************************************************
; DECLARE CP/M 2.2 SYSTEM SIZE			      *
;******************************************************

CPM$NK	EQU	60	;SYSTEM SIZE K BYTES.

;******************************************************
; DOUBLE D HARDWARE PARAMETER - SYSTEM PORT ADDRESS   *
;******************************************************

D$PORT	EQU	043H	;DOUBLE D PORT ADDRESS.

;******************************************************
; SELECT NUMBER OF DISK DRIVES USED		      *
;******************************************************

N$DRVS	EQU	2	;SELECT 1 TO 4 DRIVES.

;******************************************************
; DISK OPERATING SYSTEM ADDRESSES.		      *
;******************************************************

K$B	EQU	1024		;1K BYTE SIZE.
CPM$SZ	EQU	CPM$NK * K$B	;TOP SYSTEM ADDRESS.
CPM$BS	EQU	CPM$SZ-(20*K$B) ;CP/M BIAS VALUE.

TPA	EQU	0100H		;ADDRESS OF TPA.
CCP	EQU	CPM$BS+3400H	;ADDRESS OF CCP.
BDOS	EQU	CPM$BS+3C00H	;ADDRESS OF BDOS.
BIOS	EQU	CPM$BS+4A00H	;ADDRESS OF BIOS
BIOS$R	EQU	1000H-BIOS	;DDT OFFSET 1000H LOAD.
BOOT	EQU	0F000H		;BOOT PROM JUMP TABLE.
IO$LOC	EQU	0003H		;I/O BYTE LOCATION.
DF$LOC	EQU	0004H		;DRIVE ASSIGN LOCATION.

;******************************************************
; DOUBLE D SYSTEM PARAMETERS			      *
;******************************************************

IOBYTE	EQU	00000000B	;INITIAL IOBYTE VALUE.
DF$DRV	EQU	0		;INITIAL DEFAULT DRV.
SEC$SZ	EQU	0080H		;BYTES PER SECTOR.
FMT$SZ	EQU	0100H		;FORMAT BUFF SIZE.

;******************************************************
; DOUBLE D HARDWARE COMMANDS			      *
;******************************************************

DC$SIN	EQU  00000001B	;SWITCH DD BANK 0 INTO SYSTEM.
DC$MB0	EQU  00000001B	;SELECT DOUBLE D BANK 0.
DC$MB1	EQU  00000011B	;SELECT DOUBLE D BANK 1.
DC$SOT	EQU  00000000B	;SWITCH DD MEM OUT OF SYSTEM.
DC$INT	EQU  00000010B	;ISSUE DD Z80A INTERRUPT.

;******************************************************
; DISK CONTROLLER MODULE LINKAGE (DCM - VER 2.2)      *
;******************************************************
	
;*******( DCM ADDRESSES DEFINED )*********************

DD$CBT	EQU	0370H	;COMMAND BLOCK  (BANK 0).
DD$BUF	EQU	0380H	;SECTOR BUFFER  (BANK 0).
DD$FBF	EQU	0300H	;FORMAT BUFFER	(BANK 1).
DD$DPB	EQU	03A0H	;ID SEC DPB	(BANK 0).
DD$DDF	EQU	03B1H	;ID SEC FLAGS	(BANK 0).

;*******( DCM COMMANDS )*******************************

DC$LOG	EQU	000H	;LOG ON DISKETTE.
DC$RDS	EQU	001H	;READ SECTOR.
DC$WRS	EQU	002H	;WRITE SECTOR.
DC$FMT	EQU	003H	;FORMAT TRACK.
DC$LST	EQU	005H	;LIST CHARACTER.
DC$LCK	EQU	006H	;LIST STATUS.

;******************************************************
; ASSEMBLER DIRECTIVES				      *
;******************************************************

	ORG	BIOS

;******************************************************
; BIOS JUMP VECTOR TABLE			      *
;******************************************************

	JMP	INIT		;COLD START ENTRY
	JMP	WARM		;RELOAD CCP/BDOS
	JMP	CNS$CK		;GET CONSOLE STATUS
	JMP	CNS$IN		;CONSOLE INPUT
	JMP	CNS$OT		;CONSOLE OUTPUT
	JMP	LIST		;PRINTER OUTPUT
	JMP	PUNCH		;PUNCH OUTPUT
	JMP	READER		;READER INPUT
	JMP	HOME		;HOME SELECTED DRIVE
	JMP	SELDSK		;SELECT DISK DRIVE
	JMP	SETTRK		;SET TRACK NUMBER
	JMP	SETSEC		;SET SECTOR NUMBER
	JMP	SETDMA		;SET TRANSFER ADDRESS
	JMP	DISKRD		;PERFORM DISK READ
	JMP	DISKWR		;PERFORM DISK WRITE
	JMP	LISTST		;RETURN LIST STAT
	JMP	SECTRN		;TRANSLATE SECTOR
	JMP	FORMAT		;FORMAT A TRACK

;******************************************************
; COLD START ENTRY - ISSUE SIGN ON MESSAGE	      *
;******************************************************

INIT:	LXI	SP,0080H	;SET UP STACK AREA.
	LXI	H,MSG$SO	;SIGN ON MSG ADDR.
	CALL	MSG$OT		;ISSUE MESSAGE.
	JMP	CPM$LD		;LOAD CCP/BDOS.

;******************************************************
; WARM BOOT ENTRY - LOADS CCP/BDOS - INITIALIZES      *
;******************************************************

;*******( SET UP FOR CCP/BDOS LOAD )*******************

WARM:	LDA	IO$LOC		;GET I/O BYTE VALUE.
	STA	IO$IMG		;STORE I/O VALUE.
       	MOV	A,C
	CPI	0A5h		;signal from priam driver
	jz	PRIAM
        MVI	D,N$DRVS	;copy of number of drives

PRIAM:
;	MOV	A,D		;number of drives in D.(for ZCPR)
        ;Number of drives now in D regardless of signal from priam driver

	LDA	DF$LOC		;GET DEFAULT DRIVE.

;        MOV	B,A		;save a good copy (for ZCPR)
;	ANI	OFh		;mask out user area (for ZCPR)
	CMP	D		;check legal drive.
;	mov	A,B		;restore good copy (for ZCPR)

;	CPI	N$DRVS		;CHECK LEGAL DRIVE.
	JC	WRM$OK		;IF LEGAL, GO OK.
	XRA	A		;SET DRIVE TO A.
WRM$OK:	STA	DF$IMG		;STORE IN IMAGE.
	LXI	SP,0080H	;SET UP STACK.
CPM$LD:	MVI	A,DF$DRV	;INIT DEFAULT DRIVE.
	STA	BT$DRV		;SELECT DISK.
	LXI	B,CCP		;CP/M CCP ADDRESS.
	CALL	SETDMA		;SET DMA ADDR.
	MVI	C,2		;CCP 1ST SECTOR.
	CALL	SETSEC		;SET SECTOR NMBR.
	MVI	C,1		;CCP/BDOS TRACK.
	CALL	SETTRK		;SET TRACK NUMBER.

;*******( LOAD CCP/BDOS )******************************

W$READ:	CALL	DISKRD		;READ ONE SECTOR.
	ANA	A		;SET FLAGS.
	JNZ	W$EROR		;EXIT IF ERROR.
	LDA	BT$SEC		;GET SECTOR NMBR.
	CPI	45		;LAST SECTOR CHECK.
	JZ	W$ZRPG		;GOTO ZERO PAGE SET.
	INR	A		;INCREMENT SECTOR.
	STA	BT$SEC		;STORE NEXT SECTOR.
	LXI	D,SEC$SZ	;GET SECTOR SIZE.
	LHLD	BT$DMA		;GET TRANSFER ADDR.
	DAD	D		;CALCULATE NEW ADDR.
	SHLD	BT$DMA		;SET NEW ADDRESS.
	JMP	W$READ		;DO ANOTHER WARM READ.

;*******( READ ERROR DETECTED )************************

W$EROR:	LXI	H,MSG$LE	;GET ERROR MESAAGE.
	CALL	MSG$OT		;ISSUE MESSAGE.
	HLT			;OR GOTO MONITOR

;*******( INITIALIZE SYSTEM PARAMETERS )***************

W$ZRPG:	LXI	B,8		;BASE IMAGE SIZE.
	LXI	D,0		;BASE ADDRESS SET.
	LXI	H,BS$IMG	;BASE IMAGE ADDR.
	CALL	BLOCK		;BLOCK MOVE ROUTINE.
	LXI	H,0080H		;DEFAULT SECTOR BUFF.
	SHLD	BT$DMA		;SET TRANSFER ADDRESS.

;*******( JUMP TO CCP )********************************

	LDA	DF$LOC		;GET CURRENT DSK NMBR.
	MOV	C,A		;SEND TO THE CCP.
	JMP	CCP		;JUMP INTO CCP CP/M.

;******************************************************
; CONSOLE LINKAGE DEFINITIONS - BOOT PROM ADDRESSES   *
;******************************************************

;CNS$CK	EQU	BOOT+006H	;CHECK CONSOLE INPUT.
;CNS$IN	EQU	BOOT+009H	;READ CONSOLE INPUT.
;CNS$OT	EQU	BOOT+00CH	;CHARACTER TO CONSOLE.

CNS$CK: IN      010H
        XRI     0
        ANI     1
        RZ
        LDA     0FFh
        RET


CNS$IN:
        CALL    CNS$CK
        JZ      CNS$IN
        IN      011H
        ANI     07Fh
        RET

CNS$OT:
        IN      010H
        XRI     0
        ANI     2
        JZ      CNS$OT
        MOV     A,C
        OUT     011H
        RET
;******************************************************
; READER AND PUNCH DRIVERS - USER SHOULD DEFINED      *
;******************************************************

READER:	MVI  A,CNTL$Z!RET	;RETURN END OF FILE.
PUNCH:	RET			;NOT IMPLEMENTED.

;******************************************************
; PRINTER DRIVER AREA - DCM SERIAL PORT LINKAGE       *
;******************************************************

LIST:	MOV	A,C		;LIST CHAR TO ACUM.
	STA	BT$CHR		;STORE LIST CHARACTER.
	MVI	A,DC$SIN	;LOAD SWITCH MEM CMND.
	OUT	D$PORT		;ISSUE HARDWARE CMND.
	MVI	A,DC$LST	;DCM LIST COMMAND.
	CALL	DSK$EX		;CALL DISK EXECUTE.
	JMP	DSK$OK		;RETURN TO CALLER.

LISTST:	MVI	A,DC$SIN	;LOAD SWITCH MEM CMND.
	OUT	D$PORT		;ISSUE HARDWARE CMND.
	MVI	A,DC$LCK	;DCM LIST STAT CMND.
	CALL	DSK$EX		;CALL DISK EXECUTE.
	CALL	DSK$OK		;SWITCH DD MEM OUT.
	LDA	BT$STS		;LOAD RETURN STATUS.
	RET			;RETURN TO CALLER.

;******************************************************
; HOME - SET TRACK TO ZERO			      *
;******************************************************

HOME:	MVI	C,0		;C REGISTER TO ZERO.
	JMP	SETTRK		;PERFORM SET TRACK.

;******************************************************
; SELECT DISK DRIVE - CHECK FOR LOGON		      *
;******************************************************

SELDSK:	LXI	H,0		;ERROR RETURN CODE.
	MOV	A,C		;PUT DRIVE NMBR IN A.
	CPI	N$DRVS		;CHECK IF LEGAL DRIVE.
	RNC			;NO CARRY IF ILLEGAL.
	STA	BT$DRV		;STORE DRIVE NUMBER.
	MOV	A,E		;CHECK IF LOG-ON REQ.
	STA	LOG$RQ		;STORE LOGON REGISTER.
RETDSK:	LDA	BT$DRV		;GET DRIVE NUMBER.
	MOV	L,A		;L SET DISK NUMBER.
	MVI	H,0		;ZERO H REGISTER.
	DAD	H		;*2.
	DAD	H		;*4.
	DAD	H		;*8.
	DAD	H		;*16 (SIZE OF HEADER).
	LXI	D,D0$DPH	;DRIVE 0 D$P$H.
	DAD	D		;HLSET DRIVE N DPH.
	JMP	LOG$ON		;GO CHECK LOG-ON.

;******************************************************
; SET TRACK NUMBER				      *
;******************************************************

SETTRK:	MOV	A,C		;MOVE TRACK NUMBER.
	STA	BT$TRK		;SAVE TRACK NUMBER.
	RET			;RETURN TO CALLER.

;******************************************************
; SET SECTOR NUMBER				      *
;******************************************************

SETSEC:	MOV	A,C		;MOVE SECTOR NUMBER.
	STA	BT$SEC		;SAVE SECTOR NUMBER.
	RET			;RETURN TO CALLER.

;******************************************************
; SET MEMORY ADDRESS FOR DISK TRANSFER		      *
;******************************************************

SETDMA:	MOV	H,B		;HIGH ORDER MOVE.
	MOV	L,C		;LOW ORDER MOVE.
	SHLD	BT$DMA		;SAVE TRANSFER ADDRESS.
	RET			;RETURN TO CALLER.

;******************************************************
; READ A DISK SECTOR ROUTINE			      *
;******************************************************

DISKRD:	MVI	A,DC$SIN	;SWITCH DD INTO SYSTEM.
	OUT	D$PORT		;ISSUE DD COMMAND.
	MVI	A,DC$RDS	;READ SECTOR COMMAND.
	CALL	DSK$EX		;PERFORM OPERATION.
	JNZ	DSK$ER		;ERROR EXIT.
	LHLD	BT$DMA		;LOAD USER BUF ADDRESS
	XCHG			;MOVE HL TO DE.
	LXI	B,DD$BUF	;LOAD BUFFER OFFSET.
	LHLD	D$ADDR		;LOAD DD WINDOW ADDR.
	DAD	B		;HL NOW SECTOR BUFFER.
	LXI	B,SEC$SZ	;LOAD SECTOR SIZE.
	CALL	BLOCK		;BLOCK MOVE ROUTINE.
	JMP	DSK$OK		;NORMAL RETURN.

;******************************************************
; WRITE A DISK SECTOR ROUTINE			      *
;******************************************************

DISKWR:	MVI	A,DC$SIN	;SWITCH DD INTO SYSTEM.
	OUT	D$PORT		;ISSUE HARDWARE CMND.
	LXI	B,SEC$SZ	;LOAD SECTOR SIZE.
	LHLD	D$ADDR		;DD SYSTEM ADDRESS.
	LXI	D,DD$BUF	;DD BUFFER OFFSET.
	DAD	D		;HL NOW DD BUF ADDR.
	XCHG			;DE NOW DD BUF ADDR.
	LHLD	BT$DMA		;HL NOW USER BUF ADDR.
	CALL	BLOCK		;BLOCK MOVE ROUTINE.
	MVI	A,DC$WRS	;LOAD WRITE SEC CMND.
	CALL	DSK$EX		;CALL DISK EXECUTIVE.
	JZ	DSK$OK		;JUMP IF WRITE OK.
	JMP	DSK$ER		;ERROR EXIT.

;******************************************************
; DISK READ/WRITE EXITS				      *	
;******************************************************

DSK$OK:	MVI	A,DC$SOT	;SWITCH DD OUT OF SYS.
	OUT	D$PORT		;ISSSUE HARDWARE CMND.
	XRA	A		;ZERO A REGISTER.
	RET			;NORMAL EXIT.

DSK$ER:	MVI	A,DC$SOT	;SWITCH DD OUT OF SYS.
	OUT	D$PORT		;ISSSUE HARDWARE CMND.
	MVI	A,0FFH		;LOAD ERROR FLAGS.
	RET			;ERROR EXIT.

;******************************************************
; TRANSLATE SECTOR NUMBER			      *
;******************************************************

SECTRN:	MOV	A,D		;TESTING TBL ADDR.
	ORA	E		;ADDR IN REG DE.
	JZ	NOTRAN		;IF ZERO, NO TRANS.
	XCHG			;(HL) NOW TRANS TBL.
	DAD	B		;(HL) NOW TRANS SECTOR.
	MOV	L,M		;L IS TRANSLATED SEC.
	MVI	H,0		;HIGH ORDER BYTE ZERO.
	RET			;RETURN TO CALLER.
NOTRAN:	LXI	H,1		;SET HL TO ONE.
	DAD	B		;ADD SEC NMBR TO HL.
	RET			;RETURN TO CALLER.

;******************************************************
; FORMAT A DISK TRACK ROUTINE			      *
;******************************************************

FORMAT:	MVI	A,DC$SIN	;SWITCH DD INTO SYSTEM.
	OUT	D$PORT		;ISSUE HARDWARE CMND.
	MVI	A,DC$MB1	;SELECT DD BANK 1.
	OUT	D$PORT		;ISSUE HARDWARE CMND.
	LXI	B,FMT$SZ	;FORMAT PROG SIZE.
	LHLD	D$ADDR		;DD SYSTEM ADDRESS.
	LXI	D,DD$FBF	;DD FORMAT BUF OFFSET.
	DAD	D		;HL NOW DD FBUF ADDR.
	XCHG			;DE NOW DD FBUF ADDR.
	LHLD	BT$DMA		;FORMAT PROGRAM ADDR.
	CALL	BLOCK		;BLOCK MOVE ROUTINE.
	MVI	A,DC$MB0	;RESELECT DD BANK 0.
	OUT	D$PORT		;ISSUE TO DD HARDWARE.
	MVI	A,DC$FMT	;LOAD FORMAT TRK CMND.
	CALL	DSK$EX		;CALL DISK EXECUTIVE.
	CALL	DSK$OK		;SWITCH DD MEM OUT.
	LDA	BT$STS		;LOAD FORMAT STATUS.
	RET			;FORMAT EXIT.

;******************************************************
; DOUBLE D EXECUTION SUBROUTINE			      *
;******************************************************

;*******( COMMAND BLOCK TO DOUBLE D AND EXEC )*********

DSK$EX:	STA	BT$CMD		;STORE DCM COMMAND.
	LXI	B,7		;NMBR BYTE TO MOVE.
	LXI	D,DD$CBT	;COMMAND BYTE OFFSET.
	LHLD	D$ADDR		;DD SYS ADDRESS.
	DAD	D		;HL NOW PTS CMND BLK.
	XCHG			;NOW ADDR IN DE.
	LXI	H,BT$CMD	;BIOS CMND BLOCK.
	CALL	BLOCK		;PERFORM BLOCK MOVE.
	MVI	A,DC$INT	;LOAD DD INTERRUPT.
	OUT	D$PORT		;ISSUE DD INTERRUPT.

;*******( WAIT FOR DOUBLE D HALT )*********************

	LDA	D$HALT		;LOAD HALT BIT MASK.
	MOV	B,A		;MASK IN B REGISTER.
DSK$WT:	IN	D$PORT		;READ DD STATUS.
	ANA	B		;TEST HALT* FLAG.
	JNZ	DSK$WT		;TEST UNTIL HALTED.

;*******( GET DOUBLE D STATUS )************************

	MVI	A,DC$SIN	;SWITCH DD INTO SYS.
	OUT	D$PORT		;ISSUE HARDWARE CMND.
	XCHG			;EXCHANGE SRC/DSTN.
	MOV	A,M		;STATUS INTO A REG.
	STAX	D		;STORE STATUS BYTE.
	ANA	A		;TEST FOR ERRORS.
	RET			;RETURN TO CALLER.

;******************************************************
; LOG-ON  -   SET DISK PARAMETER BLOCK		      *
;******************************************************

;*******( CHECK IF LOG-ON REQUESTED )******************

LOG$ON:	LDA	LOG$RQ		;CHECK LOG REQUEST.
	ANI	001H		;LOG ON BIT TEST.
	JNZ	DSK$OK		;RETURN, NO LOG-ON.

;*******( READ IDENTITY SECTOR )***********************

	SHLD	DT$PTR		;STORE DRV TBL PNTR.
	MVI	A,DC$SIN	;SWITCH DD INTO SYS.
	OUT	D$PORT		;ISSUE HARDWARE CMND.
	MVI	A,DC$LOG	;LOAD DCM LOG-ON CMND.
	CALL	DSK$EX		;PERFORM DISK OP.
	JZ	LOG$CK		;GO TO LOGON ERROR.
	LXI	H,0		;ERROR, BAD LOG ON.
	JMP	DSK$ER		;BIOS EXIT.
;	
;*******( CHECK FOR JADE ID )**************************

LOG$CK:	LXI	D,DD$BUF	;DD BUFFER OFFSET.
	LHLD	D$ADDR		;DD SYS ADDRESS.
	DAD	D		;HL NOW PNTS BUFFER.
	LXI	D,JADEID	;DE PNTS BIOS ID.
	MVI	B,ID$SZE	;SET LABEL SIZE.
LOG$ID:	LDAX	D!	INX D	;GET LABEL CHARACTER.
	CMP	M!	INX H	;DOES ID SECTOR MATCH.
	JNZ	LG3740		;ASSUME DISKETTE 3740.
	DCR	B		;DECREMENT COUNT.
	JNZ	LOG$ID		;CHECK IF ANOTHER CHR.

;*******( DISKETTE CONTAINS ID )***********************

	CALL	TRNONE		;ASSUME DDENS.
	CALL	DPB$AD		;GET DPB ADDR IN DE.
	LXI	B,DD$DPB	;DPB ADDR OFFSET.
	LHLD	D$ADDR		;DD SYSTEM ADDRESS.
	DAD	B		;HL NOW AT ID DPB.
	LXI	B,DPB$SZ	;DPB SIZE IN BYTES.
	CALL	BLOCK		;MOVE INTO DPB.
	LXI	D,DD$DDF	;ID DTA DNS OFFSET.
	LHLD	D$ADDR		;DD SYSTEM ADDR.
	DAD	D		;HL POINTS FLAGS.
	MOV	A,M		;LOAD FLAGS.
	ANI	04H		;TEST DATA DENSITY.
	CZ	TR3740		;IF 0 USE 3740 TRN.
	LHLD	DT$PTR		;RELOAD POINTER.
	JMP	DSK$OK		;EXIT BIOS JUMP.

;*******( ASSUME 3740 DISKETTE )***********************

LG3740:	CALL	TR3740		;SET SECTOR TRANSLATE.
	CALL	DPB$AD		;SET REGISTER DE.
	LXI	B,DPB$SZ	;DPB SIZE IN BYTES.
	LXI	H,SD$PBK	;ADDRESS OF BLK IMAGE.
	CALL	BLOCK		;MOVE INTO DPB.
	LHLD	DT$PTR		;RELOAD POINTER.
	JMP	DSK$OK		;EXIT BIOS JUMP.

;*******( SET 3740 SECTOR TRANSLATION )****************

TR3740:	LXI	D,SDTRAN	;SECTOR TRAN TBL ADDR.
	LHLD	DT$PTR		;ADDR DISK PARA HDER.
	MOV	M,E		;LOW ORDER ADDR.
	INX	H		;POINT NEXT BYTE.
	MOV	M,D		;HIGH ORDER ADDR.
	RET			;RETURN TO LOG USER.

;*******( SET NO SECTOR TRANSLATION )******************

TRNONE:	XRA	A		;ZERO A REGISTER.
	LHLD	DT$PTR		;ADDR OF PARA HDER.
	MOV	M,A		;ZERO LOW ORDER ADDR.
	INX	H		;NEXT BYTE.
	MOV	M,A		;ZERO HIGH BYTE.
	RET			;RETURN TO LOG USER.

;*******( GET DRIVE PARA BLK ADDR )********************

DPB$AD:	LHLD	DT$PTR		;ADDR DISK PARA HDER.
	LXI	D,10		;DPB TBL PNTR OFFSET.
	DAD	D		;NOW AT DPB PNTR.
	MOV	E,M		;LOW ORDER ADDR.
	INX	H		;NEXT BYTE.
	MOV	D,M		;HIGH ORDER ADDR.
	RET			;RETURN TO LOG USER.

;******************************************************
; BLOCK MOVE SUBROUTINE - Z80 LDIR WILL FUNCTION HERE *
;******************************************************

BLOCK:	MOV	A,M!	INX H	;GET EACH BYTE.
	STAX	D!	INX D	;STORE EACH BYTE.
	DCX B!	MOV A,B! ORA C	;DEC LENGTH (MAX 64K).
	JNZ	BLOCK!	RET	;FINISH BLOCK AND RET.

;******************************************************
; MESSAGE DISPLAY ROUTINE - HL REG POINTS TO STRING   *
;******************************************************

MSG$OT:	MOV	A,M		;LOAD CHARACTER/BYTE.
	CPI	EOM!	RZ	;EXIT IF TERMINATOR.
	MOV C,A! CALL	CNS$OT	;DISPLAY CHARACTER.
	INX H!	JMP	MSG$OT	;REPEAT FOR NEXT BYTE.

;******************************************************
; DOUBLE D - DCM COMMAND BLOCK BUFFER		      *
;******************************************************

BT$CMD:	DB	0		;DCM COMMAND.
BT$DRV:	DB	0		;DRIVE NUMBER.
BT$TRK:	DB	0		;TRACK NUMBER.
BT$SEC:	DB	0		;SECTOR NUMBER.
BT$SP0:	DB	0		;SPARE BYTE 0.
BT$CHR:	DB	0		;LIST CHARACTER.
BT$MOD:	DB	00000000B	;MODE CONTROLS.
BT$STS:	DB	0		;COMMAND STATUS.

;******************************************************
; BIOS VARIABLE STORAGE				      *
;******************************************************

BT$DMA:	DW	0		;SYSTEM TRANSFER ADDR.
DT$PTR:	DW	0		;DRIVE TABLE POINTER.
LOG$RQ:	DB	0		;LOG ON REQUEST REG.

;******************************************************
; DOUBLE D - MEMORY ASSIGNMENTS  (40-4F HEX)	      *
;******************************************************

D$ADDR	EQU	0040H	;DD SYSTEM WINDOW ADDR POINTER.
D$HALT	EQU	0042H	;DD HALT STATUS MASK LOCATION.

;******************************************************
; 3740 FORMAT PARAMETERS ******** CP/M SINGLE DENSITY *
;******************************************************

;*******( SINGLE DENSITY CP/M SECTORING )**************

SDTRAN:	DB	01H,07H,0DH,13H,19H,05H,0BH,11H,17H,03H
	DB	09H,0FH,15H,02H,08H,0EH,14H,1AH,06H,0CH
	DB	12H,18H,04H,0AH,10H,16H

;*******( DEFAULT DISK PARAMETER BLOCK )***************

SD$PBK:	DW	26	;SECTORS PER TRACK.
	DB	3	;BLOCK SHIFT FACTOR.
	DB	7	;BLOCK MASK.
	DB	0	;NULL MASK.
	DW	242	;DISK SIZE - 1.
	DW	63	;DIRECTORY MAX.
	DB	11000000B ;ALLOÃ 0.
	DB	0	;ALLOC 1.
	DW	16	;CHECK SIZE.
	DW	2	;TRACK OFFSET.

;******************************************************
; ZERO PAGE IMAGE - BLOCK MOVED TO BASE PAGE	      *
;******************************************************

BS$IMG:	JMP	BIOS+03H	;WARM BOOT VECTOR.
IO$IMG:	DB	IOBYTE		;I/O BYTE IMAGE.
DF$IMG:	DB	DF$DRV		;DEFAULT DRIVE IMG.
	JMP	BDOS+06H	;BDOS CALL VECTOR.

;******************************************************
; MESSAGES					      *
;******************************************************

MSG$SO:	DB	CR,LF,CR,LF
	DB	'JADE COMPUTER PRODUCTS',CR,LF
	DB	'0' + CPM$NK / 10,'0' + CPM$NK MOD 10
	DB	'K CP/M 2.2 DDBIOS2',CR,LF,CR,LF,EOM

MSG$LE:	DB	CR,LF,'CCP/BDOS LOAD ERROR',EOM

LF	EQU	00AH		;ASCII LINE FEED.
CR	EQU	00DH		;CARRAIGE RETURN.
EOM	EQU	'$'		;END OF STRING.
CNTL$Z	EQU	01AH		;CONTROL-Z (EOF).

;******************************************************
; ID LABEL DEFINITIONS				      *
;******************************************************

JADEID:	DB    'Jade DD '	;ID LABEL.
ID$SZE	EQU   $-JADEID		;LABEL SIZE.

;******************************************************
; DRIVE PARAMETER HEADER AREA			      *
;******************************************************

D0$DPH:	DW	0		;SECTOR TRAN TBL.
	DW	0		;SCRATCH AREA.
	DW	0		;SCRATCH AREA.
	DW	0		;SCRATCH AREA.
	DW	DIR$BF		;DIRECTORY BUFFER.
	DW	D0$DPB		;DRIVE PARAM BLK.
	DW	D0$CHK		;DRIVE CHANGE BLK.
	DW	D0$ALL		;DRIVE ALLOCATION.

	IF	(1-N$DRVS) SHR 15	;TEST SIGN BIT.
D1$DPH:	DW	0,0,0,0
	DW	DIR$BF,D1$DPB
	DW	D1$CHK,D1$ALL
	ENDIF

	IF	(2-N$DRVS) SHR 15
D2$DPH:	DW	0,0,0,0
	DW	DIR$BF,D2$DPB
	DW	D2$CHK,D2$ALL
	ENDIF

	IF	(3-N$DRVS) SHR 15
D3$DPH:	DW	0,0,0,0
	DW	DIR$BF,D3$DPB
	DW	D3$CHK,D3$ALL
	ENDIF

;******************************************************
; BIOS PROGRAM AREA REMAINING			      *
;******************************************************

BIOS$U	EQU	BIOS + K$B	;BEGIN SCRATCH AREA.
BIOS$F	EQU	BIOS$U - $	;NUMBER OF BYTES FREE.

	IF	BIOS$F SHR 15	;TEST PROG OVERFLOW.
ERROR	EQU	1/0		;GENERATE ERROR.
	ENDIF

F$AREA:	DS	BIOS$F		;USE UP PROG AREA.

;******************************************************
; DIRECTORY BUFFER AREA	- BEGINNING OF SCRATCH AREA   *
;******************************************************

DIR$BF:	DS	SEC$SZ

;******************************************************
; DRIVE TABLE ENTRY - SIZES			      *
;******************************************************

DPB$SZ	EQU	15		;PARAMETER BLOCK SIZE.
ALL$SZ	EQU	33		;ALLOCATION INFO SIZE.
CHK$SZ	EQU	32		;CHANGED DISK SCRATCH.

;******************************************************
; DRIVE TABLES - SCRATCH AREAS 			      *
;******************************************************

D0$DPB:	DS	DPB$SZ		;DISK PARAMETER BLOCK.
D0$ALL:	DS	ALL$SZ		;DISK ALLOCATION INFO.
D0$CHK:	DS	CHK$SZ		;DISK CHANGED SCRATCH.

	IF	(1-N$DRVS) SHR 15
D1$DPB:	DS	DPB$SZ		;DISK PARAMETER BLOCK.
D1$ALL:	DS	ALL$SZ		;DISK ALLOCATION INFO.
D1$CHK:	DS	CHK$SZ		;DISK CHANGED SCRATCH.
	ENDIF

	IF	(2-N$DRVS) SHR 15
D2$DPB:	DS	DPB$SZ		;DISK PARAMETER BLOCK.
D2$ALL:	DS	ALL$SZ		;DISK ALLOCATION INFO.
D2$CHK:	DS	CHK$SZ		;DISK CHANGED SCRATCH.
	ENDIF

	IF	(3-N$DRVS) SHR 15
D3$DPB:	DS	DPB$SZ		;DISK PARAMETER BLOCK.
D3$ALL:	DS	ALL$SZ		;DISK ALLOCATION INFO.
D3$CHK:	DS	CHK$SZ		;DISK CHANGED SCRATCH.
	ENDIF

;******************************************************
	END

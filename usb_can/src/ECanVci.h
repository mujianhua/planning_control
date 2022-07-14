#ifndef _ECANVCI_H_

#define _ECANVCI_H_




#define uint8_t unsigned char
#define int8_t signed char
#define uint16_t unsigned short int
#define int16_t signed short int
#define uint32_t unsigned int
#define int32_t signed int
#define uint64_t unsigned long long int
#define int64_t signed long long int
#define DWORD unsigned int
#define UINT unsigned int
#define USHORT unsigned short int
#define CHAR signed char
#define UCHAR unsigned char
#define BYTE unsigned char
#define ULONG unsigned int
#define INT signed int
#define PVOID void*
#define LPVOID void*
#define HANDLE void*
#define BOOL BYTE

typedef DWORD             ECAN_RESULT;
typedef DWORD            ECAN_HANDLE;



typedef struct
{
  int32_t       id;             /* can-id                                   */
  uint8_t       len;            /* length of message: 0-8                   */
  uint8_t       msg_lost;       /* count of lost rx-messages                */
  uint8_t       reserved[2];    /* reserved                                 */
  uint8_t       data[8];        /* 8 data-bytes                             */
} CMSG;


typedef struct
{
  int32_t       evid;          /* event-id: possible range:EV_BASE...EV_LAST */
  uint8_t       len;           /* length of message: 0-8                     */
  uint8_t       reserved[3];   /* reserved                                   */
  union
  {
    uint8_t  c[8];
    uint16_t s[4];
    uint32_t l[2];
  } evdata;
} EVMSG;


typedef struct
{
  int32_t id;                   /* can-id                                    */
  uint8_t len;                  /* length of message: 0-8                    */
  uint8_t msg_lost ;            /* count of lost rx-messages                 */
  uint8_t reserved[2] ;         /* reserved                                  */
  uint8_t data[8] ;             /* 8 data-bytes                              */
  uint64_t timestamp ;          /* time stamp of this message                */
} CMSG_T;





typedef  struct  _BOARD_INFO{
		USHORT	hw_Version;
		USHORT	fw_Version;
		USHORT	dr_Version;
		USHORT	in_Version;
		USHORT	irq_Num;
		BYTE	can_Num;
		CHAR	str_Serial_Num[20];
		CHAR	str_hw_Type[40];
		USHORT	Reserved[4];
} BOARD_INFO,*P_BOARD_INFO; 


typedef  struct  _CAN_OBJ{
	UINT	ID;
	UINT	TimeStamp;
	BYTE	TimeFlag;
	BYTE	SendType;
	BYTE	RemoteFlag;
	BYTE	ExternFlag;
	BYTE	DataLen;
	BYTE	Data[8];
	BYTE	Reserved[3];
}CAN_OBJ,*P_CAN_OBJ;


typedef struct _CAN_STATUS{
	UCHAR	ErrInterrupt;
	UCHAR	regMode;
	UCHAR	regStatus;
	UCHAR	regALCapture;
	UCHAR	regECCapture; 
	UCHAR	regEWLimit;
	UCHAR	regRECounter; 
	UCHAR	regTECounter;
	DWORD	Reserved;
}CAN_STATUS,*P_CAN_STATUS;


typedef struct _ERR_INFO{
		UINT	ErrCode;
		BYTE	Passive_ErrData[3];
		BYTE	ArLost_ErrData;
} ERR_INFO,*P_ERR_INFO;


typedef struct _INIT_CONFIG{
	DWORD	AccCode;
	DWORD	AccMask;
	DWORD	Reserved;
	UCHAR	Filter;
	UCHAR	Timing0;	
	UCHAR	Timing1;	
	UCHAR	Mode;
}INIT_CONFIG,*P_INIT_CONFIG;


typedef struct _FILTER_RECORD{
	DWORD ExtFrame;	
	DWORD Start;
	DWORD End;
}FILTER_RECORD,*P_FILTER_RECORD;



//�������÷���״ֵ̬
#define	STATUS_OK					1
#define STATUS_ERR					0
#define ECAN_ERR					0

#define ECAN_SUCCESS                1




//CAN������
#define	ERR_CAN_OVERFLOW			0x0001	
#define	ERR_CAN_ERRALARM			0x0002	
#define	ERR_CAN_PASSIVE				0x0004	
#define	ERR_CAN_LOSE				0x0008	
#define	ERR_CAN_BUSERR				0x0010	
#define	ERR_CAN_REG_FULL			0x0020	
#define	ERR_CAN_REG_OVER			0x0040	
#define	ERR_CAN_ZHUDONG	    		0x0080	

//ͨ�ô�����
#define	ERR_DEVICEOPENED			0x0100	
#define	ERR_DEVICEOPEN				0x0200	
#define	ERR_DEVICENOTOPEN			0x0400	
#define	ERR_BUFFEROVERFLOW			0x0800	
#define	ERR_DEVICENOTEXIST			0x1000
#define	ERR_LOADKERNELDLL			0x2000	
#define ERR_CMDFAILED				0x4000	//�ڴ治��

#define ECAN_RX_TIMEOUT                0xE0000001
#define ECAN_TX_TIMEOUT                0xE0000002
#define ECAN_TX_ERROR                  0xE0000004



/*------------------ Defines ------------------------------------------------*/

#define ECAN_EV_BASE                   0x40000000
#define ECAN_EV_USER                   0x40000080
#define ECAN_EV_LAST                   0x400000FF

#define ECAN_EV_CAN_ERROR              ECAN_EV_BASE
#define ECAN_EV_BAUD_CHANGE    (ECAN_EV_BASE + 0x1)

#define ECAN_EV_ADD_ID_ZERO    (ECAN_EV_BASE + 0x2)
#define ECAN_EV_ADD_ID    (ECAN_EV_BASE + 0x3)
#define ECAN_EV_ADD_ID_SA    (ECAN_EV_BASE + 0x4)
#define ECAN_EV_ADD_ID_RANG    (ECAN_EV_BASE + 0x5)
#define ECAN_EV_RESET    (ECAN_EV_BASE + 0x6)
#define ECAN_EV_ADD_MASK    (ECAN_EV_BASE + 0x7)
#define ECAN_EV_Mode   (ECAN_EV_BASE + 0xc)


#define ECAN_IOCTL_GET_TIMESTAMP_FREQ    0x0007   /* Get timestamp frequency in Hz  */
#define ECAN_IOCTL_GET_TIMESTAMP         0x0008   /* Get timestamp counter   */  





#ifdef __cplusplus

extern "C"

 {

#endif 

//int __stdcall example(int value); 
//DWORD __stdcall OpenDevice(DWORD DeviceType,DWORD DeviceInd,DWORD Reserved);
//DWORD __stdcall StartCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
//DWORD __stdcall ReadErrInfo(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,P_ERR_INFO pErrInfo);

DWORD  OpenDevice(DWORD DeviceType,DWORD DeviceInd,DWORD Reserved);
DWORD  CloseDevice(DWORD DeviceType,DWORD DeviceInd);

DWORD  InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, P_INIT_CONFIG pInitConfig);

DWORD  ReadBoardInfo(DWORD DeviceType,DWORD DeviceInd,P_BOARD_INFO pInfo);
DWORD  ReadErrInfo(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,P_ERR_INFO pErrInfo);
DWORD  ReadCANStatus(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,P_CAN_STATUS pCANStatus);

DWORD  GetReference(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,DWORD RefType,PVOID pData);
DWORD  SetReference(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,DWORD RefType,PVOID pData);
DWORD  GetReceiveNum(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
DWORD  ClearBuffer(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);

DWORD  StartCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
DWORD  ResetCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
DWORD  Transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,P_CAN_OBJ pSend,ULONG Len);
DWORD  Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,P_CAN_OBJ pReceive,ULONG Len,INT WaitTime);







#ifdef __cplusplus

}     

#endif

#endif

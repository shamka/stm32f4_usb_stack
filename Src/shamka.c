#include "shamka.h"


#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h> 
#include <stdlib.h>

#define MAGIC_UARTRX_BUFF 64
#define MAGIC_HID_BUFF 64
#define MAGIC_HID_UPDATE 32*4
#define MAGIC_HID_TIMEOUT (uint32_t)(30*(1000/(MAGIC_HID_UPDATE)))

#define MAGIC_MSD_BUFF 4096


/* SCSI Commands */
#define SCSI_FORMAT_UNIT                            0x04
#define SCSI_INQUIRY                                0x12
#define SCSI_MODE_SELECT6                           0x15
#define SCSI_MODE_SELECT10                          0x55
#define SCSI_MODE_SENSE6                            0x1A
#define SCSI_MODE_SENSE10                           0x5A
#define SCSI_ALLOW_MEDIUM_REMOVAL                   0x1E
#define SCSI_READ6                                  0x08
#define SCSI_READ10                                 0x28
#define SCSI_READ12                                 0xA8
#define SCSI_READ16                                 0x88

#define SCSI_READ_CAPACITY10                        0x25
#define SCSI_READ_CAPACITY16                        0x9E

#define SCSI_REQUEST_SENSE                          0x03
#define SCSI_START_STOP_UNIT                        0x1B
#define SCSI_TEST_UNIT_READY                        0x00
#define SCSI_WRITE6                                 0x0A
#define SCSI_WRITE10                                0x2A
#define SCSI_WRITE12                                0xAA
#define SCSI_WRITE16                                0x8A

#define SCSI_VERIFY10                               0x2F
#define SCSI_VERIFY12                               0xAF
#define SCSI_VERIFY16                               0x8F

#define SCSI_SEND_DIAGNOSTIC                        0x1D
#define SCSI_READ_FORMAT_CAPACITIES                 0x23
/* SCSI ER1 */
#define NO_SENSE                                    0
#define RECOVERED_ERROR                             1
#define NOT_READY                                   2
#define MEDIUM_ERROR                                3
#define HARDWARE_ERROR                              4
#define ILLEGAL_REQUEST                             5
#define UNIT_ATTENTION                              6
#define DATA_PROTECT                                7
#define BLANK_CHECK                                 8
#define VENDOR_SPECIFIC                             9
#define COPY_ABORTED                                10
#define ABORTED_COMMAND                             11
#define VOLUME_OVERFLOW                             13
#define MISCOMPARE                                  14

/* SCSI ER2 */
#define INVALID_CDB                                 0x20
#define INVALID_FIELED_IN_COMMAND                   0x24
#define PARAMETER_LIST_LENGTH_ERROR                 0x1A
#define INVALID_FIELD_IN_PARAMETER_LIST             0x26
#define ADDRESS_OUT_OF_RANGE                        0x21
#define MEDIUM_NOT_PRESENT                          0x3A
#define MEDIUM_HAVE_CHANGED                         0x28
#define WRITE_PROTECTED                             0x27 
#define UNRECOVERED_READ_ERROR			    0x11
#define WRITE_FAULT				    0x03 

#define READ_FORMAT_CAPACITY_DATA_LEN               0x0C
#define READ_CAPACITY10_DATA_LEN                    0x08
#define MODE_SENSE10_DATA_LEN                       0x08
#define MODE_SENSE6_DATA_LEN                        0x04
#define REQUEST_SENSE_DATA_LEN                      0x12
#define STANDARD_INQUIRY_DATA_LEN                   0x24
#define BLKVFY                                      0x04

#define USBD_BOT_CBW_SIGNATURE             0x43425355
#define USBD_BOT_CSW_SIGNATURE             0x53425355
#define USBD_BOT_CBW_LENGTH                31
#define USBD_BOT_CSW_LENGTH                13

#define SENSE_LIST_DEEPTH                  4

#define msd_LUN0_CAP ((128<<(20-9))-1)

#define USBD_BOT_PREIDLE                   0
#define USBD_BOT_IDLE                      1
#define USBD_BOT_STALL_INB                 2
#define USBD_BOT_STALL_INB2                3
#define USBD_BOT_LAST_DATA_IN              4
#define USBD_BOT_ER                        5
#define USBD_BOT_DATA_IN                   6

//TYPEDEFS
typedef enum{
    SETUP=0,
    DATA=1,
    ANS=2,
} EtypeStage0;

#pragma pack(push, 1)
typedef struct{
    uint8_t *buff;
    uint32_t len;
    uint8_t epnum;
} usbTransStruct;

typedef struct {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
}usbSetup;

typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
}usbDescriptor6;
typedef struct usbStt{
    int32_t sended;
    int32_t left;
    Tcallback cb;
    Tcallback cbAfterStallClear;
    EtypeCallback type;
    uint8_t *buff;
    uint8_t enp;
} usbStt;

typedef struct{
    uint8_t reportId;
    union{
        struct {
            uint8_t changesMask;
            uint8_t changesData;
        } ;
        struct{
            uint8_t setChanges;
        } ;
        struct {
            uint8_t current;
        } ;
        struct {
            uint8_t setMask;
            uint8_t setData;
        } ;
        struct{
            uint8_t boot1;
        } ;
        struct{
            uint8_t led;
        } ;
    };
}hidReports;

typedef struct{
    uint8_t mask;
    uint8_t data;
} hidData;

typedef struct{
  uint32_t dSignature;
  uint32_t dTag;
  uint32_t dDataLength;
  uint8_t  bmFlags;
  uint8_t  bLUN;
  uint8_t  bCBLength;
  uint8_t  CB[16];
  uint8_t  ReservedForAlign;
}
USBD_MSC_BOT_CBWTypeDef;


typedef struct{
  uint32_t dSignature;
  uint32_t dTag;
  uint32_t dDataResidue;
  uint8_t  bStatus;
  uint8_t  ReservedForAlign[3];  
}
USBD_MSC_BOT_CSWTypeDef;

typedef struct _SENSE_ITEM {                
  char Skey;
  union {
    struct _ASCs {
      char ASC;
      char ASCQ;
    }b;
    unsigned int	ASC;
    char *pData;
  } w;
}
USBD_SCSI_SenseTypeDef; 
#pragma pack(pop)


//CONSTS
const uint8_t PropReq[]={
	0x42,0x00,          //2
	0x00,0x01,          //4
	0x01,0x00,0x00,0x00,//8
	0x00,0x00,0x00,0x00,//12
	0x00,0x02,0x00,0x00,//16
	0x00,0x02,0x00,0x00,//20
	0x00,0x18,0x15,0x00,//24
	0x01,0x00,0x00,0x00,//28
	0x3F,0x01,0x00,0x00,//32
	0x7F,0x00,0x00,0x00,//36
	0xF0,0xFF,0x07,0x10,//40
	0x08,0x00,          //42
	0x07,0x1F,          //44
	0x00,0x02,0x00,0x00,//48
	0x00,0x02,0x00,0x00,//52
	0x00,0x00,0x00,0x00,//56
	0x00,0x00,0x00,0x00,//60
	'3',0x00,
        '.',0x00,
        '2',0x00,
};
const uint8_t hidDesc[]={
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    
    0x85, 0x02,                    //   REPORT_ID (2)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    
    0x85, 0x03,                    //   REPORT_ID (3)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x10,                    //   REPORT_COUNT (16)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    
    0x85, 0x04,                    //   REPORT_ID (4)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x91, 0x42,                    //   OUTPUT (Data,Var,Abs,Null)
    
    0x85, 0x06,                    //   REPORT_ID (6)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x10,                    //   REPORT_COUNT (16)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    
    0x85, 0x07,                    //   REPORT_ID (7)
    0x09, 0x02,                    //   USAGE (Vendor Usage 2)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    
    0x85, 0x08,                    //   REPORT_ID (8)
    0x09, 0x02,                    //   USAGE (Vendor Usage 2)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0xc0,                           // END_COLLECTION
    
    
    
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    
    0x85, 0x15,                    //   REPORT_ID (21)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    
    0x85, 0x16,                    //   REPORT_ID (22)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x08,                    //   USAGE_MAXIMUM (Do Not Disturb)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION
};
const uint8_t conf1Desc[]={
    9,2,(9 + 9+9+7+7 + 9+7+7 + 9+7+7),0,(3),1,0,0x80,100, //CONFIG

    9,4,HID_IF,0,2,0x03,0,0,3, //INTRFACE HID_IF
    9,0x21,0x11,0x01,0x00,0x01,0x22,sizeof(hidDesc)&255,sizeof(hidDesc)>>8,
    
    7,5,HID_INT_IN,3,64,0,32,
    7,5,HID_INT_OUT,3,64,0,32,

    9,4,CDC_IF,0,2,0xFF,0,0,2, //INTRFACE CDC_IF2
    7,5,CDC_OUT,2,64,0,0,
    7,5,CDC_IN,2,64,0,0,
    
    9,4,MSD_IF,0,2,0x08,0x06,0x50,5, //INTRFACE MSD_IF3
    7,5,MSD_OUT,2,64,0,0,
    7,5,MSD_IN,2,64,0,0,
    
    
    
};
const usbDescriptor6 devDesc={18,1,0x200,0xEF,0x02,0x01,64,0x0483,0x5741,0x200,0,4,1,1};
const uint8_t STRINGS_0[]={4,3,9,4};
const uint8_t STRINGS_1[]= {2+2 *8,3,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'@',0};
const uint8_t STRINGS_2[]= {2+2*11,3,'C',0,'D',0,'C',0,'-',0,'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'4',0};
const uint8_t STRINGS_3[]= {2+2*11,3,'H',0,'I',0,'D',0,'-',0,'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'4',0};
const uint8_t STRINGS_4[]= {2+2*11,3,'B',0,'I',0,'G',0,'-',0,'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'4',0};
const uint8_t STRINGS_5[]= {2+2*11,3,'M',0,'S',0,'D',0,'-',0,'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'4',0};
const uint8_t msd_LUN0_INQUIRY[36]={
  /* LUN 0 */
  0x00,		
  0x80,		
  0x02,		
  0x02,
  (36 - 5),
  0x00,
  0x00,	
  0x00,
  'S', 'T', 'M', '3', '2', '-', 'F', '4', /* Manufacturer : 8 bytes */
  'M', 'S', 'D', ' ', 'L', 'U', 'N', '0', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1',                     /* Version      : 4 Bytes */
};
const uint8_t msd_Page00_Inquiry_Data[7] = {//7						
	0x00,		
	0x00, 
	0x00, 
	(7 - 4),
	0x00, 
	0x80, 
	0x83 
};  

//VARS
const uint8_t cdcCpErStat[0x13];
uint8_t cdcCpFlow[0x10];
uint8_t cdcCpChars[6];
uint8_t cdcCpBaudRate[4];
usbStt usbIN[4];
usbStt usbOUT[4];
EtypeStage0 ep0Stage;
int16_t LEDSTOBT=-1;
uint8_t usb_state,SCROLL_LOCK_STATE,dmaCDC;
uint8_t cdcInput[CDCINPUT_BUFF+4];
uint8_t uartInput[UARTINPUT_BUFF*2+4];
uint8_t lineCoding[16];
volatile uint8_t uartBuff,firstCDC;
extern UART_HandleTypeDef huart3;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern osTimerId statChangeHandle;
extern osThreadId usbTransHandle;
extern osMessageQId usbTransmitQueueHandle;
extern osSemaphoreId goToSendUSBHandle;

uint8_t hidInput[MAGIC_HID_BUFF+4];
hidData hiddata;
uint32_t timeout;


uint8_t msdInput[MAGIC_MSD_BUFF];
uint8_t msdOutput[MAGIC_MSD_BUFF];
USBD_MSC_BOT_CBWTypeDef msdCbw;
USBD_MSC_BOT_CSWTypeDef msdCsw;
USBD_SCSI_SenseTypeDef   scsi_sense;// [SENSE_LIST_DEEPTH];
uint8_t msd_lun_ready[1]={1};
uint8_t msdState;


//FUNCTIONS 
uint8_t shamka_setLineCoding(struct usbStt* p);
uint8_t cdc_cp_nullF(struct usbStt* p);
uint8_t hidCallback(struct usbStt* p);
uint8_t cdcCallback(struct usbStt* p);
uint8_t msdCallback(struct usbStt* p);
uint8_t shamka_setSpeed();

//USB STACK
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG_USB_STACK
    printf("RESET\r\n");
#endif
    usb_state|=1;
    timeout=0;
    osTimerStop(statChangeHandle);
    
    HAL_PCD_EP_Close(hpcd,0x80);
    HAL_PCD_EP_Close(hpcd,0x00);
    HAL_PCD_EP_Close(hpcd,0x81);
    HAL_PCD_EP_Close(hpcd,0x01);
    HAL_PCD_EP_Close(hpcd,0x82);
    HAL_PCD_EP_Close(hpcd,0x02);
    HAL_PCD_EP_Close(hpcd,0x83);
    HAL_PCD_EP_Close(hpcd,0x03);
    
    HAL_PCD_EP_Open(hpcd,0x00,64,0);
    HAL_PCD_EP_Open(hpcd,0x80,64,0);
    
    
};
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
    if(!(usb_state&1))return;
#ifdef _DEBUG_USB_STACK
    printf("SUSPEND\r\n");
#endif
    //printf("suspend STOP\r\n");
    osTimerStop(statChangeHandle);
    usb_state&=~1;
    __HAL_PCD_GATE_PHYCLOCK(hpcd);
}
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG_USB_STACK
    printf("RESUME\r\n");
#endif
    firstCDC=1;
    //printf("Resume START\r\n");
    osTimerStart(statChangeHandle,MAGIC_HID_UPDATE);
    usb_state|=1;
}
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG_USB_STACK
    printf("CONNECT\r\n");
#endif
}
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG_USB_STACK
    printf("DISCONNECT\r\n");
#endif
}
void shamkaUSBtrans(uint8_t epnum,uint8_t* buff,uint32_t len,uint8_t(*callback)(struct usbStt*),EtypeCallback type){
    epnum&=0x7f;
#ifdef _DEBUG_USB_STACK
#ifndef _DEBUG_USB0
        if(epnum!=0)
#endif
    if(len==0)
        printf("Ptrans%d: ZLP\r\n",epnum);
    else
        printf("Ptrans%d: %d\r\n",epnum,len);
#endif
    usbIN[epnum].cb=callback;
    usbIN[epnum].type=type;
    usbIN[epnum].buff=buff;
    usbIN[epnum].left=len;
    usbIN[epnum].enp=0x80|epnum;
    usbIN[epnum].sended=len;
    HAL_PCD_EP_Transmit(&hpcd_USB_OTG_FS,epnum,buff,len);
};
void shUSBtrans(uint8_t epnum,uint8_t* buff,uint32_t len){
    
    usbTransStruct *tr = (usbTransStruct*)malloc(sizeof(usbTransStruct));
    tr->buff=buff;
    tr->len=len;
    tr->epnum=epnum;
    osMessagePut(usbTransmitQueueHandle,(uint32_t)tr,osWaitForever);
};
void shamkaUSBrecv(uint8_t epnum,uint8_t* buff,uint32_t len,uint8_t(*callback)(struct usbStt*),EtypeCallback type){
    epnum&=0x7f;
#ifdef _DEBUG_USB_STACK
#ifndef _DEBUG_USB0
        if(epnum!=0)
#endif
    if(len==0){
        printf("Precv%d: ZLP\r\n",epnum);
    }else{
        printf("Precv%d: %d\r\n",epnum,len);
    }
#endif
    usbOUT[epnum].cb=callback;
    usbOUT[epnum].type=type;
    usbOUT[epnum].buff=buff;
    usbOUT[epnum].left=len;
    usbOUT[epnum].enp=epnum&0x7f;
    usbOUT[epnum].sended=0;
    HAL_PCD_EP_Receive(&hpcd_USB_OTG_FS,epnum,buff,len);
};
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
    PCD_EPTypeDef ep=hpcd->OUT_ep[epnum];
    usbStt *epS=&usbOUT[epnum];
    
    uint8_t last=ep.xfer_count!=ep.maxpacket;
    epS->left-=ep.xfer_count;
    epS->sended+=ep.xfer_count;
    
#ifdef _DEBUG_USB_STACK
#ifndef _DEBUG_USB0
        if(epnum!=0)
#endif
    if(ep.xfer_count==0)
        printf("Recv%d: ZLP\r\n",epnum);
    else
        printf("Recv%d: %d\r\n",epnum,ep.xfer_count);
#endif
    
    if(epS->left>0 && !last){
#ifdef _DEBUG_USB_STACK
#ifndef _DEBUG_USB0
        if(epnum!=0)
#endif
        printf("Precv%d: %d\r\n",epnum,epS->left);
#endif
      epS->type|=0x20;
      if((epS->cb==0) || ((epS->type&0x10) == 0) || !epS->cb(epS))
        HAL_PCD_EP_Receive(hpcd,epnum,ep.xfer_buff,epS->left);
    }
    else{
        if(epS->type&0x0f==ZLP || (last && epS->type&0x0f==ZLPF)){
#ifdef _DEBUG_USB_STACK
#ifndef _DEBUG_USB0
        if(epnum!=0)
#endif
            printf("Precv%d: ZLP\r\n",epnum);
#endif
            epS->type=(EtypeCallback)((uint8_t)(epS->type&0x10)|(uint8_t)NONE);
            HAL_PCD_EP_Receive(hpcd,epnum,0,0);
        }
        else{
            if(epnum==0){
                if(ep0Stage==DATA){
                    ep0Stage++;
                    shamkaUSBtrans(0,(uint8_t*)epS,0,epS->cb,NONE);
                    return;
                }
            }
            if(epS->cb!=0){
                epS->type&=0x1F;
                epS->cb(epS);
            }
        }
    }
    
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
    PCD_EPTypeDef ep=hpcd->IN_ep[epnum];
    usbStt *epS=&usbIN[epnum];
    
#ifdef _DEBUG_USB_STACK
#ifndef _DEBUG_USB0
        if(epnum!=0)
#endif
    if(ep.xfer_count==0){
        printf("Trans%d: ZLP\r\n",epnum);
    }
    else{
        printf("Trans%d: %d\r\n",epnum,ep.xfer_count);
    }
#endif
    epS->left-=ep.xfer_count;
    
    if(epS->left>0){
#ifdef _DEBUG_USB_STACK
#ifndef _DEBUG_USB0
        if(epnum!=0)
#endif
        printf("Ptrans%d: %d\r\n",epnum,epS->left);
#endif
      epS->type|=0x20;
      if((epS->cb==0) || (epS->type&0x10 == 0) || !epS->cb(epS))
        HAL_PCD_EP_Transmit(hpcd,epnum,ep.xfer_buff,epS->left);
    }
    else{
        if(epS->type&0x0f==ZLP || (epS->type&0x0f==ZLPF && ep.xfer_count==ep.maxpacket)){
#ifdef _DEBUG_USB_STACK
#ifndef _DEBUG_USB0
        if(epnum!=0)
#endif
            printf("Ptrans%d: ZLP\r\n",epnum);
#endif
            epS->type=(EtypeCallback)((uint8_t)(epS->type&0x10)|(uint8_t)NONE);
            HAL_PCD_EP_Transmit(hpcd,epnum,0,0);
        }
        else{
            if(epnum==0){
                if(ep0Stage==DATA){
                    ep0Stage++;
                    shamkaUSBrecv(0,(uint8_t*)epS,0,epS->cb,NONE);
                    return;
                }
            }
            if(epS->cb!=0){
                epS->type&=0x1F;
                epS->cb(epS);
            }
        }
    }
    
    
};

//SETUP USB STAGE WITH INLINES
inline static void shamkaUSbMSDSetup(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
    switch(setup->bRequest){
    case 0xFE://GET MAX LUN
      shamkaUSBtrans(0,(uint8_t*)&STRINGS_1[3],1,0,NONE);
      break;
    case 0xFF://Bulk-Only Mass Storage Reset
      HAL_PCD_EP_ClrStall(&hpcd_USB_OTG_FS,MSD_IN);
      HAL_PCD_EP_ClrStall(&hpcd_USB_OTG_FS,MSD_OUT);
      HAL_PCD_EP_Flush(&hpcd_USB_OTG_FS,MSD_IN);
      HAL_PCD_EP_Flush(&hpcd_USB_OTG_FS,MSD_OUT);
      msdState=USBD_BOT_IDLE;
      shamkaUSBtrans(0,(uint8_t*)&STRINGS_1[3],1,0,NONE);
      shamkaUSBrecv(MSD_OUT,(uint8_t*)&msdCbw,USBD_BOT_CBW_LENGTH,&msdCallback,NONE);
      break;
    }
}
inline static void shamkaUSbHIDSetup(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
    switch(setup->bRequest){
    case 0x0A://SET IDLE
        //printf("hidSetup START\r\n");
        osTimerStart(statChangeHandle,32*4);
        shamkaUSBtrans(0,0,0,0,NONE);
        break;
    }
}
inline static void usbSStandart(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
    static uint8_t* desc;
    static uint16_t len;
    switch(setup->bRequest){
    case 11:
      if(setup->wIndex==MSD_IF){
        msdState=USBD_BOT_IDLE;
        shamkaUSBrecv(MSD_OUT,(uint8_t*)&msdCbw,USBD_BOT_CBW_LENGTH,&msdCallback,NONE);
      }
      break;
    case 0://GET_STATUS
        shamkaUSBtrans(0,(uint8_t*)&STRINGS_1[3],1,0,NONE);
        return;
    case 1://CLEAR FEATURE
        if(setup->bmRequestType==2){
          HAL_PCD_EP_ClrStall(&hpcd_USB_OTG_FS,setup->wIndex);
          if(setup->wIndex&0x80){
            if(usbIN[setup->wIndex&0xf].cbAfterStallClear!=NULL)
              usbIN[setup->wIndex&0xf].cbAfterStallClear(&usbIN[setup->wIndex&0xf]);
          }else{
            if(usbOUT[setup->wIndex&0xf].cbAfterStallClear!=NULL)
              usbOUT[setup->wIndex&0xf].cbAfterStallClear(&usbOUT[setup->wIndex&0xf]);
          }
        }
        shamkaUSBtrans(0,0,0,0,NONE);
        return;
    case 5:
        //SET ADDRESS
        HAL_PCD_SetAddress(hpcd, setup->wValue);
        shamkaUSBtrans(0,0,0,0,NONE);
        return;
    case 6://GET_DESCRIPTOR
        switch(setup->wValue>>8){
        case 1:desc=(uint8_t*)&devDesc;len=sizeof(devDesc);break;
        case 2:desc=(uint8_t*)conf1Desc;len=sizeof(conf1Desc);break;
        case 3:
            switch((setup->wValue&0xff)){
            case 0:desc=(uint8_t*)STRINGS_0;len=sizeof(STRINGS_0);break;
            case 1:desc=(uint8_t*)STRINGS_1;len=sizeof(STRINGS_1);break;
            case 2:desc=(uint8_t*)STRINGS_2;len=sizeof(STRINGS_2);break;
            case 3:desc=(uint8_t*)STRINGS_3;len=sizeof(STRINGS_3);break;
            case 4:desc=(uint8_t*)STRINGS_4;len=sizeof(STRINGS_4);break;
            case 5:desc=(uint8_t*)STRINGS_5;len=sizeof(STRINGS_5);break;
            default:goto usbStall_01;
            };break;
        case 0x22:desc=(uint8_t*)&hidDesc;len=sizeof(hidDesc);break;
        default:goto usbStall_01;
        }
        shamkaUSBtrans(0,desc,MIN(len,setup->wLength),0,NONE);
        return;
    case 9://SET_CONF
        
        HAL_PCD_EP_Open(hpcd,CDC_OUT,64,2);
        HAL_PCD_EP_Open(hpcd,CDC_IN,64,2);
        //HAL_PCD_EP_Open(hpcd,CDC_INT,64,3);
        
        HAL_PCD_EP_Open(hpcd,MSD_OUT,64,2);
        HAL_PCD_EP_Open(hpcd,MSD_IN,64,2);
        usbOUT[MSD_OUT&0xf].cbAfterStallClear=usbIN[MSD_IN&0xf].cbAfterStallClear=&msdCallback;
        shamkaUSBrecv(MSD_OUT,(uint8_t*)&msdCbw,USBD_BOT_CBW_LENGTH,&msdCallback,NONE);
        msdState=USBD_BOT_IDLE;
        //scsi_sense_tail=0;
        
        HAL_PCD_EP_Open(hpcd,HID_INT_OUT,64,3);
        HAL_PCD_EP_Open(hpcd,HID_INT_IN,64,3);
        firstCDC=1;
       
        shamkaUSBrecv(HID_INT_OUT,hidInput,MAGIC_HID_BUFF,&hidCallback,NONE);
        shamkaUSBtrans(0,0,0,0,NONE);
        return;
    }
usbStall_01:
#ifdef _DEBUG_USB_STACK
    printf("STALL: 0x%02X\r\n",setup->bmRequestType&0x80);
#endif
    HAL_PCD_EP_SetStall(hpcd,setup->bmRequestType&0x80);
}
inline static void usbSClass(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
    switch(setup->bmRequestType&0x1f){
    case 1://INTERFACE
        switch(setup->wIndex){
        case MSD_IF:
            shamkaUSbMSDSetup(hpcd,setup,IN);
            break;
        case HID_IF:
            shamkaUSbHIDSetup(hpcd,setup,IN);
            break;
        case CDC_IF:
            break;
        }
        break;
    }
}
inline static void usbSVendor(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
    switch(setup->bRequest){
    case 0xFF:
      switch(setup->wValue){
      case 0x370B:
        shamkaUSBtrans(0,(uint8_t*)&conf1Desc[1],1,0,NONE);
        break;
      }
      break;
    case 0x0F://GET_PROPS
      shamkaUSBtrans(0,(uint8_t*)PropReq,MIN(setup->wLength,sizeof(PropReq)),0,NONE);
      break;
    case 0x03://SET_LINE_CTL
      lineCoding[12]=0;
      lineCoding[13]=0;
      lineCoding[14]=setup->wValue>>8;
      shamkaUSBtrans(0,0,0,0,NONE);
      break;
    case 0x13://SET_FLOW
      shamkaUSBrecv(0,cdcCpFlow,MIN(setup->wLength,sizeof(cdcCpFlow)),&cdc_cp_nullF,NONE);
      break;
    case 0x19://SET_CHARS
      shamkaUSBrecv(0,cdcCpChars,MIN(setup->wLength,sizeof(cdcCpChars)),&cdc_cp_nullF,NONE);
      break;
    case 0x1E://SET_BAUDRATE
      if(firstCDC){
          shamkaUSBrecv(CDC_OUT,cdcInput,64,&cdcCallback,NONE);
          firstCDC=0;
      };
      shamkaUSBrecv(0,&lineCoding[8],MIN(setup->wLength,4),&shamka_setLineCoding,NONE);
      break;
    case 0x00://IFC_ENABLE
      lineCoding[7]=setup->wValue&1;
      if(setup->wValue==0){
        *(uint32_t*)&lineCoding[8]=1382400;
        shamka_setSpeed();
      }
      shamkaUSBtrans(0,0,0,0,NONE);
      break;
    case 0x08://GET_MDMSTS
      shamkaUSBtrans(0,(uint8_t*)&STRINGS_1[3],1,0,NONE);
      break;
    case 0x10://GET_COMM_STATUS
      shamkaUSBtrans(0,(uint8_t*)cdcCpErStat,MIN(setup->wLength,sizeof(cdcCpErStat)),&shamka_setLineCoding,NONE);
      break;
    case 0x07://SET_MHS
      lineCoding[7]=(setup->wValue&3)|((setup->wValue>>4)&0x30);
      shamkaUSBtrans(0,0,0,0,NONE);
      break;
    case 0x12://PURGE   <<----------------------------------------------------
      shamkaUSBtrans(0,0,0,0,NONE);
      break;
    }
}
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG_USB0
    printf("\r\nbm/R: %02X %02X\r\nVIL:%04X %04X %04X\r\n\r\n",
           ((usbSetup*)hpcd->Setup)->bmRequestType, 
           ((usbSetup*)hpcd->Setup)->bRequest,
           ((usbSetup*)hpcd->Setup)->wValue,
           ((usbSetup*)hpcd->Setup)->wIndex,
           ((usbSetup*)hpcd->Setup)->wLength
               );
#endif
    ep0Stage=((usbSetup*)hpcd->Setup)->wLength>0?DATA:ANS;
    switch((((usbSetup*)hpcd->Setup)->bmRequestType>>5)&3){
    case 0://Standard
        usbSStandart(hpcd,((usbSetup*)hpcd->Setup), (((usbSetup*)hpcd->Setup)->bmRequestType&0x80)?1:0);
        break;
    case 1://Class
        usbSClass(hpcd,((usbSetup*)hpcd->Setup), (((usbSetup*)hpcd->Setup)->bmRequestType&0x80)?1:0);
        break;
    case 2://Vendor
        usbSVendor(hpcd,((usbSetup*)hpcd->Setup), (((usbSetup*)hpcd->Setup)->bmRequestType&0x80)?1:0);
        break;
    default:
        break;
    }
    
};


//HID
void hidSendRep3(){
  static uint8_t buff3[3]={3,0,0};
  buff3[1]=(hiddata.mask);
  buff3[2]=0;
  if(HAL_GPIO_ReadPin(LED_GPIO_Port,      LED_Pin))      buff3[2]|=(1<<0);
  if(HAL_GPIO_ReadPin(BOOT1_GPIO_Port,    BOOT1_Pin))    buff3[2]|=(1<<1);
  if(HAL_GPIO_ReadPin(BT_LED1_GPIO_Port,  BT_LED1_Pin))  buff3[2]|=(1<<4);
  if(HAL_GPIO_ReadPin(BT_LED2_GPIO_Port,  BT_LED2_Pin))  buff3[2]|=(1<<5);
  if(HAL_GPIO_ReadPin(BT_RESET_GPIO_Port, BT_RESET_Pin)) buff3[2]|=(1<<6);
  if(HAL_GPIO_ReadPin(BT_KEY_GPIO_Port,   BT_KEY_Pin))   buff3[2]|=(1<<7);
  shUSBtrans(HID_INT_IN,buff3,sizeof(buff3));
}
void statChangeTimer(void const * argument){
    static uint8_t oldDataValue[2]={1,0};
    timeout++;
    if(timeout>MAGIC_HID_TIMEOUT){
        timeout=0xFFFFFFFF;
        //printf("statChangeTimer STOP\r\n");
        osTimerStop(statChangeHandle);
        hidSendRep3();
        return;
    }
    if(hiddata.mask==0)return;
    hiddata.data=0;
    if((hiddata.mask&(1<<0))&& HAL_GPIO_ReadPin(LED_GPIO_Port,      LED_Pin))      hiddata.data|=(1<<0);
    if((hiddata.mask&(1<<1))&& HAL_GPIO_ReadPin(BOOT1_GPIO_Port,    BOOT1_Pin))    hiddata.data|=(1<<1);
    if((hiddata.mask&(1<<4))&& HAL_GPIO_ReadPin(BT_LED1_GPIO_Port,  BT_LED1_Pin))  hiddata.data|=(1<<4);
    if((hiddata.mask&(1<<5))&& HAL_GPIO_ReadPin(BT_LED2_GPIO_Port,  BT_LED2_Pin))  hiddata.data|=(1<<5);
    if((hiddata.mask&(1<<6))&& HAL_GPIO_ReadPin(BT_RESET_GPIO_Port, BT_RESET_Pin)) hiddata.data|=(1<<6);
    if((hiddata.mask&(1<<7))&& HAL_GPIO_ReadPin(BT_KEY_GPIO_Port,   BT_KEY_Pin))   hiddata.data|=(1<<7);
    if(oldDataValue[1]!=hiddata.data){
        oldDataValue[1]=hiddata.data;
        shUSBtrans(HID_INT_IN,oldDataValue,sizeof(oldDataValue));
    }
}
void tTimer(){
    if(timeout==0xFFFFFFFF){
        timeout=0;
        //printf("tTimer START\r\n");
        osTimerStart(statChangeHandle,MAGIC_HID_UPDATE);
    }
    timeout=0;
}
uint8_t hidCallback(struct usbStt* p){
    hidReports *report=(hidReports*)p->buff;
    switch(report->reportId){
    case 2:tTimer();
        if(p->sended!=2)break;
        hiddata.mask=report->setChanges;
        break;
    case 4:tTimer();
        if(p->sended!=2)break;
        hidSendRep3();
        break;
    case 6:tTimer();
        if(p->sended!=3)break;
        if(report->setMask&(1<<0))HAL_GPIO_WritePin(LED_GPIO_Port,      LED_Pin,     (report->setData&(1<<0)) ?GPIO_PIN_SET:GPIO_PIN_RESET);
        if(report->setMask&(1<<6))HAL_GPIO_WritePin(BT_RESET_GPIO_Port, BT_RESET_Pin,(report->setData&(1<<6)) ?GPIO_PIN_SET:GPIO_PIN_RESET);
        if(report->setMask&(1<<7))HAL_GPIO_WritePin(BT_KEY_GPIO_Port,   BT_KEY_Pin,  (report->setData&(1<<7)) ?GPIO_PIN_SET:GPIO_PIN_RESET);
        break;
    case 8:tTimer();
        if(p->sended!=2)break;
        HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,(report->led)?GPIO_PIN_SET:GPIO_PIN_RESET);
        break;
    case 22:
      if(p->sended!=2)break;
      SCROLL_LOCK_STATE = (report->boot1 & 0x04)?1:0;
      if(HAL_GPIO_ReadPin(BT_LED2_GPIO_Port,  BT_LED2_Pin)){
        LEDSTOBT=report->boot1;
      }
      break;
    }
    shamkaUSBrecv(HID_INT_OUT,p->buff,MAGIC_HID_BUFF,&hidCallback,NONE);
    return 0;
};


//CDC
uint8_t shamka_setSpeed(){
    
    huart3.Instance = USART3;
    huart3.Init.BaudRate = *(uint32_t*)&lineCoding[8];
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        huart3.Init.BaudRate = *(uint32_t*)&lineCoding[0];
        if (HAL_UART_Init(&huart3) != HAL_OK)
        {
            _Error_Handler(__FILE__, __LINE__);
        }
    }
    else{
        memcpy(&lineCoding[0],&lineCoding[8],7);
    }
#ifdef _DEBUG_USB_CDC
    printf("CDC set speed: %d\r\n",*(uint32_t*)&lineCoding[0]);
#endif
    return 0;
}
uint8_t shamka_setLineCoding(struct usbStt* p){
    p=&usbOUT[p->enp&0x0f];
    if(p->sended==7){
        HAL_UART_Abort(&huart3);
        uartBuff=0;
        shamka_setSpeed();
#ifdef _DEBUG_USB_UART
        printf("UART3 start DMA\r\n");
#endif
        HAL_UART_Receive_DMA(&huart3, &uartInput[uartBuff], 64);
    }
    else if(p->sended==4){
        HAL_UART_Abort(&huart3);
        uartBuff=0;
        shamka_setSpeed();
#ifdef _DEBUG_USB_UART
        printf("UART3 start DMA\r\n");
#endif
        HAL_UART_Receive_DMA(&huart3, &uartInput[uartBuff], 64);
    }
    return 0;
};
uint8_t cdcCallback(struct usbStt* p){
  while(HAL_UART_Transmit_DMA(&huart3,p->buff,p->sended)==HAL_BUSY){
    osDelay(64);
  }
  dmaCDC=1;
  return 0;
};
uint8_t cdc_cp_nullF(struct usbStt* p){
  return 0;
};

//USART
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(dmaCDC==1){
    dmaCDC=0;
    shamkaUSBrecv(CDC_OUT,cdcInput,64,&cdcCallback,NONE);
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(uartBuff==0){
        HAL_UART_Receive_DMA(&huart3, &uartInput[MAGIC_UARTRX_BUFF], MAGIC_UARTRX_BUFF);
        if(lineCoding[7]&0x11)shUSBtrans(CDC_IN,&uartInput[0],MAGIC_UARTRX_BUFF);
        uartBuff=MAGIC_UARTRX_BUFF;
    }
    else{
        HAL_UART_Receive_DMA(&huart3, &uartInput[0], MAGIC_UARTRX_BUFF);
        //shamkaUSBtrans(CDC_IN,&uartInput[MAGIC_UARTRX_BUFF],MAGIC_UARTRX_BUFF,0,NONE);
        if(lineCoding[7]&0x11)shUSBtrans(CDC_IN,&uartInput[MAGIC_UARTRX_BUFF],MAGIC_UARTRX_BUFF);
        uartBuff=0;
    }
#ifdef _DEBUG_USB_UART
    printf("UART3 DMA COMPLETE\r\n");
#endif

}
void HAL_UART_RxIdleCallback(UART_HandleTypeDef* huart){
    static uint16_t rxXferCount = 0;
    if(huart->RxXferSize==0)return;
    if(huart->hdmarx != NULL)
    {
        rxXferCount = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        if(rxXferCount==0){
            if(lineCoding[7]&0x11)shUSBtrans(CDC_IN,0,0);
#ifdef _DEBUG_USB_UART
            printf("UART3 IDLE 64\r\n");
#endif
            return;
        };
        
        /* Determine how many items of data have been received */
        //rxXferCount = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        HAL_UART_Abort(huart);
        huart->RxXferCount = 0;
        /* Check if a transmit process is ongoing or not */
        if(huart->gState == HAL_UART_STATE_BUSY_TX_RX)
        {
            huart->gState = HAL_UART_STATE_BUSY_TX;
        }
        else
        {
            huart->gState = HAL_UART_STATE_READY;
        }
        if(uartBuff==0){
            HAL_UART_Receive_DMA(&huart3, &uartInput[MAGIC_UARTRX_BUFF], MAGIC_UARTRX_BUFF);
            //shamkaUSBtrans(CDC_IN,&uartInput[0],rxXferCount,0,NONE);
            if(lineCoding[7]&0x11)shUSBtrans(CDC_IN,&uartInput[0],rxXferCount);
            uartBuff=MAGIC_UARTRX_BUFF;
        }
        else{
            HAL_UART_Receive_DMA(&huart3, &uartInput[0], MAGIC_UARTRX_BUFF);
            //shamkaUSBtrans(CDC_IN,&uartInput[MAGIC_UARTRX_BUFF],rxXferCount,0,NONE);
            if(lineCoding[7]&0x11)shUSBtrans(CDC_IN,&uartInput[MAGIC_UARTRX_BUFF],rxXferCount);
            uartBuff=0;
        }
#ifdef _DEBUG_USB_UART
    printf("UART3 IDLE\r\n");
#endif

    }
}

//MSD
uint8_t readToBuffFrom(uint32_t lba,uint16_t len){
  static uint32_t lbaStart,curLba;
  static uint16_t lenStart,curLen;
  if(len!=0){
    lbaStart=curLba=lba;
    lenStart=curLen=len;
  }
  
  return 0;
}
void SCSI_SenseCode(uint8_t sKey, uint8_t ASC){
  scsi_sense.Skey  = sKey;
  msdCsw.bStatus=1;
  scsi_sense.w.ASC = ASC << 8;
}
uint8_t msdCallback(struct usbStt* p){
  static uint8_t whatEP;
  static int32_t toSend=-1,toRecv=-1,rSend=-1;
  static uint32_t trans,innerSend;
  static uint8_t *send,*recv;
  
    toSend=-1;
    toRecv=-1;
  
  whatEP=msdState;
  switch(msdState){
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  case USBD_BOT_IDLE:{
    rSend=-1;
    if(msdCbw.dSignature!=USBD_BOT_CBW_SIGNATURE){
        whatEP=3;
        goto msdCallback_setStall;
    }
    msdCsw.dTag=msdCbw.dTag;
    trans=msdCbw.dDataLength;
    msdCsw.bStatus=0;
    switch(msdCbw.bLUN){
    case 0:
      break;
    default:
      SCSI_SenseCode(NOT_READY, MEDIUM_NOT_PRESENT);
      msdCsw.bStatus=1;
      if(trans>0)goto msdCallback_stallINb;
      msdCsw.dDataResidue=trans;
      goto msdCallback_dataIn;
    }
    switch(msdCbw.CB[0]){
//------------------------------------------------------------------------------
    case SCSI_TEST_UNIT_READY:{
      if(msd_lun_ready[msdCbw.bLUN]){
        toSend=0;
      }else{
        SCSI_SenseCode(NOT_READY,MEDIUM_NOT_PRESENT);
      }
      break;}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
    case SCSI_REQUEST_SENSE:{
      if(msdCbw.bLUN==0){
        memset(msdInput,0,REQUEST_SENSE_DATA_LEN);
        msdInput[0] = 0x70;		
        msdInput[7] = REQUEST_SENSE_DATA_LEN - 6;	
        msdInput[2] =  scsi_sense.Skey;		
        msdInput[12] = scsi_sense.w.b.ASCQ;	
        msdInput[13] = scsi_sense.w.b.ASC;	

        send=msdInput;
        toSend=REQUEST_SENSE_DATA_LEN;
      }
      break;}
//------------------------------------------------------------------------------
    case SCSI_READ10:{
      if(msdCbw.bLUN==0){
        if(!readToBuffFrom((msdCbw.CB[2]<<24)|(msdCbw.CB[3]<<16)|(msdCbw.CB[4]<<8)|(msdCbw.CB[5]),(msdCbw.CB[7]<<8)|(msdCbw.CB[8]))){
          send=msdInput;
          toSend=trans;
          rSend=MAGIC_MSD_BUFF;
        }
      }
      break;}
//------------------------------------------------------------------------------
    case SCSI_INQUIRY:{
      
      //trans=(msdCbw.CB[3]<<8)|(msdCbw.CB[4]);
      if(msdCbw.bLUN==0){
        if(msdCbw.CB[1]==1){
          send=(uint8_t*)msd_Page00_Inquiry_Data;
          toSend=sizeof(msd_Page00_Inquiry_Data);
        }
        else{
          send=(uint8_t*)msd_LUN0_INQUIRY;
          toSend=sizeof(msd_LUN0_INQUIRY);
        }
      }
      break;}
//------------------------------------------------------------------------------
    case SCSI_READ_FORMAT_CAPACITIES:{
      if(msdCbw.bLUN==0){
        *((uint32_t*)&msdInput[0])=8<<24;
        msdInput[4]=(uint8_t)(msd_LUN0_CAP>>24);
        msdInput[5]=(uint8_t)(msd_LUN0_CAP>>16);
        msdInput[6]=(uint8_t)(msd_LUN0_CAP>>8);
        msdInput[7]=(uint8_t)(msd_LUN0_CAP>>0);
        *((uint32_t*)&msdInput[8])=0x00020002;

        send=msdInput;
        toSend=12;
      }
      break;}
//------------------------------------------------------------------------------
    case SCSI_READ_CAPACITY10:{
      if(msdCbw.bLUN==0){
        msdInput[0]=(uint8_t)(msd_LUN0_CAP>>24);
        msdInput[1]=(uint8_t)(msd_LUN0_CAP>>16);
        msdInput[2]=(uint8_t)(msd_LUN0_CAP>>8);
        msdInput[3]=(uint8_t)(msd_LUN0_CAP>>0);
        *((uint32_t*)&msdInput[4])=0x00020000;

        send=msdInput;
        toSend=8;
      }
      break;}
//------------------------------------------------------------------------------
    case SCSI_MODE_SENSE6:{
      if(msdCbw.bLUN==0){
        send=msdInput;
        memset(msdInput,0,8);
        toSend=8;
      }
      break;}
//------------------------------------------------------------------------------
    case SCSI_START_STOP_UNIT:
    case SCSI_ALLOW_MEDIUM_REMOVAL:{
      if(msdCbw.bLUN==0){
        toSend=0;
      }
      break;}
//------------------------------------------------------------------------------
    default:
      SCSI_SenseCode(/*msdCbw.bLUN,*/ILLEGAL_REQUEST,INVALID_CDB);
    }
    break;
  }
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  case USBD_BOT_LAST_DATA_IN:{
msdCallback_dataIn:
    rSend=-1;
    send=(uint8_t*)&msdCsw;
    toSend=USBD_BOT_CSW_LENGTH;
    msdState=USBD_BOT_PREIDLE;
    break;}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  case USBD_BOT_DATA_IN:{
    toSend=innerSend;
    switch(msdCbw.CB[0]){
//------------------------------------------------------------------------------
    case SCSI_READ10:{
      if(msdCbw.bLUN==0){
        if(!readToBuffFrom(0,0)){
          send=msdInput;
        }
      }
      break;}
//------------------------------------------------------------------------------
      
    }
    break;}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  case USBD_BOT_PREIDLE:
//msdCallback_preIdle:
    msdState=USBD_BOT_IDLE;
    recv=(uint8_t*)&msdCbw;
    toRecv=USBD_BOT_CBW_LENGTH;
    break;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  case USBD_BOT_STALL_INB:
msdCallback_stallINb:
    msdState=USBD_BOT_LAST_DATA_IN;
    whatEP=1;
    goto msdCallback_setStall;
    break;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//  case USBD_BOT_ER:
//msdCallback_erINb:
//    msdCsw.bStatus=1;
//    msdCsw.dDataResidue=trans;
//    send=(uint8_t*)&msdCsw;
//    toSend=USBD_BOT_CSW_LENGTH;
//    msdState=USBD_BOT_PREIDLE;
//    break;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//  case USBD_BOT_STALL_INB2:
//    msdState=USBD_BOT_IDLE;
//    recv=(uint8_t*)&msdCbw;
//    toRecv=USBD_BOT_CBW_LENGTH;
//
//    break;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  }
  if(msdCsw.bStatus==1 && whatEP==USBD_BOT_IDLE){
    whatEP=1;
    if(trans>0)goto msdCallback_stallINb;
    goto msdCallback_tr0send0;
  }
  if(trans==0 && toSend==0){
msdCallback_tr0send0:
    whatEP=USBD_BOT_LAST_DATA_IN;
    goto msdCallback_dataIn;
  }
  else if(toSend>=0){
    if(whatEP==USBD_BOT_IDLE){
      msdCsw.dDataResidue=trans-toSend;
      if(trans==toSend){
        if(rSend==-1 || toSend<=rSend){
          msdState=USBD_BOT_LAST_DATA_IN;
        }else{
          msdState=USBD_BOT_DATA_IN;
          innerSend=toSend-rSend;
          toSend=rSend;
        }
        msdCsw.bStatus=0;
      }
      else if(trans>toSend){
        msdState=USBD_BOT_STALL_INB;
        msdCsw.bStatus=0;
      }
      else{
        toSend=trans;
        msdState=USBD_BOT_LAST_DATA_IN;
        msdCsw.bStatus=2;
      }
    }
    else if(whatEP==USBD_BOT_DATA_IN){
      toSend=MIN(rSend,innerSend);
      innerSend-=toSend;
      if(innerSend<=rSend){
        msdState=USBD_BOT_LAST_DATA_IN;
      }
    }
    
    shamkaUSBtrans(MSD_IN,send,toSend,&msdCallback,NONE);
  }
  if(toRecv>=0)shamkaUSBrecv(MSD_OUT,recv,MIN(MAGIC_MSD_BUFF,toRecv),&msdCallback,NONE);
  return 0;
msdCallback_setStall:
  if(whatEP&1)HAL_PCD_EP_SetStall(&hpcd_USB_OTG_FS,MSD_IN);
  if(whatEP&2){
    HAL_PCD_EP_SetStall(&hpcd_USB_OTG_FS,MSD_OUT);
  }
  if(whatEP&4){
    shamkaUSBrecv(p->enp,(uint8_t*)&msdCbw,USBD_BOT_CBW_LENGTH,&msdCallback,NONE);
  }
  return 0;
};

//FREERTOS
void StartDefaultTask(void const * argument){
    static uint32_t scrollOn=4;
    static const uint8_t buffSCON[9]={21,0,0,0x47,0,0,0,0,0};
    static const uint8_t buffSCOFF[9]={21,0,0,0,0,0,0,0,0};
    
    static char btLedsSend[]="NUM: 0\r\nCAPS: 0\r\nSCROLL: 0\r\n\r\n";
    
    memset(lineCoding,0,sizeof(lineCoding));
    memset(&hiddata,0,sizeof(hiddata));
    memset(&usbIN,0,sizeof(usbIN));
    memset(&usbOUT,0,sizeof(usbOUT));
    do{
      scrollOn--;
      usbIN[scrollOn].enp=0x80|scrollOn;
      usbOUT[scrollOn].enp=scrollOn;
    }while(scrollOn>0);
    
    msdCsw.dSignature=USBD_BOT_CSW_SIGNATURE;

    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x30);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x30);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 0x30);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 3, 0x30);
    HAL_PCD_Start(&hpcd_USB_OTG_FS);
    
    for(;;){
      osDelay(1000);
      if(SCROLL_LOCK_STATE){
        scrollOn++;
        if(scrollOn>30){
          osDelay(50);
          shUSBtrans(HID_INT_IN,(uint8_t*)buffSCON,sizeof(buffSCON));
          osDelay(100);
          shUSBtrans(HID_INT_IN,(uint8_t*)buffSCOFF,sizeof(buffSCOFF));
          osDelay(1000);
          shUSBtrans(HID_INT_IN,(uint8_t*)buffSCON,sizeof(buffSCON));
          osDelay(100);
          shUSBtrans(HID_INT_IN,(uint8_t*)buffSCOFF,sizeof(buffSCOFF));
          scrollOn=0;
        }
      }
      else{
        scrollOn=0;
      }
      if(LEDSTOBT!=-1){
        btLedsSend[5]=(LEDSTOBT&1)?0x31:0x30;
        btLedsSend[14]=(LEDSTOBT&2)?0x31:0x30;
        btLedsSend[25]=(LEDSTOBT&4)?0x31:0x30;
        if(HAL_UART_Transmit_DMA(&huart3,(uint8_t*)btLedsSend,sizeof(btLedsSend))!=HAL_BUSY){
          dmaCDC=0;
          LEDSTOBT=-1;
        }
      }
    }
}


usbTransStruct *foFree;
uint8_t memFreeAfterSend(struct usbStt* p){
  if(foFree!=NULL){
    free(foFree);
    foFree=NULL;
  }
  osSemaphoreRelease(goToSendUSBHandle);
  return 0;
}
void usbTransmit(void const * argument){
  for(;;)
  {
    if((osSemaphoreWait(goToSendUSBHandle,5000)==0) && (foFree!=NULL)){
      free(foFree);
#ifdef _DEBUG_USB_CDC
      printf("SORRY CDC IN TIMEOUT");
#endif
      foFree=NULL;
    };
    osEvent q = osMessageGet(usbTransmitQueueHandle,osWaitForever);
    foFree = (usbTransStruct*)q.value.p;
    shamkaUSBtrans(foFree->epnum,foFree->buff,foFree->len,&memFreeAfterSend,NONE);
  }
}
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
    uint8_t *buff;
    EtypeCallback type;
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
#pragma pack(pop)


//CONSTS
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
    9,2,(9+8+ 9+5+4+5+5+7 + 9+7+7 + 9+(9)+7+7),0,(3),1,0,0x80,100, //CONFIG

    9,4,HID_IF,0,2,0x03,0,0,3, //INTRFACE HID_IF
    9,0x21,0x11,0x01,0x00,0x01,0x22,sizeof(hidDesc)&255,sizeof(hidDesc)>>8,
    
    7,5,HID_INT_IN,3,64,0,32,
    7,5,HID_INT_OUT,3,64,0,32,


    8,11,CDC_IF1,2,2,2,0,2,        //IAD
    9,4,CDC_IF1,0,1,2,2,1,0,     //INTERFACE CDC_IF1
    5,0x24,0,0x10,0x01,
    4,0x24,2,2,
    5,0x24,6,0,1,
    5,0x24,1,3,1,
    7,5,CDC_INT,3,64,0,0xFF,
    
    9,4,CDC_IF2,0,2,0x0A,0,0,0, //INTRFACE CDC_IF2
    7,5,CDC_OUT,2,64,0,0,
    7,5,CDC_IN,2,64,0,0,
    
};
const usbDescriptor6 devDesc={18,1,0x200,0xEF,0x02,0x01,64,0x0483,0x5741,0x200,0,4,1,1};
const uint8_t STRINGS_0[]={4,3,9,4};
const uint8_t STRINGS_1[]= {2+2 *8,3,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'!',0};
const uint8_t STRINGS_2[]= {2+2*11,3,'V',0,'C',0,'P',0,'-',0,'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'4',0};
const uint8_t STRINGS_3[]= {2+2*11,3,'H',0,'I',0,'D',0,'-',0,'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'4',0};
const uint8_t STRINGS_4[]= {2+2*11,3,'B',0,'I',0,'G',0,'-',0,'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'4',0};

//VARS
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

//FUNCTIONS 
void shamka_setLineCoding(struct usbStt* p);
void cdcCallback(struct usbStt* p);
void cdcCallback2(struct usbStt* p);


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
    HAL_PCD_EP_Close(hpcd,CDC_OUT);
    HAL_PCD_EP_Close(hpcd,CDC_IN);
    HAL_PCD_EP_Close(hpcd,CDC_INT);
    HAL_PCD_EP_Close(hpcd,HID_INT_OUT);
    HAL_PCD_EP_Close(hpcd,HID_INT_IN);
    
    HAL_PCD_EP_Open(hpcd,0x00,64,0);
    HAL_PCD_EP_Open(hpcd,0x80,64,0);
    
    HAL_PCD_EP_Open(hpcd,CDC_OUT,64,2);
    HAL_PCD_EP_Open(hpcd,CDC_IN,64,2);
    HAL_PCD_EP_Open(hpcd,CDC_INT,64,3);
    
    HAL_PCD_EP_Open(hpcd,HID_INT_OUT,64,3);
    HAL_PCD_EP_Open(hpcd,HID_INT_IN,64,3);
    firstCDC=1;
    
};
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
    if(!(usb_state&1))return;
#ifdef _DEBUG_USB_STACK
    printf("SUSPEND\r\n");
#endif
    osTimerStop(statChangeHandle);
    usb_state&=~1;
    __HAL_PCD_GATE_PHYCLOCK(hpcd);
}
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG_USB_STACK
    printf("RESUME\r\n");
#endif
    firstCDC=1;
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
void shamkaUSBtrans(uint8_t epnum,uint8_t* buff,uint32_t len,void(*callback)(struct usbStt*),EtypeCallback type){
    epnum&=0x7f;
#ifdef _DEBUG_USB_STACK
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
void shamkaUSBrecv(uint8_t epnum,uint8_t* buff,uint32_t len,void(*callback)(struct usbStt*),EtypeCallback type){
    epnum&=0x7f;
#ifdef _DEBUG_USB_STACK
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
    if(ep.xfer_count==0)
        printf("Recv%d: ZLP\r\n",epnum);
    else
        printf("Recv%d: %d\r\n",epnum,ep.xfer_count);
#endif
    
    if(epS->left>0 && !last){
#ifdef _DEBUG_USB_STACK
        printf("Precv%d: %d\r\n",epnum,epS->left);
#endif
        HAL_PCD_EP_Receive(hpcd,epnum,ep.xfer_buff,epS->left);
    }
    else{
        if(epS->type==ZLP || (last && epS->type==ZLPF)){
#ifdef _DEBUG_USB_STACK
            printf("Precv%d: ZLP\r\n",epnum);
#endif
            epS->type=NONE;
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
                epS->cb(epS);
            }
        }
    }
    
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
    PCD_EPTypeDef ep=hpcd->IN_ep[epnum];
    usbStt *epS=&usbIN[epnum];
    
#ifdef _DEBUG_USB_STACK
    if(ep.xfer_count==0)
        printf("Trans%d: ZLP\r\n",epnum);
    else
        printf("Trans%d: %d\r\n",epnum,ep.xfer_count);
#endif
    epS->left-=ep.xfer_count;
    
    if(epS->left>0){
#ifdef _DEBUG_USB_STACK
        printf("Ptrans%d: %d\r\n",epnum,epS->left);
#endif
        HAL_PCD_EP_Transmit(hpcd,epnum,ep.xfer_buff,epS->left);
    }
    else{
        if(epS->type==ZLP || (epS->type==ZLPF && ep.xfer_count==ep.maxpacket)){
#ifdef _DEBUG_USB_STACK
            printf("Ptrans%d: ZLP\r\n",epnum);
#endif
            epS->type=NONE;
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
                epS->cb(epS);
            }
        }
    }
    
    
};

//SETUP USB STAGE WITH INLINES
inline static void shamkaUSbHIDSetup(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
    switch(setup->bRequest){
    case 0x0A://SET IDLE
        osTimerStart(statChangeHandle,32*4);
        shamkaUSBtrans(0,0,0,0,NONE);
        break;
    }
}
inline static void shamkaUSbCDCSetup(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
    switch(setup->bRequest){
    case 0x20://SET LINE CODING
        shamkaUSBrecv(0,&lineCoding[8],MIN(7,setup->wLength),&shamka_setLineCoding,NONE);
        break;
    case 0x21://GET LINE CODING
        shamkaUSBtrans(0,lineCoding,MIN(7,setup->wLength),0,NONE);
        break;
    case 0x22://SET CONTROL LINE STATE
        lineCoding[7]=(uint8_t)setup->wValue;
        //HAL_GPIO_WritePin(BT_KEY_GPIO_Port,BT_KEY_Pin,lineCoding[7]&1?GPIO_PIN_SET:GPIO_PIN_RESET);
        //HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,lineCoding[7]&1?GPIO_PIN_SET:GPIO_PIN_RESET);
        shamkaUSBtrans(0,0,0,0,NONE);
        if(firstCDC){
          shamkaUSBrecv(CDC_OUT,cdcInput,64,&cdcCallback,NONE);
          firstCDC=0;
        };
        break;
    }
}
inline static void usbSStandart(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
    uint8_t* desc;
    uint16_t len;
    switch(setup->bRequest){
    case 1://CLEAR FEATURE
        shamkaUSBtrans(0,0,0,0,NONE);
        
        //shamkaUSBtrans(HID_INT_IN,&lineCoding[15],1,0,NONE);
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
            default:goto usbStall_01;
            };break;
        case 0x22:desc=(uint8_t*)&hidDesc;len=sizeof(hidDesc);break;
        default:goto usbStall_01;
        }
        shamkaUSBtrans(0,desc,MIN(len,setup->wLength),0,NONE);
        return;
    case 9://SET_CONF
        
        shamkaUSBrecv(HID_INT_OUT,hidInput,MAGIC_HID_BUFF,&cdcCallback2,NONE);
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
        case CDC_IF1:
        case CDC_IF2:
            shamkaUSbCDCSetup(hpcd,setup,IN);
            break;
        case HID_IF:
            shamkaUSbHIDSetup(hpcd,setup,IN);
            break;
        }
        break;
    }
}
inline static void usbSVendor(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
    
}
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG_USB_STACK
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
        osTimerStart(statChangeHandle,MAGIC_HID_UPDATE);
    }
    timeout=0;
}
void cdcCallback2(struct usbStt* p){
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
    shamkaUSBrecv(HID_INT_OUT,p->buff,MAGIC_HID_BUFF,&cdcCallback2,NONE);
};


//CDC
void shamka_setSpeed(){
    
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
}
void shamka_setLineCoding(struct usbStt* p){
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
};
void cdcCallback(struct usbStt* p){
  while(HAL_UART_Transmit_DMA(&huart3,p->buff,p->sended)==HAL_BUSY){
    osDelay(64);
  }
  dmaCDC=1;
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
        shUSBtrans(CDC_IN,&uartInput[0],MAGIC_UARTRX_BUFF);
        uartBuff=MAGIC_UARTRX_BUFF;
    }
    else{
        HAL_UART_Receive_DMA(&huart3, &uartInput[0], MAGIC_UARTRX_BUFF);
        //shamkaUSBtrans(CDC_IN,&uartInput[MAGIC_UARTRX_BUFF],MAGIC_UARTRX_BUFF,0,NONE);
        shUSBtrans(CDC_IN,&uartInput[MAGIC_UARTRX_BUFF],MAGIC_UARTRX_BUFF);
        uartBuff=0;
    }
#ifdef _DEBUG_USB_UART
    printf("UART3 DMA COMPLETE\r\n");
#endif

}
void HAL_UART_RxIdleCallback(UART_HandleTypeDef* huart){
    static uint16_t rxXferCount = 0;
    if(huart->hdmarx != NULL)
    {
        rxXferCount = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        if(rxXferCount==0){
            shUSBtrans(CDC_IN,0,0);
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
            shUSBtrans(CDC_IN,&uartInput[0],rxXferCount);
            uartBuff=MAGIC_UARTRX_BUFF;
        }
        else{
            HAL_UART_Receive_DMA(&huart3, &uartInput[0], MAGIC_UARTRX_BUFF);
            //shamkaUSBtrans(CDC_IN,&uartInput[MAGIC_UARTRX_BUFF],rxXferCount,0,NONE);
            shUSBtrans(CDC_IN,&uartInput[MAGIC_UARTRX_BUFF],rxXferCount);
            uartBuff=0;
        }
#ifdef _DEBUG_USB_UART
    printf("UART3 IDLE\r\n");
#endif

    }
}


//FREERTOS
void StartDefaultTask(void const * argument){
    static uint32_t scrollOn=0;
    static uint8_t buffSCON[9]={21,0,0,0x47,0,0,0,0,0};
    static uint8_t buffSCOFF[9]={21,0,0,0,0,0,0,0,0};
    
    static char btLedsSend[]="NUM: 0\r\nCAPS: 0\r\nSCROLL: 0\r\n\r\n";
    
    memset(lineCoding,0,sizeof(lineCoding));
    memset(&hiddata,0,sizeof(hiddata));

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
          shUSBtrans(HID_INT_IN,buffSCON,sizeof(buffSCON));
          osDelay(100);
          shUSBtrans(HID_INT_IN,buffSCOFF,sizeof(buffSCOFF));
          osDelay(1000);
          shUSBtrans(HID_INT_IN,buffSCON,sizeof(buffSCON));
          osDelay(100);
          shUSBtrans(HID_INT_IN,buffSCOFF,sizeof(buffSCOFF));
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
void memFreeAfterSend(struct usbStt* p){
    free(foFree);
    osSemaphoreRelease(goToSendUSBHandle);
}
void usbTransmit(void const * argument)
{
  for(;;)
  {
    osSemaphoreWait(goToSendUSBHandle,osWaitForever);
    osEvent q = osMessageGet(usbTransmitQueueHandle,osWaitForever);
    foFree = (usbTransStruct*)q.value.p;
    shamkaUSBtrans(foFree->epnum,foFree->buff,foFree->len,&memFreeAfterSend,NONE);
  }
}
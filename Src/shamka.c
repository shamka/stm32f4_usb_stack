#include "shamka.h"

//VARS
usbStt usbIN[4];
usbStt usbOUT[4];
EtypeStage0 ep0Stage;
uint8_t usb_state;
uint8_t cdcInput[CDCINPUT_BUFF+4];
uint8_t uartInput[UARTINPUT_BUFF*2+4];
uint8_t lineCoding[16];
uint8_t uartStart,uartEnd,uartStop;
extern UART_HandleTypeDef huart3;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern osMutexId uartCdcHandle;

//CONSTS
const uint8_t hidDesc[]={
0x05,0x01,
0x15,0x00,
0x09,0x04,
0xa1,0x01,
0x05,0x02,
0x09,0xbb,
0x15,0x81,
0x25,0x7f,
0x75,0x08,
0x95,0x01,
0x81,0x02,
0x05,0x01,
0x09,0x01,
0xa1,0x00,
0x09,0x30,
0x09,0x31,
0x95,0x02,
0x81,0x02,
0xc0,0x09,
0x39,0x15,
0x00,0x25,
0x30,0x35,
0x00,0x46,
0x0e,0x01,
0x65,0x14,
0x75,0x04,
0x95,0x01,
0x81,0x02,
0x05,0x09,
0x19,0x01,
0x29,0x04,
0x15,0x00,
0x25,0x01,
0x75,0x01,
0x95,0x04,
0x55,0x00,
0x65,0x00,
0x81,0x02,
0xc0,
};
const uint8_t conf1Desc[]={
  9,2,(9+8+ 9+5+4+5+5+7 + 9+7+7 + 9+(9)+7+7),0,(3),1,0,0x80,100, //CONFIG
    8,11,0,2,2,2,0,2,        //IAD
      9,4,0,0,1,2,2,1,0,     //INTERFACE 0
        5,0x24,0,0x10,0x01,
        4,0x24,2,2,
        5,0x24,6,0,1,
        5,0x24,1,3,1,
        7,5,CDC_INT,3,64,0,0xFF,
        
      9,4,1,0,2,0x0A,0,0,0, //INTRFACE 1
        7,5,CDC_OUT,2,64,0,0,
        7,5,CDC_IN,2,64,0,0,
        
      9,4,2,0,2,0x03,0,0,3, //INTRFACE 2
        9,0x21,0x11,0x01,0x00,0x01,0x22,sizeof(hidDesc)&255,sizeof(hidDesc)>>8,
        
        7,5,HID_INT_IN,3,64,0,32,
        7,5,HID_INT_OUT,3,64,0,32,
};
const usbDescriptor6 devDesc={18,1,0x200,0xEF,0x02,0x01,64,0x0483,0x5741,0x200,0,0,1,1};
const uint8_t STRINGS_0[]={4,3,9,4};
const uint8_t STRINGS_1[]= {2+2 *8,3,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'0',0,'@',0};
const uint8_t STRINGS_2[]= {2+2*11,3,'V',0,'C',0,'P',0,'-',0,'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'4',0};
const uint8_t STRINGS_3[]= {2+2*11,3,'H',0,'I',0,'D',0,'-',0,'S',0,'T',0,'M',0,'3',0,'2',0,'F',0,'4',0};


//FUNCTIONS 



//USB STACK
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG
  printf("RESET\r\n");
#endif
  usb_state|=1;

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
  

      
};
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd){
  if(!(usb_state&1))return;
#ifdef _DEBUG
  printf("SUSPEND\r\n");
#endif
  usb_state&=~1;
  __HAL_PCD_GATE_PHYCLOCK(hpcd);
}
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG
  printf("RESUME\r\n");
#endif
  usb_state|=1;
}
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG
  printf("CONNECT\r\n");
#endif
}
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG
  printf("DISCONNECT\r\n");
#endif
}
void shamkaUSBtrans(uint8_t epnum,uint8_t* buff,uint32_t len,void(*callback)(struct usbStt*),EtypeCallback type){
  epnum&=0x7f;
#ifdef _DEBUG
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
void shamkaUSBrecv(uint8_t epnum,uint8_t* buff,uint32_t len,void(*callback)(struct usbStt*),EtypeCallback type){
  epnum&=0x7f;
#ifdef _DEBUG
  if(len==0)
  printf("Precv%d: ZLP\r\n",epnum);
    else
  printf("Precv%d: %d\r\n",epnum,len);
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
  
#ifdef _DEBUG
  if(ep.xfer_count==0)
  printf("Recv%d: ZLP\r\n",epnum);
    else
  printf("Recv%d: %d\r\n",epnum,ep.xfer_count);
#endif

  if(epS->left>0 && !last){
#ifdef _DEBUG
    printf("Precv%d: %d\r\n",epnum,epS->left);
#endif
    HAL_PCD_EP_Receive(hpcd,epnum,ep.xfer_buff,epS->left);
  }
  else{
    if(epS->type==ZLP || (last && epS->type==ZLPF)){
#ifdef _DEBUG
      printf("Precv%d: ZLP\r\n",epnum);
#endif
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
        //FREERTOS
        //if(osMessagePut(myUSBcallbacksHandle,(uint32_t)epS,10)!=osOK)printf("HAL_PCD_DataOutStageCallback FREERTOS addToQueue ERROR\r\n");
        epS->cb(epS);
      }
    }
  }
  
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum){
  PCD_EPTypeDef ep=hpcd->IN_ep[epnum];
  usbStt *epS=&usbIN[epnum];

#ifdef _DEBUG
  if(ep.xfer_count==0)
  printf("Trans%d: ZLP\r\n",epnum);
    else
  printf("Trans%d: %d\r\n",epnum,ep.xfer_count);
#endif
  epS->left-=ep.xfer_count;
  
  if(epS->left>0){
#ifdef _DEBUG
    printf("Ptrans%d: %d\r\n",epnum,epS->left);
#endif
    HAL_PCD_EP_Transmit(hpcd,epnum,ep.xfer_buff,epS->left);
  }
  else{
    if(epS->type==ZLP || (epS->type==ZLPF && ep.xfer_count==ep.maxpacket)){
#ifdef _DEBUG
      printf("Ptrans%d: ZLP\r\n",epnum);
#endif
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
        //FREERTOS
        //if(osMessagePut(myUSBcallbacksHandle,(uint32_t)epS,10)!=osOK)printf("HAL_PCD_DataInStageCallback FREERTOS addToQueue ERROR\r\n");
        epS->cb(epS);
      }
    }
  }
  
  
};

//SETUP USB STAGE WITH INLINES
inline static void shamkaUSbHIDSetup(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
  switch(setup->bRequest){
  case 0x0A://SET IDLE
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
    shamkaUSBrecv(CDC_OUT,cdcInput,64,&cdcCallback,NONE);
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
      default:goto usbStall_01;
      };break;
    case 0x22:desc=(uint8_t*)&hidDesc;len=sizeof(hidDesc);break;
    default:goto usbStall_01;
    }
    shamkaUSBtrans(0,desc,MIN(len,setup->wLength),0,NONE);
    return;
  case 9://SET_CONF
    shamkaUSBtrans(0,0,0,0,NONE);
    return;
  }
usbStall_01:
#ifdef _DEBUG
  printf("STALL: 0x%02X\r\n",setup->bmRequestType&0x80);
#endif
  HAL_PCD_EP_SetStall(hpcd,setup->bmRequestType&0x80);
}
inline static void usbSClass(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){
  switch(setup->bmRequestType&0x1f){
  case 1://INTERFACE
    switch(setup->wIndex){
    case 0:
    case 1:
      shamkaUSbCDCSetup(hpcd,setup,IN);
      break;
    case 2:
      shamkaUSbHIDSetup(hpcd,setup,IN);
      break;
    }
    break;
  }
}
inline static void usbSVendor(PCD_HandleTypeDef *hpcd, usbSetup *setup, uint8_t IN){

}
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd){
#ifdef _DEBUG
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
#ifdef _DEBUG
  printf("CDC set speed: %d\r\n",*(uint32_t*)&lineCoding[0]);
#endif
}
void shamka_setLineCoding(struct usbStt* p){
  p=&usbOUT[p->enp&0x0f];
  if(p->sended==7){
    HAL_UART_AbortReceive_IT(&huart3);
    shamka_setSpeed();
    uartStart=uartEnd=uartStop=0;
    HAL_UART_Receive_IT(&huart3, &uartInput[uartStart], 1);
  }
};
void cdcCallback(struct usbStt* p){
  HAL_UART_Transmit_IT(&huart3,p->buff,p->sended);
};
void cdcCallback2(struct usbStt* p){
  int16_t len;
  uartEnd+=p->sended;
  p->sended=0;
  if(uartEnd>=sizeof(uartInput)-4)
    uartEnd-=(sizeof(uartInput)-4);
  
  len = uartStart - uartEnd;
  if(len==0 || !uartStop){
    if(len>0){
      shamkaUSBtrans(CDC_IN,&uartInput[uartEnd],len,&cdcCallback2,NONE);
    }
    else if(len<0){
      shamkaUSBtrans(CDC_IN,&uartInput[uartEnd],sizeof(uartInput)-4 - uartEnd,&cdcCallback2,NONE);
    }
    else{
      osSemaphoreRelease(uartCdcHandle);
    }
  }
  else{
    
  }
  
};


//USART

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  shamkaUSBrecv(CDC_OUT,cdcInput,64,&cdcCallback,NONE);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  uartStart++;
  if(uartStart>=UARTINPUT_BUFF*2)uartStart=0;
  if(uartStart!=uartEnd){
    HAL_UART_Receive_IT(&huart3, &uartInput[uartStart], 1);
  }
  else uartStop=1;
  
}

//FREERTOS
void uartToCdc(void const * argument){
  for(;;){
    osDelay(50);
    osSemaphoreWait(uartCdcHandle,osWaitForever);
    cdcCallback2(&usbIN[CDC_IN&0x7f]);
  }
};
void StartDefaultTask(void const * argument){
  static uint8_t leds=0xff,oldLeds=0;
  memset(lineCoding,0,sizeof(lineCoding));
   
  HAL_PCD_Start(&hpcd_USB_OTG_FS);
  HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 0x40);
  HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 3, 0x40);

  osDelay(3000);
  
  
  for(;;){
    osDelay(1000);
    leds=HAL_GPIO_ReadPin(LED_GPIO_Port,LED_Pin)?1:0;
    leds|=HAL_GPIO_ReadPin(LED_GPIO_Port,LED_Pin)?1<<1:0;
    leds|=HAL_GPIO_ReadPin(BOOT1_GPIO_Port,BOOT1_Pin)?1<<2:0;
    if(oldLeds!=leds){
      oldLeds=leds;
      //shamkaUSBtrans(HID_INT_IN,(uint8_t*)&devDesc,1,0,NONE);
    }
  }
}

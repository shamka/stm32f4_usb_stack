#ifndef __shamka_H
#define __shamka_H

#define MIN(a,b) a<b?a:b
#define MAX(a,b) a>b?a:b
#define _DEBUG


#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <string.h>

#define CDCINPUT_BUFF 64
#define UARTINPUT_BUFF 64

#define CDC_IN 0x83
#define CDC_OUT 0x03
#define CDC_INT 0x81
#define HID_INT_IN 0x82
#define HID_INT_OUT 0x02



//extern uint8_t cdcInput[CDCINPUT_BUFF+4];
//extern uint8_t uartInput[UARTINPUT_BUFF*2+4];
//
//extern uint8_t lineCoding[16];
//extern UART_HandleTypeDef huart3;
//extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
//extern uint8_t uartStart,uartEnd,uartStop;

//TYPEDEFS
struct usbStt;
typedef void (*Tcallback)(struct usbStt*);

typedef enum{
  NONE=0,
  ZLPF=1,
  ZLP=2,
}EtypeCallback;
typedef enum{
  SETUP=0,
  DATA=1,
  ANS=2,
}EtypeStage0;
#pragma pack(push, 1)
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
#pragma pack(pop)


//PROTO FUNCTIONS

void shamka_setLineCoding(struct usbStt* p);
void cdcCallback(struct usbStt* p);
void cdcCallback2(struct usbStt* p);


void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);

void shamkaUSBtrans(uint8_t epnum,uint8_t* buff,uint32_t len,void(*callback)(struct usbStt*),EtypeCallback type);
void shamkaUSBrecv(uint8_t epnum,uint8_t* buff,uint32_t len,void(*callback)(struct usbStt*),EtypeCallback type);








#endif

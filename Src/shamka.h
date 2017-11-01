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
#define UARTINPUT_BUFF 2560

#define CDC_IN 0x83
#define CDC_OUT 0x03
#define CDC_INT 0x81
#define HID_INT_IN 0x82
#define HID_INT_OUT 0x02


//TYPEDEFS
struct usbStt;
typedef void (*Tcallback)(struct usbStt*);

typedef enum{
  NONE=0,
  ZLPF=1,
  ZLP=2,
}EtypeCallback;


//PROTO FUNCTIONS

void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart);
void shamkaUSBtrans(uint8_t epnum,uint8_t* buff,uint32_t len,void(*callback)(struct usbStt*),EtypeCallback type);
void shamkaUSBrecv(uint8_t epnum,uint8_t* buff,uint32_t len,void(*callback)(struct usbStt*),EtypeCallback type);


#endif

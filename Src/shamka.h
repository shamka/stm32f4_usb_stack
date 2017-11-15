#ifndef __shamka_H
#define __shamka_H

#ifdef STM32F407xx
#include <stm32f4xx.h>
#else
#error "UNDEFINED CHIP"
#endif


#define MIN(a,b) a<b?a:b
#define MAX(a,b) a>b?a:b

#define CDCINPUT_BUFF 64
#define UARTINPUT_BUFF 2560

#define CDC_IN 0x82
#define CDC_OUT 0x02
#define CDC_IF 1

#define HID_INT_IN 0x81
#define HID_INT_OUT 0x01
#define HID_IF 0

#define MSD_OUT 0x03
#define MSD_IN 0x83
#define MSD_IF 3

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

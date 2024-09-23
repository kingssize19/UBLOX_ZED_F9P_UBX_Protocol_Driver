#ifndef SERIALCOM_H_INCLUDED
#define SERIALCOM_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include "driver.h"


#define SIZE        (uint8_t)100           /*<! RX Buffer SIZE (100 byte veri alıyor. GNSS tarafından maksimum 256 byte bilgi gelebilir.) */
#define COM_PORT "COM6"                    /*<! GNSS'in baglı oldugu COM port                                                             */


void OpenSerialPort(const char* port_name);                         /*<! Serial port configuration and connection   */
void CloseSerialPort();                                             /*<! Serial port close                          */
void SendMessageToGNSS(void* msg, MSG_TYPE_t msgType);             /*<! Send command to GNSS                       */
uint8_t ReceiveMessageFromGNSS(uint8_t* buffer, size_t max_size);   /*<! Receive Message from GNSS                  */
void ClearBuffer(uint8_t* buffer, size_t size);

//void SendMessageToGNSS(CFG_MSG_t* msg);


#endif // SERIALCOM_H_INCLUDED

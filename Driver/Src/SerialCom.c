#include "SerialCom.h"


HANDLE hSerial;

void OpenSerialPort(const char* port_name)
{
    hSerial = CreateFile(port_name, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE) {
        printf("Error opening serial port\n");
        exit(1);
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        printf("Error getting serial port state\n");
        exit(1);
    }

    // Baudrate 57600 olarak ayarlanýyor
    dcbSerialParams.BaudRate = CBR_57600;  // 57600 baudrate
    dcbSerialParams.ByteSize = 8;           // 8 data bits
    dcbSerialParams.StopBits = ONESTOPBIT;  // 1 stop bit
    dcbSerialParams.Parity   = NOPARITY;    // No parity

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        printf("Error setting serial port state\n");
        exit(1);
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;

    SetCommTimeouts(hSerial, &timeouts);
}

void CloseSerialPort()
 {
    CloseHandle(hSerial);
}
void SendMessageToGNSS(void* msg, MSG_TYPE_t msgType)
 {

     if (msgType == CFG_MSG_TYPE) {
        CFG_MSG_t* data = (CFG_MSG_t *)msg;
        DWORD bytes_written;
        WriteFile(hSerial, data, sizeof(CFG_MSG_t), &bytes_written, NULL);

        printf("Gonderilen Mesaj: ");
        uint8_t* send_data = (uint8_t*)msg;
        for (size_t i = 0; i < sizeof(CFG_MSG_t); ++i) {
            printf("0x%02X ", send_data[i]);
        }
        printf("\n");
     }

     else if (msgType == CFG_MSG2_TYPE) {
        CFG_MSG2_t* data = (CFG_MSG2_t *)msg;
        DWORD bytes_written;
        WriteFile(hSerial, data, sizeof(CFG_MSG2_t), &bytes_written, NULL);
        printf("Gonderilen Mesaj: ");
        uint8_t* send_data = (uint8_t*)msg;
        for (size_t i = 0; i < sizeof(CFG_MSG2_t); ++i) {
            printf("0x%02X ", send_data[i]);
        }
        printf("\n");
     }

     else if (msgType == CFG_CFG_TYPE) {
        CFG_CFG_t* data = (CFG_CFG_t *)msg;
        DWORD bytes_written;
        WriteFile(hSerial, data, sizeof(CFG_CFG_t), &bytes_written, NULL);
        printf("Gonderilen Mesaj: ");
        uint8_t* send_data = (uint8_t*)msg;
        for (size_t i = 0; i < sizeof(CFG_CFG_t); ++i) {
            printf("0x%02X ", send_data[i]);
        }
        printf("\n");
     }

     else if (msgType == CFG_RATE_TYPE) {
        CFG_RATE_t* data = (CFG_RATE_t *)msg;
        DWORD bytes_written;
        WriteFile(hSerial, data, sizeof(CFG_RATE_t), &bytes_written, NULL);
        printf("Gonderilen Mesaj: ");
        uint8_t* send_data = (uint8_t *)msg;
        for(size_t i = 0; i < sizeof(CFG_RATE_t); ++i) {
            printf("0x%02X ", send_data[i]);
        }
        printf("\n");
     }

}


uint8_t ReceiveMessageFromGNSS(uint8_t* buffer, size_t max_size) {
    DWORD bytes_read;
    BOOL result = ReadFile(hSerial, buffer, max_size, &bytes_read, NULL);

    if (!result || bytes_read == 0) {
        printf("Timeout: Yanit alinamadi.\n");
        return -1;
    }
    return bytes_read;  
}


void ClearBuffer(uint8_t* buffer, size_t size) {

    for (size_t i = 0; i < size; ++i)
        buffer[i] = 0;
}





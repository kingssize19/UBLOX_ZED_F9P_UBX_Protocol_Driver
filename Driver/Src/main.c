#include "SerialCom.h"


int main() {

    OpenSerialPort(COM_PORT);


    void* msg;
    uint8_t buffer[SIZE];
    uint8_t bytes_received;




    RATE_Config(&msg, FAST_MODE);
    bytes_received = ReceiveMessageFromGNSS(buffer, SIZE);
    print_Message(buffer, SIZE, bytes_received);
    ParseMessage(buffer, UBX_ACK_TYPE);


    CloseSerialPort();

    return 0;
}






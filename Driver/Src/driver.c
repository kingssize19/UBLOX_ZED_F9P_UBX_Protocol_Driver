#include "SerialCom.h"



void ConfigurationMessage(void *msg, MSG_TYPE_t msgType, uint8_t msgClass, uint8_t msgId, uint8_t state)
{


    if (msgType == CFG_MSG_TYPE) {
        CFG_MSG_t *cfg_msg = (CFG_MSG_t *)msg;
        cfg_msg->syncChar1 = SYNC_CHAR1;
        cfg_msg->syncChar2 = SYNC_CHAR2;
        cfg_msg->cfgClass  = UBX_CFG_CLASS;
        cfg_msg->cfgId     = UBX_CFG_MSG_ID;
        cfg_msg->length1   = 0x03;
        cfg_msg->length2   = NULL_BYTE;
        cfg_msg->msgClass  = msgClass;
        cfg_msg->msgId     = msgId;
        cfg_msg->state     = state;
    }

    else if (msgType == CFG_CFG_TYPE) {

            //For clear configuration
            if (state == CLEAR) {
                CFG_CFG_t* cfg_msg  = (CFG_CFG_t *)msg;
                cfg_msg->syncChar1  = SYNC_CHAR1;
                cfg_msg->syncChar2  = SYNC_CHAR2;
                cfg_msg->cfgClass   = UBX_CFG_CLASS;
                cfg_msg->cfgId      = UBX_CFG_CFG_ID;
                cfg_msg->length1    = 0x0D;
                cfg_msg->length2    = NULL_BYTE;
                cfg_msg->ClearMask1 = 0xFF;
                cfg_msg->clearMask2 = 0xFF;
                cfg_msg->clearMask3 = NULL_BYTE;
                cfg_msg->clearMask4 = NULL_BYTE;
                cfg_msg->saveMask1  = NULL_BYTE;
                cfg_msg->saveMask2  = NULL_BYTE;
                cfg_msg->saveMask3  = NULL_BYTE;
                cfg_msg->saveMask4  = NULL_BYTE;
                cfg_msg->loadMask1  = NULL_BYTE;
                cfg_msg->loadMask2  = NULL_BYTE;
                cfg_msg->loadMask3  = NULL_BYTE;
                cfg_msg->loadMask4  = NULL_BYTE;
                cfg_msg->deviceMask = 0x03;
            }

            //For save Configuration
            else if (state == SAVE) {
                CFG_CFG_t* cfg_msg  = (CFG_CFG_t *)msg;
                cfg_msg->syncChar1  = SYNC_CHAR1;
                cfg_msg->syncChar2  = SYNC_CHAR2;
                cfg_msg->cfgClass   = UBX_CFG_CLASS;
                cfg_msg->cfgId      = UBX_CFG_CFG_ID;
                cfg_msg->length1    = 0x0D;
                cfg_msg->length2    = NULL_BYTE;
                cfg_msg->ClearMask1 = NULL_BYTE;
                cfg_msg->clearMask2 = NULL_BYTE;
                cfg_msg->clearMask3 = NULL_BYTE;
                cfg_msg->clearMask4 = NULL_BYTE;
                cfg_msg->saveMask1  = 0xFF;
                cfg_msg->saveMask2  = 0xFF;
                cfg_msg->saveMask3  = NULL_BYTE;
                cfg_msg->saveMask4  = NULL_BYTE;
                cfg_msg->loadMask1  = NULL_BYTE;
                cfg_msg->loadMask2  = NULL_BYTE;
                cfg_msg->loadMask3  = NULL_BYTE;
                cfg_msg->loadMask4  = NULL_BYTE;
                cfg_msg->deviceMask = 0x03;
            }
    }

    else if (msgType == CFG_RATE_TYPE)
    {
        if (state == FAST_MODE) {
            CFG_RATE_t* cfg_msg   = (CFG_RATE_t *)msg;
            cfg_msg->syncChar1    = SYNC_CHAR1;
            cfg_msg->syncChar2    = SYNC_CHAR2;
            cfg_msg->rateClass    = UBX_CFG_CLASS;
            cfg_msg->rateId       = UBX_CFG_RATE_ID;
            cfg_msg->length1      = 0x06;
            cfg_msg->length2      = NULL_BYTE;
            cfg_msg->msRate1      = 0x64;
            cfg_msg->msRate2      = NULL_BYTE;
            cfg_msg->navRate1     = 0x01;
            cfg_msg->navRate2     = NULL_BYTE;
            cfg_msg->timeSystem1  = 0x01;
            cfg_msg->timeSystem2  = NULL_BYTE;
        }

        else if (state == MIDDLE_MODE) {

            CFG_RATE_t* cfg_msg   = (CFG_RATE_t *)msg;
            cfg_msg->syncChar1    = SYNC_CHAR1;
            cfg_msg->syncChar2    = SYNC_CHAR2;
            cfg_msg->rateClass    = UBX_CFG_CLASS;
            cfg_msg->rateId       = UBX_CFG_RATE_ID;
            cfg_msg->length1      = 0x06;
            cfg_msg->length2      = NULL_BYTE;
            cfg_msg->msRate1      = 0x08;
            cfg_msg->msRate2      = 0x3E;
            cfg_msg->navRate1     = 0x01;
            cfg_msg->navRate2     = NULL_BYTE;
            cfg_msg->timeSystem1  = 0x01;
            cfg_msg->timeSystem2  = NULL_BYTE;
        }

        else if (state == SLOW_MODE) {

            CFG_RATE_t* cfg_msg   = (CFG_RATE_t *)msg;
            cfg_msg->syncChar1    = SYNC_CHAR1;
            cfg_msg->syncChar2    = SYNC_CHAR2;
            cfg_msg->rateClass    = UBX_CFG_CLASS;
            cfg_msg->rateId       = UBX_CFG_RATE_ID;
            cfg_msg->length1      = 0x06;
            cfg_msg->length2      = NULL_BYTE;
            cfg_msg->msRate1      = 0x10;
            cfg_msg->msRate2      = 0x27;
            cfg_msg->navRate1     = 0x01;
            cfg_msg->navRate2     = NULL_BYTE;
            cfg_msg->timeSystem1  = 0x01;
            cfg_msg->timeSystem2  = NULL_BYTE;
        }
    }


}

void ConfigurationMessageRate(void* msg, uint8_t msgClass, uint8_t msgId, uint8_t state, uint8_t ui_interface)
{
    CFG_MSG2_t *cfg_msg = (CFG_MSG2_t *)msg;
    cfg_msg->syncChar1 = SYNC_CHAR1;
    cfg_msg->syncChar2 = SYNC_CHAR2;
    cfg_msg->cfgClass = UBX_CFG_CLASS;
    cfg_msg->cfgId    = UBX_CFG_MSG_ID;
    cfg_msg->length1  = 0x08;
    cfg_msg->length2  = NULL_BYTE;
    cfg_msg->msgClass = msgClass;
    cfg_msg->msgId    = msgId;

    if (state == ENABLE && ui_interface == UI_I2C)
        {
        cfg_msg->rateI2C    = 0x01;
        cfg_msg->rateUART1  = NULL_BYTE;
        cfg_msg->rateUART2  = NULL_BYTE;
        cfg_msg->rateUSB    = NULL_BYTE;
        cfg_msg->rateSPI    = NULL_BYTE;
    }

    if (state == ENABLE && ui_interface == UI_UART1)
        {
        cfg_msg->rateI2C    = NULL_BYTE;
        cfg_msg->rateUART1  = 0x01;
        cfg_msg->rateUART2  = NULL_BYTE;
        cfg_msg->rateUSB    = NULL_BYTE;
        cfg_msg->rateSPI    = NULL_BYTE;
    }

    else if (state == ENABLE && ui_interface == UI_UART2)
        {
        cfg_msg->rateI2C    = NULL_BYTE;
        cfg_msg->rateUART1  = NULL_BYTE;
        cfg_msg->rateUART2  = 0x01;
        cfg_msg->rateUSB    = NULL_BYTE;
        cfg_msg->rateSPI    = NULL_BYTE;
    }

    else if (state == ENABLE && ui_interface == UI_USB)
        {
        cfg_msg->rateI2C    = NULL_BYTE;
        cfg_msg->rateUART1  = NULL_BYTE;
        cfg_msg->rateUART2  = NULL_BYTE;
        cfg_msg->rateUSB    = 0x01;
        cfg_msg->rateSPI    = NULL_BYTE;
    }

    else if (state == ENABLE && ui_interface == UI_SPI)
        {
        cfg_msg->rateI2C    = NULL_BYTE;
        cfg_msg->rateUART1  = NULL_BYTE;
        cfg_msg->rateUART2  = NULL_BYTE;
        cfg_msg->rateUSB    = NULL_BYTE;
        cfg_msg->rateSPI    = 0x01;
    }

    else if (state == DISABLE && ui_interface == UI_ALL)
        {
        cfg_msg->rateI2C    = NULL_BYTE;
        cfg_msg->rateUART1  = NULL_BYTE;
        cfg_msg->rateUART2  = NULL_BYTE;
        cfg_msg->rateUSB    = NULL_BYTE;
        cfg_msg->rateSPI    = NULL_BYTE;

    }

    cfg_msg->rateEmpty = NULL_BYTE;
}

//B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 C2 01 00 01 00 01 00 00 00 00 00 B8 42
void UART_Config(void* msg, PORT_TYPE_t prtType, uint8_t portId, uint32_t baudRate)
{

    if (portId == UART1_PORT || portId == UART2_PORT)
    {
        CFG_PRT_t* configMsg = (CFG_PRT_t *)msg;
        configMsg->syncChar1 = SYNC_CHAR1;
        configMsg->syncChar2 = SYNC_CHAR2;
        configMsg->prtClass  = UBX_CFG_CLASS;
        configMsg->prtId     = UBX_CFG_PRT_ID;
        configMsg->lenght1   = 0x14;
        configMsg->length2   = NULL_BYTE;
        configMsg->portId    = portId;
        configMsg->reserved1 = NULL_BYTE;
        configMsg->txReady1  = NULL_BYTE;
        configMsg->txReady2  = NULL_BYTE;
        configMsg->mode1     = 0xD0;
        configMsg->mode2     = 0x08;
        configMsg->mode3     = NULL_BYTE;
        configMsg->mode4     = NULL_BYTE;

        if (baudRate == 4800) {
            configMsg->baudRate1 = 0xC0;
            configMsg->baudRate2 = 0x12;
            configMsg->baudRate3 = NULL_BYTE;
            configMsg->baudRate4 = NULL_BYTE;
        }

        else if (baudRate == 9600) {

            configMsg->baudRate1 = 0x80;
            configMsg->baudRate2 = 0x25;
            configMsg->baudRate3 = NULL_BYTE;
            configMsg->baudRate4 = NULL_BYTE;
        }

        else if (baudRate == 19200) {

            configMsg->baudRate1 = NULL_BYTE;
            configMsg->baudRate2 = 0x4B;
            configMsg->baudRate3 = NULL_BYTE;
            configMsg->baudRate4 = NULL_BYTE;
        }

        else if (baudRate == 38400) {

            configMsg->baudRate1 = NULL_BYTE;
            configMsg->baudRate2 = 0x96;
            configMsg->baudRate3 = NULL_BYTE;
            configMsg->baudRate4 = NULL_BYTE;
        }

        else if (baudRate == 57600) {

            configMsg->baudRate1 = NULL_BYTE;
            configMsg->baudRate2 = 0xE1;
            configMsg->baudRate3 = NULL_BYTE;
            configMsg->baudRate4 = NULL_BYTE;
        }

        else if (baudRate == 115200) {

            configMsg->baudRate1 = NULL_BYTE;
            configMsg->baudRate2 = 0xC2;
            configMsg->baudRate3 = 0x01;
            configMsg->baudRate4 = NULL_BYTE;
        }

        else if (baudRate == 230400) {

            configMsg->baudRate1 = NULL_BYTE;
            configMsg->baudRate2 = 0x84;
            configMsg->baudRate3 = 0x03;
            configMsg->baudRate4 = NULL_BYTE;
        }

        else if (baudRate == 460800) {

            configMsg->baudRate1 = NULL_BYTE;
            configMsg->baudRate2 = 0x08;
            configMsg->baudRate3 = 0x07;
            configMsg->baudRate4 = NULL_BYTE;
        }

        else if (baudRate == 921600) {

            configMsg->baudRate1 = NULL_BYTE;
            configMsg->baudRate2 = 0x10;
            configMsg->baudRate3 = 0x0E;
            configMsg->baudRate4 = NULL_BYTE;
        }

        configMsg->inProtoMask1 = 0x01;
        configMsg->inProtoMask2 = NULL_BYTE;
        configMsg->outProtoMask1 = 0x01;
        configMsg->outProtoMask2 = NULL_BYTE;
        configMsg->flag1 = NULL_BYTE;
        configMsg->flag2 = NULL_BYTE;
        configMsg->reserved2 = NULL_BYTE;
        configMsg->reserved3 = NULL_BYTE;

    }

}

void CalculateChecksum(void* msg, size_t length, uint8_t* checksumA, uint8_t* checksumB)
{
    uint8_t* data = (uint8_t *)msg;
    *checksumA    = NULL_BYTE;
    *checksumB    = NULL_BYTE;

    for (size_t i = 2; i < length; ++i)
        {
        *checksumA += data[i];
        *checksumB += *checksumA;
    }

    *checksumA &= 0xFF;
    *checksumB &= 0xFF;
}


//Calculate checksum
void CalculateChecksumForType(void *msg, MSG_TYPE_t msgType)
{
    uint8_t CK_A;
    uint8_t CK_B;
    size_t length;

    switch (msgType) {

        case CFG_CFG_TYPE :
            length = sizeof(CFG_CFG_t) - 2;
            CalculateChecksum((void *)msg, length, &CK_A, &CK_B);
            ((CFG_CFG_t *)msg)->checksumA = CK_A;
            ((CFG_CFG_t *)msg)->checksumB = CK_B;
            break;

        case CFG_MSG_TYPE :
            length = sizeof(CFG_MSG_t) - 2;
            CalculateChecksum((void *)msg, length, &CK_A, &CK_B);
            ((CFG_MSG_t *)msg)->checksumA = CK_A;
            ((CFG_MSG_t *)msg)->checksumB = CK_B;
            break;

        case CFG_MSG2_TYPE :
            length = sizeof(CFG_MSG2_t) - 2;
            CalculateChecksum((void *)msg, length, &CK_A, &CK_B);
            ((CFG_MSG2_t *)msg)->checksumA = CK_A;
            ((CFG_MSG2_t *)msg)->checksumB = CK_B;
            break;

        case CFG_RATE_TYPE :
            length = sizeof(CFG_RATE_t) - 2;
            CalculateChecksum((void *)msg, length, &CK_A, &CK_B);
            ((CFG_RATE_t *)msg)->checksumA = CK_A;
            ((CFG_RATE_t *)msg)->checksumB = CK_B;
            break;

    default: break;
    }

}



//Function that parses the incoming message
void ParseMessage(uint8_t* buffer, MSG_TYPE_t msgType)
 {

    if (msgType == UBX_ACK_TYPE)
    {
        UBX_ACK_t *msg_ack = (UBX_ACK_t *)buffer;

        if (msg_ack->ackClass == 0x05 && msg_ack->ackId == 0x01)
        {
            printf("\n\nKonfigurasyon ayari kabul edildi.\n\n");
        }

        else if (msg_ack->ackClass == 0x05 && msg_ack->ackId == 0x00)
        {
            printf("\n\nKonfigurasyon kabul edilmedi.\n\n");
        }
    }
}



//Function for Configuration


/*<! Open the NAV-PVT packet                            */
void NAV_PVT_OPEN_Config(void* msg)
 {

    CFG_MSG_t* configMsg = (CFG_MSG_t *)msg;
    ConfigurationMessage(configMsg, CFG_MSG_TYPE, UBX_NAV_CLASS, UBX_NAV_PVT_ID, ENABLE);
    CalculateChecksumForType(configMsg, CFG_MSG_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG_TYPE);

}

/*<! Close the NAV-PVT packet                           */
void NAV_PVT_CLOSE_Config(void* msg) {

    CFG_MSG_t* configMsg = (CFG_MSG_t *)msg;
    ConfigurationMessage(configMsg, CFG_MSG_TYPE, UBX_NAV_CLASS, UBX_NAV_PVT_ID, DISABLE);
    CalculateChecksumForType(configMsg, CFG_MSG_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG_TYPE);
}

/*<! Open the MON-HW  packet                            */
void MON_HW_OPEN_Config(void* msg) {

    CFG_MSG_t* configMsg = (CFG_MSG_t *)msg;
    ConfigurationMessage(configMsg, CFG_MSG_TYPE, UBX_MON_CLASS, UBX_MON_HW_ID, ENABLE);
    CalculateChecksumForType(configMsg, CFG_MSG_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG_TYPE);
}

/*<! Close the MON-HW  packet                           */
void MON_HW_CLOSE_Config(void* msg) {

    CFG_MSG_t* configMsg = (CFG_MSG_t *)msg;
    ConfigurationMessage(configMsg, CFG_MSG_TYPE, UBX_MON_CLASS, UBX_MON_HW_ID, DISABLE);
    CalculateChecksumForType(configMsg, CFG_MSG_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG_TYPE);
}

/*<! Open the NAV-PVT packet for the relevant interface */
void NAV_PVT_OPEN_Rate(void* msg, uint8_t ui_interface) {

    CFG_MSG2_t* configMsg = (CFG_MSG2_t *)msg;
    ConfigurationMessageRate(configMsg, UBX_NAV_CLASS, UBX_NAV_PVT_ID, ENABLE, ui_interface);
    CalculateChecksumForType(configMsg, CFG_MSG2_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG2_TYPE);

}

/*<! Open the MON-HW  packet for the relevant interface */
void MON_HW_OPEN_Rate(void* msg, uint8_t ui_interface) {

    CFG_MSG2_t* configMsg = (CFG_MSG2_t *)msg;
    ConfigurationMessageRate(configMsg, UBX_MON_CLASS, UBX_MON_HW_ID, ENABLE, ui_interface);
    CalculateChecksumForType(configMsg, CFG_MSG2_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG2_TYPE);
}

/*<! Save Configuration Function                        */
void SAVE_Config(void* msg) {

    CFG_CFG_t* configMsg = (CFG_CFG_t *)msg;
    ConfigurationMessage(configMsg, CFG_CFG_TYPE, UBX_CFG_CLASS, UBX_CFG_CFG_ID, SAVE);
    CalculateChecksumForType(configMsg, CFG_CFG_TYPE);
    SendMessageToGNSS(configMsg, CFG_CFG_TYPE);
}


/*<! Clear Configuration Function                        */
void CLEAR_Config(void* msg) {

    CFG_CFG_t* configMsg = (CFG_CFG_t *)msg;
    ConfigurationMessage(configMsg, CFG_CFG_TYPE, UBX_CFG_CLASS, UBX_CFG_CFG_ID, CLEAR);
    CalculateChecksumForType(configMsg, CFG_CFG_TYPE);
    SendMessageToGNSS(configMsg, CFG_CFG_TYPE);
}



void NMEA_GGA_CLOSE_Config(void* msg) {

    CFG_MSG2_t* configMsg = (CFG_MSG2_t *)msg;
    ConfigurationMessageRate(configMsg, NMEA_STD_CLASS, NMEA_STD_GGA_ID, DISABLE, UI_ALL);
    CalculateChecksumForType(configMsg, CFG_MSG2_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG2_TYPE);

}

void NMEA_GLL_CLOSE_Config(void* msg) {

    CFG_MSG2_t* configMsg = (CFG_MSG2_t *)msg;
    ConfigurationMessageRate(configMsg, NMEA_STD_CLASS, NMEA_STD_GLL_ID, DISABLE, UI_ALL);
    CalculateChecksumForType(configMsg, CFG_MSG2_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG2_TYPE);
}

void NMEA_GSA_CLOSE_Config(void* msg) {

    CFG_MSG2_t* configMsg = (CFG_MSG2_t *)msg;
    ConfigurationMessageRate(configMsg, NMEA_STD_CLASS, NMEA_STD_GSA_ID, DISABLE, UI_ALL);
    CalculateChecksumForType(configMsg, CFG_MSG2_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG2_TYPE);
}

void NMEA_GSV_CLOSE_Config(void* msg) {

    CFG_MSG2_t* configMsg = (CFG_MSG2_t *)msg;
    ConfigurationMessageRate(configMsg, NMEA_STD_CLASS, NMEA_STD_GSV_ID, DISABLE, UI_ALL);
    CalculateChecksumForType(configMsg, CFG_MSG2_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG2_TYPE);
}

void NMEA_RMC_CLOSE_Config(void* msg) {

    CFG_MSG2_t* configMsg = (CFG_MSG2_t *)msg;
    ConfigurationMessageRate(configMsg, NMEA_STD_CLASS, NMEA_STD_RMC_ID, DISABLE, UI_ALL);
    CalculateChecksumForType(configMsg, CFG_MSG2_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG2_TYPE);
}

void NMEA_VTG_CLOSE_Config(void* msg) {

    CFG_MSG2_t* configMsg = (CFG_MSG2_t *)msg;
    ConfigurationMessageRate(configMsg, NMEA_STD_CLASS, NMEA_STD_VTG_ID, DISABLE, UI_ALL);
    CalculateChecksumForType(configMsg, CFG_MSG2_TYPE);
    SendMessageToGNSS(configMsg, CFG_MSG2_TYPE);
}

void RATE_Config(void *msg, uint8_t mode) {

    CFG_RATE_t* configMsg = (CFG_RATE_t *)msg;
    ConfigurationMessage(configMsg, CFG_RATE_TYPE, UBX_CFG_CLASS, UBX_CFG_RATE_ID, mode);
    CalculateChecksumForType(configMsg, CFG_RATE_TYPE);
    SendMessageToGNSS(configMsg, CFG_RATE_TYPE);

}




void print_Message(const uint8_t* buffer, uint8_t size, uint8_t bytes_received)
{
    if (bytes_received > 0) {
        printf("Alinan Yanit: ");
        for (size_t i = 0; i < bytes_received; ++i) {
            printf("0x%02X ", buffer[i]);
        }
        printf("\n");
    }
}




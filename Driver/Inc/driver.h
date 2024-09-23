#ifndef DRIVER_H_INCLUDED
#define DRIVER_H_INCLUDED

#include <stdint.h>


#define ENABLE         (0x01)
#define DISABLE        (0x00)
#define RESERVED       (0x00)


#define CLEAR          (0x00)
#define SAVE           (0x01)

#define NULL_BYTE      (0x00)

#define SLOW_MODE      (0x10)
#define MIDDLE_MODE    (0x80)
#define FAST_MODE      (0x64)

#define UI_I2C         (0x00)
#define UI_UART1       (0x01)
#define UI_UART2       (0x02)
#define UI_USB         (0x03)
#define UI_SPI         (0x04)
#define UI_ALL         (0xFF)


#define I2C_PORT       (0x00)
#define UART1_PORT     (0x01)
#define UART2_PORT     (0x02)
#define USB_PORT       (0x03)
#define SPI_PORT       (0x04)


/*<!	Header Message address definitions		*/
#define 	SYNC_CHAR1			(0xB5)				/*<! Preamble sync character 1	*/
#define 	SYNC_CHAR2			(0x62)				/*<! Preamble sync character 2	*/


/*
 *
 *  UBX MESSAGE INFORMATION SECTION
 *
 */

/*<!	UBX-ACK class & id macro definitions 	*/
#define		UBX_ACK_CLASS					(0x05)				/*<! UBX-ACK class address 		*/
#define 	UBX_ACK_ACK_ID					(0x01)				/*<! UBX-ACK-ACK id address		*/
#define 	UBX_ACK_NAK_ID					(0x00)				/*<! UBX-ACK-NAK id address		*/


/*<! 	UBX-CFG class & id macro definitions 	*/
#define UBX_CFG_CLASS						(0x06)				/*<! 	UBX-CFG class address 								*/
#define UBX_CFG_ANT_ID						(0x13)				/*<! 	Antenna Control Settings 							*/
#define UBX_CFG_CFG_ID						(0x09)				/*<!  	Clear, save and load configurations					*/
#define UBX_CFG_DAT_ID						(0x06)				/*<! 	Set user-defined datum								*/
#define UBX_CFG_DGNSS_ID					(0x70)				/*<! 	DGNSS configuration 								*/
#define UBX_CFG_GEOFENCE_ID 				(0x69)				/*<! 	Geofencing configuration 							*/
#define UBX_CFG_GNSS_ID						(0x3E)				/*<! 	GNSS system configuration 							*/
#define UBX_CFG_INF_ID						(0x02)				/*<! 	Poll configuration for one protocol 				*/
#define UBX_CFG_ITFM_ID						(0x39)				/*<! 	Jamming/interference monitor configuration			*/
#define UBX_CFG_LOGFILTER_ID				(0x47)				/*<! 	Data logger configuration							*/
#define	UBX_CFG_MSG_ID						(0x01)				/*<! 	Poll a message configuration						*/
#define UBX_CFG_NAV5_ID						(0x24)				/*<! 	Navigation engine settings							*/
#define UBX_CFG_NAVX5_ID					(0x23)				/*<! 	Navigation engine expert settings					*/
#define UBX_CFG_NMEA_ID						(0x17)				/*<! 	Extended NMEA protocol configuration V1				*/
#define UBX_CFG_ODO_ID						(0x1E)				/*<! 	Odometer, low-speed COG engine settings				*/
#define UBX_CFG_PRT_ID						(0x00)				/*<! 	Polls the configuration for one I/O port			*/
#define UBX_CFG_PWR_ID						(0x57)				/*<! 	Put receiver in a defined power state				*/
#define UBX_CFG_RATE_ID						(0x08)				/*<! 	Navigation/measurement rate settings				*/
#define UBX_CFG_RINV_ID						(0x34)				/*<! 	Contents of remote inventory						*/
#define UBX_CFG_RST_ID						(0x04)				/*<! 	Reset receiver / Clear backup data structures		*/
#define UBX_CFG_SBAS_ID						(0x16)				/*<! 	SBAS configuration									*/
#define	UBX_CFG_TMODE3_ID					(0x71)				/*<! 	Time mode settings 3								*/
#define UBX_CFG_TP5_ID						(0x31)				/*<! 	Time pulse parameters								*/
#define UBX_CFG_USB_ID						(0x1B)				/*<! 	USB configuration									*/
#define UBX_CFG_VALDEL						(0x8C)				/*<! 	Delete configuration item values					*/
#define UBX_CFG_VALGET						(0x8B)				/*<!    Get configuration items								*/
#define UBX_CFG_VALSET						(0x8A)				/*<! 	Set configuration item values						*/

/*<! 	UBX-INF class & id macro definitions	 */
#define	UBX_INF_CLASS						(0x04)				/*<! UBX-INF class address									*/
#define UBX_INF_DEBUG_ID					(0x04)				/*<! ASCII output with debug contents						*/
#define UBX_INF_ERROR_ID					(0x00)				/*<! ASCII output with error contents						*/
#define UBX_INF_NOTICE_ID					(0x02)				/*<! ASCII output with informational contents				*/
#define UBX_INF_TEST_ID						(0x03)				/*<! ASCII output with test contents						*/
#define	UBX_INF_WARNING_ID					(0x01)				/*<! ASCII output with warning contents						*/


/*<! 	UBX-LOG class & id macro definitions	 */
#define UBX_LOG_CLASS						(0x21)				/*<! UBX_LOG classs address									*/
#define UBX_LOG_CREATE_ID					(0x07)				/*<! Create log file										*/
#define UBX_LOG_ERASE_ID					(0x03)				/*<! Erase logged data										*/
#define UBX_LOG_FINDTIME_ID					(0x0E)				/*<! Find index of a log entry based on a given time		*/
#define UBX_LOG_INFO_ID						(0x08)				/*<! Poll for log information								*/
#define UBX_LOG_RETRIEVE_ID					(0x09)				/*<! Request log data										*/
#define UBX_LOG_RETRIEVEPOS_ID				(0x0B)				/*<! Position fix log entry									*/
#define UBX_LOG_RETRIEVEPOSEXTRA_ID			(0x0F)				/*<! Odometer log entry										*/
#define UBX_LOG_RETRIEVESTRING_ID			(0x0D)				/*<! Byte string log entry									*/
#define UBX_LOG_STRING_ID					(0x04)				/*<! Store arbitrary string in on-board flash				*/


/*<!	UBX-MGA class & id macro definitions	 */
#define UBX_MGA_CLASS						(0x13)				/*<! UBX-MGA class address									*/
#define UBX_MGA_ACK_ID						(0x60)				/*<! Multiple GNSS acknowledge message						*/
#define UBX_MGA_BDS_ID						(0x03)				/*<! BeiDou ephemeris assistance							*/
#define UBX_MGA_DBD_ID						(0x80)				/*<! Poll the navigation database							*/
#define UBX_MGA_GAL_ID						(0x02)				/*<! Galileo ephemeris assistance							*/
#define UBX_MGA_GLO_ID						(0x06)				/*<! GLONASS ephemeris assistance							*/
#define UBX_MGA_GPS_ID						(0x00)				/*<! GPS ephemeris assistance								*/
#define UBX_MGA_INI_ID						(0x40)				/*<! Initial position assistance							*/
#define UBX_MGA_QZSS_ID						(0x05)				/*<! QZSS ephemeris assistance								*/


/*<! UBX-MON class & id macro definitions		*/
#define UBX_MON_CLASS						(0x0A)				/*<! UBX-MON class address									*/
#define UBX_MON_COMMS_ID					(0x36)				/*<! Communication port information							*/
#define UBX_MON_GNSS_ID						(0x28)				/*<! Information message major GNSS selection				*/
#define UBX_MON_HW_ID						(0x09)				/*<! Hardware status										*/
#define UBX_MON_HW2_ID						(0x0B)				/*<! Extended hardware status								*/
#define UBX_MON_HW3_ID						(0x37)				/*<! I/O pin status											*/
#define UBX_MON_IO_ID						(0x02)				/*<! I/O system status										*/
#define UBX_MON_MSGPP_ID					(0x06)				/*<! Message parse and process status						*/
#define UBX_MON_PATCH_ID					(0x27)				/*<! Installed patches										*/
#define UBX_MON_RF_ID						(0x38)				/*<! RF information											*/
#define UBX_MON_RXBUF_ID					(0x07)				/*<! Receiver buffer status									*/
#define UBX_MON_RXR_ID						(0x21)				/*<! Receiver status information							*/
#define UBX_MON_SPAN_ID						(0x31)				/*<! Signal characteristics									*/
#define UBX_MON_TXBUF_ID					(0x08)				/*<! Transmitter buffer status								*/
#define UBX_MON_VER_ID						(0x04)				/*<! Receiver and software version							*/


/*<! UBX-NAV class & id macro definitions		*/
#define UBX_NAV_CLASS						(0x01)				/*<! UBX-NAV class address									*/
#define UBX_NAV_CLOCK_ID					(0x22)				/*<! Clock solution											*/
#define UBX_NAV_DOP_ID						(0x04)				/*<! Dilution of precision 									*/
#define UBX_NAV_EOE_ID						(0x61)				/*<! End of epoch											*/
#define UBX_NAV_GEOFENCE_ID					(0x39)				/*<! Geofencing status										*/
#define UBX_NAV_HPPOSECEF_ID				(0x13)				/*<! High precision position solution in ECEF				*/
#define UBX_NAV_HPPOSLLH_ID					(0x14)				/*<! High precision geodetic position solution				*/
#define UBX_NAV_ODO_ID						(0x09)				/*<! Odometer solution										*/
#define UBX_NAV_ORB_ID						(0x34)				/*<! GNSS orbit database info								*/
#define UBX_NAV_POSECEF_ID					(0x01)				/*<! Position solution in ECEF								*/
#define UBX_NAV_POSLLH_ID					(0x02)				/*<! Geodetic position solution								*/
#define UBX_NAV_PVT_ID						(0x07)				/*<! Navigation position velocity time solution				*/
#define UBX_NAV_RELPOSNED_ID				(0x3C)				/*<! Relative positioning information in NED frame			*/
#define UBX_NAV_RESETODO_ID					(0x10)				/*<! Reset odometer											*/
#define UBX_NAV_SAT_ID						(0x35)				/*<! Satellite information									*/
#define UBX_NAV_SBAS_ID						(0x32)				/*<! SBAS status data										*/
#define UBX_NAV_SIG_ID						(0x43)				/*<! Signal information										*/
#define UBX_NAV_SLAS_ID						(0x42)				/*<! QZSS L1S SLAS status data								*/
#define UBX_NAV_STATUS_ID					(0x03)				/*<! Receiver navigation status								*/
#define UBX_NAV_SVIN_ID						(0x3B)				/*<! Survey-in data											*/
#define UBX_NAV_TIMEBDS_ID					(0x24)				/*<! BeiDou time solution									*/
#define UBX_NAV_TIMEGAL_ID					(0x25)				/*<! Galileo time solution									*/
#define UBX_NAV_TIMEGLO_ID					(0x23)				/*<! GLONASS time solution									*/
#define UBX_NAV_TIMELS_ID					(0x26)				/*<! Leap second event information							*/
#define UBX_NAV_TIMEQZSS_ID					(0x27)				/*<! QZSS time solution										*/
#define UBX_NAV_TIMEUTC_ID					(0x21)				/*<! UTC time solution										*/
#define UBX_NAV_VELECEF_ID					(0x11)				/*<! Velocity solution in ECEF								*/
#define UBX_NAV_VELNED_ID					(0x12)				/*<! Velocity solution in NED frame							*/


/*<! UBX-RMX class & id macro definitions		*/
#define UBX_RXM_CLASS						(0x02)				/*<! UBX-RXM class address									*/
#define UBX_RXM_MEASX_ID					(0x14)				/*<! Satellite measurements for RRLP						*/
#define UBX_RXM_PMREQ_ID					(0x41)				/*<! Power management request								*/
#define UBX_RXM_RAWX_ID						(0x15)				/*<! Multi-GNSS raw measurements							*/
#define UBX_RXM_RLM_ID						(0x59)				/*<! Galileo SAR short-RLM report							*/
#define UBX_RXM_RTCM_ID						(0x32)				/*<! RTCM input status										*/
#define UBX_RXM_SFRBX_ID					(0x13)				/*<! Broadcast navigation data subframe						*/


/*<! UBX-SEC class & id macro definitions		*/
#define UBX_SEC_CLASS						(0x27)				/*<! UBX-SEC class address									*/
#define UBX_SEC_UNIQID_ID					(0x03)				/*<! Unique chip ID											*/

/*<! UBX-TIM class & id macro definitions		*/
#define UBX_TIM_CLASS						(0x0D)				/*<! UBX-TIM class address									*/
#define UBX_TIM_TIM2_ID						(0x03)				/*<! Time mark data											*/
#define UBX_TIM_TP_ID						(0x01)				/*<! Time pulse time data									*/
#define UBX_TIM_VRFY_ID						(0x06)				/*<! Sourced time verification 								*/


/*<! UBX-UPD class & id macro definitions		*/
#define UBX_UPD_CLASS						(0x09)				/*<! UBX-UPD class address									*/
#define UBX_UPD_SOS_ID						(0x14)				/*<! Poll backup restore status								*/


/*
 *
 *  NMEA MESSAGE INFORMATION SECTION
 *
 */

/*<! NMEA packets Class & Id definitions        */
#define NMEA_STD_CLASS                  (0xF0)

#define NMEA_STD_DTM_ID                 (0x0A)
#define NMEA_STD_GAQ_ID                 (0x45)
#define NMEA_STD_GBQ_ID                 (0x44)
#define NMEA_STD_GBS_ID                 (0x09)
#define NMEA_STD_GGA_ID                 (0x00)
#define NMEA_STD_GLL_ID                 (0x01)
#define NMEA_STD_GLQ_ID                 (0x43)
#define NMEA_STD_GNQ_ID                 (0x42)
#define NMEA_STD_GNS_ID                 (0x0D)
#define NMEA_STD_GPQ_ID                 (0x40)
#define NMEA_STD_GQQ_ID                 (0x47)
#define NMEA_STD_GRS_ID                 (0x06)
#define NMEA_STD_GSA_ID                 (0x02)
#define NMEA_STD_GST_ID                 (0x07)
#define NMEA_STD_GSV_ID                 (0x03)
#define NMEA_STD_RLM_ID                 (0x0B)
#define NMEA_STD_RMC_ID                 (0x04)
#define NMEA_STD_TXT_ID                 (0x41)
#define NMEA_STD_VLW_ID                 (0x0F)
#define NMEA_STD_VTG_ID                 (0x05)
#define NMEA_STD_ZDA_ID                 (0x08)

#define NMEA_PUBX_CLASS                 (0xF1)

#define NMEA_PUBX_CONFIG_ID             (0x41)
#define NMEA_PUBX_POSITION_ID           (0x00)
#define NMEA_PUBX_RATE_ID               (0x40)
#define NMEA_PUBX_SVSTATUS_ID           (0x03)
#define NMEA_PUBX_TIME_ID               (0x04)





/*<! Enum to select the message type for Configuration  */

typedef enum
{
    UBX_ACK_TYPE,
    CFG_CFG_TYPE,
    CFG_MSG_TYPE,
    CFG_MSG2_TYPE,
    CFG_RATE_TYPE,


}MSG_TYPE_t;


typedef enum
{
    CFG_PRT_TYPE,

}PORT_TYPE_t;

/*<! UBX-ACK Structure Definitions  */

typedef struct
{

	 uint8_t  syncChar1;			   		/*<!   		B5 Header first byte  									*/
	 uint8_t  syncChar2;				    /*<!  	 	62 Header second byte 									*/
	 uint8_t  ackClass;				        /*<!   		05 ACK Class  byte	 									*/
	 uint8_t  ackId;					    /*<!        ACK ID byte 	01 (for ACK-ACK)                     	*/
     uint8_t  length1;
     uint8_t  length2;						/*<! 		Payload Length (Always 2 byte)							*/
	 uint8_t  messageClass;			        /*<! 		Message Class byte										*/
	 uint8_t  messageId;				    /*<! 		Message ID	byte										*/
	 uint8_t  checksumA;		            /*<! 		First checksum byte										*/
	 uint8_t  checksumB;			        /*<! 		Second checksum byte									*/

}UBX_ACK_t;

void Parse_ACK_Message(uint8_t* message);			// Function to parse the UBX_ACK message

/*<! UBX-NACK Structure Definitions     */

typedef struct
{

    uint8_t syncChar1;                          /*<! B5 Header first byte */
    uint8_t syncChar2;                          /*<! 62 Header second byte*/
    uint8_t nackClass;                          /*<! 05 NACK Class byte*/
    uint8_t nackId;                             /*<! NACK ID byte  00 (for NACK-NACK)*/
    uint8_t length1;                            /*<! Payload first byte */
    uint8_t length2;                            /*<! Payload second byte */
    uint8_t messageClass;                       /*<! Message Class byte*/
    uint8_t messageId;                          /*<! Message ID byte*/
    uint8_t checksumA;                          /*<! First checksum byte*/
    uint8_t checksumB;                          /*<! Second checksum byte*/

}UBX_NACK_t;

/*<! CFG-CFG structure Definitions      */

typedef struct
{
    uint8_t syncChar1;     /*<! B5 Header first byte*/
    uint8_t syncChar2;     /*<! 62 Header second byte*/
    uint8_t cfgClass;       /*<! 06 CFG Class byte*/
    uint8_t cfgId;          /*<! 09 CFG Class ID*/
    uint8_t length1;        /*<! Payload first byte*/
    uint8_t length2;        /*<! Payload second byte*/
    uint8_t ClearMask1;     /*<! First byte for clear mask. (0xFF for clear configuration)*/
    uint8_t clearMask2;     /*<! Second byte for clear mask. (0xFF for clear configuration) */
    uint8_t clearMask3;     /*<! Third byte for clear mask. (0x00)*/
    uint8_t clearMask4;     /*<! Fourth byte for clear mask. (0x00)*/
    uint8_t saveMask1;      /*<! First byte for save mask. (0xFF for save configuration)*/
    uint8_t saveMask2;      /*<! Second byte for save mask. (0xFF for save configuration)*/
    uint8_t saveMask3;      /*<! Third byte for save mask. (0x00)*/
    uint8_t saveMask4;      /*<! Fourth byte for save mask. (0x00)*/
    uint8_t loadMask1;      /*<! First byte for load mask. (0xFF for load configuration)*/
    uint8_t loadMask2;      /*<! Second byte for load mask. (0xFF for load configuration)*/
    uint8_t loadMask3;      /*<! Third byte for load mask. (0x00)*/
    uint8_t loadMask4;      /*<! Fourth byte for load mask (0x00)*/
    uint8_t deviceMask;     /*<! device mask for configuration. (used 0x03)*/
                            /*<! bit 0 : devBBR
                                 bit 1 : devFlash
                                 bit 2 : devEEPROM
                                 bit 4 : deSpiFlash  */
    uint8_t checksumA;                          /*<! First checksum byte*/
    uint8_t checksumB;                          /*<! Second checksum byte*/


}CFG_CFG_t;



/*<! CFG-MSG Structure Definitions  */

typedef struct
{

    uint8_t syncChar1;
    uint8_t syncChar2;
    uint8_t cfgClass;
    uint8_t cfgId;
    uint8_t length1;
    uint8_t length2;
    uint8_t msgClass;
    uint8_t msgId;
    uint8_t state;
    uint8_t checksumA;
    uint8_t checksumB;

}CFG_MSG_t;

/*<! CFG-MSG2 Structure Definitions  */

typedef struct
{

    uint8_t syncChar1;
    uint8_t syncChar2;
    uint8_t cfgClass;
    uint8_t cfgId;
    uint8_t length1;
    uint8_t length2;
    uint8_t msgClass;
    uint8_t msgId;
    uint8_t rateI2C;
    uint8_t rateUART1;
    uint8_t rateUART2;
    uint8_t rateUSB;
    uint8_t rateSPI;
    uint8_t rateEmpty;
    uint8_t checksumA;
    uint8_t checksumB;

}CFG_MSG2_t;


typedef struct
{
    uint8_t syncChar1;
    uint8_t syncChar2;
    uint8_t rateClass;
    uint8_t rateId;
    uint8_t length1;
    uint8_t length2;
    uint8_t msRate1;
    uint8_t msRate2;
    uint8_t navRate1;
    uint8_t navRate2;
    uint8_t timeSystem1;
    uint8_t timeSystem2;
    uint8_t checksumA;
    uint8_t checksumB;

}CFG_RATE_t;



typedef struct {

    uint8_t syncChar1;
    uint8_t syncChar2;
    uint8_t prtClass;
    uint8_t prtId;
    uint8_t lenght1;
    uint8_t length2;
    uint8_t portId;
    uint8_t reserved1;
    uint8_t txReady1;
    uint8_t txReady2;
    uint8_t mode1;
    uint8_t mode2;
    uint8_t mode3;
    uint8_t mode4;
    uint8_t baudRate1;
    uint8_t baudRate2;
    uint8_t baudRate3;
    uint8_t baudRate4;
    uint8_t inProtoMask1;
    uint8_t inProtoMask2;
    uint8_t outProtoMask1;
    uint8_t outProtoMask2;
    uint8_t flag1;
    uint8_t flag2;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t checksumA;
    uint8_t checksumB;

}CFG_PRT_t;


//Function Declared
void ConfigurationMessage(void *msg, MSG_TYPE_t msgType, uint8_t msgClass, uint8_t msgId, uint8_t state);
void CalculateChecksum(void* msg, size_t length, uint8_t* checksumA, uint8_t* checksumB);
void CalculateChecksumForType(void *msg, MSG_TYPE_t msgType);
void ParseMessage(uint8_t* buffer, MSG_TYPE_t msgType);


//Packet Configuration Declared
void NAV_PVT_OPEN_Config(void* msg);
void MON_HW_OPEN_Config(void* msg);

void NAV_PVT_CLOSE_Config(void* msg);
void MON_HW_CLOSE_Config(void* msg);

void NAV_PVT_OPEN_Rate(void* msg, uint8_t ui_interface);
void MON_HW_OPEN_Rate(void* msg, uint8_t ui_interface);

void SAVE_Config(void* msg);
void CLEAR_Config(void* msg);


void NMEA_GGA_CLOSE_Config(void* msg);
void NMEA_GLL_CLOSE_Config(void* msg);
void NMEA_GSA_CLOSE_Config(void* msg);
void NMEA_GSV_CLOSE_Config(void* msg);
void NMEA_RMC_CLOSE_Config(void* msg);
void NMEA_VTG_CLOSE_Config(void* msg);

void RATE_Config(void *msg, uint8_t mode);
void UART_Config(void* msg, PORT_TYPE_t prtType, uint8_t portId, uint32_t baudRate);

void print_Message(const uint8_t* buffer, uint8_t size, uint8_t bytes_received);





#include <string.h>

#endif



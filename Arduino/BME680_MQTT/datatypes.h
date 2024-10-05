/*---------------------------------------------------------------------------
	Project:	      APRS over MQTT

	Module:		      Definitions

	File Name:	      datatypes.h

	Revision:	      1.00

	Author:			  Martin Alcock, VE6VH

	Description:      Structure definitions and prototypes for the APRS over MQTT project

					  This software module and all others are the exclusive property of the Alberta Digital
					  Radio Communications Society and others (?the owners?), all rights are reserved.

					  The owners grant licence to any Amateur for personal or club use, and only on hardware
					  designed by them expressly for this purpose, use on any other hardware is prohibited.

					  This file, and others may be copied or distributed only with the inclusion of this
					  notice.

					  No warranty, either express or implied or transfer of rights is granted in this
					  licence and the owner is not liable for any outcome whatsoever arising from such usage.

					  Copyright � 2023, Alberta Digital Radio Communications Society


	Revision History:

---------------------------------------------------------------------------*/
#ifndef _DATATYPES_H_
#define _DATATYPES_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "cJSON.h"

#ifdef __cplusplus
extern "C"
{
#endif

// local defines and configurations
// set these to one to include the relevant code, else 0
#define	USE_AQI				1			// use AQI frame format
#define	USE_UIFRAME		0			// use a UI frame
#define	USE_TELEM			0			// use telemetry
// AQI definitions
#define SHORT_MSG_LEN 	500	 		// short message length
#define	LONG_MSG_LEN		1000		// longer message length
// telemetry I/O definitions
#define	MAX_ADC				8			// max adc's
#define	MAX_INP				5			// max number of inputs
#define	MAX_OUT				5			// max number of outputs

// MQTT payload frame types
#define	MQTT_AQI_PAYLOAD	  0			// AQI application
#define	MQTT_UIFRAM_PAYLOAD	1			// UI Frame
#define	MQTT_TELEM_PAYLOAD	2			// Telemetry 
#define	N_MQTT_PAYLOADS		  3			// number currently defined

// define a large integer
#ifndef LARGE_INTEGER
typedef union _LARGE_INTEGER {
	struct {
		uint32_t LowPart;
		uint32_t  HighPart;
	} DUMMYSTRUCTNAME;
	struct {
		uint32_t 	LowPart;
		uint32_t  HighPart;
	} u;
	uint64_t QuadPart;
} LARGE_INTEGER;
#endif

// MQTT common fields
typedef struct mqtt_hdr_t
{
	// device and frame info
	uint8_t			PayloadType;	// type of payload
	char			*HwType;		// hardware type
	char			*DeviceID;	  	// device ID

	// location: JSON Array
	double 			latitude;		  // position data
	double 			longitude;
	float 			altitude; 		// altitude
	float 			speed;		  	// speed
	float 			hdg;			    // heading
	bool   			fixvalid;		  // fix is valid
	// batttery
	int    			batt_level;
	LARGE_INTEGER	epoch;	    // epoch time	
} MQTT_HDR;

/* 	@brief generate the appropriate MQTT publish/subscribe topic from header data
	@param mqtt_hdr_t Struct containing the message header
*/
char *Get_MQTT_PubTopic(MQTT_HDR *hdr);
char *Get_MQTT_SubTopic(MQTT_HDR *hdr);

// additional structure and methods for AQI application
#if USE_AQI
// struct to hold AQI readings
typedef struct mqtt_AQI_t
{
	// BME630 output
	double  temperature; 	// you can do the arithmetic to convert these to float in bme680.c if you wish. the log shows how.
	double 	pressure;
	double 	relative_humidity;
	double 	air_qual;
} MQTT_AQI;
/* 	@brief JSON Message creator for an AQI message
	@param data a cJSON object in which to store the data
	@param mqtt_hdr_t Struct containing the message header
	@param mqtt_AQI_t Struct contains the message body
*/
bool Format_AQI_MQTT(cJSON *json_object, struct mqtt_hdr_t *hdr_data, struct mqtt_AQI_t *bme680);
#endif

// additional structures and methods for UIFrame forwarding
#if USE_UIFRAME
// struct to hold an I-Gate message
typedef struct mqtt_UIFrame_t
{
	char	*Source;		// source callsign
	char	*Dest;			// destination callsign
	char	*Repeaters;		// repeater field
	char 	*Frame;			// data frame
} MQTT_UIFRAME;
/* 	@brief JSON Message creator for an APRS message (pass through)
	@param data a cJSON object in which to store the data
	@param mqtt_hdr_t Struct containing the message header
	@param mqtt_UIFrame_t Struct contains the UI Frame
*/
bool Format_UIFrame_MQTT(cJSON *json_object, struct mqtt_hdr_t *hdr_data, struct mqtt_UIFrame_t *frame);
#endif

// additional structures and methods for Telemetry forwarding
#if USE_TELEM
// struct for a telemetry packet
typedef struct mqtt_Telemetry_t
{
	uint32_t	RadioID;			// ID of the radio
	uint8_t		nADC;				// number of ADC's
	uint16_t	ADCData[MAX_ADC];	// ADC Data
	uint8_t		nINP;				// number of inputs
	bool		INPData[MAX_INP];	// input data
	uint8_t		nOUT;				// number of outputs
	bool		OUTData[MAX_OUT];	// output data
} MQTT_TELEM;
/* 	@brief JSON Message creator for an APRS message (pass through)
	@param data a cJSON object in which to store the data
	@param mqtt_hdr_t Struct containing the message header
	@param mqtt_UIFrame_t Struct contains the UI Frame
*/
bool Format_Telemetry_MQTT(cJSON *json_object, struct mqtt_hdr_t *hdr_data, struct mqtt_Telemetry_t *Telemetry);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _DATATYPES_H_ */

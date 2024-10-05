/*---------------------------------------------------------------------------
	Project:	      APRS over MQTT

	Module:		      MQTT payload creator

	File Name:	      mqtt_payload.c

	Revision:	      1.00

	Author:			  Martin Alcock, VE6VH

	Description:      This code formats the JSON string using the cJSON library.

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
#include <string.h>

#include "cJSON.h"
#include "datatypes.h"

/*
 *	Pubclising topic processing
*/
char *MQTT_PubTopics[N_MQTT_PAYLOADS] = {
	"mqtt_aprs/aqi/",
	"mqtt_aprs/uiframe/",
	"mqtt_aprs/telem/",

};

char *MQTT_SubTopics[N_MQTT_PAYLOADS] = {
	"aprs_mqtt/aqi/",
	"aprs_mqtt/uiframe/",
	"aprs_mqtt/telem/",
};

// return the correct publishing topic based on payload type
char *Get_MQTT_PubTopic(MQTT_HDR *hdr)
{
  static char pubBuffer[100];
	switch (hdr->PayloadType) {

	case MQTT_AQI_PAYLOAD:
	case MQTT_UIFRAM_PAYLOAD:
	case MQTT_TELEM_PAYLOAD:
		strcpy(pubBuffer, MQTT_PubTopics[hdr->PayloadType]);
		strcat(pubBuffer, hdr->HwType);
		return pubBuffer;

	default:
		return NULL;
	}
}

char *Get_MQTT_SubTopic(MQTT_HDR *hdr)
{
  static char subBuffer[100];
	switch (hdr->PayloadType) {

	case MQTT_AQI_PAYLOAD:
	case MQTT_UIFRAM_PAYLOAD:
	case MQTT_TELEM_PAYLOAD:
		strcpy(subBuffer, MQTT_SubTopics[hdr->PayloadType]);
		strcat(subBuffer, hdr->DeviceID);
		return subBuffer;

	default:
		return NULL;
	}
}

/*
 *  header record is the same for all record type
*/
bool Format_MQTT_hdr(cJSON *root_object, struct mqtt_hdr_t *hdr_data)
{
	cJSON *IDString, *TimeStamp;
	cJSON *PositionData, *PositionItem;
	cJSON *JSONLat, *JSONLong, *JSONAlt, *JSONSpeed, *JSONHdg, *JSONFix;
	cJSON *JSONBatt;

	/*
	 *	Start with the device ID
	 */
	if ((IDString = cJSON_CreateString(hdr_data->DeviceID)) == NULL)
		return false;
	cJSON_AddItemToObject(root_object, "Device ID", IDString);

	/*
	 *	Add the position array
	 */
	if((PositionData = cJSON_CreateArray()) == NULL)
		return false;
	cJSON_AddItemToObject(root_object, "Position", PositionData);
	if ((PositionItem = cJSON_CreateObject()) == NULL)
		return false;
	cJSON_AddItemToArray(PositionData, PositionItem);

	// latitude
	if ((JSONLat = cJSON_CreateNumber(hdr_data->latitude)) == NULL)
		return false;
	cJSON_AddItemToObject(PositionItem, "Latitude", JSONLat);
	// longitude
	if ((JSONLong = cJSON_CreateNumber(hdr_data->longitude)) == NULL)
		return false;
	cJSON_AddItemToObject(PositionItem, "Longitude", JSONLong);
	// altitude
	if ((JSONAlt = cJSON_CreateNumber(hdr_data->altitude)) == NULL)
		return false;
	cJSON_AddItemToObject(PositionItem, "Altitude", JSONAlt);
	// speed
	if ((JSONSpeed = cJSON_CreateNumber(hdr_data->speed)) == NULL)
		return false;
	cJSON_AddItemToObject(PositionItem, "Speed", JSONSpeed);
	// heading
	if ((JSONHdg = cJSON_CreateNumber(hdr_data->hdg)) == NULL)
		return false;
	cJSON_AddItemToObject(PositionItem, "Heading", JSONHdg);
	// and finally, fix validity
	if ((JSONFix = cJSON_CreateBool(hdr_data->fixvalid)) == NULL)
		return false;
	cJSON_AddItemToObject(PositionItem, "Valid", JSONFix);
	/*
	 *	Add the battery level
	 */
	if ((JSONBatt = cJSON_CreateNumber(hdr_data->batt_level)) == NULL)
		return false;
	cJSON_AddItemToObject(root_object, "Battery Level", JSONBatt);
	/*
	 *	Finally add the timestamp
	 */
	if ((TimeStamp = cJSON_CreateNumber((double)hdr_data->epoch.QuadPart)) == NULL)
		return false;
	cJSON_AddItemToObject(root_object, "Epoch Time", TimeStamp);
    
    return true;
}

#if USE_AQI
/*
 * Add the fields for the BME680 application
*/
bool Format_AQI_Data(cJSON *root_object, struct mqtt_AQI_t *bme680)
{
    cJSON *AQIData, *AQIItem;
    cJSON *JSONTemp, *JSONPressure, *JSONHumidity, *JSONIAQ;

	/*
	 *	Add the BME680 data
	 */
	if ((AQIData = cJSON_CreateArray()) == NULL)
		return false;
	cJSON_AddItemToObject(root_object, "DataFrame", AQIData);
	if ((AQIItem = cJSON_CreateObject()) == NULL)
		return false;
	cJSON_AddItemToArray(AQIData, AQIItem);
	// temperature
	if ((JSONTemp = cJSON_CreateNumber(bme680->temperature)) == NULL)
		return false;
	cJSON_AddItemToObject(AQIItem, "Temperature", JSONTemp);
	// pressure
	if ((JSONPressure = cJSON_CreateNumber(bme680->pressure)) == NULL)
		return false;
	cJSON_AddItemToObject(AQIItem, "Pressure", JSONPressure);
	// humidity
	if ((JSONHumidity = cJSON_CreateNumber(bme680->relative_humidity)) == NULL)
		return false;
	cJSON_AddItemToObject(AQIItem, "Humidity", JSONHumidity);
	// Gas Resistance
	if ((JSONIAQ = cJSON_CreateNumber(bme680->air_qual)) == NULL)
		return false;
	cJSON_AddItemToObject(AQIItem, "IAQ", JSONIAQ);
	// completed
	return true;
}

// build a payload record for the AQI app. 
bool Format_AQI_MQTT(cJSON *json_object, struct mqtt_hdr_t *hdr_data, struct mqtt_AQI_t *bme680)
{
    if(!Format_MQTT_hdr(json_object, hdr_data))
        return false;

    return(Format_AQI_Data(json_object, bme680));
}
#endif

// huild a payload record for the UI forwarding APP
#if USE_UIFRAME
// format the contents of a UI Frame
bool Format_UIFrame_Data(cJSON *root_object, struct mqtt_UIFrame_t *UIFrame)
{
    cJSON *FrameArray, *FrameItem;
    cJSON *JSONSource, *JSONDest, *JSONRpt, *JSONFrame;

	/*
	 *	Add the Frame data
	 */
	if ((FrameArray = cJSON_CreateArray()) == NULL)
		return false;
	cJSON_AddItemToObject(root_object, "DataFrame", FrameArray);
    //
	if ((FrameItem = cJSON_CreateObject()) == NULL)
		return false;
	cJSON_AddItemToArray(FrameArray, FrameItem);

	// Source callsign
	if ((JSONSource = cJSON_CreateString(UIFrame->Source)) == NULL)
		return false;
	cJSON_AddItemToObject(FrameItem, "Source", JSONSource);

    // Destination callsign
	if ((JSONDest = cJSON_CreateString(UIFrame->Dest)) == NULL)
		return false;
	cJSON_AddItemToObject(FrameItem, "Destination", JSONDest);

    // repeaters
	if ((JSONRpt = cJSON_CreateString(UIFrame->Source)) == NULL)
		return false;
	cJSON_AddItemToObject(FrameItem, "Repeaters", JSONRpt);

    // Frame data
	if ((JSONFrame = cJSON_CreateString(UIFrame->Frame)) == NULL)
		return false;
	cJSON_AddItemToObject(FrameItem, "Frame", JSONFrame);

	return true;
}

// build a payload record for the repeater app
bool Format_UIFrame_MQTT(cJSON *json_object, struct mqtt_hdr_t *hdr_data, struct mqtt_UIFrame_t *frame)
{
    if(!Format_MQTT_hdr(json_object, hdr_data))
        return false;

    return(Format_UIFrame_Data(json_object, frame));
}
#endif

#if USE_TELEM
// format the contents of a telemetry frame
bool Format_MQTT_Telem(cJSON *root_object, struct mqtt_Telemetry_t *telemetry)
{
    cJSON *TelemArray, *TelemItem;
    cJSON *JSONRadioID, *JSONNumADC, *JSONNumInp, *JSONNumOut;
    cJSON *JSON_ADCArray, *JSON_ADCItem, *JSON_ADCValue;
	cJSON *JSON_INPArray, *JSON_INPItem, *JSON_INPValue;
	cJSON *JSON_OUTArray, *JSON_OUTItem, *JSON_OUTValue;

    char inputID[2] = { '\0', '\0' };

	/*
	 *	Add the telemetry frame
	 */
	if ((TelemArray = cJSON_CreateArray()) == NULL)
		return false;
	cJSON_AddItemToObject(root_object, "DataFrame", TelemArray);
    //
	if ((TelemItem = cJSON_CreateObject()) == NULL)
		return false;
	cJSON_AddItemToArray(TelemArray, TelemItem);

	// RadioID 
	if ((JSONRadioID = cJSON_CreateNumber(telemetry->RadioID)) == NULL)
		return false;
	cJSON_AddItemToObject(TelemItem, "RadioID", JSONRadioID);

    // Number of ADC's
	if ((JSONNumADC = cJSON_CreateNumber(telemetry->nADC)) == NULL)
		return false;
	cJSON_AddItemToObject(TelemItem, "Num_ADC", JSONNumADC);

    // ADC data
    if ((JSON_ADCArray = cJSON_CreateArray()) == NULL)
		return false;
	if ((JSON_ADCItem = cJSON_CreateObject()) == NULL)
		return false;
	cJSON_AddItemToArray(JSON_ADCArray, JSON_ADCItem);
    for(int i=0;i<telemetry->nADC;i++)      {
        inputID[0] = i + '0';
        if((JSON_ADCValue = cJSON_CreateNumber(telemetry->ADCData[i])) == NULL)
            return false;
        cJSON_AddItemToObject(JSON_ADCItem, inputID, JSON_ADCValue);                  
    }
	cJSON_AddItemToObject(TelemItem, "ADC_Data", JSON_ADCArray);    

    // Number of Inputs
	if ((JSONNumInp = cJSON_CreateNumber(telemetry->nINP)) == NULL)
		return false;
	cJSON_AddItemToObject(TelemItem, "Num_INP", JSONNumInp);

    // Input data
    if ((JSON_INPArray = cJSON_CreateArray()) == NULL)
		return false;
	if ((JSON_INPItem = cJSON_CreateObject()) == NULL)
		return false;
	cJSON_AddItemToArray(JSON_INPArray, JSON_INPItem);
    for(int i=0;i<telemetry->nINP;i++)      {
        inputID[0] = i + '0';
        if((JSON_INPValue = cJSON_CreateBool(telemetry->INPData[i])) == NULL)
            return false;
        cJSON_AddItemToObject(JSON_INPItem, inputID, JSON_INPValue);                  
    }
	cJSON_AddItemToObject(TelemItem, "INP_Data", JSON_INPArray);   	

    // Number of outputs
	if ((JSONNumOut = cJSON_CreateNumber(telemetry->nOUT)) == NULL)
		return false;
	cJSON_AddItemToObject(TelemItem, "Num_OUT", JSONNumOut);

	// Output data
    if ((JSON_OUTArray = cJSON_CreateArray()) == NULL)
		return false;
	if ((JSON_OUTItem = cJSON_CreateObject()) == NULL)
		return false;
	cJSON_AddItemToArray(JSON_OUTArray, JSON_OUTItem);
    for(int i=0;i<telemetry->nOUT;i++)      {
        inputID[0] = i + '0';
        if((JSON_OUTValue = cJSON_CreateBool(telemetry->OUTData[i])) == NULL)
            return false;
        cJSON_AddItemToObject(JSON_OUTItem, inputID, JSON_OUTValue);                  
    }
	cJSON_AddItemToObject(TelemItem, "OUT_Data", JSON_OUTArray);   	

	return true;
}

// buld a payload record for a Telemetry Frame
bool Format_Telemetry_MQTT(cJSON *json_object, struct mqtt_hdr_t *hdr_data, struct mqtt_Telemetry_t *telemetry)
{
        if(!Format_MQTT_hdr(json_object, hdr_data))
            return false;

        return(Format_MQTT_Telem(json_object, telemetry));
}
#endif





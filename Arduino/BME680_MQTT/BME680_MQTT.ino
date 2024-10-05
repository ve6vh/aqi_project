/********************************************************
 * AQI PROJECT - BME680 - BOSCH GAS SENSOR - MQTT       *
 *          BY PAULO RANZANI                            *
 *          24 AUG 2024 - LAST UPDATE 06 SEP 2024       *
 *          ARDUINO-2.3.2                               *
 *          UNO R4 WIFI MICROCONTROLLER                 *
 *          ESP32-S3 384 ROM 512KB SRAM                 *
 *          I2C Interface:                              *
 *          SDA=A4 SCL=A5 LOW ADDR=0x76 HIGH ADDR=0x77  *
 *          MQTT BROKER ECLIPSE MOSQUITTO               *
 *------------------------------------------------------*
 * CALGARY Sea Level:        1,048m         3,438ft     *
 *                       LAT 51.044733 LON -114.071883  *
 *                           51°02.68'N     114°04.31'W *
 *                           5102.68N       11404.31W   *
 *                      GRID DO21XB10JR                 *
 *------------------------------------------------------*
 * Temperature Parameters      Range:-40-85°C -40-185°F *
 * Absolute Accuracy:          ±0.5-1.0°C               *
 * Resolution:                 0.01°C                   *
 *------------------------------------------------------*
 * Pressure Parameters	Range: 300-1100hPa 30K-110KPa   *
 *                             approx. 4.35-15.95PSI    *
 * Absolute Accuracy:          ±0.6hPa                  *
 * Resolution:                 0.18Pa                   *
 *1atm = 760mmHg = 1013.25mbar = 101.325hPa = 1013.25Pa *
 *------------------------------------------------------*
 * Humidity Parameters	       Range: 10-90%RH          *
 * Absolute Accuracy:          ±3%RH (from 20-80%RH)    *
 * Resolution:                 0.008%RH                 *
 *------------------------------------------------------*
 * Gas Sensor Parameters	     F1 score for H₂S         *
 *                             scanning:0.92*           *
 * Standard scan speed:        10.8s/scan               *
 * Sensor-to-sensor deviation: +/-15%                   *
 * Output data processing:                              *
 * Index for Air Quality       (IAQ)                    *
 * bVOC-& CO₂-equivalents      (ppm)                    *
 * Gas scan result             (%)                      *
 * Raw Gas Index               0-9                      *
 * Heater Profile Length       0-10                     *
 * IAQ Range                   0-500 (larger is worse)  *
 ********************************************************/
 
 /********************************************************
    Updated by Martin Alcock, VE6VH to use ADRCS
    data structures for MQTT
 ********************************************************/
 
#include <WiFiS3.h>
#include "Wire.h"
#include "ArduinoGraphics.h"            //https://github.com/arduino-libraries/ArduinoGraphics
#include "Arduino_LED_Matrix.h"
#include "config.h"
#include <bsec2.h>
#include <ArduinoMqttClient.h>

#include "datatypes.h"
#include "cJSON.h"

// local defines
#define TX_PAYLOAD_BUFFER_SIZE  700         // sizeof (Mqtt payload buffer)
#define BME68X_WAITING          100         // waiting for data
#define MAX_MQTT_ATTEMPTS       10          // max attemtpts to connect to broker

// debug options (1 enables; 0 disables)
#define DUMP_PAYLOAD            0           // dump the payload to serial port
#define SHORT_FUSE              0           // shorten time between posts
#define WIFI_STATS              0           // dump the wifi stats

// main state machine
#define IDLE                    0           // not doing anything yet
#define WAITING                 1           // waiting for data
#define SENDING                 2           // sending data
#define SLEEPING                3           // sleeping
uint8_t fsm;                                // state var

// classes used by this code...
ArduinoLEDMatrix matrix;
Bsec2 envSensor;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// data structures for MQTT payload
// Header data
MQTT_HDR mqttHdr;

// application data frame
MQTT_AQI mqttAQI = {
    .temperature = 25.0,                        // temperature
    .pressure = 100.0,                          // pressure in hPa
    .relative_humidity = 30,                    // humidity
    .air_qual = 0                               // air quality
};

// sensor configuration
bsecSensor sensorList[] =
{
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT
};  

// Data strings from config file
const char *networkID=  CONFIG_NETWORKID;
const char *pass=       CONFIG_PASS;
const char *broker=     CONFIG_BROKER;
int port=               CONFIG_PORT;
const char *mqttUser=   CONFIG_MQTT_USER;            
const char *mqttPass=   CONFIG_MQTT_PASS;
const char *PayloadType = MQTT_AQI_PAYLOAD;
const char *HwType =    "ArduinoR4";
const char *DeviceID = CONFIG_DEVICE;     

char *pubTopic;                     // publication topic
char *subTopic;                     // subscription topic
char payload[TX_PAYLOAD_BUFFER_SIZE];    // mqtt payload data

// time between posts
#if SHORT_FUSE
#define     POST_INTERVAL   4       // 4 minute post interval
#else
#define     POST_INTERVAL   15      // 15 minute post interval
#endif

unsigned int   int_Secs = POST_INTERVAL*60;
unsigned long intReload = ((unsigned long)int_Secs * 1000L);
unsigned long interval;

// misc vars
char buffer[50];
unsigned long currentMillis=0;
unsigned long previousMillis=0;
unsigned long Minutes=0;
int Count=0;
boolean dataReady = false;

void setup(void)
{
  pinMode(13,OUTPUT);
  
  //Serial port setup
  Serial.begin(9600);
  while(!Serial){
    delay(10);
  }
  Serial.println();
  
  //i2C setup
  Serial.println("Configuring I2C....");
  Wire.begin();
    
  //BME680 Configuration
  // configure the sensor, halt in error state
  // if we cannot proceed...
  Serial.println("Configuring BME680....");
  if(!envSensor.begin(BME68X_I2C_ADDR_HIGH,Wire))
  {
      if(!checkBsecStatus(envSensor))
        errLeds();
  }
  if(!envSensor.updateSubscription(sensorList,ARRAY_LEN(sensorList),BSEC_SAMPLE_RATE_ULP))
  {
      if(!checkBsecStatus(envSensor))
        errLeds();
  }
  envSensor.attachCallback(newDataCallback);
  
  // announce the library version
  String verMsg="BSEC2 library version..: "+String(envSensor.version.major)+"."+String(envSensor.version.minor)+"."+String(envSensor.version.major_bugfix)+"."+String(envSensor.version.minor_bugfix);
  Serial.println(verMsg);
    
  // Wifi Setup
  Serial.println("Configuring WiFi....");
  if(WiFi.status()==WL_NO_MODULE)
  {
    Serial.println("Communication with wifi module failed: Cannot proceed");
    while(true);
  }
  
  // connect to network. OK to use delay() here as we are not doing
  // anything else. We want the wifi connection to stay up
  Serial.println("Connecting to WiFi network " + String(CONFIG_NETWORKID) + "(" + String(CONFIG_PASS) + ")");
  while(WiFi.begin(networkID,pass)!=WL_CONNECTED)
  {
      delay(5000);
      Serial.println("Retrying...");
  }
  Serial.println("Connected");

#if WIFI_STATS
  dumpWifiStats();
#endif

  //MQTT Setup
  // Complete the header struct setup
  mqttHdr.PayloadType = MQTT_AQI_PAYLOAD;
  mqttHdr.HwType = (char *)HwType;   
  mqttHdr.DeviceID = (char *)DeviceID;
  // 
  mqttHdr.latitude = (double)CONFIG_GPS_LAT; 
  mqttHdr.longitude = (double)CONFIG_GPS_LONG; 
  mqttHdr.altitude = 1020;
  mqttHdr.speed = 0;
  mqttHdr.hdg = 0;
  mqttHdr.fixvalid = true;
  //
  mqttHdr.batt_level = 100;  
  mqttHdr.epoch.QuadPart = 0;

  // callback not fired yet...
  dataReady = false;                          // no data yet...

  // get the publication topic
  pubTopic = Get_MQTT_PubTopic(&mqttHdr);
  subTopic = Get_MQTT_SubTopic(&mqttHdr);

  // set the username and password
  mqttClient.setUsernamePassword(mqttUser, mqttPass);

  // connect to the broker: Ok to wait here as well...
  Serial.println("Connecting to the MQTT broker...." + String(broker) + ":" + String(port));
  
  // loop until connected, retry every 5 seconds
  int connAttempts = 1;
  while(!mqttClient.connect(broker,port))
  {
    if(connAttempts > MAX_MQTT_ATTEMPTS)    {
      Serial.println("Max number of connection attempts exceeded. ");
        errLeds();
    }
    // will only come here if the attempt failed...
    Serial.print("MQTT Connection attempt " + String(connAttempts++) + " failed, error code= ");
    Serial.print(mqttClient.connectError());
    Serial.println(", Retrying In 5 seconds");
    delay(5000);
  }
  Serial.println("Connected");
 
  // LCD Matrix
  matrix.begin();
  matrix.loadFrame(LEDMATRIX_HEART_BIG);
  delay(2000);
  
  // Final setup
  Serial.println("Setup completed successfully");
}

// main processing loop
void loop(void)
{
  mqttClient.poll();        // keep alive for mqtt connection

  if(!envSensor.run())
  {
    // go to error routine if status is not OK
    if(!checkBsecStatus(envSensor))
      errLeds();
    else  return;
  }

  // run the state machine
  switch(fsm) {

  //idle: setup completed
  case  IDLE:
    Serial.println("Waiting for sensor data...");
    fsm = WAITING;
    break;

  // waiting for sensor data
  case  WAITING:
    if(!dataReady)         // wait for data to arrive
      return;
    Serial.println("Sensor data ready");      
    fsm = SENDING;
    break;
  
  // sending data
  case  SENDING:
    sendAPRSMessage();
    dataReady = false;
    interval = intReload;       // setup delay
    previousMillis = millis();    
    fsm = SLEEPING;
    break;
  
  // waiting for next update interval
  case  SLEEPING:
    currentMillis = millis()-previousMillis;
    if(currentMillis < interval)    {
      Minutes=(previousMillis+interval-millis())/60000L;
      String timeMsg=String(Minutes+1);
      timeMsg.trim();
      timeMsg=timeMsg+" min";
      MatrixText(timeMsg);
    } else {
      fsm = IDLE;
    }
    break;
  }

}

// process a callback from the BSEC2 library
// update the data structures for the next post
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
  // return if nothing sent
  if(!outputs.nOutputs)
  {
      return;
  }
  matrix.loadFrame(SMILE);
  // process the sensor data

  for(uint8_t i=0;i<outputs.nOutputs;i++)
  {
    
    const bsecData output=outputs.output[i];
    double dsignal = roundf((double)output.signal*100.0)/100.0;
    
    // store data by sensor ID
    switch (output.sensor_id)
    {
      case BSEC_OUTPUT_RAW_TEMPERATURE:
        mqttAQI.temperature = dsignal;
        break;
        
      case BSEC_OUTPUT_RAW_PRESSURE:
        mqttAQI.pressure =  dsignal;
        break;
        
      case BSEC_OUTPUT_RAW_HUMIDITY:
        mqttAQI.relative_humidity = dsignal;
        break;
        
      case BSEC_OUTPUT_RAW_GAS:
        break;
        
      case BSEC_OUTPUT_IAQ:
        mqttAQI.air_qual = dsignal;
        break;

      // unused sensor data
      case BSEC_OUTPUT_CO2_EQUIVALENT:
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
      default:
        break;
    }
  }
  dataReady = true;             // indicate that we have data
}

/*
 *  Send the data frame to APRS via MQTT
*/
void sendAPRSMessage(void)
{
  cJSON *aprs;               // message to go to aprs
  
  // format the aprs message
  if((aprs = cJSON_CreateObject()) == NULL)  {
      Serial.println("Cannot create cJSON payload object");
      return;
  }

  // fill in remaining fields
  mqttHdr.epoch.QuadPart = (uint64_t)millis();

  if(!Format_AQI_MQTT(aprs, &mqttHdr, &mqttAQI))  {
      Serial.println("Unable to format message");
      return;
  }
  if(!cJSON_PrintPreallocated(aprs, payload, TX_PAYLOAD_BUFFER_SIZE, true))  {
      Serial.println("Message format failed");
      return;
  }
  cJSON_Delete(aprs);

  // now send it
  mqttClient.beginMessage(String(pubTopic));
  mqttClient.print(String(payload));
  mqttClient.endMessage();
  Count+=1;
  
  String pubMsg = "Message "+ String(Count) + " published.. to " + pubTopic;
  Serial.println(pubMsg);
  
  // optional: print payload 
#if DUMP_PAYLOAD
  pubMsg="Payload ::" + String(payload);
  Serial.println(pubMsg);
#endif
}

// check the BSEC Status
boolean checkBsecStatus(Bsec2 bsec)
{
  if(bsec.status<BSEC_OK){
    Serial.println("BSEC status error (" + String(bsec.status) + ") ");
      return false;
  }
  else if(bsec.status>BSEC_OK){
    Serial.println("BSEC status Warning code.....: " + String(bsec.status));
    return true;
  }
  if(bsec.sensor.status<BME68X_OK){
    Serial.println("BME68X Sensor Error code.....: " + String(bsec.sensor.status));
    return false;
  }
  else if(bsec.sensor.status>BME68X_OK) {
    if(bsec.sensor.status == BME68X_WAITING)  {
          Serial.println("Waiting for BME68X Sensor measurement");
          return false;
    }
    Serial.println("BME68X Sensor Warning code...: " + String(bsec.sensor.status));
    return true;
  }
  // none of the above...
  return false;
}

// dump some Wifi Stats
void dumpWifiStats()
{

  // check the firmware version
  String FV=WiFi.firmwareVersion();
  Serial.println("Firmware version......: "+FV);
  if(FV<WIFI_FIRMWARE_LATEST_VERSION)
  {
    Serial.println("Firmware upgrade is available to current version:" + String(WIFI_FIRMWARE_LATEST_VERSION));
  }
   
  // Dump some stats about Wifi Connection
  Serial.print("Network ID............: ");
  Serial.println(networkID);
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address............: ");
  Serial.println(ip);
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC Address...........: ");
  for(int i=0;i<6;i++)
  {
    if(i>0)   {
      Serial.print(":");
    }
    if(mac[i]<16)  {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
  }
  Serial.println();
  byte encryption = WiFi.encryptionType();
  
  Serial.print("Encryption Type.......: ");
  Serial.print(encryption,HEX);
  switch (encryption)
  {
    case ENC_TYPE_WEP:
      Serial.println(" - WEP");
      break;
    case ENC_TYPE_WPA:
      Serial.println(" - WPA");
      break;
    case ENC_TYPE_WPA2:
      Serial.println(" - WPA2");
      break;
    case ENC_TYPE_WPA3:
      Serial.print(" - WPA3");
      break;   
    case ENC_TYPE_NONE:
      Serial.println(" = NONE");
      break;
    case ENC_TYPE_AUTO:
      Serial.println(" - AUTO");
      break;
    case ENC_TYPE_UNKNOWN:
    default:
      Serial.println(" - UNKNOWN");
      break;
  }
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID Mac address.....: ");
  for(int i=0;i<6;i++)
  {
    if(i>0){Serial.print(":");}
    if(bssid[i]<16){Serial.print("0");}
    Serial.print(bssid[i],HEX);
  }
  Serial.println();
  long rssi = WiFi.RSSI();
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// output text to the LED matrix
void MatrixText(String text)
{
  text="  "+text+"  ";
  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(50);
  matrix.textFont(Font_5x7);
  matrix.beginText(0,1,0xFFFFFF);
  matrix.println(text.c_str());
  matrix.endText(SCROLL_LEFT);
}

// flash the error LED: 1 sec on, 1 sec off
void errLeds(void)
{
  while(1)
  {
    digitalWrite(13,HIGH);
    delay(1000);
    digitalWrite(13,LOW);
    delay(1000);
  }
}

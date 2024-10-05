// local WIFI network
#define CONFIG_NETWORKID  "<your wifi network>"
#define CONFIG_PASS       "<your wifi password>"

// MQTT broker
#define CONFIG_BROKER     "aprs.adrcs.org"
#define CONFIG_PORT        7000
#define CONFIG_MQTT_USER  "<broker user name>"
#define CONFIG_MQTT_PASS  "<broker password>"

// your device ID and lat/long
#define CONFIG_DEVICE     "40367091005101"
#define CONFIG_GPS_LAT    51.14339              // must be correct
#define CONFIG_GPS_LONG   -114.17631            // you can obtain it from google maps

// MATRIX SIZE: 8x12 pixels
// SMILE - PAULO RANZANI
const uint32_t SMILE[3] PROGMEM={0x0F010829,0x42042942,0x641080F0};
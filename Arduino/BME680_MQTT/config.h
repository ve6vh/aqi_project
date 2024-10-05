// local WIFI network
#define CONFIG_NETWORKID  "Labrat"
#define CONFIG_PASS       "nimh_secret"

// MQTT broker
#define CONFIG_BROKER     "aprs.adrcs.org"
#define CONFIG_PORT        7000
#define CONFIG_MQTT_USER  "aqi"
#define CONFIG_MQTT_PASS  "bosch"

// your device ID and lat/long
#define CONFIG_DEVICE     "40367091005101"
#define CONFIG_GPS_LAT    51.14339              // must be correct
#define CONFIG_GPS_LONG   -114.17631            // you can obtain it from google maps

// MATRIX SIZE: 8x12 pixels
// SMILE - PAULO RANZANI
const uint32_t SMILE[3] PROGMEM={0x0F010829,0x42042942,0x641080F0};
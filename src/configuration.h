#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//************ RADIO MODULE SETTINGS *************
#define NODEID              artekroom    // name of node - look nodes.h
#define NETWORKID           100           //210
#define BAND                RF69_433MHZ   // (433, 868 or 915 MHz)
#define GATEWAYID           1
#define ENCRYPTKEY          "test"    // 16 character/bytes the same on all nodes
#define ACK_TIME 30

//#define ENABLE_ATC
#define ATC_RSSI -75                      //AUTO TRANSMISSION CONTROL

//*********** SENSORS ******************
// Used devices (comment or uncomment if not used)
//#define LDR_SENSOR                    // use LDR sensor
//#define BH1750_SENSOR                   // use BH1750 light sensor
//#define DS18B20                       // use 1WIE DS18B20
#define DS_COUNT           1	    // TO FIX; ONLY 1 - 3 sensors

//#define SHT21_SENSOR                  // use SHT21
//#define BMP_SENSOR                    // use BMP085 or BMP180
//#define DHT_SENSOR                    // DHT11 or DHT22
//#define DHT_SENSOR_TYPE    DHT22
//#define DHT_SENSOR_PORT    P2	    // TODO!!
//#define AIRQ                  	    // Air Quality - MQ-135 eg. FC-22
//#define LPG                           // LPG - MQ-6
//#define RELAY                         // SSR Relay
//#define SOIL                          // Soil Moisture Sensor

//*********** OTHER ******************
//#define LED_ON                        // use act led for transmission
//#define OLED
//#define FAKE_BAT

//#define DEV_MODE                      // display output only at console, do not send packages via radio
#define DEBUG                         // debug mode - serial output
//#define DEBUG_BAUD      57600         // if not define baud rate = 9600bps

//*********** BATTERY SAVING ********
#define VCC_OK    3.40                  // enough power for normal 1-minute sends
#define VCC_LOW   3.30                  // sleep for 1 minute, then try again
#define VCC_DOZE  3.20                  // sleep for 5 minutes, then try again
                                        // sleep for 10 minutes, then try again
#define VCC_SLEEP_MINS(x) ((x) >= VCC_LOW ? 1 : (x) >= VCC_DOZE ? 5 : 10)

#define VCC_FINAL 3.10                  // send anyway, might be our last swan song

#endif //__CONFIGURATION_H

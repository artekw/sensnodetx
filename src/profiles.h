#ifndef PROFILES_H
#define PROFILES_H

/* Defines:
  * sensors
  * board rev (BOARD_REV variable)
  * wireless module (RFM69 variable)
*/

// rooms

#if NODEID == artekroom
#define BOARD_REV         3
#define DS18B20
#define LED_ON
#endif

#if NODEID == foliowiec
  #define BOARD_REV       2
  #define DS18B20
#endif

#if NODEID == babcia
  #define BOARD_REV       4
  #define DS18B20
  #define RELAY
#endif

#if NODEID == garaz
  #define BOARD_REV       3
  #define SHT21_SENSOR
  #define LPG
#endif

#if NODEID == pepper
  #define BOARD_REV       4
  #define LED_ON 
  #define SOIL
  #define BH1750_SENSOR
#endif

// others


#if NODEID == testnode
  #define BOARD_REV       4
  #define DS18B20
  #define LED_ON
#endif

#if NODEID == car
  #define BOARD_REV       4
  #define DS18B20
  #define LED_ON
#endif

#if NODEID == outnode
  #define BOARD_REV 4
  #define DS18B20
#endif

#endif //__PROFILES_H

#include <Arduino.h>

 /*
SensnodeTX v5.0-dev
Written by Artur Wronowski <artur.wronowski@digi-led.pl>
Based od Jeenode project by Jean-Claude Wippler <jcw@wippler.nl>

TODO:
- wiartomierz //#define ObwAnem 0.25434 // meters
*/

#include "nodes.h"
#include "configuration.h"
#include "profiles.h"

#include <Wire.h>

#if defined DS18B20
   #include <OneWire.h>
   #include <DallasTemperature.h>
#endif
// lib for SHT21 and BMP085
#if defined SHT21_SENSOR || defined BMP_SENSOR 
   #include <BMP085.h>
   #include <SHT2x.h>
#endif
// lib for BH1750
#ifdef BH1750_SENSOR
    #include <BH1750.h>
#endif

// radio RFM69
#include <LowPower.h>
#include <RFM69.h>
#include <SPI.h>

 // Jee Ports
#include <JeeLib.h>

 // DHT11 and DHT22 from https://github.com/adafruit/DHT-sensor-library
#ifdef DHT_SENSOR
   #include "DHT.h"
#endif
#ifdef OLED
   #include <OzOLED.h>
   #include <stdlib.h>
#endif

// EEPROM needed for relay states
//#include <EEPROM.h>
#include "boards.h"


RFM69 radio;

#if defined ENABLE_ATC
  RFM69_ATC radio;
#endif


 /*************************************************************/

#ifdef DS18B20
  byte ds_array[DS_COUNT];
#endif

#ifdef DHT_SENSOR
  float h; // define humidity DTH variable
  float t; //define temperature DTH variable
  #define DHTTYPE           DHT_SENSOR_TYPE
  #define DHTPIN            4  // port P1 = digital 4
#endif

#if defined SHT21_SENSOR || defined BMP_SENSOR || defined BH1750_SENSOR
  #define I2C
#endif

#ifdef DEV_MODE
  #define DEBUG
#endif

#ifndef DEBUG_BAUD
  #define DEBUG_BAUD        9600
#endif

/*************************************************************/

unsigned int adcreading;
byte numberOfDevices;
int volts;
byte receivedOK = 0;
byte relay_eeprom_addr = 0;

char buf[10];
char buff[50];
char nodeData[192];

// structure of sended data

// typedef struct {
// #if defined BH1750_SENSOR
//   uint16_t light;
// #endif
// #if defined SHT21_SENSOR || defined DHT_SENSOR
//   int humi;
// #endif
// #if defined DS18B20 && defined DS_COUNT > 1
//   int temp0;
//   int temp1;
//   int temp2;
// #else
//   #if defined DS18B20 || defined SHT21_SENSOR || defined DHT_SENSOR || defined BMP_SENSOR
//     int temp;
//   #endif
// #endif
// #if defined BMP_SENSOR
//   int pressure;
// #endif
// #ifdef AIRQ
//   int co2;
// #endif
// #ifdef LPG
//   int lpg;
// #endif
// #ifdef SOIL
//   int soil;
// #endif
// #ifdef RELAY
//   byte relay :1;
// #endif
//   int battvol;
// } PayloadTX;
// PayloadTX measure;


#if defined BH1750_SENSOR
  uint16_t light;
#endif
#if defined SHT21_SENSOR || defined DHT_SENSOR
  char humi[10];
#endif
#if defined DS18B20 && defined DS_COUNT > 1
  char temp0[10];
  char temp1[10];
  char temp2[10];
#else
  #if defined DS18B20 || defined SHT21_SENSOR || defined DHT_SENSOR || defined BMP_SENSOR
    char temp[10];
  #endif
#endif
#if defined BMP_SENSOR
  int pressure;
#endif
#ifdef AIRQ
  int co2;
#endif
#ifdef LPG
  int lpg;
#endif
#ifdef SOIL
  int soil;
#endif
#ifdef RELAY
  byte relay :1;
#endif
  char battvol[10];

typedef struct {
    byte destnode;
    byte cmd;
    byte state :1;
} PayloadRX;
PayloadRX rxdata;


#ifdef DS18B20
  #define TEMPERATURE_PRECISION 9
  OneWire oneWire(ONEWIRE_DATA);
  DallasTemperature sensors(&oneWire);
  DeviceAddress tempDeviceAddress;
#endif

#ifdef DHT_SENSOR
  DHT dht(DHTPIN, DHTTYPE);
#endif

#ifdef RELAY
  #define RELAY_PIN 1
  Port relay (RELAY_PIN); //P1
#endif

#ifdef LPG
  #define LPG_PIN 2
  Port lpg (LPG_PIN);    //P2
#endif

#ifdef AIRQ
  #define AIRQ_PIN 3
  Port air (AIRQ_PIN);  //P3
#endif

#ifdef SOIL
  #define SOIL_PIN 4    //P4
  Port soil (SOIL_PIN);
#endif

#ifdef BH1750_SENSOR
  BH1750 lightMeter(0x23);
#endif


void setup()
{
  radio.initialize(BAND, NODEID, NETWORKID);
  radio.sleep();
  radio.encrypt(ENCRYPTKEY);

#ifdef ENABLE_ATC
  radio.enableAutoPower(ATC_RSSI);
#endif

  sprintf(buff, "\nTransmitting at %d Mhz...", BAND == RF69_433MHZ ? 433 : BAND == RF69_868MHZ ? 868 : 915);

#ifdef DHT_SENSOR
  dht.begin();
#endif

#ifdef I2C
  Wire.begin();
#endif

#ifdef DEBUG
  Serial.begin(DEBUG_BAUD);
#endif

#ifdef DS18B20
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
#endif

#ifdef AIRQ
  air.mode(INPUT);
#endif

#ifdef RELAY
  relay.mode(OUTPUT);
#endif

#ifdef LPG
  lpg.mode(INPUT);
#endif

#ifdef OLED
  OzOled.init();
  OzOled.clearDisplay();
  OzOled.setPageMode();
#endif

#ifdef BH1750_SENSOR
  lightMeter.begin();
#endif
}


void doReport()
{

  // send ACK
  if (radio.receiveDone())
  {
    if (radio.ACKRequested()) {
      radio.sendACK();
    }
  }
  radio.sendWithRetry(GATEWAYID, nodeData, sizeof nodeData + 1);
}

// void doReceive() {
//   if (rf12_recvDone() && rf12_crc == 0 && rf12_len == sizeof rxdata) {
//     #ifdef LED_ON
//       recvLED();
//     #endif
//     memcpy(&rxdata, (void*) rf12_data, sizeof rxdata);
//     if(RF12_WANTS_ACK){
//       rf12_sendStart(RF12_ACK_REPLY,0,0);
//     }
//   }

// #ifdef RELAY
//   if (rxdata.destnode == NODEID) {
//     // Custom commands
//     // eg. ON/OFF SSR Relay
//     if (rxdata.cmd == 1) {
//       if (rxdata.state == 1) {
//         receivedOK = 1;
//         EEPROM.write(relay_eeprom_addr, rxdata.state);
//         relay.digiWrite(1);
//       }
//       else {
//         receivedOK = 0;
//         EEPROM.write(relay_eeprom_addr, rxdata.state);
//         relay.digiWrite(0);

//       }
//     }
//   }
// #endif
// }


long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result;
  return result;
}


int battVolts(void) {
  #if defined(__AVR_ATmega168__) 
    analogReference(DEFAULT);
  #else
    analogReference(INTERNAL);
  #endif
  float vccref = readVcc()/1000.0;
  #if BOARD_REV == 4
    float resistor_tolerance = 1.06; // 1%
    // R6 = 1M %1, R7 = 10M %1
	adcreading = analogRead(BAT_VOL) * 10 * resistor_tolerance;
  #else
  // R6 = 1M %1, R7 = 1M %1
	adcreading = analogRead(BAT_VOL) * 2;
  #endif
  float battvol = (adcreading / 1023.0) * vccref;
  return battvol * 1000;
}


void sendLED(byte on) {
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, on);
  delay(100);
}


void recvLED() {
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, HIGH);
  delay(50);
  digitalWrite(ACT_LED, LOW);
  delay(100);
  digitalWrite(ACT_LED, HIGH);
  delay(50);
  digitalWrite(ACT_LED, LOW);
}


void transmissionRS()
{
  #ifdef DEV_MODE
    Serial.println("==DEV MODE==");
    Serial.print("NODEID: ");
    Serial.print(NODEID);
    delay(2);
  #endif
  //sendLED(1);
  Serial.println(' ');
  delay(2);
  #ifdef BH1750_SENSOR
    Serial.print("LIGHT ");
    Serial.println(light);
    delay(2);
  #endif
  #if defined DHT_SENSOR || defined SHT21_SENSOR
    Serial.print("HUMI ");
    Serial.println(humi);
  #endif
  delay(2);
  #if defined DS_COUNT && defined DS_COUNT > 1
      for (byte i=0; i < DS_COUNT; i++) { 
        Serial.print("TEMP");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(ds_array[i]);
      }
  #else
      #if defined DS18B20 || defined SHT21_SENSOR || defined DHT_SENSOR || defined BMP_SENSOR
        Serial.print("TEMP ");
        Serial.println(temp);
      #endif
  #endif
  #if defined BMP_SENSOR
    Serial.print("PRES ");
    Serial.println(measure.pressure);
  #endif
  delay(2);
  #ifdef AIRQ
    Serial.print("CO2 ");
    Serial.println(co2);
  #endif
  delay(2);
  #ifdef LPG
    Serial.print("LPG ");
    Serial.println(lpg);
  #endif
  #ifdef SOIL
    Serial.print("SOIL ");
    Serial.println(soil);
  #endif
  Serial.print("BATVOL ");
  Serial.println(battvol);
  delay(2);
  #ifdef RELAY
    Serial.println(' ');
    Serial.print("RECEIVED ");
    Serial.println(EEPROM.read(0));
    Serial.println(' ');
    delay(2);
  #endif
  //sendLED(0);
}


void doMeasure() {
  float tmp;
#if defined FAKE_BAT
  tmp = 3.7;
#else
  tmp = battVolts() * 0.001;
#endif
  dtostrf(tmp, 3, 2, battvol);
  sprintf(buf, "bat:%s ", battvol);
  strcat(nodeData, buf);


#ifdef LDR_SENSOR
     measure.light = analogRead(LDR_PIN);
#endif

#ifdef I2C
  #ifdef BH1750_SENSOR
    light = lightMeter.readLightLevel(); //lux
  #endif

  #ifdef SHT21_SENSOR
    float shthumi = SHT2x.GetHumidity();
    humi = shthumi;
    #ifndef DS18B20
      float shttemp = SHT2x.GetTemperature();
      temp = shttemp;
    #endif
  #endif

  #ifdef BMP_SENSOR 
    Sleepy::loseSomeTime(250);
    BMP085.getCalData();
    BMP085.readSensor();
    measure.pressure = ((BMP085.press*10*10) + 16);
  #endif
#endif

#ifdef DS18B20
  #ifdef DEBUG
    Serial.print("DS18B20 found: ");
    Serial.println(numberOfDevices, DEC);
  #endif

  #if DS_COUNT > 1  // TODO
    for(byte i=0; i < DS_COUNT; i++) {
      sensors.requestTemperatures();
      tmp = sensors.getTempCByIndex(i);
      ds_array[i] = tmp * 10;
    }
    measure.temp0 = ds_array[0];
    measure.temp1 = ds_array[1];
    measure.temp2 = ds_array[2];
  #else
    sensors.requestTemperatures();
    tmp = sensors.getTempCByIndex(0);
    // measure.temp = tmp * 10;
    dtostrf(tmp, 3, 1, temp);
    sprintf(buf, "temp:%s", temp);
    strcat(nodeData, buf);
  #endif
#endif


#ifdef DHT_SENSOR
 #ifndef SHT21
   float h = dht.readHumidity();
   #ifndef DS18B20
     float t = dht.readTemperature();
   #endif
   if (isnan(h) || isnan(t)) {
     return;
   } 
   else {
     measure.humi = h*10;
     #ifndef DS18B20
       measure.temp = t*10;
     #endif
   }
  #endif
#endif

#ifdef AIRQ
  byte i = 0;
  float val = 0;
  for(i=0;i<20;i++) {
    val += air.anaRead();
    delay(100);
  }
  measure.co2 = (val/20.0) * 10;
#endif

#ifdef LPG
  byte i = 0;
  float val = 0;
  for(i=0;i<20;i++) {
    val += lpg.anaRead();
    delay(100);
  }
  measure.lpg = (val/20.0) * 10;
#endif

#ifdef RELAY
  measure.relay = EEPROM.read(relay_eeprom_addr);
#endif

#ifdef SOIL
  byte i = 0;
  float val = 0;
  for(i=0;i<20;i++) {
    val += soil.anaRead();
    delay(100);
  }
  //measure.soil = map((val/20.0), 750, 1023, 100, 0);
  //measure.soil = map((val/20.0), 0, 1023, 0, 100);
  measure.soil = (val/20.0);
#endif
}

// void doDisplay() {
// #ifdef OLED
//   OzOled.setCursorXY(0,0);
//   OzOled.printString("-=");
//   OzOled.setCursorXY(4,0);
//   OzOled.printString(NODENAME);
//   OzOled.setCursorXY(14,0);
//   OzOled.printString("=-");

//   OzOled.setCursorXY(0,4);
//   OzOled.printString("BAT:");
//   OzOled.setCursorXY(6,4);
//   OzOled.printNumber((float)measure.battvol/1000, 2);
//   OzOled.setCursorXY(13,4);
//   OzOled.printString("V");
// /*
//   OzOled.setCursorXY(0,6);
//   OzOled.printString("LUPD:");
//   OzOled.setCursorXY(6,6);
//   OzOled.printNumber((float)(m * 60)+s);
//   OzOled.setCursorXY(13,6);
//   OzOled.printString("s");
// */
//   #ifdef DS18B20
//     OzOled.setCursorXY(0,2);
//     OzOled.printString("TEMP:");
//     OzOled.setCursorXY(6,2);
//     OzOled.printNumber((float)measure.temp/10, 2);
//     OzOled.setCursorXY(13,2);
//     OzOled.printString("*C");
//   #endif

//   #ifdef AIRQ
//     OzOled.setCursorXY(0,6);
//     OzOled.printString("AIRQ:");
//     OzOled.setCursorXY(6,6);
//     OzOled.printNumber((float)measure.co2/10, 2);
//     OzOled.setCursorXY(13,6);
//     OzOled.printString("ppm");
//   #endif
// #endif
// }


void sendPayload()
{
  doMeasure(); // measure
  #ifdef DEBUG
    transmissionRS();
  #endif
  #ifdef LED_ON
    sendLED(1);
  #endif
  #ifndef DEV_MODE
    doReport(); // send
  #endif
  #ifdef LED_ON
    sendLED(0);
  #endif
  #ifdef OLED
    doDisplay();
  #endif
}


// Main loop
void loop() {
  int vcc;

  #if defined FAKE_BAT
    vcc = 5000;
  #else
    vcc = battVolts() * 0.001;
  #endif

  #ifdef RELAY
    relay.digiWrite(EEPROM.read(0));
  #endif

    if (vcc <= VCC_FINAL) { // ostatni mozliwy pakiet
      sendPayload();
      vcc = 1; // don't even try reading VCC after this send
    }

    if (vcc >= VCC_OK) { // wszytko ok
      sendPayload();
    }

    int minutes = VCC_SLEEP_MINS(vcc);

    while (minutes-- > 0)
      #ifndef DEV_MODE
        #ifndef RELAY
          // 15 * 4s = 60sek 
          for (int i = 0; i < 15; ++i)
          {
            LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
          }
        #else
          for (int i = 0; i < 60000/70; ++i) {
            Sleepy::loseSomeTime(10);
            doReceive();
          }
        #endif
      #else
        #ifndef RELAY
          Sleepy::loseSomeTime(6000);
        #else
          for (int i = 0; i < 6000/70; ++i) {
            Sleepy::loseSomeTime(10);
            doReceive();
          }
        #endif
      #endif
    memset(nodeData, 0, sizeof(nodeData));
}

/*
  (c) 2023 by Cory Griffis

  it is released under Creative Commons Attrib Share-Alike License
  You are free to use/extend this library but please abide with the CC-BY-SA license:
  http://creativecommons.org/licenses/by-sa/3.0/



*/


#include <Adafruit_EEPROM_I2C.h>
//#include <Adafruit_FRAM_I2C.h>

#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

#include <SD.h>

#include <Adafruit_SleepyDog.h>

#define USE_FEATHER_OLED true
#define USE_I2C_EEPROM true

#ifdef USE_I2C_FRAM
  #include <Adafruit_FRAM_I2C.h>
#endif

#ifdef USE_I2C_EEPROM
  #include <Adafruit_EEPROM_I2C.h>
  Adafruit_EEPROM_I2C fram = Adafruit_EEPROM_I2C();
#endif

#ifdef USE_SPI_FRAM
  #include <Adafruit_FRAM_SPI.h>
  uint8_t FRAM_SCK = 14;
  uint8_t FRAM_MISO = 12;
  uint8_t FRAM_MOSI = 13;
  uint8_t FRAM_CS = 15;
  Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_SCK, FRAM_MISO, FRAM_MOSI, FRAM_CS);
#endif

#define mem_SSID (uint32_t)0x0000
#define mem_PASSWORD (uint32_t)0x0025

#define mem_DEV_USER (uint32_t)0x0050
#define mem_DEV_PASSWORD (uint32_t)0x0075

#define mem_SECURTIY_TYPE 
#define mem_DNS_SERVER (uint32_t)0x0200
#define mem_NTP_SERVER (uint32_t)0x0240
#define mem_DEFAULT_DNS_SERVER
#define mem_DEFAULT_NTP_SERVER

#define mem_PRESSURE_OFFSET (uint32_t)0x0400
#define mem_BAR_ENABLE (uint32_t)0x0406
#define mem_INFCE_THERMOMETER_ENABLE (uint32_t)0x0407
#define mem_INFCE_HUMIDITY_ENABLE (uint32_t)0x0408
#define mem_INFCE_THERMOMETER_UNIT (uint32_t)0x0409

#define mem_WUNDERGROUND_ENABLE (uint32_t)0x0100
#define mem_WUNDERGROUND_TEMP_ID (uint32_t)0x0101
#define mem_WUNDERGROUND_HUMIDITY_ID (uint32_t)0x0102
#define mem_WUNDERGROUND_TIME_SOURCE (uint32_t)0x0103

#define mem_WUNDERGROUNDID (uint32_t)0x0125
#define mem_WUNDERGROUNDPASSWORD (uint32_t)0x0150

#define WundergroundStationIDLength 64
#define WundergroundStationIDPassword 64

#include <Adafruit_FeatherOLED.h>
#include <Adafruit_SSD1306.h>

Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

//#include <postParser.h>

#define _USE_RTC_PCF8523 true
#define USE_WUNDERGROUND_INFCE
#define _USE_TH_SENSOR
//#define DAVISRFM69_DEBUG
#define _INFCE_SEND_TEMP_HUMIDITY
#ifdef _USE_TH_SENSOR

static void xDisplayTask(void* pvParameters);

//static void readTempHumiditySensor();
//  Ticker readTHSensorTicker(readTempHumiditySensor,2,0);
float temperature = NAN, humidity = NAN;
float tempf = NAN, tempc = NAN;
bool QueueThermometerForInterfaces = true;
bool QueueHumidityForInterfaces = true;
bool UseCelcius = false;
#endif

#include <Ethernet.h>
#include <Arduino.h>
#include <EthernetUdp.h>
#include <DNS.h>
#include <SPI.h>
// FreeRTOS Libraries
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <FreeRTOS_SAMD21.h>
#include <croutine.h>
#include <deprecated_definitions.h>
#include <error_hooks.h>
#include <event_groups.h>
#include <list.h>
#include <message_buffer.h>
#include <mpu_prototypes.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <runTimeStats_hooks.h>
#include <semphr.h>
#include <stack_macros.h>
#include <stream_buffer.h>
#include <task.h>
#include <timers.h>

//
#include <Wire.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "DavisRFM69.h"
#include "PacketFifo.h"
#include "RFM69registers.h"

#include <Dns.h>

#define DATASTRINGLENGTH 256
#define MAXDATASTRINGS 1

typedef struct DataString {
  char TheString[DATASTRINGLENGTH];
};


typedef struct DatalogArray {
  DataString DataStringArray[2];
};

DatalogArray DataStrings;
String DatalogString;

// Wunderground interface
//#define USE_WUNDERGROUND_INFCE

#ifdef USE_WUNDERGROUND_INFCE
#include "WundergroundInfce.h"

IPAddress wundergroundIP(0, 0, 0, 0);
//HTTPClient httpWunderground;
bool WundergroundInfceEnable = true;
EthernetClient WundergroundEthernetCclient;

#include "WundergroundInfce.h"

#define wx_version String("00.01.00");

#define SM_Wunderground_Infce_Init (byte)91
#define SM_Wunderground_Infce_Get_IP_By_DNS (byte)99
#define SM_Wunderground_Infce_Make_HTTP_Request (byte)101
#define SM_Wunderground_Infce_WaitForHTTP_Response (byte)111
#define SM_Wunderground_CloseConnections (byte)112
#define SM_Wunderground_Infce_Idle (byte)255
#define SM_Wunderground_Infce_No_Link (byte)254
#define SM_Wunderground_Infce_Assemble_Payload (byte)121

// Define sensor types accepted by Wunderground interface
enum {
  WU_S_TEMPC_T = 1,
  WU_S_TEMPC2_T = 2,
  WU_S_TEMPC3_T = 3,
  WU_S_TEMPC4_T = 4,
  WU_S_INDOORTEMPC_T = 5,
  WU_S_SOILTEMPC_T = 6,
  WU_S_SOILTEMPC2_T = 7,
  WU_S_SOILTEMPC3_T = 8,
  WU_S_TEMPF_T = 9,
  WU_S_TEMPF2_T = 10,
  WU_S_TEMPF3_T = 11,
  WU_S_TEMPF4_T = 12,
  WU_S_INDOORTEMPF_T = 13,
  WU_S_SOILTEMPF_T = 14,
  WU_S_SOILTEMPF2_T = 15,
  WU_S_SOILTEMPF3_T = 16,
  WU_S_BAROMIN_T = 17,
  WU_S_HUMIDITY_T = 18,
  WU_S_INDOORHUMIDITY_T = 19

} Wu_Sensor_t;

enum {
  WU_NOW_T = 1,
  WU_RTC_T = 2,
  WU_NTP_T = 3
} Wu_Time_t;


String Wundergroundpayload;
String WundergroundHTTPString;
String WundergroundResponse;
String date_str;
int WundergroundResponseCode;
String WUurl = "https://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?";
String WUget = "/weatherstation/updateweatherstation.php?";
String WU_station_id = "";   //Wunderground station ID
String WU_station_pwd = "";  //# Wunderground station password
String WUcreds;              // = "ID=" + WU_station_id + "&PASSWORD="+ WU_station_pwd;
const char* url = "weatherstation.wunderground.com";
String action_str = "&action=updateraw";
String W_Software_Type = "&softwaretype=M0WX%20version" + wx_version;
//  bool shouldUpdateWundergroundInfce = false;
// bool WundergroundInfceEnable = false;
char WundergroundStationID[WundergroundStationIDLength] = "";
char WundergroundStationPassword[WundergroundStationIDPassword] = "";
//struct repeating_timer WU_Update_timer;
uint8_t thermometer1Type = WU_S_TEMPF_T;
uint8_t humidity1_sensor_type = WU_S_HUMIDITY_T;
uint8_t WundergroundTimeSource = 1;

static void xUpdateWundergroundInfce(void* pvParameters);  // Must be called in main loop

#endif
/*
  Definitions for RFM69 Radio
*/
#define LED 13
#define SERIAL_BAUD 19200

// If using Feather M0 with RF69 module
//  #define RFM69_ENABLE 9
#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4
#define RFM69_TIMEOUT 12
#define LED 13

#define mem_Davis_Station_0_Active (uint32_t)0x0210
#define mem_Davis_Station_1_Active (uint32_t)0x0220
#define mem_Davis_Station_2_Active (uint32_t)0x0230
#define mem_Davis_Station_3_Active (uint32_t)0x0240
#define mem_Davis_Station_4_Active (uint32_t)0x0250
#define mem_Davis_Station_5_Active (uint32_t)0x0260
#define mem_Davis_Station_6_Active (uint32_t)0x0270
#define mem_Davis_Station_7_Active (uint32_t)0x0280

DavisRFM69 radio(RFM69_CS, 3, true, 3);

// id, type, active
Station stations[8] = {
  { .id = 0,
    .type = STYPE_VUE,
    .active = true},
      { .id = 1,
    .type = STYPE_VUE,
    .active = false},
      { .id = 2,
    .type = STYPE_VUE,
    .active = false},
      { .id = 3,
    .type = STYPE_VUE,
    .active = false },
      { .id = 4,
    .type = STYPE_VUE,
    .active = false },
      { .id = 5,
    .type = STYPE_VUE,
    .active = false },
      { .id = 6,
    .type = STYPE_VUE,
    .active = false },
      { .id = 7,
    .type = STYPE_VUE,
    .active = false },
};

int stationSel = 0;

// RTC Definitions
#ifdef _USE_DS3231
#include <RTClib.h>
RTC_DS3231 rtc;
#ifndef _USE_RTC
#define _USE_RTC true
#endif
#endif

#ifdef _USE_RTC_PCF8523
#ifndef _USE_RTC
#define _USE_RTC
#endif
#include <RTClib.h>
RTC_PCF8523 rtc;
#endif

//  #include <RTClib.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0x98, 0x76, 0xB6, 0x12, 0x3c, 0xa6
};

// default IP address
//IPAddress ip(192, 168, 1, 177);
// webserver definitions

static void xHTTPUpdateTask(void* pvParameters);  // Webserver Task
EthernetClient HTTPClient;                        // The Web Client
TaskHandle_t xHTTPClientTaskHandle;               // Task Handle for Webserver Task
// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);
DNSClient dnsclient;

// NTP Client definitions
#include <NTPClient.h>  // NTP Client Library
//extern EthernetClass Ethernet;
DateTime now;      // Current DateTime
DateTime ntptime;  // NTP Time
//WiFiUDP wifiUdp;
IPAddress ntpip(192, 168, 1, 20);
EthernetUDP eth0udp;                             // UDP Socket for NTP Client
NTPClient timeClient(eth0udp, ntpip);            // NTP Client
static void xNTPClientTask(void* pvParameters);  // NTP Client Task
TaskHandle_t xNTPClientTaskHandle;               // NTP Client Task Handle

// Semaphore Definitions
SemaphoreHandle_t I2CBusSemaphore;   // Arbitrate I2C bus access
SemaphoreHandle_t Eth0Semaphore;     // Arbitrate Eth0 bus access
StaticSemaphore_t xSPIBusSemaphore;  // Arbitrate the SPI bus
SemaphoreHandle_t SPIBusSemaphore;
static void xinterruptHandlertask(void* pvParameters);
static void xReadRadioTask(void* pvParameters);
TaskHandle_t xReadRadioTaskHandle;
TaskHandle_t xinterrupttaskhandle;
TaskHandle_t xUpdateWundergroundInfcetaskhandle;
TaskHandle_t xWatchdogTaskHandle;
TaskHandle_t xDataSamplerTaskHandle;
QueueHandle_t DataStringQueueHandle;
TaskHandle_t xDataloggerTaskHandle;
TaskHandle_t xDisplayTaskHandle;
StaticQueue_t DataStringQueue;
//uint8_t DataStringQueueArray[MAXDATASTRINGS];
TaskHandle_t xButtonHandlerTaskHandle;
void setup() {
  Serial.begin(115200);
  delay(1000);
  //while(!Serial);
  // SPI.begin(); // initialize SPI
  digitalWrite(LED, LOW);
  pinMode(LED, OUTPUT);
  // perform hard reset on RFM69 module
  digitalWrite(RFM69_RST, LOW);
  pinMode(RFM69_RST, OUTPUT);
  delay(10);
  pinMode(RFM69_RST, INPUT_PULLUP);
  delay(10);
  // initialize mutexes
  I2CBusSemaphore = xSemaphoreCreateMutex();  // Create I2C Semaphore
  //Eth0Semaphore = xSemaphoreCreateMutex(); // Create ETH0 Semaphore
  SPIBusSemaphore = xSemaphoreCreateMutex();
  pinMode(8, INPUT_PULLUP);  // RF69 Enable pin
                             // pinMode(10, INPUT_PULLUP); // Ethernet Feather CS pin
#ifdef _USE_RTC
  // Initialize the RTC
  Serial.println("Initialize RTC");
  if (rtc.begin(&Wire)) {
    now = rtc.now();
  } else {
    Serial.println("Could not initialize RTC!");
  }
#endif  // USE_RTC

// Initialize NVRAM
  Serial.println("Initialize NVRAM");
  if(fram.begin(0x50)){
     //loadCredentials();            // Load WLAN credentials from network
     LoadSensorsFromDisk();
     //RestoreDefaults();
     LoadWundergroundCredentials();// Load Wunderground Interface credentials
  }
  else{
    Serial.println("could not initialize fram");
  }

    oled.init();

  // Initialize Ethernet
  Ethernet.init(10);
  // start the Ethernet connection and the server:
  Ethernet.begin(mac);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1);  // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  dnsclient.begin(IPAddress(1, 1, 1, 1));

  timeClient.begin();  // Start NTP Client

  server.begin();  // start webserver
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  HTTPClient.setConnectionTimeout(2000);
  xTaskCreate(xHTTPUpdateTask, "HTTP Update Task", 256, NULL, tskIDLE_PRIORITY + 1, &xHTTPClientTaskHandle);  // Start web server task

  // pinMode(10, INPUT_PULLUP); // Ethernet Feather CS pin

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  radio.setStations(stations, 1);
  radio.initialize(FREQ_BAND_US);
  radio.setBandwidth(RF69_DAVIS_BW_WIDE);

  xTaskCreate(xReadRadioTask, "Radio Task", 256, NULL, tskIDLE_PRIORITY + 3, &xReadRadioTaskHandle);
  radio.xReadRadioTaskHandle = xReadRadioTaskHandle;
  xTaskCreate(xNTPClientTask, "NTP Task", 512, NULL, tskIDLE_PRIORITY + 1, &xNTPClientTaskHandle);  // Start NTP Update task

  DataStringQueueHandle = xQueueCreate(MAXDATASTRINGS, DATASTRINGLENGTH);
  xTaskCreate(xDataSamplerTask, "Datasampler", 256, NULL, tskIDLE_PRIORITY + 1, &xDataSamplerTaskHandle);
  xTaskCreate(xDataloggerTask, "Datalogger", 256, NULL, tskIDLE_PRIORITY + 1, &xDataloggerTaskHandle);
  xTaskCreate(xUpdateWundergroundInfce, "Wunderground Interface Task", 512, NULL, tskIDLE_PRIORITY + 1, &xUpdateWundergroundInfcetaskhandle);
  xTaskCreate(xDisplayTask, "xDisplay Task", 256, NULL, tskIDLE_PRIORITY + 1, &xDisplayTaskHandle);
  xTaskCreate(xButtonHandlerTask, "xButton Handler Task", 256, NULL, tskIDLE_PRIORITY + 1, &xButtonHandlerTaskHandle);
  Serial.println("Boot complete!");
  vTaskStartScheduler();  // Start task scheduler
}

void loop() {
}

void Blink(byte PIN, int DELAY_MS) {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);
  vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
  //delay(DELAY_MS);
  //    delay(100);
  //  vTaskDelay( DELAY_MS / portTICK_PERIOD_MS ); // Sleep for 100ms
  digitalWrite(PIN, LOW);
}

/*
void xBlinkTask(void *pvParameters)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  vTaskDelay( DELAY_MS/portTICK_PERIOD_MS );
  //delay(DELAY_MS);
//    delay(100);
//  vTaskDelay( DELAY_MS / portTICK_PERIOD_MS ); // Sleep for 100ms
  digitalWrite(PIN,LOW);
}
*/

void print_value(char* vname, char* value, const __FlashStringHelper* sep) {
  Serial.print(vname);
  Serial.print(F(":"));
  Serial.print(value);
  Serial.print(sep);
}

void print_value(char* vname, int value, const __FlashStringHelper* sep) {
  Serial.print(vname);
  Serial.print(F(":"));
  Serial.print(value);
  Serial.print(sep);
}

void print_value(char* vname, float value, const __FlashStringHelper* sep) {
  Serial.print(vname);
  Serial.print(F(":"));
  Serial.print(value, 1);
  Serial.print(sep);
}

void print_value(char* vname, long value, const __FlashStringHelper* sep) {
  Serial.print(vname);
  Serial.print(F(":"));
  Serial.print(value);
  Serial.print(sep);
}

void print_value(char* vname, uint32_t value, const __FlashStringHelper* sep) {
  Serial.print(vname);
  Serial.print(F(":"));
  Serial.print(value);
  Serial.print(sep);
}

void decode_packet(RadioData* rd) {

  // for more about the protocol see:
  // https://github.com/dekay/DavisRFM69/wiki/Message-Protocol
  int val;
  byte* packet = rd->packet;

  Serial.print(F("raw:"));
  printHex(packet, 10);
  Serial.print(F(", "));
  print_value("station", packet[0] & 0x7, F(", "));


  Serial.print(F("packets:"));
  Serial.print(radio.packets);
  Serial.print('/');
  Serial.print(radio.lostPackets);
  Serial.print('/');
  Serial.print((float)(radio.packets * 100.0 / (radio.packets + radio.lostPackets)));
  Serial.print(F(", "));

  print_value("channel", rd->channel, F(", "));
  print_value("rssi", -rd->rssi, F(", "));

  print_value("batt", (char*)(packet[0] & 0x8 ? "err" : "ok"), F(", "));

  // All packet payload values are printed unconditionally, either properly
  // calculated or flagged with a special "missing sensor" value, mostly -1.
  // It's the CPE's responsibility to interpret our output accordingly.

  byte id = radio.DATA[0] & 7;
  int stIx = radio.findStation(id);

  // wind data is present in every packet, windd == 0 (packet[2] == 0) means there's no anemometer
  if (packet[2] != 0) {
    if (stations[stIx].type == STYPE_VUE) {
      val = (packet[2] << 1) | (packet[4] & 2) >> 1;
      val = round(val * 360 / 512);
    } else {
      val = 9 + round((packet[2] - 1) * 342.0 / 255.0);
    }
  } else {
    val = 0;
  }
  print_value("windv", packet[1], F(", "));
  print_value("winddraw", packet[2], F(", "));
  print_value("windd", val, F(", "));

  switch (packet[0] >> 4) {

    case VP2P_UV:
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3ff) {
        print_value("uv", (float)(val / 50.0), F(", "));
      } else {
        print_value("uv", -1, F(", "));
      }
      break;

    case VP2P_SOLAR:
      val = word(packet[3], packet[4]) >> 6;
      if (val < 0x3fe) {
        print_value("solar", (float)(val * 1.757936), F(", "));
      } else {
        print_value("solar", -1, F(", "));
      }
      break;

    case VP2P_RAIN:
      if (packet[3] == 0x80) {
        print_value("rain", -1, F(", "));
      } else {
        print_value("rain", packet[3], F(", "));
      }
      break;

    case VP2P_RAINSECS:
      // light rain:  byte4[5:4] as value[9:8] and byte3[7:0] as value[7:0] - 10 bits total
      // strong rain: byte4[5:4] as value[5:4] and byte3[7:4] as value[3:0] - 6 bits total
      val = (packet[4] & 0x30) << 4 | packet[3];
      if (val == 0x3ff) {
        print_value("rainsecs", -1, F(", "));
      } else {
        if ((packet[4] & 0x40) == 0) val >>= 4;  // packet[4] bit 6: strong == 0, light == 1
        print_value("rainsecs", val, F(", "));
      }
      break;

    case VP2P_TEMP:
      //if (packet[3] == 0xff) {
      //        print_value("temp", -100, F(", "));
      //} else {
      {
        //val = (int)packet[3] << 4 | packet[4] >> 4;
        //val = (packet[3]* 256 + packet[4]) / 160;
        val = ((int16_t)((packet[3] << 8) | packet[4])) / 16;
        print_value("temp", (float)(val / 10.0), F(", "));
#ifdef _USE_TH_SENSOR
        temperature = (float)(val / 10.0);
        tempf = temperature;
        tempc = (temperature - 32) * 5 / 9;
#endif
      }
      break;

    case VP2P_HUMIDITY:
      val = ((packet[4] >> 4) << 8 | packet[3]) / 10;  // 0 -> no sensor
      print_value("rh", (float)val, F(", "));
#ifdef _USE_TH_SENSOR
      humidity = (float)val;
#endif
      break;

    case VP2P_WINDGUST:
      print_value("windgust", packet[3], F(", "));
      // gustref is the index of the last message 9 packet containing the gust (max wind speed).
      if (packet[3] != 0) {
        print_value("gustref", packet[5] & 0xf0 >> 4, F(", "));
      }
      break;

    case VP2P_SOIL_LEAF:
      // currently not processed but algorithm is known
      // see https://github.com/matthewwall/weewx-meteostick/blob/master/bin/user/meteostick.py
      print_value("soilleaf", -1, F(", "));
      break;

    case VUEP_VCAP:
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
      print_value("vcap", (float)(val / 100.0), F(", "));
      break;

    case VUEP_VSOLAR:
      val = (packet[3] << 2) | (packet[4] & 0xc0) >> 6;
      print_value("vsolar", (float)(val / 100.0), F(", "));
  }

  print_value("fei", round(rd->fei * RF69_FSTEP / 1000), F(", "));
  print_value("delta", rd->delta, F(""));

  Serial.println();
  Serial.flush();
}

void printHex(volatile byte* packet, byte len) {
  for (byte i = 0; i < len; i++) {
    if (!(packet[i] & 0xf0)) Serial.print('0');
      Serial.print(packet[i], HEX);
    if (i < len - 1) Serial.print('-');
  }
}

static void xReadRadioTask(void* pvParameters) {
  long startTime, endTime;
  uint32_t ulInterruptStatus;
  bool PayloadReady = false;
  bool Timeout = false;

  pinMode(RFM69_TIMEOUT, INPUT_PULLUP);

  attachInterrupt(RFM69_TIMEOUT, timeoutISR, RISING),    // DIO1 RFM69 Timeout
  attachInterrupt(RFM69_INT, interruptHandler, CHANGE);  // DIO0 PayloadReady

  while (true) {

    if (xSemaphoreTake(SPIBusSemaphore, 1)) {  // we need eth0 semaphore to update time over NTP
      //Serial.println("xRadio Task has Mutex");
      if ((ulInterruptStatus & 0x01) != 0x00) {
        PayloadReady = true;
      } else {
        PayloadReady = false;
      }
      if ((ulInterruptStatus & 0x02) != 0x00) {
        Timeout = true;
      } else {
        Timeout = false;
      }
      xTaskNotifyStateClear(NULL);
      // Check PayloadReady interrupt and clear the FIFO if needed
      if (PayloadReady) {  // PayloadReady task notification from ISR
        if (radio._mode == RF69_MODE_RX && (radio.readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
          radio.FEI = word(radio.readReg(REG_FEIMSB), radio.readReg(REG_FEILSB));
          radio.setMode(RF69_MODE_STANDBY);
          radio.select();                 // Select RFM69 module, disabling interrupts
          SPI.transfer(REG_FIFO & 0x7f);  // Initiates FIFO transfer

          for (int i = 0; i < DAVIS_PACKET_LEN; i++) radio.DATA[i] = radio.reverseBits(SPI.transfer(0));  // Reads FIFO

          radio._packetReceived = true;

          radio.handleRadioInt();

          radio.unselect();  // Unselect RFM69 module, enabling interrupts
          //ulTaskNotifyValueClearIndexed(NULL,0,0x01); // Clear notification bit for PayloadReady interrupt
          //xTaskNotify(xWatchdogTaskHandle,0x01,eSetBits);
          PayloadReady = false;
        }
      }
      // blink the LED if necessary
      if (radio.mode == SM_RECEIVING) {
        digitalWrite(LED, HIGH);
      } else if (radio.mode == SM_SEARCHING) {
        //    Blink(LED, 15);
        //    Blink(LED, 15);
        //    Blink(LED, 15);
        //delay(100);
        //    vTaskDelay( 100/portTICK_PERIOD_MS );
      } else {
        digitalWrite(LED, LOW);
      }
      radio.loop();

      if (radio.fifo.hasElements()) {
        decode_packet(radio.fifo.dequeue());
      } else {
      }

      //  Serial.println("Radio Task gives up mutex");
      xSemaphoreGive(SPIBusSemaphore);
    }

    // need to share the mcu but wake up quickly if a packet arrives or timeout occurs
    // In theory we could sleep indefinitely and switch channels on timeout;
    // the original loop() tries to set the channel about 15ms before the tx is expected.
    // slow loop times cause high packet loss. Yield() would give better packet loss results
    // but the high priority task would rule the mcu.
    xTaskNotifyWait(0, 0, &ulInterruptStatus, 5);
  }
}

/* 
// NTP Task polls NTP and updates the RTC
// The Davis ISS transmitter hops between 51 channels. 
// Thus it can take 1 or more minutes to sync with
// a Davis ISS transmitter. 
// 
// The NTP client has a 6 second timeout. Its important
// we poll NTP less than once per minute to allow the radio
// to find and sync a station. 
// Need to write a thread-safe NTP client. 
*/
static void xNTPClientTask(void* pvParameters) {
  long taskStart, taskEnd;
  //timeClient.setUpdateInterval(60000);
  while (true) {
    taskStart = millis();
    bool shouldUpdateRTC = false;              // Only update RTC if NTP update is successfull
    if (xSemaphoreTake(SPIBusSemaphore, 1)) {  // we need SPI bus semaphore to update time over NTP
                                               //Serial.println("NTP Task has mutex");
                                               // timeClient can block for up to 6s for a timeout, or 10ms for a local time server.
                                               // may need figure out how to change the timeout to 100ms or better.
      if (timeClient.forceUpdate()) {          // since we need to hold the SPI bus, it makes more sense to force an update rather than poll the function using update(). Just need to force update less than once per minute.
        if (timeClient.isTimeSet()) {          // sanity check
          shouldUpdateRTC = true;              // Update the RTC only when necessary
          //debug strings
          //Serial.println("Updated RTC time!");
          now = DateTime(timeClient.getEpochTime());
          //String timedate = String(now.year())+String("-")+String(now.month())+String("-")+String(now.day())+String(" ")+String(now.hour())+String(":")+String(now.minute())+String(":")+String(now.second());
        }
      } else {
        //          Serial.println("Could not update NTP time!");
      }
      xSemaphoreGive(SPIBusSemaphore);
      //    Serial.println("NTP task gives up mutex");
    } else {
      Serial.println("NTP task could not obtain mutex");
    }
//taskYIELD();
// NTP may only return true once per minute. Update the RTC only if necessary.
#ifdef _USE_RTC
    if (shouldUpdateRTC) {
      if (xSemaphoreTake(I2CBusSemaphore, 5000)) {
        rtc.adjust(DateTime(timeClient.getEpochTime()));
        now = rtc.now();
        xSemaphoreGive(I2CBusSemaphore);
      }
    } else {
    }
#endif
    Serial.println(now.timestamp());
    Serial.flush();
    taskEnd = millis() - taskStart;
    //Serial.print("NTP task: "); Serial.println(taskEnd);
    // Sleep
    if (shouldUpdateRTC) {  // If the time was updated, sleep for 10 minutes
      vTaskDelay(600000 / portTICK_PERIOD_MS);
    } else {  // If the time could not be updated, sleep for just 2 minutes.
      vTaskDelay(120000 / portTICK_PERIOD_MS);
    }
  }
}  // end of thread

// DIO 3 Sync Address Found
// The sync word is the start of the packet.
// we can use the time the packet arrived to sync
// with the transmitter.
void SyncAddressISR() {
  radio.SyncAddressSeen = micros();
  //Serial.print(radio.SyncAddressSeen);
  //Serial.println(": Sync Word ");
}

// DIO 2 Fifo Not Empty
// Copy the timestamp the Sync Address was seen.
// This is to grab the timestamp in case another
// sync address is seen before PayloadReady
void FifoNotEmptyISR() {
  radio.Fifo_Not_Empty = micros();
}

// DIO 1 Timeout ISR
// Radio will give a timeout interrupt if RSSI threshold is not reached.
// used to wake up the main task to change channels if necessary.
void timeoutISR() {
  BaseType_t xHigherPriorityTaskWoken;
  //uint32_t ulStatusRegister;
  xTaskNotifyFromISR(xReadRadioTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
#ifdef DAVISRFM69_DEBUG
  Serial.println("Timeout");
#endif
}
// DIO0 PayloadReady ISR
// Tells the main task data is available in the FIFO
void interruptHandler() {
  uint32_t ulStatusRegister;
  BaseType_t pxHigherPriorityTaskWoken;
  //    Serial.println("Interrupt");
  radio.PayloadReady = digitalRead(RFM69_INT);
  if (radio.PayloadReady) {
    radio.PayloadReadyTicks = xTaskGetTickCount();
    xTaskNotifyFromISR(xReadRadioTaskHandle, 0x01, eSetBits, &pxHigherPriorityTaskWoken);
  }
}

// the current mode (rx/tx/standby/sleep) is "ready"
void ModeReadyInterrupt() {
  radio.ModeReady = digitalRead(6);
}

// Webserver task polls for web clients and serves webpages
// to be inplemented
static void xHTTPUpdateTask(void* pvParameters) {
  char PostActionBuf[64];
  while (true) {
    if (xSemaphoreTake(SPIBusSemaphore, 5)) {
      HTTPClient = server.available();
      if (HTTPClient) {
        Serial.println("new client");
        // an HTTP request ends with a blank line
        bool currentLineIsBlank = true;
        if (HTTPClient.connected()) {
 //         PostParser postParser = PostParser(HTTPClient);  // create our parser
          while (HTTPClient.available()) {
            char c = HTTPClient.read();
 //           postParser.addHeaderCharacter(c);  // compose the header
            Serial.write(c);
            // if you've gotten to the end of the line (received a newline
            // character) and the line is blank, the HTTP request has ended,
            // so you can send a reply
            if (c == '\n' && currentLineIsBlank) {

/*
              postParser.grabPayload();
              postParser.getField("i").toCharArray(PostActionBuf, sizeof(PostActionBuf));
              Serial.println(PostActionBuf);
              postParser.getField("p").toCharArray(PostActionBuf, sizeof(PostActionBuf));
              Serial.println(PostActionBuf);

              Serial.println(postParser.getHeader());   // print the header for debugging
              delay(10);                                //used to make sure the 2 serial prints don't overlap each other
              Serial.println(postParser.getPayload());  // print the payload for debugging
*/

              // send a standard HTTP response header
              HTTPClient.println("HTTP/1.1 200 OK");
              HTTPClient.println("Content-Type: text/html");
              HTTPClient.println("Connection: close");  // the connection will be closed after completion of the response
                                                        // HTTPClient.println("Refresh: 5");  // refresh the page automatically every 5 sec
              HTTPClient.println();
              HTTPClient.println("<!DOCTYPE HTML>");
              HTTPClient.println("<html>");

              //String Page;
              HTTPClient.println("<!DOCTYPE html><html lang='en'><head>");
              HTTPClient.println("<meta name='viewport' content='width=device-width'>");
              HTTPClient.println("<title>CaptivePortal</title></head><body>");
              HTTPClient.println("<nav style='border:1px solid black;'>");
              HTTPClient.println("<ul>");
              HTTPClient.println("<a href='/weather'>weather</a> | ");
              HTTPClient.println("<a href='/wifi'>wifi</a> | ");
              HTTPClient.println("<a href='/sensors'>sensors</a> | ");
              HTTPClient.println("<a href='/config'>config</a>");
              HTTPClient.println("</ul>");
              HTTPClient.println("</nav>");
              // User Credentials section
              HTTPClient.println("\r\n<form method='POST' action='savedevicecredentials'>");
              HTTPClient.println("<form>");
              HTTPClient.println("<h1>Device Credentials</h1>");
              HTTPClient.println("<Label for 'DevicePassword'>Device Password:</label>");
              HTTPClient.println("<Input type='password' id='DevicePassword' name='d'");
              HTTPClient.println("<input type='submit' value='Save'/>");
              HTTPClient.println("</form>");
              HTTPClient.println("<h1>network</h1>");
              HTTPClient.println("</table>");
              HTTPClient.println("\r\n<br /><form method='POST' action='networksave'>");
              HTTPClient.println("<label>NTP Server</label>");
              HTTPClient.println("<input type='url' placeholder='pool.ntp.org'");
              //  HTTPClient.println F("'network'");
              HTTPClient.println(" name='n'/>");
              HTTPClient.println("<br /><label>DNS Server</label>");
              HTTPClient.println("<input type='text' placeholder='password' name='p'/>");
              HTTPClient.println("<br /><input type='submit' value='Save'/></form>");
              HTTPClient.println("<br />");

              HTTPClient.println("<h1>wunderground</h1>");
              HTTPClient.println("</table>");
              HTTPClient.println("<form method='POST' action='wundergroundsave'>");
              HTTPClient.println("<label>Enable</label>");
              HTTPClient.println("<input type='checkbox' ");
              //  if(WundergroundInfceEnable==true){
              //    HTTPClient.println("checked");
              //  }
              HTTPClient.println(" name='e'/><br />");

              //  HTTPClient.println("<Label for 'WundergroundTimeSource'>Time Source </label>");
              //  HTTPClient.println("<select name='z' id='WundergroundTimeSource'>");
              //  HTTPClient.println("<option value='1'");if(WundergroundTimeSource==WU_NOW_T){HTTPClient.println("selected");} HTTPClient.println(">&now</option>");
              //  #ifdef _USE_RTC
              //    HTTPClient.println("<option value='2'");if(WundergroundTimeSource==WU_RTC_T){HTTPClient.println("selected");} HTTPClient.println(">rtc</option>");
              //  #endif
              //  HTTPClient.println("<option value='3'");if(WundergroundTimeSource==WU_NTP_T){HTTPClient.println("selected");} HTTPClient.println(">ntp</option>");
              //  HTTPClient.println("</select><br />");

              HTTPClient.println("<label>Wunderground ID</label>");
              HTTPClient.println("<input type='text' placeholder='ID'");

              //  if(strlen(WundergroundStationID)>0){
              //      HTTPClient.println(" value='");
              //HTTPClient.println String(WundergroundStationID);
              //      HTTPClient.println("'");
              //  }
              HTTPClient.println(" name='i'/>");
              HTTPClient.println("<br /><label>Wunderground Password</label>");
              HTTPClient.println("<input type='text' placeholder='password' value='");
              //HTTPClient.println String(WundergroundStationPassword);
              HTTPClient.println("' name='p'/><br />");
              HTTPClient.println("<Label for 'WundergroundThermometerID'>Thermometer ID </label>");
              HTTPClient.println("<select name='t' id='WundergroundThermometerID'>");
              //    if(UseCelcius){
              //      HTTPClient.println("<option value='");Page + String(WU_S_TEMPC_T);HTTPClient.println("'");if(thermometer1Type==WU_S_TEMPC_T){HTTPClient.println("selected");} HTTPClient.println(">tempc</option>");
              //      HTTPClient.println("<option value='");Page + String(WU_S_TEMPC2_T);HTTPClient.println("'");if(thermometer1Type==WU_S_TEMPC2_T){HTTPClient.println("selected");} HTTPClient.println(">tempc2</option>");
              //      HTTPClient.println("<option value='");Page + String(WU_S_TEMPC3_T);if(thermometer1Type==WU_S_TEMPC3_T){HTTPClient.println F("selected");} HTTPClient.println F(">tempc3</option>");
              //      HTTPClient.println("<option value='");Page + String(WU_S_TEMPC4_T);if(thermometer1Type==WU_S_TEMPC4_T){HTTPClient.println F("selected");} HTTPClient.println F(">tempc4</option>");
              //      HTTPClient.println("<option value='");Page + String(WU_S_INDOORTEMPC_T);if(thermometer1Type==WU_S_INDOORTEMPC_T){HTTPClient.println F("selected");} HTTPClient.println F(">indoortempc</option>");
              //      HTTPClient.println("<option value='");Page + String(WU_S_SOILTEMPC_T);if(thermometer1Type==WU_S_SOILTEMPC_T){HTTPClient.println F("selected");} HTTPClient.println F(">soiltempc</option>");
              //      HTTPClient.println("<option value='7'");if(thermometer1Type==7){HTTPClient.println F("selected");} HTTPClient.println F(">soiltempc2</option>");
              //      HTTPClient.println("<option value='8'");if(thermometer1Type==8){HTTPClient.println F("selected");} HTTPClient.println F(">soiltempc3</option>");
              //    }
              //    else
              //    {
              //      HTTPClient.println F("<option value='9'");if(thermometer1Type==9){HTTPClient.println F("selected");} HTTPClient.println F(">tempf</option>");
              //      HTTPClient.println F("<option value='10'");if(thermometer1Type==10){HTTPClient.println F("selected");} HTTPClient.println F(">tempf2</option>");
              //      HTTPClient.println F("<option value='11'");if(thermometer1Type==11){HTTPClient.println F("selected");} HTTPClient.println F(">tempf3</option>");
              //      HTTPClient.println F("<option value='12'");if(thermometer1Type==12){HTTPClient.println F("selected");} HTTPClient.println F(">tempf4</option>");
              //      HTTPClient.println F("<option value='13'");if(thermometer1Type==13){HTTPClient.println F("selected");} HTTPClient.println F(">indoortempf</option>");
              //      HTTPClient.println F("<option value='14'");if(thermometer1Type==14){HTTPClient.println F("selected");} HTTPClient.println F(">soiltemptempf</option>");
              //      HTTPClient.println F("<option value='15'");if(thermometer1Type==15){HTTPClient.println F("selected");} HTTPClient.println F(">soiltempf2</option>");
              //      HTTPClient.println F("<option value='16'");if(thermometer1Type==16){HTTPClient.println F("selected");} HTTPClient.println F(">soiltemp34</option>");
              //    }
              //  HTTPClient.println F("</select><br />");
              //
              HTTPClient.println("<Label for 'WundergroundHumidity1ID'>Humidity Sensor ID </label>");
              HTTPClient.println("<select name='h' id='WundergroundHumidity1ID'>");
              //HTTPClient.println F("<option value='18'");if(humidity1_sensor_type==WU_S_HUMIDITY_T){HTTPClient.println F("selected");} HTTPClient.println F(">humidity</option>");
              //HTTPClient.println F("<option value='19'");if(humidity1_sensor_type==WU_S_INDOORHUMIDITY_T){HTTPClient.println F("selected");} HTTPClient.println F(">indoorhumidity</option>");
              HTTPClient.println("</select><br />");
              HTTPClient.println("<br /><input type='submit' value='Save'/></form>");
              HTTPClient.println("<br />");

              HTTPClient.println("<label>");
              //HTTPClient.println String(WundergroundHTTPString);
              HTTPClient.println("</label>");
              HTTPClient.println("<br />");

              HTTPClient.println("<label>");
              // HTTPClient.println String(WundergroundResponseCode);
              HTTPClient.println("</label>");
              HTTPClient.println("<br />");

              HTTPClient.println("<label>");
              // HTTPClient.println(WundergrondResponse);
              HTTPClient.println("</label>");
              HTTPClient.println("<br />");

              HTTPClient.println("<h2>Restore Defaults</h2>");
              HTTPClient.println("\r\n<br /><form method='POST' action='restoredefaults'><br />");
              HTTPClient.println("<br /><input type='submit' value='Save'/></form>");

              HTTPClient.println("</body>");

              HTTPClient.println("</html>");
              break;
            }
            if (c == '\n') {
              // you're starting a new line
              currentLineIsBlank = true;
            } else if (c != '\r') {
              // you've gotten a character on the current line
              currentLineIsBlank = false;
            }
          }
        }
        // give the web browser time to receive the data
        delay(1);
        // close the connection:
        HTTPClient.stop();
      }
      // Give up the semaphore and sleep 250ms
      xSemaphoreGive(SPIBusSemaphore);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    //taskYIELD();
  }
}



static void xDataSamplerTask(void* pvParameters) {
  int DataLogIndex = 0;
  vTaskDelay(60000 / portTICK_PERIOD_MS);
  while (true) {
    if (xSemaphoreTake(I2CBusSemaphore, 5)) {
      now = rtc.now();
      xSemaphoreGive(I2CBusSemaphore);
    }
    DatalogString = "";
    DatalogString += "time:" + now.timestamp() + ",";
    DatalogString += "temp:";
    DatalogString += String(temperature);
    DatalogString += "F,";
    DatalogString += "rh:";
    DatalogString += String(humidity);
    //  DatalogString += 13;//cr
    //  DatalogString += 10;//lf
    DatalogString.toCharArray(&DataStrings.DataStringArray[DataLogIndex].TheString[0], DATASTRINGLENGTH);
    xQueueSend(DataStringQueueHandle, &DataStrings.DataStringArray[DataLogIndex].TheString[0], 1000);
    Serial.println(DataStrings.DataStringArray[DataLogIndex].TheString);
    DataLogIndex++;
    if (DataLogIndex >= MAXDATASTRINGS) { DataLogIndex = 0; }
    vTaskDelay(30000 / portTICK_PERIOD_MS);
  }
}

#define DL_PGM_STATE_INIT (byte)0
#define DL_PGM_STATE_READ_QUEUE (byte)5
#define DL_PGM_STATE_OPEN_FILE (byte)10
#define DL_PGM_STATE_OPEN_FOLDER (byte)9
#define DL_PGM_STATE_WRITE_FILE (byte)15
#define DL_PGM_STATE_IDLE (byte)99
#define DL_PGM_STATE_NO_SD_CARD (byte)255

char DataPayload[DATASTRINGLENGTH];
File dataFile;

static void xDataloggerTask(void* pvParameters) {
  byte DL_pgm_state = DL_PGM_STATE_INIT;
  vTaskDelay(60000 / portTICK_PERIOD_MS);
  while (true) {
    if (xSemaphoreTake(SPIBusSemaphore, 5)) {
      switch (DL_pgm_state) {
        case DL_PGM_STATE_INIT:
          {
            if (SD.begin(9)) {
              Serial.println("SD card found!");
              DL_pgm_state = DL_PGM_STATE_READ_QUEUE;
            } else {
              Serial.println("No SD card found!");
              DL_pgm_state = DL_PGM_STATE_NO_SD_CARD;
            }

            break;
          }
        case DL_PGM_STATE_READ_QUEUE:
          {
            if (DataStringQueueHandle == NULL) {  // No Queue
              DL_pgm_state = DL_PGM_STATE_IDLE;
              break;
            } else if (xQueueReceive(DataStringQueueHandle, &DataPayload, 1) == pdPASS) {  // Item found in the queue
              //Serial.println("Item found in Queue!");
              DL_pgm_state = DL_PGM_STATE_OPEN_FILE;
            } else {
              DL_pgm_state = DL_PGM_STATE_IDLE;  // nothing in the queue
            }
            break;
          }
        case DL_PGM_STATE_OPEN_FOLDER:
          {
            break;
          }
        case DL_PGM_STATE_OPEN_FILE:
          {
            dataFile = SD.open("datalog.txt", FILE_WRITE);  // Try opening the file
            if (dataFile) {                                 // we opened the file
              dataFile.println(DataPayload);
              dataFile.close();

              if (xQueuePeek(DataStringQueueHandle, &DataPayload, 1) == pdTRUE) {
                DL_pgm_state = DL_PGM_STATE_READ_QUEUE;
              } else {
                DL_pgm_state = DL_PGM_STATE_IDLE;
              }
            } else {
              Serial.println("Could not open file");
              DL_pgm_state = DL_PGM_STATE_INIT;
            }
            break;
          }
        case DL_PGM_STATE_WRITE_FILE:
          {
            if (dataFile) {
              dataFile.println(DataPayload);
              dataFile.close();
              if (xQueuePeek(DataStringQueueHandle, &DataPayload, 1) == pdTRUE) {
                DL_pgm_state = DL_PGM_STATE_READ_QUEUE;
              } else {
                DL_pgm_state = DL_PGM_STATE_IDLE;
              }

            } else {
              DL_pgm_state = DL_PGM_STATE_INIT;
            }
            break;
          }
        case DL_PGM_STATE_NO_SD_CARD:
          {
            Serial.println("No SD card! could not init datalogger. ");
            DL_pgm_state = DL_PGM_STATE_INIT;
            break;
          }
        case DL_PGM_STATE_IDLE:
          {
            if (xQueuePeek(DataStringQueueHandle, &DataPayload, 1) == pdTRUE) {
              DL_pgm_state = DL_PGM_STATE_READ_QUEUE;
            } else {
              DL_pgm_state = DL_PGM_STATE_IDLE;
            }
            break;
          }
        default:
          {
            DL_pgm_state = DL_PGM_STATE_INIT;
            break;
          }
      }
      // open the file
      //
      xSemaphoreGive(SPIBusSemaphore);
    }       //  end try to get semaphore
    else {  // could not obtain semaphore
    }
    if (DL_PGM_STATE_IDLE) {
      vTaskDelay(10000 / portTICK_PERIOD_MS);
    } else {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

enum xDisplayAction{
  DISPLAY_SET,
  DISPLAY_UPDATE,
  DISPLAY_SAVE,
  DISPLAY_OK,
  DISPLAY_NEXT_FIELD
};

enum xButtonState {
  BUTTON_UP,
  BUTTON_DOWN
};

enum xButtonEventType{
  BUTTON_PRESSED,
  BUTTON_RELEASED,
  BUTTON_HELD
};

class xButton {
public:
  volatile xButtonState ButtonState;
  volatile xButtonState OldButtonState;
  void (*button_action_ptr)(void);        // Function pointer for a Button Action
  void (*button_press_handler)(void);
  void (*button_release_handler)(void);
  uint32_t buttonDownTime, buttonUpTime;  // time the button was pushed or released.
  xButton(uint8_t Pin);
  void init(void (*ptr)(void),QueueHandle_t queue);
  void setButtonHandler(void (*button_action_ptr)(void));
  uint8_t InterruptPin;
  QueueHandle_t eventQueue;
private:
  static void isr(void);
};

xButton::xButton(uint8_t Pin) {
  InterruptPin = Pin;
}

void xButton::init(void (*ptr)(void),QueueHandle_t queue) {
  eventQueue = queue;
  button_action_ptr=ptr;
  pinMode(InterruptPin, INPUT_PULLUP); 
  attachInterrupt(InterruptPin, button_action_ptr, CHANGE);
}

void xButton::isr(){
}

class xButtonEvent{
  public:
  xButtonEvent(xButton* _Button,xButtonEventType _EventType);
  xButton *Button;
  xButtonEventType EventType;
};

xButtonEvent::xButtonEvent(xButton* _Button,xButtonEventType _EventType){
  Button = _Button;
  EventType = _EventType;
}

QueueHandle_t ButtonEventQueue;
QueueHandle_t DisplayQueue;

#define UP_PIN 11
#define DOWN_PIN 5
#define ENTER_PIN 6
#define DEBOUNCE_MILLIS 250

xButton up(UP_PIN);
xButton down(DOWN_PIN);
xButton enter(ENTER_PIN);

void xUp(){
  if(digitalRead(up.InterruptPin)){
    up.ButtonState = BUTTON_UP;
  } else {
    up.ButtonState = BUTTON_DOWN;
  }
  if((up.OldButtonState == BUTTON_UP)&(up.ButtonState==BUTTON_DOWN)){ // button was pressed
    if(millis()-up.buttonDownTime>DEBOUNCE_MILLIS){// Last button press was past the Debounce time
      xButtonEvent ButtonPress(&up,BUTTON_PRESSED);
      xQueueSend(up.eventQueue,&ButtonPress, 1000);
    }
  } else { // button was released 
  if(millis()-up.buttonUpTime>DEBOUNCE_MILLIS){// last button release was past the Debounce time
      xButtonEvent ButtonRelease(&up,BUTTON_RELEASED);
      xQueueSend(up.eventQueue,&ButtonRelease, 1000);
    }
  }
  if(up.ButtonState==BUTTON_UP){
      up.buttonUpTime = millis();}
  else {
    up.buttonDownTime = millis();}
    up.OldButtonState = up.ButtonState;
}

void xDown(){
  if(digitalRead(down.InterruptPin)){
    down.ButtonState = BUTTON_UP;
  } else {
    down.ButtonState = BUTTON_DOWN;
  }
  if((down.OldButtonState == BUTTON_UP)&(down.ButtonState==BUTTON_DOWN)){ // button was pressed
    if(millis()-down.buttonDownTime>DEBOUNCE_MILLIS){// Last button press was past the Debounce time
      xButtonEvent ButtonPress(&down,BUTTON_PRESSED);
      xQueueSend(down.eventQueue,&ButtonPress, 1000);
    }
  } else { // button was released 
  if(millis()-down.buttonUpTime>DEBOUNCE_MILLIS){// last button release was past the Debounce time
      xButtonEvent ButtonRelease(&down,BUTTON_RELEASED); // Create a button event for the queue
      xQueueSend(down.eventQueue,&ButtonRelease, 1000);  // Queue the button event to the button handler
    }
  }
  if(down.ButtonState==BUTTON_UP){
      down.buttonUpTime = millis();}
  else {
    down.buttonDownTime = millis();}
    down.OldButtonState = down.ButtonState;
}

void xEnter(){
  if(digitalRead(enter.InterruptPin)){
    enter.ButtonState = BUTTON_UP;
  } else {
    enter.ButtonState = BUTTON_DOWN;
  }
  if((enter.OldButtonState == BUTTON_UP)&(enter.ButtonState==BUTTON_DOWN)){ // button was pressed
    if(millis()-enter.buttonDownTime>DEBOUNCE_MILLIS){// Last button press was past the Debounce time
      xButtonEvent ButtonPress(&enter,BUTTON_PRESSED);
      xQueueSend(enter.eventQueue,&ButtonPress, 1000);
    }
  } else if ((enter.OldButtonState == BUTTON_DOWN)&(enter.ButtonState==BUTTON_UP)){ // button was released 
  if(millis()-enter.buttonUpTime>DEBOUNCE_MILLIS){// last button release was past the Debounce time
      xButtonEvent ButtonRelease(&enter,BUTTON_RELEASED);
      xQueueSend(enter.eventQueue,&ButtonRelease, 1000);
    }
  }
  if(enter.ButtonState==BUTTON_UP){
      enter.buttonUpTime = millis();}
  else {
    enter.buttonDownTime = millis();}
    enter.OldButtonState = enter.ButtonState;
}

xButtonEvent ButtonEvent(&enter,BUTTON_PRESSED);

static void xButtonHandlerTask(void* pvParameters) {

  ButtonEventQueue = xQueueCreate(2,sizeof(xButtonEvent));

  up.init(xUp,ButtonEventQueue);
  down.init(xDown,ButtonEventQueue);
  enter.init(xEnter,ButtonEventQueue);

  up.button_press_handler=xUpPress;
  up.button_release_handler=xUpRelease;

  down.button_press_handler=xDownPress;
  down.button_release_handler=xDownRelease;

  enter.button_press_handler=xEnterPress;
  enter.button_release_handler=xEnterRelease;

  while (true) {
    if(xQueueReceive(ButtonEventQueue, &ButtonEvent, 5000) == pdTRUE){
      if(ButtonEvent.EventType==BUTTON_PRESSED){
        if(!ButtonEvent.Button->button_press_handler==NULL){
          ButtonEvent.Button->button_press_handler();
        }
      } else if(ButtonEvent.EventType==BUTTON_RELEASED){
        if(!ButtonEvent.Button->button_release_handler==NULL){
          ButtonEvent.Button->button_release_handler();
        }
      }
    }
    taskYIELD();
  }
}

enum xMenuFunction_T {
  Function,  // Menu Action is a Function
  Menu       // Menu Action loads another Menu
};

class xDisplayItem {
  public:
  virtual void init();
  virtual void display();
  virtual bool isMenu(){return false;}
};

void xDisplayItem::display(){}
void xDisplayItem::init(){}

#define MAXMENUITEMS 8

// xMenu will be a subclass of xDisplayItem
// When we render the menu we can call display()
class xMenu : public xDisplayItem {

  typedef struct xMenuItem {
  xMenuFunction_T MenuFunctionType;  // Determine if Menu Item is a Function or a sub-menu
  void (*function_ptr)(void);        // Function pointer for a Menu Action
  String text;                   // menu item
  xMenu *SubMenu;
};

  int numButtons;
  int sel = 0; // menu item selected
  int DisplayLines = 4; // max # lines to display for the menu
  int FirstLine = 0; // first menu item we should display
  int EndLine = 4; // last menu item we should display
  xMenuItem MenuItems[8]; // list of menu items to display
  int items = 0;
  public:
    void init();
    void AddMenuItemSubmenu(String _text, xMenu *_SubMenu);
    void AddMenuItemFunction(String _text, void (*ptr)(void));
    void display(); // display the menu
    virtual void xMenuUp();
    virtual void xMenuDown();
    virtual void xMenuEnter();
    bool isMenu(){return true;}
    friend class xMenuItem;
    int getSel();
};

int xMenu::getSel(){
  return sel;
}
void xMenu::init(){
  for(int i=0;i<8;i++){
    MenuItems[i].text = "";
    MenuItems[i].function_ptr = NULL;
  }
  sel=0;
  items = 0;
  FirstLine = 0;
  EndLine = 4;
}
void xMenu::display(){
  oled.clearDisplay();
  oled.setCursor(0, 0);
  for(int i=FirstLine; i<items; i++){
    if(i==sel){oled.print("*");} else {oled.print(" ");} // show * if menu line is selected
    oled.println(MenuItems[i].text); // Print the menu item to OLED
  }
  oled.display();
  //Serial.println("update menu display");
}

//void xMenu::renderMenu(){}

void xMenu::xMenuUp(){
  if(sel-1<0){sel = 0;}
    else sel--;
  if(sel<FirstLine) {FirstLine=sel;EndLine=FirstLine+DisplayLines-1;}
}
void xMenu::xMenuDown(){
  Serial.println(sel);
  Serial.println(FirstLine);

    if(sel+1<items){
      sel++;
      if(sel>=EndLine){
        FirstLine++;
      }
    }
}

void xMenu::xMenuEnter(){
  if(MenuItems[sel].MenuFunctionType==Function){
    MenuItems[sel].function_ptr();
  }
  else if (MenuItems[sel].MenuFunctionType==Menu){
  }
}

void xMenu::AddMenuItemFunction(String _text,void (*ptr)(void)){
  Serial.print("Menu Function: ");Serial.println(_text);
  if((items<MAXMENUITEMS)){
    MenuItems[items].MenuFunctionType=Function;
    MenuItems[items].text=_text;
    MenuItems[items].function_ptr=ptr;
    items++;
    if(items>EndLine){EndLine = DisplayLines;} else {EndLine = items;}
  }
}

void xMenu::AddMenuItemSubmenu(String _text, xMenu *_SubMenu){
  Serial.print("Submenu: ");Serial.println(_text);
  if((items<MAXMENUITEMS)){
    MenuItems[items].MenuFunctionType=Menu;
    MenuItems[items].text=_text;
    MenuItems[items].SubMenu=_SubMenu;
    items++;
    if(items>EndLine){EndLine = DisplayLines;} else {EndLine = items;}
  }
}

#define MaxCharList 39

const char CharList[MaxCharList] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','0','1','2','3','4','5','6','7','8','9','<','>',' '};

class TextField : public xDisplayItem{
  public:
    char InputString[64];
    int index=0; // index within the string
    int currentchar=0; // 
    void nextChar();
    void previousChar();
    void enterChar();
    void display();
};

class YNField : public xDisplayItem{
  public:
    bool YN = false;
    void setField(bool _yn);
    void next();
    void display();
};

void YNField::display(){
  if(YN==true){
    oled.print("Y");
  } else {
    oled.print("N");
  }
}

void YNField::next(){
  if(YN==true) {YN=false;}
  else {YN=true;}
}

void YNField::setField(bool _yn){
  YN=_yn;
}


/*
  @brief CHOICE field definition
*/
#ifndef CHOICETEXTLENGTH
  #define CHOICETEXTLENGTH 32
#endif
#ifndef CHOICEITEMS
  #define CHOICEITEMS 8
#endif
class Choice: public xDisplayItem{
  struct choiceItem{
    String text;
  };
    int numChoices=0;
  public:
    int sel=0;
    void addChoice(String _text);
    choiceItem choicelist[CHOICEITEMS];
    void nextChoice(); 
    void prevChoice(); 
    void enterChoice(); 
    void display();
    void init();
};

void Choice::init(){
  for(int i=0;i<CHOICEITEMS;i++){
    choicelist[i].text="";
    sel=0;
    numChoices=0;
  }
}

void Choice::addChoice(String _text){
  if(numChoices+1<CHOICEITEMS){
    choicelist[numChoices].text=_text;
    numChoices++;
  }
}

void Choice::nextChoice(){
  if(sel+1<numChoices){
    sel++;
  }
}

void Choice::prevChoice(){
  if(sel-1>=0){
    sel--;
  }
}

void Choice::enterChoice(){

}

void Choice::display(){
  oled.print(choicelist[sel].text);
}

/*
  @brief XDisplay Class Definition. User will define .init() and .update() functions. 
    update() will draw the display. .init() will setup any display fields. 
*/
class xDisplay {
  public:
    xDisplayItem *CurrentDisplayItem;
    int _currentDisplayItem;
    xMenu *TheMenu;
    Choice *TheChoice;
    TextField *TheTextField;
    YNField *TheYNField;
    bool AddDisplayItem(xDisplayItem *displayItem);
    void setMenu(xMenu *_Menu);
    virtual void init();
    virtual void update();
    TickType_t LastUpdate;
    TickType_t RefreshTimer = 5000;
    TickType_t DisplayTimeout = 0;
    virtual void NextDisplayItem();
    virtual void ok();
    virtual void saveDisplay();
  private:
    xDisplayItem *DisplayItemList;
};

void xDisplay::NextDisplayItem(){}
void xDisplay::ok(){}
void xDisplay::saveDisplay(){}

void xDisplay::setMenu(xMenu *_Menu){
  TheMenu = _Menu;
}

class xDisplayEvent{
  public:
    xDisplayAction DisplayAction;
    xDisplay *Display;
};

void xDisplay::init(){}

void xDisplay::update(){}

/*

static void xWatchdogTask(void *pvParameters){
uint32_t ulNotifiedValue;
int countdownMS = Watchdog.enable(6000000);
Watchdog.reset();
while(true){
  xTaskNotifyWait(0x01,0xFF,&ulNotifiedValue,portMAX_DELAY);
  Watchdog.reset();
  Serial.println("Watchdog fed");
}}

*/

class xDefaultDisplay:public xDisplay{
  void init();
  void update();
};

void xDefaultDisplay::init(){
  up.button_press_handler=xUpPress;
  up.button_release_handler=xUpRelease;
  down.button_press_handler=xDownPress;
  down.button_release_handler=xDownRelease;
  enter.button_press_handler=xEnterPress;
  enter.button_release_handler=xEnterRelease;
}

void xDefaultDisplay::update(){
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.print("Temp: ");
      oled.print(temperature);
      oled.println();
      oled.print("Humidity: ");
      oled.print(humidity);
      oled.println("%");
      oled.display();
      //Serial.println("default display update");
}

void TextField::display(){
  for(int i=0;i<32;i++){
    if(i==index){
      oled.print(CharList[currentchar]);
    }
    else
     oled.print(InputString[i]);
  }
}

void TextField::nextChar(){
  if(currentchar+1>MaxCharList){currentchar = MaxCharList;} else currentchar++;
  //InputString[index]=CharList[currentchar];
  Serial.println(CharList[currentchar]);
}

void TextField::previousChar(){
  if(currentchar-1<0){currentchar = 0;} else currentchar--;
  //InputString[index]=CharList[currentchar];
  Serial.println(CharList[currentchar]);
}

void TextField::enterChar(){
  if(CharList[currentchar]==' '){
    SaveTextField();}
  else {
    if(index+1<=64){
      index++;
    } else index=0;
  }
  Serial.println(index);
}

/*
  @brief The Main Menu
*/
class xMainMenuDisplay:public xDisplay{
  xMenu MainMenu;
  void init();
  void update();
};

void xMainMenuDisplay::init(){
  TheMenu = &MainMenu;
  MainMenu.init();
  MainMenu.AddMenuItemFunction("Network Status",SetNetworkStatusDisplay);
  MainMenu.AddMenuItemFunction("Stations",OpenEditStationMenu);
  MainMenu.AddMenuItemFunction("Sensors",SetNetworkStatusDisplay);
  MainMenu.AddMenuItemFunction("Interfaces",SetInterfacesDisplay);
  MainMenu.AddMenuItemFunction("Settings",SetSettingsDisplay);

  up.button_press_handler=xUpMenuPress;
  down.button_press_handler=xDownMenuPress;
  enter.button_press_handler=xEnterMenuPress;
  //Serial.println("Main Menu Init");
}

void xMainMenuDisplay::update(){
  MainMenu.display();
  //Serial.println("MainMenuDisplay");
}

class xNetStatusDisplay:public xDisplay{
  void init();
  void update();
};

void xNetStatusDisplay::init(){
}

void xNetStatusDisplay::update(){
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(Ethernet.localIP());
  oled.display();
  up.button_press_handler=SetDefaultDisplay;
  down.button_press_handler=SetDefaultDisplay;
  enter.button_press_handler=SetDefaultDisplay;
}

class xSettingsDisplay:public xDisplay{
  xMenu SettingsMenu;
  void init();
  void update();
};

void xSettingsDisplay::init(){
  TheMenu = &SettingsMenu;
  SettingsMenu.init();
  SettingsMenu.AddMenuItemFunction("Network",SetDefaultDisplay);
  SettingsMenu.AddMenuItemFunction("System",SetDefaultDisplay);
  SettingsMenu.AddMenuItemFunction("Display",SetDefaultDisplay);
  up.button_press_handler=xUpMenuPress;
  down.button_press_handler=xDownMenuPress;
  enter.button_press_handler=xEnterMenuPress;
}

void xSettingsDisplay::update(){
    TheMenu->display();
}

/*
@class xInterfacesDisplay
@brief Interface Settings Menu. User will choose which Interface to edit. 
*/
class xInterfacesDisplay:public xDisplay{
  xMenu InterfaceSettingsMenu;
  void init();
  void update();
};

void xInterfacesDisplay::init(){
  TheMenu = &InterfaceSettingsMenu;
  InterfaceSettingsMenu.init();
  InterfaceSettingsMenu.AddMenuItemFunction("Wunderground",SetWundergroundSettingsDisplay);
  up.button_press_handler=xUpMenuPress;
  down.button_press_handler=xDownMenuPress;
  enter.button_press_handler=xEnterMenuPress;
}

void xInterfacesDisplay::update(){
  TheMenu->display();
}

/*
  @class xWundergroundSettingsDisplay
  @brief Wunderground Interface Settings Menu
*/
class xWundergroundSettingsDisplay:public xDisplay{
  xMenu WundergroundSettingsMenu;
  void init();
  void update();
};

void xWundergroundSettingsDisplay::init(){
  TheMenu=&WundergroundSettingsMenu;
  WundergroundSettingsMenu.init();
  WundergroundSettingsMenu.AddMenuItemFunction("Wunderground Interface Active",SetWundergroundEditActiveDisplay);
  WundergroundSettingsMenu.AddMenuItemFunction("Wunderground Interface Name",SetWundergroundEditNameDisplay);
  WundergroundSettingsMenu.AddMenuItemFunction("Wunderground Interface Password",SetWundergroundEditPasswordDisplay);
  WundergroundSettingsMenu.AddMenuItemFunction("Wunderground Interface Sensors",OpenEditWundergroundSensorsMenu);
  up.button_press_handler=xUpMenuPress;
  down.button_press_handler=xDownMenuPress;
  enter.button_press_handler=xEnterMenuPress;
}

void xWundergroundSettingsDisplay::update(){
  TheMenu->display();
}

class xWundergroundEditStationActiveDisplay:public xDisplay{
  void init();
  void update();
  void saveDisplay();
  YNField _WundergroundStationActive;
};

class xWundergroundEditStationNameDisplay:public xDisplay{
  void init();
  void update();
  void saveDisplay();
  TextField _WundergroundStationName;
};

class xWundergroundEditStationPasswordDisplay:public xDisplay{
  void init();
  void update();
  void saveDisplay();
  TextField _WundergroundStationPassword;
};

void xWundergroundEditStationNameDisplay::saveDisplay(){
  memcpy(WundergroundStationID,_WundergroundStationName.InputString,WundergroundStationIDLength);
  SaveWundergroundCredentials();
  SetDefaultDisplay();
}

void xWundergroundEditStationPasswordDisplay::saveDisplay(){
  memcpy(WundergroundStationPassword,_WundergroundStationPassword.InputString,WundergroundStationIDLength);
  SaveWundergroundCredentials();
  SetDefaultDisplay();
}

/*
  @class xWundergroundEditStationNameDisplay
  @brief Default the Wunderground Interface Station Name field to WundergroundStationID
*/
void xWundergroundEditStationNameDisplay::init(){
  TheTextField=&_WundergroundStationName;
  memcpy(_WundergroundStationName.InputString,WundergroundStationID,64);
  up.button_press_handler=xUpTextfieldPress;
  down.button_press_handler=xDownTextfieldPress;
  enter.button_press_handler=xEnterTextfieldPress;
}
/*
  @class xWundergroundEditStationNameDisplay
  @brief Draws the Wunderground Interface Edit Station Name display 
*/
void xWundergroundEditStationNameDisplay::update(){
  oled.clearDisplay();
  oled.setCursor(0, 0);
  _WundergroundStationName.display();
  oled.display();
}
/*
  @class xWundergroundEditStationPasswordDisplay
  @brief Init function for xWundergroundEditStationPasswordDisplay class. 
    Default to WundergroundStationPassword
*/
void xWundergroundEditStationPasswordDisplay::init(){
  TheTextField=&_WundergroundStationPassword;
  memcpy(_WundergroundStationPassword.InputString,WundergroundStationPassword,64);
  up.button_press_handler=xUpTextfieldPress;
  down.button_press_handler=xDownTextfieldPress;
  enter.button_press_handler=xEnterTextfieldPress;
}

/* 
  @class xWundergroundEditStationPasswordDisplay
  @brief Draw the xWundergroundEditStationPasswordDisplay
 */
void xWundergroundEditStationPasswordDisplay::update(){
  oled.clearDisplay();
  oled.setCursor(0, 0);
  _WundergroundStationPassword.display();
  oled.display();
}


/*
  @brief Edit Davis Station list 
*/
class xWXStationMenu:public xDisplay{
  public: 
  xMenu WXStationList;
  void init();
  void update();
  void saveDisplay();
};

void xWXStationMenu::init(){
  TheMenu=&WXStationList;
  WXStationList.AddMenuItemFunction("Station 1",OpenEditStationDisplay);
  WXStationList.AddMenuItemFunction("Station 2",OpenEditStationDisplay);
  WXStationList.AddMenuItemFunction("Station 3",OpenEditStationDisplay);
  WXStationList.AddMenuItemFunction("Station 4",OpenEditStationDisplay);
  WXStationList.AddMenuItemFunction("Station 5",OpenEditStationDisplay);
  WXStationList.AddMenuItemFunction("Station 6",OpenEditStationDisplay);
  WXStationList.AddMenuItemFunction("Station 7",OpenEditStationDisplay);
  WXStationList.AddMenuItemFunction("Station 8",OpenEditStationDisplay);
  up.button_press_handler=xUpMenuPress;
  down.button_press_handler=xDownMenuPress;
  enter.button_press_handler=xEnterMenuPress;
}

void xWXStationMenu::update(){
  WXStationList.display();
}

void xWXStationMenu::saveDisplay(){

}

class xEditDavisStationDisplay:public xDisplay{
  xMenu DavisStationMenu;
  void init();
  void update();
  void saveDisplay();
};

void xEditDavisStationDisplay::init(){
  TheMenu=&DavisStationMenu;
  DavisStationMenu.AddMenuItemFunction("Active",OpenEditDavisStationActiveDisplay);
//  DavisStationMenu.AddMenuItemFunction("Sensors",OpenEditDavisStationSensorsDisplay);
  up.button_press_handler=xUpMenuPress;
  down.button_press_handler=xDownMenuPress;
  enter.button_press_handler=xEnterMenuPress;
}

  void xEditDavisStationDisplay::update(){
    oled.clearDisplay();
    oled.setCursor(0, 0);
    DavisStationMenu.display();
    oled.display();
  }

  void xEditDavisStationDisplay::saveDisplay(){}

class xEditDavisStationActiveDisplay:public xDisplay{
  public:
    YNField _DavisStationActive;
    void init();
    void update();
    void saveDisplay();
};

void xEditDavisStationActiveDisplay::init(){
  TheYNField=&_DavisStationActive;
  up.button_press_handler=xUpPressYN;
  down.button_press_handler=xDownPressYN;
  enter.button_press_handler=xEnterPressYN;
  _DavisStationActive.YN=stations[stationSel].active;

}

void xEditDavisStationActiveDisplay::update(){
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print("Active: ");
  _DavisStationActive.display();
  oled.display();
}

void xEditDavisStationActiveDisplay::saveDisplay(){
}

class xEditDavisStationSensorsDisplay:public xDisplay{
};

/*
  @brief E
*/
class xEditStationSensorsMenu:public xDisplay{
  xMenu StationSensorsMenu;
    public:
    void init();
    void update();
    void saveDisplay();
};

void xEditStationSensorsMenu::init(){
  TheMenu=&StationSensorsMenu;
 // StationSensorsMenu.
}

void xEditStationSensorsMenu::update(){
  StationSensorsMenu.display();
}

void xEditStationSensorsMenu::saveDisplay(){

}

/*
  @brief Wunderground Interface Sensor Menu. Lets the user choose which Stations and Sensors to use for the Wunderground interface. 
*/
class xEditWundergroundSensorsMenu: public xDisplay{
  xMenu WundergroundSensorsMenu;
    public:
    void init();
    void update();
    void saveDisplay();
};

void xEditWundergroundSensorsMenu::init(){
      TheMenu=&WundergroundSensorsMenu;
      WundergroundSensorsMenu.init();
      WundergroundSensorsMenu.AddMenuItemFunction("Thermometer Station:",OpenEditWundergroundEditTemperatureSensorDisplay);
      WundergroundSensorsMenu.AddMenuItemFunction("Humidity Station:",OpenEditWundergroundEditHumiditySensorDisplay);
//      WundergroundSensorsMenu.AddMenuItemFunction("Anemometer Station: ",OpenEditWundergroundAnemometerDisplay);
//      WundergroundSensorsMenu.AddMenuItemFunction("Wind Direction Station: ",OpenEditWundergroundWindDirectionSensorDisplay);
      //WundergroundSensorsMenu.AddMenuItemFunction("Barometric Pressure Station: ",);
    up.button_press_handler=xUpMenuPress;
    down.button_press_handler=xDownMenuPress;
    enter.button_press_handler=xEnterMenuPress;
    }

void xEditWundergroundSensorsMenu::update(){
  WundergroundSensorsMenu.display();
}

void xEditWundergroundSensorsMenu::saveDisplay(){
  SetDefaultDisplay();
}

/*
  Edit Humidity Sensor for Wunderground Interface
*/

class xEditWundergroundHumidityDisplay:public xDisplay{
  xMenu WundergroundHumiditySensorMenu;
    public:
    void init();
    void update();
    void saveDisplay();
};

void xEditWundergroundHumidityDisplay::init(){
  TheMenu=&WundergroundHumiditySensorMenu;
  WundergroundHumiditySensorMenu.init();
  WundergroundHumiditySensorMenu.AddMenuItemFunction("Active: ",OpenEditWundergroundEditHumidityActiveDisplay);
  WundergroundHumiditySensorMenu.AddMenuItemFunction("Station: ",OpenEditWundergroundEditHumidityStationChoiceDisplay);
  WundergroundHumiditySensorMenu.AddMenuItemFunction("Sensor: ",OpenEditWundergroundEditHumidityStationSensorChoiceDisplay);
  up.button_press_handler=xUpMenuPress;
  down.button_press_handler=xDownMenuPress;
  enter.button_press_handler=xEnterMenuPress;
}

void xEditWundergroundHumidityDisplay::update(){
  oled.clearDisplay();
  oled.setCursor(0, 0);
  WundergroundHumiditySensorMenu.display();
  oled.display();
}

void xEditWundergroundHumidityDisplay::saveDisplay(){}

/*
  @class xEditWundergroundHumidityActiveDisplay
  @brief Sets QueueHumidityForInterfaces
*/
class xEditWundergroundHumidityActiveDisplay:public xDisplay{
  YNField _Wunderground_Humidity_Sensor_Active;
    public:
    void init();
    void update();
    void saveDisplay();
};

void xEditWundergroundHumidityActiveDisplay::init(){
  TheYNField = &_Wunderground_Humidity_Sensor_Active;
  _Wunderground_Humidity_Sensor_Active.YN=QueueHumidityForInterfaces;
  up.button_press_handler=xUpPressYN;
  down.button_press_handler=xDownPressYN;
  enter.button_press_handler=xEnterPressYN;
}

void xEditWundergroundHumidityActiveDisplay::update(){
  oled.clearDisplay();
  oled.setCursor(0, 0);
  _Wunderground_Humidity_Sensor_Active.display();
  oled.display();
}

void xEditWundergroundHumidityActiveDisplay::saveDisplay(){
  QueueHumidityForInterfaces=_Wunderground_Humidity_Sensor_Active.YN;
  saveHumidityToDisk();
  SetDefaultDisplay();
 }

/*
  @brief Choose which station to pull humidity for Wunderground Interface
*/
class xEditWundergroundHumidityStationChoiceDisplay:public xDisplay{
  Choice HumidityStation;
    public:
    void init();
    void update();
    void saveDisplay();
};

    void xEditWundergroundHumidityStationChoiceDisplay::init(){}
    void xEditWundergroundHumidityStationChoiceDisplay::update(){}
    void xEditWundergroundHumidityStationChoiceDisplay::saveDisplay(){}

// Choose the Humidity sensor within the Station
class xEditWundergroundHumidityStationSensorChoiceDisplay:public xDisplay{
    public:
    void init();
    void update();
    void saveDisplay();
};

    void xEditWundergroundHumidityStationSensorChoiceDisplay::init(){}
    void xEditWundergroundHumidityStationSensorChoiceDisplay::update(){}
    void xEditWundergroundHumidityStationSensorChoiceDisplay::saveDisplay(){}

/*
  Edit Wunderground Anemometer 
*/

class xEditWundergroundAnemometerDisplay:public xDisplay{
    public:
    void init();
    void update();
    void saveDisplay();
};

void xEditWundergroundAnemometerDisplay::init(){}
void xEditWundergroundAnemometerDisplay::update(){}
void xEditWundergroundAnemometerDisplay::saveDisplay(){
  SetDefaultDisplay();
}

/*
  @class xEditWundergroundThermometerDisplay
*/
class xEditWundergroundThermometerDisplayMenu:public xDisplay{
  xMenu WundergroundThermometerSettingsMenu;
    public:
    void init();
    void update();
    void saveDisplay();
};

void xEditWundergroundThermometerDisplayMenu::init(){
  TheMenu=&WundergroundThermometerSettingsMenu;
  WundergroundThermometerSettingsMenu.init();
  WundergroundThermometerSettingsMenu.AddMenuItemFunction("Active: ",OpenEditWundergroundThermometerActiveDisplay);
  WundergroundThermometerSettingsMenu.AddMenuItemFunction("Station: ",OpenEditWundergroundThermometerStationDisplay);
  WundergroundThermometerSettingsMenu.AddMenuItemFunction("Sensor: ",OpenEditWundergroundThermometerSensorDisplay);
  up.button_press_handler=xUpMenuPress;
  down.button_press_handler=xDownMenuPress;
  enter.button_press_handler=xEnterMenuPress;
}
void xEditWundergroundThermometerDisplayMenu::update(){
  WundergroundThermometerSettingsMenu.display();
}
void xEditWundergroundThermometerDisplayMenu::saveDisplay(){
}

class xEditWundergroundThermometerActiveDisplay:public xDisplay{
  YNField _WundergroundThermometerActive;
    public:
    void init();
    void update();
    void saveDisplay();
};
void xEditWundergroundThermometerActiveDisplay::init(){
  TheYNField=&_WundergroundThermometerActive;
  _WundergroundThermometerActive.YN=QueueThermometerForInterfaces;
  up.button_press_handler=xUpPressYN;
  down.button_press_handler=xDownPressYN;
  enter.button_press_handler=xEnterPressYN;
}
void xEditWundergroundThermometerActiveDisplay::update(){
  oled.clearDisplay();
  oled.setCursor(0, 0);
   _WundergroundThermometerActive.display();
  oled.display();
}
void xEditWundergroundThermometerActiveDisplay::saveDisplay(){
  QueueThermometerForInterfaces=_WundergroundThermometerActive.YN;
  saveThermometerToDisk();
  SetDefaultDisplay();
}

/*
  @class 
  @brief
*/

class xEditWundergroundWindDirectionDisplay:public xDisplay{
    public:
    void init();
    void update();
    void saveDisplay();
};

void xEditWundergroundWindDirectionDisplay::init(){}
void xEditWundergroundWindDirectionDisplay::update(){

}
void xEditWundergroundWindDirectionDisplay::saveDisplay(){
  SetDefaultDisplay();
}

/*
  @brief xDisplay definitions
*/
xDisplay *TheDisplay; // the Default display
xDisplayEvent DisplayEvent; 
xDefaultDisplay _DefaultDisplay;
xMainMenuDisplay MainMenuDisplay; // Main Menu
xNetStatusDisplay NetStatusDisplay;
xSettingsDisplay SettingsDisplay; // Settings Menu
xInterfacesDisplay InterfaceSettingsDisplay; // Edit Interfaces Menu
xWXStationMenu _xWXStationMenu;
xEditDavisStationDisplay _xEditDavisStationDisplay;
xEditDavisStationActiveDisplay _xEditDavisStationActiveDisplay;
xEditStationSensorsMenu _xEditStationSensorsMenu;

/* Wunderground Interface Display definitions*/
xWundergroundSettingsDisplay WundergroundSettingsDisplay; 
xEditWundergroundSensorsMenu _xEditWundergroundSensorsMenu;
xWundergroundEditStationNameDisplay _xWundergroundEditStationNameDisplay;
xWundergroundEditStationPasswordDisplay _xWundergroundEditStationPasswordDisplay;
xWundergroundEditStationActiveDisplay _xWundergroundEditStationActiveDisplay;


xEditWundergroundHumidityDisplay _xEditWundergroundHumidityDisplay; // Humidity Sensor Settings Menu for Wunderground Interface
xEditWundergroundHumidityActiveDisplay _xEditWundergroundHumidityActiveDisplay; // Humidity Sensor Active Field for Wunderground Interface

xEditWundergroundThermometerDisplayMenu _xEditWundergroundThermometerDisplayMenu;
xEditWundergroundThermometerActiveDisplay _xEditWundergroundThermometerActiveDisplay;

xEditWundergroundAnemometerDisplay _xEditWundergroundAnemometerDisplay;
//xEditWundergroundThermometerDisplay _xEditWundergroundThermometerDisplay;
xEditWundergroundWindDirectionDisplay _xEditWundergroundWindDirectionDisplay;

static void xDisplayTask(void* pvParameters) {
  DisplayQueue = xQueueCreate(2,sizeof(xDisplayEvent));
  TheDisplay = &_DefaultDisplay;
  DisplayEvent.DisplayAction=DISPLAY_UPDATE;
  xQueueSend(DisplayQueue,&DisplayEvent, 1000);
  while (true) {
  //try to get time from RTC
  if(xQueuePeek(DisplayQueue, &DisplayEvent, 10000)==pdTRUE){
    if (xSemaphoreTake(I2CBusSemaphore, 5)) {
      now = rtc.now();
      if(xQueueReceive(DisplayQueue, &DisplayEvent, 1)==pdTRUE){
        Serial.println("display event");
        if(DisplayEvent.DisplayAction==DISPLAY_SET){
          TheDisplay=DisplayEvent.Display;
          TheDisplay->init();
          TheDisplay->update();
          TheDisplay->LastUpdate = xTaskGetTickCount();
        }
        else if(DisplayEvent.DisplayAction==DISPLAY_UPDATE) {
          TheDisplay->update();
          TheDisplay->LastUpdate = xTaskGetTickCount();
        }
        else if(DisplayEvent.DisplayAction==DISPLAY_SAVE){
                TheDisplay->LastUpdate = xTaskGetTickCount();
                TheDisplay->saveDisplay();
        }
        else {}
        }
      xSemaphoreGive(I2CBusSemaphore);
    }
  } else { // refresh the display even when there are no updates queued
        if (xSemaphoreTake(I2CBusSemaphore, 5)) {
          if(xTaskGetTickCount()>(TheDisplay->LastUpdate+TheDisplay->RefreshTimer)) {
            TheDisplay->update();
            TheDisplay->LastUpdate = xTaskGetTickCount();
          }
          xSemaphoreGive(I2CBusSemaphore);
        }
  }
  taskYIELD();
  }
}

xDisplayEvent xMenuEvent;

/*
  @brief Default Button Handlers
*/
void xUpPress(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&MainMenuDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
  Serial.println("Up Pressed");
}

void xUpRelease(){
  Serial.println("Up Released");
}

void xDownPress(){
  Serial.println("Down Pressed");
}

void xDownRelease(){
  Serial.println("Down Released");
}

void xEnterPress(){
  Serial.println("Enter Pressed");
}

void xEnterRelease(){
  Serial.println("Enter");
}
/* End default button handlers */

/*
  @brief Button handlers for the MENU type
*/
void xUpMenuPress(){
  xMenuEvent.DisplayAction=DISPLAY_UPDATE;
  TheDisplay->TheMenu->xMenuUp();
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void xDownMenuPress(){
  TheDisplay->TheMenu->xMenuDown();
  xMenuEvent.DisplayAction=DISPLAY_UPDATE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);}

void xEnterMenuPress(){
  TheDisplay->TheMenu->xMenuEnter();}
/* End MENU Button Handlers*/

/*
  @brief Button handlers for the TEXT field 
*/
void xUpTextfieldPress(){
  TheDisplay->TheTextField->previousChar();
  xMenuEvent.DisplayAction=DISPLAY_UPDATE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);}

void xDownTextfieldPress(){
  TheDisplay->TheTextField->nextChar();
  xMenuEvent.DisplayAction=DISPLAY_UPDATE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);}

void xEnterTextfieldPress(){
  TheDisplay->TheTextField->enterChar();}

void SaveTextField(){
  xMenuEvent.DisplayAction=DISPLAY_SAVE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/* End TEXT field Button Handlers */

void SetDefaultDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_DefaultDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);}

void SetNetworkStatusDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&NetStatusDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);}

void SetSettingsDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&SettingsDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);}

void SetInterfacesDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&InterfaceSettingsDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);}

void SetWundergroundSettingsDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&WundergroundSettingsDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void SetWundergroundEditNameDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xWundergroundEditStationNameDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void SetWundergroundEditPasswordDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xWundergroundEditStationPasswordDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void SetWundergroundEditActiveDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xWundergroundEditStationActiveDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

/*
  @brief Button handlers for the YN field
*/

void xUpPressYN(){
  TheDisplay->TheYNField->next();
  xMenuEvent.DisplayAction=DISPLAY_UPDATE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void xDownPressYN(){
  TheDisplay->TheYNField->next();
  xMenuEvent.DisplayAction=DISPLAY_UPDATE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void xEnterPressYN(){
  xMenuEvent.DisplayAction=DISPLAY_SAVE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void SaveYNField(){

}
/* End YN Field Button Handlers */

void xWundergroundEditStationActiveDisplay::init(){
  TheDisplay->TheYNField=&_WundergroundStationActive;
  _WundergroundStationActive.setField(WundergroundInfceEnable);
  up.button_press_handler=xUpPressYN;
  down.button_press_handler=xDownPressYN;
  enter.button_press_handler=xEnterPressYN;
}

void xWundergroundEditStationActiveDisplay::update(){
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print("Active: ");
  _WundergroundStationActive.display();
  oled.display();
}

void xWundergroundEditStationActiveDisplay::saveDisplay(){
  WundergroundInfceEnable=TheDisplay->TheYNField->YN;
  SaveWundergroundCredentials();
  SetDefaultDisplay();
}

/* 
  @brief Button Handlers for CHOICE field
*/
void xUpPressChoice(){
  TheDisplay->TheChoice->prevChoice();
  xMenuEvent.DisplayAction=DISPLAY_UPDATE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void xDownPressChoice(){
  TheDisplay->TheChoice->nextChoice();
  xMenuEvent.DisplayAction=DISPLAY_UPDATE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void xEnterPressChoice(){
  TheDisplay->TheChoice->enterChoice();
  xMenuEvent.DisplayAction=DISPLAY_SAVE;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/* End CHOICE */

void OpenEditStationMenu(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xWXStationMenu;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void OpenEditStationDisplay(){
  stationSel=_xWXStationMenu.WXStationList.getSel(); 
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditDavisStationDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/*
  @brief Opens the Active: YN display for the Davis ISS station in stationSel
*/
void OpenEditDavisStationActiveDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditDavisStationActiveDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/*
  @brief Opens the Sensors menu display for the Davis ISS station in stationSel
*/
void OpenEditDavisStationSensorsDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditDavisStationDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/*
  Code to open Edit Wunderground Interface displays
*/
void OpenEditWundergroundSensorsMenu(){
      xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditWundergroundSensorsMenu;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/*
  @brief Opens the menu for the Wunderground Interface Anemometer sensor
*/
void OpenEditWundergroundAnemometerDisplay(){
    xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditWundergroundAnemometerDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/*
  @brief Opens the menu for the Wunderground Interface Wind Direction sensor
*/
void OpenEditWundergroundWindDirectionSensorDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditWundergroundWindDirectionDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/*
  @brief Opens the menu for the Wunderground Interface Temperature sensor  
*/
void OpenEditWundergroundEditTemperatureSensorDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditWundergroundThermometerDisplayMenu;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/*
  @brief Opens the menu for the Wunderground Interface Humidity sensor  
*/
void OpenEditWundergroundEditHumiditySensorDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditWundergroundHumidityDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

/*
  @brief Opens the menu for the Wunderground Interface Pressure sensor  
*/
void OpenEditWundergroundEditPressureSensorDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditDavisStationDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void OpenEditWundergroundEditHumidityActiveDisplay(){
    xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditWundergroundHumidityActiveDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
void OpenEditWundergroundEditHumidityStationChoiceDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditDavisStationDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

void OpenEditWundergroundEditHumidityStationSensorChoiceDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditDavisStationDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
/*
  @brief Wunderground Interface settings display - thermometer active. 
*/
void OpenEditWundergroundThermometerActiveDisplay(){
    xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditWundergroundThermometerActiveDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
void OpenEditWundergroundThermometerStationDisplay(){
  xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditWundergroundHumidityActiveDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}
void OpenEditWundergroundThermometerSensorDisplay(){
    xMenuEvent.DisplayAction=DISPLAY_SET;
  xMenuEvent.Display=&_xEditWundergroundHumidityActiveDisplay;
  xQueueSend(DisplayQueue,&xMenuEvent, 1000);
}

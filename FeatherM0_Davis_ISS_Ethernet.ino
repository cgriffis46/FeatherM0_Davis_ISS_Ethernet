#include <SD.h>

#include <Adafruit_SleepyDog.h>

#include <postParser.h>

/*
  (c) 2023.10.26 by Cory Griffis

  it is released under the same Creative Commons Attrib Share-Alike License
  You are free to use/extend this library but please abide with the CC-BY-SA license:
  http://creativecommons.org/licenses/by-sa/3.0/



*/


#define _USE_RTC_PCF8523 true 
#define USE_WUNDERGROUND_INFCE
#define _USE_TH_SENSOR
//#define DAVISRFM69_DEBUG
#define _INFCE_SEND_TEMP_HUMIDITY
#ifdef _USE_TH_SENSOR

//static void readTempHumiditySensor();
//  Ticker readTHSensorTicker(readTempHumiditySensor,2,0);
  float temperature = NAN, humidity = NAN;
  float tempf=NAN, tempc=NAN;
  bool QueueThermometerForInterfaces = true;
  bool QueueHumidityForInterfaces = true;
  bool UseCelcius = false;
#endif

#include <Ethernet.h>
#include <Arduino.h>
#include <EthernetUdp.h>
#include <DNS.h>
#include <SPI.h>
//#include <EEPROM.h>
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

typedef struct DataString{
  char TheString[DATASTRINGLENGTH];
};


typedef struct DatalogArray
{
  DataString DataStringArray[2];
} ; 

DatalogArray DataStrings;
String DatalogString;

// Wunderground interface 
//#define USE_WUNDERGROUND_INFCE

#ifdef USE_WUNDERGROUND_INFCE
  #include "WundergroundInfce.h"

  IPAddress wundergroundIP(0,0,0,0);
  //HTTPClient httpWunderground;
  bool WundergroundInfceEnable = true;
  EthernetClient WundergroundEthernetCclient;

  #define WundergroundStationIDLength 64
#define WundergroundStationIDPassword 64
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
  String WU_station_id = "KMAATTLE33"; //Wunderground station ID
  String WU_station_pwd = "JWM4JYWE"; //# Wunderground station password
  String WUcreds;  // = "ID=" + WU_station_id + "&PASSWORD="+ WU_station_pwd;
  const char *url = "weatherstation.wunderground.com";
  String action_str = "&action=updateraw";
  String W_Software_Type = "&softwaretype=rp2040wx%20version"+wx_version;
//  bool shouldUpdateWundergroundInfce = false;
 // bool WundergroundInfceEnable = false;
  char WundergroundStationID[WundergroundStationIDLength] = "KMAATTLE33";
  char WundergroundStationPassword[WundergroundStationIDPassword] = "JWM4JYWE";
  //struct repeating_timer WU_Update_timer;
  uint8_t thermometer1Type = WU_S_TEMPF_T;
  uint8_t humidity1_sensor_type = WU_S_HUMIDITY_T;
  uint8_t WundergroundTimeSource = 1;


static void xUpdateWundergroundInfce(void *pvParameters); // Must be called in main loop 

#endif
/*
  Definitions for RFM69 Radio
*/
#define LED 13
#define SERIAL_BAUD 19200

// If using Feather M0 with RF69 module 
  #define RFM69_CS    8
  #define RFM69_INT   3
  #define RFM69_RST   4
  #define LED        13

DavisRFM69 radio(RFM69_CS, 3, true, 3);

// id, type, active
Station stations[1] = {
  { .id = 0,
    .type =  STYPE_VUE,
    .active = true
  }
};


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

static void xHTTPUpdateTask(void *pvParameters); // Webserver Task 
EthernetClient HTTPClient; // The Web Client
TaskHandle_t xHTTPClientTaskHandle; // Task Handle for Webserver Task
// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);
DNSClient dnsclient;

// NTP Client definitions
#include <NTPClient.h> // NTP Client Library 
//extern EthernetClass Ethernet;
DateTime now; // Current DateTime
DateTime ntptime; // NTP Time
//WiFiUDP wifiUdp;
IPAddress ntpip(192, 168, 1, 20);
EthernetUDP eth0udp; // UDP Socket for NTP Client
NTPClient timeClient(eth0udp,ntpip); // NTP Client 
static void xNTPClientTask(void *pvParameters); // NTP Client Task
TaskHandle_t xNTPClientTaskHandle;  // NTP Client Task Handle

// Semaphore Definitions 
SemaphoreHandle_t I2CBusSemaphore; // Arbitrate I2C bus access
SemaphoreHandle_t Eth0Semaphore; // Arbitrate Eth0 bus access
StaticSemaphore_t xSPIBusSemaphore; // Arbitrate the SPI bus 
SemaphoreHandle_t SPIBusSemaphore;
static void xinterruptHandlertask(void *pvParameters);
static void xReadRadioTask(void *pvParameters);
TaskHandle_t xReadRadioTaskHandle;
TaskHandle_t xinterrupttaskhandle;
TaskHandle_t xUpdateWundergroundInfcetaskhandle;
TaskHandle_t xWatchdogTaskHandle;
TaskHandle_t xDataSamplerTaskHandle;
QueueHandle_t DataStringQueueHandle;
TaskHandle_t xDataloggerTaskHandle;
StaticQueue_t DataStringQueue;
//uint8_t DataStringQueueArray[MAXDATASTRINGS];

void setup() {
  Serial.begin(115200);
  delay(1000);
  //while(!Serial);
 // SPI.begin(); // initialize SPI
  pinMode(LED, OUTPUT);
  pinMode(9,INPUT_PULLUP);
  digitalWrite(LED, LOW);
  
  I2CBusSemaphore = xSemaphoreCreateMutex(); // Create I2C Semaphore
  //Eth0Semaphore = xSemaphoreCreateMutex(); // Create ETH0 Semaphore
  SPIBusSemaphore = xSemaphoreCreateMutex();
//  SPIBusSemaphore = xSemaphoreCreateBinary(); // Create SPI Semaphore 

  pinMode(8, INPUT_PULLUP); // RF69 Enable pin
 // pinMode(10, INPUT_PULLUP); // Ethernet Feather CS pin
#ifdef _USE_RTC
// Initialize the RTC 
    Serial.println("Initialize RTC");
    if(rtc.begin(&Wire)){      
//      rtc.start();
      now = rtc.now();}
  else {
    Serial.println("Could not initialize RTC!");}
#endif // USE_RTC


  // Initialize Ethernet 
  Ethernet.init(10);
  // start the Ethernet connection and the server:
  Ethernet.begin(mac);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");  }
  dnsclient.begin(IPAddress(1,1,1,1));

  timeClient.begin(); // Start NTP Client 

    server.begin(); // start webserver
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  HTTPClient.setConnectionTimeout(2000);
  xTaskCreate(xHTTPUpdateTask,     "HTTP Update Task",       256, NULL, tskIDLE_PRIORITY + 1, &xHTTPClientTaskHandle); // Start web server task

 // pinMode(10, INPUT_PULLUP); // Ethernet Feather CS pin

  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);

  radio.setStations(stations, 1);
  radio.initialize(FREQ_BAND_US);
  radio.setBandwidth(RF69_DAVIS_BW_WIDE);


  //xTaskCreate(xWatchdogTask,"Watchdog Task",256, NULL,tskIDLE_PRIORITY + 1,&xWatchdogTaskHandle);

  xTaskCreate(xReadRadioTask,"Radio Task",256, NULL,tskIDLE_PRIORITY + 2,&xReadRadioTaskHandle);
  radio.xReadRadioTaskHandle=xReadRadioTaskHandle;
  xTaskCreate(xNTPClientTask,     "NTP Task",       512, NULL, tskIDLE_PRIORITY + 1, &xNTPClientTaskHandle); // Start NTP Update task

  DataStringQueueHandle = xQueueCreate(MAXDATASTRINGS,DATASTRINGLENGTH);
  xTaskCreate(xDataSamplerTask,"Datasampler",256, NULL,tskIDLE_PRIORITY + 1,&xDataSamplerTaskHandle);
  xTaskCreate(xDataloggerTask,"Datalogger",256, NULL,tskIDLE_PRIORITY + 1,&xDataloggerTaskHandle);
  xTaskCreate(xUpdateWundergroundInfce,"Wunderground Interface Task",1024, NULL,tskIDLE_PRIORITY + 1,&xUpdateWundergroundInfcetaskhandle);
  Serial.println("Boot complete!");

  vTaskStartScheduler(); // Start task scheduler

}

void loop() {
/*
    if (radio.fifo.hasElements()) {
    decode_packet(radio.fifo.dequeue());
  }
*/
/* 
  if (radio.mode == SM_RECEIVING) {
    digitalWrite(LED, HIGH);
  } else if (radio.mode == SM_SEARCHING){
    Blink(LED, 15);
    Blink(LED, 15);
    Blink(LED, 15);
    delay(100);
  }else{
    digitalWrite(LED, LOW);
  }
*/

 // radio.loop();
/*
  pinMode(RFM69_CS, INPUT_PULLUP);
       // timeClient must be called every loop to update NTP time 
      if(timeClient.update()) {// NTP client will update about once a minute. 
          if(timeClient.isTimeSet()){ // sanity check 
       //     shouldUpdateRTC = true;// Update the RTC only when necessary
            //debug strings
            //Serial.println("Updated RTC time!");
            //String timedate = String(now.year())+String("-")+String(now.month())+String("-")+String(now.day())+String(" ")+String(now.hour())+String(":")+String(now.minute())+String(":")+String(now.second());
            //Serial.println(now.timestamp());
            }
      }
*/


}

void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  vTaskDelay( DELAY_MS/portTICK_PERIOD_MS );
  //delay(DELAY_MS);
//    delay(100);
//  vTaskDelay( DELAY_MS / portTICK_PERIOD_MS ); // Sleep for 100ms
  digitalWrite(PIN,LOW);
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
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
}

void print_value(char* vname, int value, const __FlashStringHelper* sep) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
}

void print_value(char* vname, float value, const __FlashStringHelper* sep) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value, 1); Serial.print(sep);
}

void print_value(char* vname, long value, const __FlashStringHelper* sep) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
}

void print_value(char* vname, uint32_t value, const __FlashStringHelper* sep) {
  Serial.print(vname); Serial.print(F(":")); Serial.print(value); Serial.print(sep);
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
        if ((packet[4] & 0x40) == 0) val >>= 4; // packet[4] bit 6: strong == 0, light == 1
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
        val = ((int16_t)((packet[3]<<8) | packet[4])) / 16;
        print_value("temp", (float)(val / 10.0), F(", "));
        #ifdef _USE_TH_SENSOR
          temperature = (float)(val / 10.0);
          tempf = temperature;
          tempc = (temperature -32)*5/9;
        #endif
      }
      break;

    case VP2P_HUMIDITY:
      val = ((packet[4] >> 4) << 8 | packet[3]) / 10; // 0 -> no sensor
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

static void xReadRadioTask(void *pvParameters){
  long startTime,endTime;
  uint32_t ulInterruptStatus;
  bool PayloadReady = false;
  bool Timeout = false;
//vTaskSuspend(NULL);
//radio.xWatchdogTaskHandle= xWatchdogTaskHandle;
pinMode(5,INPUT_PULLUP);
pinMode(12,INPUT_PULLUP);
pinMode(11,INPUT_PULLUP);

attachInterrupt(12,timeoutISR,RISING), // DIO1 RFM69 Timeout 
attachInterrupt(5,SyncAddressISR,RISING); // DIO 3 Sync Address
attachInterrupt(6,ModeReadyInterrupt,CHANGE); // DIO 5 ModeReady 
attachInterrupt(11,FifoNotEmptyISR,RISING); // DIO 2 FifoNotEmpty 
attachInterrupt(RFM69_INT, interruptHandler, CHANGE); // DIO0 PayloadReady

while(true){

if(xSemaphoreTake(SPIBusSemaphore,1)){// we need eth0 semaphore to update time over NTP
//Serial.println("xRadio Task has Mutex");
if((ulInterruptStatus&0x01)!=0x00){PayloadReady = true;} else {PayloadReady = false;}
if((ulInterruptStatus&0x02)!=0x00){Timeout = true;} else {Timeout = false;}
 xTaskNotifyStateClear(NULL);
// Check PayloadReady interrupt and clear the FIFO if needed 
  if(PayloadReady){ // PayloadReady task notification from ISR
    if (radio._mode == RF69_MODE_RX && (radio.readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
      radio.FEI = word(radio.readReg(REG_FEIMSB), radio.readReg(REG_FEILSB));
      radio.setMode(RF69_MODE_STANDBY); 
      radio.select();  // Select RFM69 module, disabling interrupts
      SPI.transfer(REG_FIFO & 0x7f); // Initiates FIFO transfer

      for (int i = 0; i < DAVIS_PACKET_LEN; i++) radio.DATA[i] = radio.reverseBits(SPI.transfer(0)); // Reads FIFO

      radio._packetReceived = true;

      radio.handleRadioInt();

      radio.unselect();  // Unselect RFM69 module, enabling interrupts
      //ulTaskNotifyValueClearIndexed(NULL,0,0x01); // Clear notification bit for PayloadReady interrupt
      //xTaskNotify(xWatchdogTaskHandle,0x01,eSetBits);
      PayloadReady = false;
    }}
    // blink the LED if necessary
    if (radio.mode == SM_RECEIVING) {
    digitalWrite(LED, HIGH);
  } else if (radio.mode == SM_SEARCHING){
//    Blink(LED, 15);
//    Blink(LED, 15);
//    Blink(LED, 15);
    //delay(100);
//    vTaskDelay( 100/portTICK_PERIOD_MS );
  }else{
    digitalWrite(LED, LOW);
  }
    radio.loop();

    if (radio.fifo.hasElements()) {
      decode_packet(radio.fifo.dequeue());
    } else { }

//  Serial.println("Radio Task gives up mutex");
  xSemaphoreGive( SPIBusSemaphore );
}

   // need to share the mcu but wake up quickly if a packet arrives or timeout occurs
   // In theory we could sleep indefinitely and switch channels on timeout;
   // the original loop() tries to set the channel about 15ms before the tx is expected. 
   // slow loop times cause high packet loss. Yield() would give better packet loss results
   // but the high priority task would rule the mcu. 
  xTaskNotifyWait(0,0,&ulInterruptStatus,5);
}}

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
static void xNTPClientTask(void *pvParameters){
long taskStart, taskEnd;
//timeClient.setUpdateInterval(60000);
while(true){
taskStart = millis();
bool shouldUpdateRTC = false; // Only update RTC if NTP update is successfull 
if(xSemaphoreTake(SPIBusSemaphore,1)){// we need SPI bus semaphore to update time over NTP
//Serial.println("NTP Task has mutex");
       // timeClient can block for up to 6s for a timeout, or 10ms for a local time server. 
       // may need figure out how to change the timeout to 100ms or better. 
      if(timeClient.forceUpdate()) {// since we need to hold the SPI bus, it makes more sense to force an update rather than poll the function using update(). Just need to force update less than once per minute. 
          if(timeClient.isTimeSet()){ // sanity check 
            shouldUpdateRTC = true;// Update the RTC only when necessary
            //debug strings
            //Serial.println("Updated RTC time!");
            now = DateTime(timeClient.getEpochTime());
            //String timedate = String(now.year())+String("-")+String(now.month())+String("-")+String(now.day())+String(" ")+String(now.hour())+String(":")+String(now.minute())+String(":")+String(now.second());
            }
      }
      else {
      //          Serial.println("Could not update NTP time!");
      }
    xSemaphoreGive( SPIBusSemaphore );
//    Serial.println("NTP task gives up mutex");
  } else{
    Serial.println("NTP task could not obtain mutex");
   }
//taskYIELD();
// NTP may only return true once per minute. Update the RTC only if necessary. 
#ifdef _USE_RTC
if(shouldUpdateRTC){ 
  if(xSemaphoreTake(I2CBusSemaphore,5)){
  rtc.adjust(DateTime(timeClient.getEpochTime()));
  now=rtc.now();
  xSemaphoreGive( I2CBusSemaphore );}
} else {}
#endif
Serial.println(now.timestamp());
Serial.flush();
taskEnd = millis()-taskStart;
//Serial.print("NTP task: "); Serial.println(taskEnd);
// Sleep
if(shouldUpdateRTC){ // If the time was updated, sleep for 10 minutes 
    vTaskDelay( 600000/portTICK_PERIOD_MS );
} else { // If the time could not be updated, sleep for just 2 minutes.
  vTaskDelay( 120000/portTICK_PERIOD_MS );
}

}}// end of thread

// DIO 3 Sync Address Found
// The sync word is the start of the packet. 
// we can use the time the packet arrived to sync
// with the transmitter. 
void SyncAddressISR(){
  radio.SyncAddressSeen = micros();
  //Serial.print(radio.SyncAddressSeen);
  //Serial.println(": Sync Word ");
}

// DIO 2 Fifo Not Empty
// Copy the timestamp the Sync Address was seen. 
// This is to grab the timestamp in case another 
// sync address is seen before PayloadReady
void FifoNotEmptyISR(){
  radio.Fifo_Not_Empty = micros();
}

// DIO 1 Timeout ISR
// Radio will give a timeout interrupt if RSSI threshold is not reached. 
// used to wake up the main task to change channels if necessary.
void timeoutISR(){
  BaseType_t xHigherPriorityTaskWoken;
  //uint32_t ulStatusRegister;
  xTaskNotifyFromISR(xReadRadioTaskHandle,0,eNoAction,&xHigherPriorityTaskWoken);
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
   if(radio.PayloadReady){ 
      radio.PayloadReadyTicks = xTaskGetTickCount();
      xTaskNotifyFromISR(xReadRadioTaskHandle,0x01,eSetBits,&pxHigherPriorityTaskWoken);}
}

// the current mode (rx/tx/standby/sleep) is "ready"
void ModeReadyInterrupt(){
  radio.ModeReady = digitalRead(6);

}

/*

static void xinterruptHandlertask(void *pvParameters){
uint32_t ulNotifiedValue;
byte i = 0;
attachInterrupt(RFM69_INT, interruptHandler, RISING);
vTaskSuspend(NULL);
while(true){
//if(radio.ModeReady){
  if (xSemaphoreTake(SPIBusSemaphore,100)) {
//      Serial.println("xInterruptTask took mutex");
//    radio.RSSI = radio.readRSSI();  // Read up front when it is most likely the carrier is still up
    if (radio._mode == RF69_MODE_RX && (radio.readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
      radio.FEI = word(radio.readReg(REG_FEIMSB), radio.readReg(REG_FEILSB));
      radio.setMode(RF69_MODE_STANDBY);
      radio.select();  // Select RFM69 module, disabling interrupts
      SPI.transfer(REG_FIFO & 0x7f);

      for (i = 0; i < DAVIS_PACKET_LEN; i++) radio.DATA[i] = radio.reverseBits(SPI.transfer(0));

      radio._packetReceived = true;

      radio.handleRadioInt();

      radio.unselect();  // Unselect RFM69 module, enabling interrupts
    }
    
    xSemaphoreGive(SPIBusSemaphore);
//    Serial.println("Interrupt task gives up mutex");
//    xTaskResumeAll();
//    taskEXIT_CRITICAL( );
 //   Serial.println("Radio Interrupt Task Completed");
    xTaskNotifyGive(xReadRadioTaskHandle);
    vTaskSuspend(NULL);
    //xTaskResumeFromISR(xReadRadioTaskHandle);
  //}
  } else {
      taskYIELD();
  }}
}
*/

// Webserver task polls for web clients and serves webpages 
// to be inplemented
static void xHTTPUpdateTask(void *pvParameters){
char PostActionBuf[64];
while(true){
  if(xSemaphoreTake(SPIBusSemaphore,5)){
    HTTPClient = server.available();
    if (HTTPClient) {
      Serial.println("new client");
      // an HTTP request ends with a blank line
      bool currentLineIsBlank = true;
    if (HTTPClient.connected()) {
      PostParser postParser = PostParser(HTTPClient); // create our parser
      while (HTTPClient.available()) {
        char c = HTTPClient.read();
        postParser.addHeaderCharacter(c); // compose the header
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {

        postParser.grabPayload();
        postParser.getField("i").toCharArray(PostActionBuf, sizeof(PostActionBuf));
        Serial.println(PostActionBuf);
        postParser.getField("p").toCharArray(PostActionBuf, sizeof(PostActionBuf));
        Serial.println(PostActionBuf);

            Serial.println(postParser.getHeader()); // print the header for debugging
          delay(10); //used to make sure the 2 serial prints don't overlap each other
          Serial.println(postParser.getPayload()); // print the payload for debugging


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
          HTTPClient.println( "<ul>");
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
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
      // give the web browser time to receive the data
      delay(1);
      // close the connection:
      HTTPClient.stop();


      //HTTPClient.
    }
    // Give up the semaphore and sleep 250ms
    xSemaphoreGive( SPIBusSemaphore );
  }
  vTaskDelay( 2000/portTICK_PERIOD_MS );
  //taskYIELD();
}}



static void xDataSamplerTask(void *pvParameters){
int DataLogIndex = 0;
vTaskDelay( 60000/portTICK_PERIOD_MS );
while(true){
  if(xSemaphoreTake(I2CBusSemaphore,5)){
    now=rtc.now();
  xSemaphoreGive( I2CBusSemaphore );}
  DatalogString = "";
  DatalogString += "time:"+now.timestamp()+",";
  DatalogString += "temp:";
  DatalogString += String(temperature);
  DatalogString += "F,";
  DatalogString += "rh:";
  DatalogString += String(humidity);
//  DatalogString += 13;//cr
//  DatalogString += 10;//lf
  DatalogString.toCharArray(&DataStrings.DataStringArray[DataLogIndex].TheString[0],DATASTRINGLENGTH);
  xQueueSend(DataStringQueueHandle,&DataStrings.DataStringArray[DataLogIndex].TheString[0],1000);
  Serial.print(DataStrings.DataStringArray[DataLogIndex].TheString);
  DataLogIndex++;
  if(DataLogIndex>=MAXDATASTRINGS) {DataLogIndex=0;}
  vTaskDelay( 30000/portTICK_PERIOD_MS );
}}

#define DL_PGM_STATE_INIT 0
#define DL_PGM_STATE_READ_QUEUE 5
#define DL_PGM_STATE_OPEN_FILE 10
#define DL_PGM_STATE_OPEN_FOLDER 9
#define DL_PGM_STATE_WRITE_FILE 15
#define DL_PGM_STATE_IDLE 99
#define DL_PGM_STATE_NO_SD_CARD 999

char DataPayload[DATASTRINGLENGTH];
File dataFile;

static void xDataloggerTask(void *pvParameters){
byte DL_pgm_state = DL_PGM_STATE_INIT;
vTaskDelay( 60000/portTICK_PERIOD_MS );
while(true){
if(xSemaphoreTake(SPIBusSemaphore,5)){
  switch (DL_pgm_state){
    case DL_PGM_STATE_INIT: 
    {
      if(SD.begin(9)){
        Serial.println("SD card found!");
              DL_pgm_state = DL_PGM_STATE_READ_QUEUE;
      } else {
        Serial.println("No SD card found!");
        DL_pgm_state = DL_PGM_STATE_NO_SD_CARD;
      }

      break;
    }
    case DL_PGM_STATE_READ_QUEUE:{ 
      if(DataStringQueueHandle==NULL){ // No Queue 
        DL_pgm_state = DL_PGM_STATE_IDLE;
        break;
      } else if(xQueueReceive(DataStringQueueHandle,&DataPayload,1) == pdPASS){ // Item found in the queue
        Serial.println("Item found in Queue!");
        DL_pgm_state = DL_PGM_STATE_OPEN_FILE;
      } else {
        Serial.println("Item found in Queue!");
        DL_pgm_state = DL_PGM_STATE_IDLE; // nothing in the queue 
      }
      break;
    }
    case DL_PGM_STATE_OPEN_FOLDER:{
      break;
    }
    case DL_PGM_STATE_OPEN_FILE: { 
      dataFile = SD.open("datalog.txt", FILE_WRITE); // Try opening the file 
      if (dataFile){ // we opened the file
        dataFile.println(DataPayload);
        dataFile.close();
        
        if(xQueuePeek(DataStringQueueHandle,&DataPayload,1)==pdTRUE){
        DL_pgm_state = DL_PGM_STATE_READ_QUEUE;
      } else{
        DL_pgm_state = DL_PGM_STATE_IDLE;}
      } else{
        Serial.println("Could not open file");
        DL_pgm_state = DL_PGM_STATE_INIT;
      }
      break;
    }
    case DL_PGM_STATE_WRITE_FILE:{
    if (dataFile){
      dataFile.println(DataPayload);
      dataFile.close();
      if(xQueuePeek(DataStringQueueHandle,&DataPayload,1)==pdTRUE){
        DL_pgm_state = DL_PGM_STATE_READ_QUEUE;
      } else{
        DL_pgm_state = DL_PGM_STATE_IDLE;
      }
      
    } else{
      DL_pgm_state = DL_PGM_STATE_INIT;
    }
      break;
    }
    case DL_PGM_STATE_NO_SD_CARD:{
      Serial.println("No SD card! could not init datalogger. ");
      DL_pgm_state = DL_PGM_STATE_INIT;
      break;
    }
    case DL_PGM_STATE_IDLE:{
      if(xQueuePeek(DataStringQueueHandle,&DataPayload,1)==pdTRUE){
        DL_pgm_state = DL_PGM_STATE_READ_QUEUE;
      } else{
        DL_pgm_state = DL_PGM_STATE_IDLE;
      }
      break;
    }
    default:{
      DL_pgm_state = DL_PGM_STATE_INIT;
      break;
    }
  }
// open the file
//
    xSemaphoreGive( SPIBusSemaphore );
} //  end try to get semaphore
else { // could not obtain semaphore

}
  if(DL_PGM_STATE_IDLE){
  vTaskDelay( 10000/portTICK_PERIOD_MS );} else {
    vTaskDelay( 1000/portTICK_PERIOD_MS );
  }
}}

static void xDisplayTask(void *pvParameters){
while(true){

}}

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

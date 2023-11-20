/*
  (c) 2023.10.26 by Cory Griffis

  it is released under the same Creative Commons Attrib Share-Alike License
  You are free to use/extend this library but please abide with the CC-BY-SA license:
  http://creativecommons.org/licenses/by-sa/3.0/



*/
//#define _USE_RTC_PCF8523 true 
#define DAVISRFM69_DEBUG

#include <Ethernet.h>
#include <Arduino.h>
#include <EthernetUdp.h>
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
    .active = true}
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

  #include <RTClib.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0x98, 0x76, 0xB6, 0x12, 0x3c, 0xa6
};

// default IP address
//IPAddress ip(192, 168, 1, 177);

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

void setup() {
  Serial.begin(115200);
  delay(1000);
  //while(!Serial);
 // SPI.begin(); // initialize SPI
  pinMode(LED, OUTPUT); 
  digitalWrite(LED, LOW);
  
  I2CBusSemaphore = xSemaphoreCreateMutex(); // Create I2C Semaphore
  //Eth0Semaphore = xSemaphoreCreateMutex(); // Create ETH0 Semaphore
  SPIBusSemaphore = xSemaphoreCreateMutex();
//  SPIBusSemaphore = xSemaphoreCreateBinary(); // Create SPI Semaphore 

  pinMode(8, INPUT_PULLUP); // RF69 Enable pin
 // pinMode(10, INPUT_PULLUP); // Ethernet Feather CS pin
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

  timeClient.begin(); // Start NTP Client 

 // pinMode(10, INPUT_PULLUP); // Ethernet Feather CS pin


    radio.setStations(stations, 1);
   radio.initialize(FREQ_BAND_US);
   radio.setBandwidth(RF69_DAVIS_BW_WIDE);

  xTaskCreate(xReadRadioTask,"Radio Task",256, NULL,tskIDLE_PRIORITY + 1,&xReadRadioTaskHandle);
  radio.xReadRadioTaskHandle=xReadRadioTaskHandle;
  xTaskCreate(xNTPClientTask,     "NTP Task",       1024, NULL, tskIDLE_PRIORITY + 1, &xNTPClientTaskHandle); // Start NTP Update task
  xTaskCreate(xinterruptHandlertask,"Radio Task",256, NULL,tskIDLE_PRIORITY + 3,&xinterrupttaskhandle);
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
      }
      break;

    case VP2P_HUMIDITY:
      val = ((packet[4] >> 4) << 8 | packet[3]) / 10; // 0 -> no sensor
      print_value("rh", (float)val, F(", "));
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

void ModeReadyInterrupt(){
  radio.ModeReady = digitalRead(6);
  if(radio.ModeReady) {
    //Serial.println("modeready");
    }
  else {
    //Serial.println("modenotready");
  }
}

static void xReadRadioTask(void *pvParameters){
  long startTime,endTime;
//vTaskSuspend(NULL);
pinMode(5,INPUT_PULLUP);
attachInterrupt(5, RSSIThresholdInterrupt, RISING);
attachInterrupt(6,ModeReadyInterrupt,CHANGE);
//attachInterrupt(6,ModeReadyInterruptLow,FALLING);
//attachInterrupt(9,ModeReadyInterrupt,),

while(true){
//   vTaskSuspend(NULL);
if(xSemaphoreTake(SPIBusSemaphore,1)){// we need eth0 semaphore to update time over NTP
//Serial.println("xRadio Task has Mutex");
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
  taskYIELD();
}}

// NTP Task polls NTP and updates the RTC
static void xNTPClientTask(void *pvParameters){
long taskStart, taskEnd;
//timeClient.setUpdateInterval(60000);
while(true){
taskStart = millis();
bool shouldUpdateRTC = false; // Only update RTC if NTP update is successfull 
if(xSemaphoreTake(SPIBusSemaphore,1)){// we need eth0 semaphore to update time over NTP
//Serial.println("NTP Task has mutex");
       // timeClient must be called every loop to update NTP time 
      if(timeClient.forceUpdate()) {// NTP client will update about once a minute. 
          if(timeClient.isTimeSet()){ // sanity check 
            shouldUpdateRTC = true;// Update the RTC only when necessary
            //debug strings
            //Serial.println("Updated RTC time!");
            now = DateTime(timeClient.getEpochTime());
            //String timedate = String(now.year())+String("-")+String(now.month())+String("-")+String(now.day())+String(" ")+String(now.hour())+String(":")+String(now.minute())+String(":")+String(now.second());
            Serial.println(now.timestamp());
            Serial.flush();
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
  xSemaphoreGive( I2CBusSemaphore );}
} else {}
#endif
taskEnd = millis()-taskStart;
//Serial.print("NTP task: "); Serial.println(taskEnd);
if(shouldUpdateRTC){ // If the time was updated, sleep for 10 minutes
    vTaskDelay( 600000/portTICK_PERIOD_MS );
} else { // If the time could not be updated, sleep for just 2 minutes
  vTaskDelay( 120000/portTICK_PERIOD_MS );
}

}}// end of thread

void RSSIThresholdInterrupt(){
  radio.SyncAddressSeen = xTaskGetTickCount();
  //Serial.print(radio.SyncAddressSeen);
  //Serial.println(": Sync Word ");
}

void interruptHandler() {
  uint32_t ulStatusRegister;
  BaseType_t pxHigherPriorityTaskWoken;
//    Serial.println("Interrupt");
radio.PayloadReady = digitalRead(RFM69_INT);
   if(radio.PayloadReady){ 
      radio.PayloadReadyTicks = xTaskGetTickCount();
    xTaskResumeFromISR(xinterrupttaskhandle);}
 //   vTaskNotifyGiveFromISR(xinterrupttaskhandle,&pxHigherPriorityTaskWoken);
}

static void xinterruptHandlertask(void *pvParameters){
uint32_t ulNotifiedValue;
byte i = 0;
attachInterrupt(RFM69_INT, interruptHandler, RISING);
vTaskSuspend(NULL);
while(true){

  if (xSemaphoreTake(SPIBusSemaphore,100)) {
//      Serial.println("xInterruptTask took mutex");
//          vTaskSuspendAll();
//    taskENTER_CRITICAL( );
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
  } else {
      taskYIELD();
  }}
}

/*
  Written by : Cory S Griffis
  (C) 8/10/2010
  Provided under MIT license. 
  No warranty provided. 
*/


#ifdef USE_WUNDERGROUND_INFCE

#include "WundergroundInfce.h"
#include <Ethernet.h>
char buffer[256];
static byte PgmState = SM_Wunderground_Infce_Init;
TickType_t HTTP_Timeout; 

static void xUpdateWundergroundInfce(void *pvParameters){
vTaskDelay( 60000/portTICK_PERIOD_MS );
HTTP_Timeout = xTaskGetTickCount();
while (true){
if(xSemaphoreTake(SPIBusSemaphore,1)){// we need eth0 semaphore to update time over NTP
//Serial.print("wunderground task ");Serial.println(PgmState);
switch (PgmState) {
  case SM_Wunderground_Infce_Init:
  {
    if(WundergroundInfceEnable&& (strlen(WundergroundStationID)>0) && (strlen(WundergroundStationPassword)>0)) {
        PgmState = SM_Wunderground_Infce_Idle;
    }
    break;
  }
  case SM_Wunderground_Infce_Get_IP_By_DNS:
  {
    //Ethernet.hostByName(url,wundergroundIP);
    if(dnsclient.getHostByName(url,wundergroundIP,250)>0){
      PgmState = SM_Wunderground_Infce_Assemble_Payload;
    } else {
      wundergroundIP = IPAddress(0,0,0,0);
    }
    break;
  }
  case SM_Wunderground_Infce_Make_HTTP_Request:
  {
      
    if(WundergroundEthernetCclient.connect(wundergroundIP,80)>0){
      WundergroundHTTPString=WUget+WUcreds+date_str+Wundergroundpayload+W_Software_Type+action_str;
      #ifdef _DEBUG_WU_INFCE
        Serial.println(WundergroundHTTPString);
      #endif
      WundergroundHTTPString.toCharArray(buffer, WundergroundHTTPString.length());
      #ifdef _DEBUG_WU_INFCE
        Serial.print("connected to ");
        Serial.println(WundergroundEthernetCclient.remoteIP());
      #endif
      // Make a HTTP request:
      WundergroundEthernetCclient.print("GET ");
      WundergroundEthernetCclient.print(buffer);
      WundergroundEthernetCclient.println(" HTTP/1.1");
      WundergroundEthernetCclient.println("Host: weatherstation.wunderground.com");
      WundergroundEthernetCclient.println("Connection: close");
      WundergroundEthernetCclient.println();
    }
    HTTP_Timeout = xTaskGetTickCount();
    PgmState = SM_Wunderground_Infce_WaitForHTTP_Response;
    break;
  }
  case SM_Wunderground_Infce_WaitForHTTP_Response:
  {
    uint8_t buf[80];
    int len = WundergroundEthernetCclient.available();
    if (len > 0) {
    if (len > 80) len = 80;
        PgmState = SM_Wunderground_CloseConnections;
      WundergroundEthernetCclient.read(buf, len);
      #ifdef _DEBUG_WU_INFCE
        Serial.write(buf, len); // show in the serial monitor (slows some boards)
      #endif
    } else if (xTaskGetTickCount()-HTTP_Timeout>5000) { // Timeout after 5 seconds
      PgmState = SM_Wunderground_CloseConnections;
    }

    break;
  }
  case SM_Wunderground_CloseConnections:
  { 
    #ifdef _DEBUG_WU_INFCE
      Serial.println("disconnecting.");
    #endif
    WundergroundEthernetCclient.stop();

    PgmState = SM_Wunderground_Infce_Idle;
    break;
  }
  case SM_Wunderground_Infce_Assemble_Payload:
  {
          // Assemble date_str
      date_str = "";
      DateTime now;
      date_str = "&dateutc=now";
      Wundergroundpayload = "";
      // Send temperature and humidity if necessary
      #ifdef _INFCE_SEND_TEMP_HUMIDITY
      if(QueueThermometerForInterfaces){
      if(!(temperature==NAN)){
        switch (thermometer1Type) {
          case WU_S_TEMPC_T: 
          {
            Wundergroundpayload += F("&tempc=");Wundergroundpayload += String(tempc);
            break;
          }
          case WU_S_TEMPC2_T: 
          {
            Wundergroundpayload += F("&tempc2=");Wundergroundpayload += String(tempc);
            break;
          }
          case WU_S_TEMPC3_T: 
          {
            Wundergroundpayload += F("&tempc3=");Wundergroundpayload += String(tempc);
            break;
          }
          case WU_S_TEMPC4_T:
          {
            Wundergroundpayload += F("&tempc4=") ;Wundergroundpayload += String(tempc);
            break;
          }
          case WU_S_INDOORTEMPC_T:
          {
            Wundergroundpayload += F("&indoortempc=");Wundergroundpayload += String(tempc);
            break;
          }
          case WU_S_SOILTEMPC_T:
          {
            Wundergroundpayload += F("&soiltempc=");Wundergroundpayload += String(tempc);
            break;
          }
          case WU_S_SOILTEMPC2_T:
          {
            Wundergroundpayload += F("&soiltempc2=");Wundergroundpayload += String(tempc);
            break;
          }
          case WU_S_SOILTEMPC3_T:
          {
            Wundergroundpayload += F("&soiltempc3=");Wundergroundpayload += String(tempc);
            break;
          }
          case WU_S_TEMPF_T:
          {
            Wundergroundpayload += F("&tempf=");Wundergroundpayload += String(tempf);
            break;
          }
          case WU_S_TEMPF2_T:
          {
            Wundergroundpayload += F("&tempf2=");Wundergroundpayload += String(tempf);
            break;
          }
          case WU_S_TEMPF3_T:
          {
            Wundergroundpayload += F("&tempf3=");Wundergroundpayload += String(tempf);
            break;
          }
          case WU_S_TEMPF4_T:
          {
            Wundergroundpayload += F("&tempf4=");Wundergroundpayload += String(tempf);
            break;
          }
          case WU_S_INDOORTEMPF_T:
          {
            Wundergroundpayload += F("&indoortempf=");Wundergroundpayload += String(tempf);
            break;
          }
          case WU_S_SOILTEMPF_T:
          {
            Wundergroundpayload += F("&soiltempf=");Wundergroundpayload += String(tempf);
            break;
          }
          case WU_S_SOILTEMPF2_T:
          {
            Wundergroundpayload += F("&soiltempf2=");Wundergroundpayload += String(tempf);
            break;
          }
          case WU_S_SOILTEMPF3_T:
          {
            Wundergroundpayload += F("&soiltempf3=");Wundergroundpayload += String(tempf);
            break;
          }

        }}

        }
        // Add humidity to update string 
        if(QueueHumidityForInterfaces){
          if(!(humidity==NAN)){ // sanity check 
            switch (humidity1_sensor_type) { 
              case WU_S_HUMIDITY_T: {
                Wundergroundpayload += "&humidity="+String(humidity,2);
                break;
              }
              case WU_S_INDOORHUMIDITY_T: {
                Wundergroundpayload += "&indoorhumidity="+String(humidity,2);
                break;
              }
            }
          }
        }
      #endif
      // Send barometric pressure if necessary 
      #ifdef _INFCE_SEND_BAROMETRIC_PRESSURE
        if(QueueBarometerForInterfaces==true){
          Wundergroundpayload += String("&baromin=")+String(pressure);}
      #endif
      WUcreds = "ID=" + String(WundergroundStationID) + "&PASSWORD="+ String(WundergroundStationPassword);

    if(Wundergroundpayload.length()>0){ // only transmit if we have a payload
      PgmState = SM_Wunderground_Infce_Make_HTTP_Request;
    }
    break; // assemble payload
  }
  case SM_Wunderground_Infce_Idle:
  {
    PgmState = SM_Wunderground_Infce_Get_IP_By_DNS;
    break;
  }
    default:
    {
      break;
    }
} // end switch
  xSemaphoreGive( SPIBusSemaphore );
}
   if(PgmState == SM_Wunderground_Infce_Idle){
      vTaskDelay( 60000/portTICK_PERIOD_MS );}
  else{taskYIELD();}
}} // UpdateWundergroundInfce

/* 
void handleWundergroundConfigSave(){
  memset(WundergroundStationID,0,sizeof(WundergroundStationID));
  memset(WundergroundStationPassword,0,sizeof(WundergroundStationPassword));
  if(server.arg("e").equals("on")) {
    WundergroundInfceEnable = true;
  }
  else{
    WundergroundInfceEnable = false;
  }
  server.arg("i").toCharArray(WundergroundStationID, sizeof(WundergroundStationID) - 1);
  server.arg("p").toCharArray(WundergroundStationPassword, sizeof(WundergroundStationPassword) - 1);
  Serial.println(thermometer1Type = server.arg("t").toInt());
  Serial.println(humidity1_sensor_type = server.arg("h").toInt());
  Serial.println(WundergroundTimeSource = server.arg("z").toInt());
  server.sendHeader("Location", "config", true);
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  server.send(302, "text/plain", "");  // Empty content inhibits Content-length header so we have to close the socket ourselves.
  server.client().stop();              // Stop is needed because we sent no content length
  SaveWundergroundCredentials();
} // handleWundergroundConfigSave()
*/

/* 

void LoadWundergroundCredentials(){
  char ok[2 + 1];
  uint8_t enable = 0;
  uint32_t address;
  uint8_t buffer1[256];
  // wunderground interface enable
  #ifdef USE_SPI_FRAM
    enable = fram.read8(mem_WUNDERGROUND_ENABLE);
  #endif
  #ifdef USE_I2C_FRAM
    enable = fram.read(mem_WUNDERGROUND_ENABLE);
  #endif
  #ifdef USE_I2C_EEPROM
    enable = fram.read(mem_WUNDERGROUND_ENABLE);
  #endif
  if(enable>0){
    Serial.println("wunderground infce enabled");
    WundergroundInfceEnable = true;
  }
  else {
    Serial.println("wunderground infce disabled");
    WundergroundInfceEnable = false;
  }
  // Wunderground ID
  for(address = 0; address <sizeof(WundergroundStationID);address++){
    #ifdef USE_SPI_FRAM
      buffer1[address] = fram.read8(mem_WUNDERGROUNDID+address);
    #endif
    #ifdef USE_I2C_FRAM
      buffer1[address] = fram.read(mem_WUNDERGROUNDID+address);
    #endif
    #ifdef USE_I2C_EEPROM
      buffer1[address] = fram.read(mem_WUNDERGROUNDID+address);
    #endif
    WundergroundStationID[address] = char(buffer1[address]);
    }
  if(strlen(WundergroundStationID)>0){
    Serial.println(WundergroundStationID);
  }
  // Wunderground Password
  for(address = 0; address <sizeof(WundergroundStationPassword);address++){
    #ifdef USE_SPI_FRAM
      buffer1[address] = fram.read8(mem_WUNDERGROUNDPASSWORD+address);
    #endif
    #ifdef USE_I2C_FRAM
      buffer1[address] = fram.read(mem_WUNDERGROUNDPASSWORD+address);
    #endif
    #ifdef USE_I2C_EEPROM
      buffer1[address] = fram.read(mem_WUNDERGROUNDPASSWORD+address);
    #endif
    WundergroundStationPassword[address] = char(buffer1[address]);}
    // thermometer 1 type 
    #ifdef USE_SPI_FRAM
      thermometer1Type = ram.read8(mem_WUNDERGROUND_TEMP_ID);
    #endif
    #ifdef USE_I2C_FRAM
      thermometer1Type = fram.read(mem_WUNDERGROUND_TEMP_ID);
    #endif
    #ifdef USE_I2C_EEPROM
      thermometer1Type = fram.read(mem_WUNDERGROUND_TEMP_ID);
    #endif
    // humidity sensor 1 type 
    #ifdef USE_SPI_FRAM
      humidity1_sensor_type = ram.read8(mem_WUNDERGROUND_HUMIDITY_ID);
    #endif
    #ifdef USE_I2C_FRAM
      humidity1_sensor_type = fram.read(mem_WUNDERGROUND_HUMIDITY_ID);
    #endif
    #ifdef USE_I2C_EEPROM
      humidity1_sensor_type = fram.read(mem_WUNDERGROUND_HUMIDITY_ID);
    #endif
    // Time Source
    #ifdef USE_SPI_FRAM
      WundergroundTimeSource = ram.read8(mem_WUNDERGROUND_TIME_SOURCE);
    #endif
    #ifdef USE_I2C_FRAM
      WundergroundTimeSource = fram.read(mem_WUNDERGROUND_TIME_SOURCE);
    #endif
    #ifdef USE_I2C_EEPROM
      WundergroundTimeSource = fram.read(mem_WUNDERGROUND_TIME_SOURCE);
    #endif

} // LoadWundergroundCredentials()

*/

/*
void SaveWundergroundCredentials(){
  uint32_t address;
  char ok[2 + 1];
  uint8_t buffer1[512];
  // wunderground interface enable 
  Serial.print("Station ID: "); Serial.println(WundergroundStationID);
  Serial.print("Station Password: "); Serial.println(WundergroundStationPassword);
  if(WundergroundInfceEnable == true){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_WUNDERGROUND_ENABLE,(uint8_t)255);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_WUNDERGROUND_ENABLE,(uint8_t)255);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_WUNDERGROUND_ENABLE,(uint8_t)255);
    #endif
    }
  else {
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_WUNDERGROUND_ENABLE,0);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_WUNDERGROUND_ENABLE,0);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_WUNDERGROUND_ENABLE,0);
    #endif
    }
  
  // wunderground interface id
  for(address = 0; address <sizeof(WundergroundStationID);address++){
    buffer1[address] = (uint8_t)WundergroundStationID[address];
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_WUNDERGROUNDID+address,buffer1[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_WUNDERGROUNDID+address,buffer1[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_WUNDERGROUNDID+address,buffer1[address]);
    #endif
    }

  // wunderground interface password
  for(address = 0; address <sizeof(WundergroundStationPassword);address++){
    buffer1[address] = (uint8_t)WundergroundStationPassword[address];
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_WUNDERGROUNDPASSWORD+address,buffer1[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_WUNDERGROUNDPASSWORD+address,buffer1[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_WUNDERGROUNDPASSWORD+address,buffer1[address]);
    #endif
    }

    // wunderground thermometer sensor ID
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_WUNDERGROUND_TEMP_ID,thermometer1Type);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_WUNDERGROUND_TEMP_ID,thermometer1Type);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_WUNDERGROUND_TEMP_ID,thermometer1Type);
    #endif

        // wunderground humidity sensor ID
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_WUNDERGROUND_HUMIDITY_ID,humidity1_sensor_type);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_WUNDERGROUND_HUMIDITY_ID,humidity1_sensor_type);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_WUNDERGROUND_HUMIDITY_ID,humidity1_sensor_type);
    #endif

    // wunderground time source 
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_WUNDERGROUND_TIME_SOURCE,WundergroundTimeSource);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_WUNDERGROUND_TIME_SOURCE,WundergroundTimeSource);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_WUNDERGROUND_TIME_SOURCE,WundergroundTimeSource);
    #endif
} // SaveWundergroundCredentials()
*/

#endif

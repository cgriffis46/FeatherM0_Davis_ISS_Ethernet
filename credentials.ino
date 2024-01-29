/*
  Written by : Cory S Griffis
  (C) 8/10/2010
  Provided under MIT license. 
  No warranty provided. 
*/

union{
  float pressureF;
  uint8_t bytes[4];
} PressureConversion;


/** Load WLAN credentials from EEPROM */

/*

void loadCredentials() {
  char ok[2 + 1];
  uint32_t address;
  uint8_t buffer1[256];
  //uint8_t passwordbuffer[sizeof(password)];
  for(address = 0; address <sizeof(ssid);address++){
      #ifdef USE_SPI_FRAM
        buffer1[address] = fram.read8(address);
      #endif
      #ifdef USE_I2C_FRAM
        buffer1[address] = fram.read(address);
      #endif
      #ifdef USE_I2C_EEPROM
        buffer1[address] = fram.read(address);
      #endif
      ssid[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
  for(address = 0; address<sizeof(password);address++){
    #ifdef USE_SPI_FRAM
      buffer1[address] = fram.read8(mem_PASSWORD+address);
    #endif
    #ifdef USE_I2C_FRAM
      buffer1[address] = fram.read(mem_PASSWORD+address);
    #endif
    #ifdef USE_I2C_EEPROM
      buffer1[address] = fram.read(mem_PASSWORD+address);
    #endif
    password[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }
  Serial.println("Recovered credentials:");
  Serial.println(ssid);
  Serial.println(strlen(password) > 0 ? "********" : "<no password>");

  for(address = 0; address <sizeof(dev_user);address++){
      #ifdef USE_SPI_FRAM
        buffer1[address] = fram.read8(mem_DEV_USER+address);
      #endif
      #ifdef USE_I2C_FRAM
        buffer1[address] = fram.read(mem_DEV_USER+address);
      #endif
      #ifdef USE_I2C_EEPROM
        buffer1[address] = fram.read(mem_DEV_USER+address);
      #endif
      dev_user[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }

  //strlen();

  for(address = 0; address <sizeof(dev_password);address++){
      #ifdef USE_SPI_FRAM
        buffer1[address] = fram.read8(mem_DEV_PASSWORD+address);
      #endif
      #ifdef USE_I2C_FRAM
        buffer1[address] = fram.read(mem_DEV_PASSWORD+address);
      #endif
      #ifdef USE_I2C_EEPROM
        buffer1[address] = fram.read(mem_DEV_PASSWORD+address);
      #endif
      dev_password[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
  }

    Serial.println("Recovered credentials:");
  Serial.println(dev_user);
  Serial.println(strlen(dev_password) > 0 ? "********" : "<no password>");

}
*/

/** Store WLAN credentials to EEPROM */
/*

void saveCredentials() {
  uint32_t address;
  char ok[2 + 1];
  uint8_t buffer1[512];
  Serial.print("SSID: ");Serial.println(ssid);
  Serial.print("Password: ");Serial.println(password);
 // memcpy(buffer1, (void *)&ssid, sizeof(ssid));
  for(address = 0; address <sizeof(ssid);address++){
    buffer1[address] = (uint8_t)ssid[address];
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(address,buffer1[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(address,buffer1[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(address,buffer1[address]);
    #endif 
    }
//    if(!fram.write8(address,buffer1[address])) {Serial.println("Could not write to fram ");}}
 for(address = 0; address <sizeof(password);address++){
    buffer1[address] = (uint8_t)password[address];
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_PASSWORD+address,buffer1[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_PASSWORD+address,buffer1[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_PASSWORD+address,buffer1[address]);
    #endif
    }

    ConnectionAttempts = 0;

}

*/

void saveSensorsToDisk(){
  uint32_t address;
  char ok[2 + 1];
  uint8_t buffer1[512];
  uint8_t enable = 0;

  #ifdef USE_BAROMETRIC_PRESSURE_SENSOR
  PressureOffset = externalPressureOffset;
  PressureConversion.pressureF = externalPressureOffset;

  for(address = 0; address <sizeof(PressureConversion);address++){
    buffer1[address] = PressureConversion.bytes[address];
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_PRESSURE_OFFSET+address,buffer1[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_PRESSURE_OFFSET+address,buffer1[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_PRESSURE_OFFSET+address,buffer1[address]);
    #endif
    }

   Serial.print("Pressure Calibration Offset: ");Serial.println(PressureConversion.pressureF);

    if(QueueBarometerForInterfaces == true){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_BAR_ENABLE,(uint8_t)255);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_BAR_ENABLE,(uint8_t)255);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_BAR_ENABLE,(uint8_t)255);
    #endif
    }
    //if(!fram.write8(mem_WUNDERGROUND_ENABLE,255)) {Serial.println("Could not write wunderground enable flag to fram ");}}
  else {
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_BAR_ENABLE,0);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_BAR_ENABLE,0);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_BAR_ENABLE,0);
    #endif
    }
#endif


}

void LoadSensorsFromDisk(){
  char ok[2 + 1];
  uint8_t enable = 0;
  uint32_t address;
  uint8_t buffer1[256];

#ifdef USE_BAROMETRIC_PRESSURE_SENSOR
  for(address = 0; address <sizeof(PressureConversion);address++){
    #ifdef USE_SPI_FRAM
       PressureConversion.bytes[address] = fram.read8(mem_PRESSURE_OFFSET+address);
    #endif
    #ifdef USE_I2C_FRAM
       PressureConversion.bytes[address] = fram.read(mem_PRESSURE_OFFSET+address);
    #endif
    #ifdef USE_I2C_EEPROM
       PressureConversion.bytes[address] = fram.read(mem_PRESSURE_OFFSET+address);
    #endif
   // WundergroundStationID[address] = char(buffer1[address]);
    //  Serial.println("could not read fram");
    }
    PressureOffset = PressureConversion.pressureF;

    Serial.print("Pressure Calibration Offset: ");Serial.println(PressureOffset);

        // Queue Baromter to Interfaces 

  #ifdef USE_SPI_FRAM
    enable = fram.read8(mem_BAR_ENABLE);
  #endif
  #ifdef USE_I2C_FRAM
    enable = fram.read(mem_BAR_ENABLE);
  #endif
  #ifdef USE_I2C_EEPROM
    enable = fram.read(mem_BAR_ENABLE);
  #endif
  if(enable>0){
    Serial.println("enabled Queue Barometers for Interfaces");
    QueueBarometerForInterfaces = true;
  }
  else {
    Serial.println("Disabled Queue Barometers for Interfaces");
    QueueBarometerForInterfaces = false;
  }
#endif
  // Load Thermometer information 
  #ifdef USE_SPI_FRAM
    enable = fram.read8(mem_INFCE_THERMOMETER_ENABLE);
  #endif
  #ifdef USE_I2C_FRAM
    enable = fram.read(mem_INFCE_THERMOMETER_ENABLE);
  #endif
  #ifdef USE_I2C_EEPROM
    enable = fram.read(mem_INFCE_THERMOMETER_ENABLE);
  #endif

  if(enable>0){
    Serial.println("enabled Queue Thermometer for Interfaces");
    QueueThermometerForInterfaces = true;
  }
  else {
    Serial.println("Disabled Queue Thermometer for Interfaces");
    QueueThermometerForInterfaces = false;
  }

  #ifdef USE_SPI_FRAM
    enable = fram.read8(mem_INFCE_THERMOMETER_UNIT);
  #endif
  #ifdef USE_I2C_FRAM
    enable = fram.read(mem_INFCE_THERMOMETER_UNIT);
  #endif
  #ifdef USE_I2C_EEPROM
    enable = fram.read(mem_INFCE_THERMOMETER_UNIT);
  #endif

  if(enable>0){
    Serial.println("Thermometer Unit: C");
    UseCelcius = true;
  }
  else {
    Serial.println("Thermometer Unit: F");
    UseCelcius = false;
  }

   #ifdef USE_SPI_FRAM
    enable = fram.read8(mem_INFCE_HUMIDITY_ENABLE);
  #endif
  #ifdef USE_I2C_FRAM
    enable = fram.read(mem_INFCE_HUMIDITY_ENABLE);
  #endif
  #ifdef USE_I2C_EEPROM
    enable = fram.read(mem_INFCE_HUMIDITY_ENABLE);
  #endif

  if(enable>0){
    Serial.println("enabled Queue Humidity for Interfaces");
    QueueHumidityForInterfaces = true;
  }
  else {
    Serial.println("Disabled Queue Humidity for Interfaces");
    QueueHumidityForInterfaces = false;
  }

}

void saveThermometerToDisk(){
  uint32_t address;
  char ok[2 + 1];
  uint8_t buffer1[512];
  uint8_t enable = 0;


    if(QueueThermometerForInterfaces == true){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_INFCE_THERMOMETER_ENABLE,(uint8_t)255);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_INFCE_THERMOMETER_ENABLE,(uint8_t)255);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_INFCE_THERMOMETER_ENABLE,(uint8_t)255);
    #endif
    }
    //if(!fram.write8(mem_WUNDERGROUND_ENABLE,255)) {Serial.println("Could not write wunderground enable flag to fram ");}}
  else {
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_INFCE_THERMOMETER_ENABLE,0);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_INFCE_THERMOMETER_ENABLE,0);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_INFCE_THERMOMETER_ENABLE,0);
    #endif
    }

  if(UseCelcius == true){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_INFCE_THERMOMETER_UNIT,(uint8_t)255);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_INFCE_THERMOMETER_UNIT,(uint8_t)255);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_INFCE_THERMOMETER_UNIT,(uint8_t)255);
    #endif
    }
    //if(!fram.write8(mem_WUNDERGROUND_ENABLE,255)) {Serial.println("Could not write wunderground enable flag to fram ");}}
  else {
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_INFCE_THERMOMETER_UNIT,0);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_INFCE_THERMOMETER_UNIT,0);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_INFCE_THERMOMETER_UNIT,0);
    #endif
    }

}

void saveHumidityToDisk(){
  uint32_t address;
  char ok[2 + 1];
  uint8_t buffer1[512];
  uint8_t enable = 0;


  if(QueueHumidityForInterfaces == true){
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_INFCE_HUMIDITY_ENABLE,(uint8_t)255);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_INFCE_HUMIDITY_ENABLE,(uint8_t)255);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_INFCE_HUMIDITY_ENABLE,(uint8_t)255);
    #endif
  }
    //if(!fram.write8(mem_WUNDERGROUND_ENABLE,255)) {Serial.println("Could not write wunderground enable flag to fram ");}}
  else {
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_INFCE_HUMIDITY_ENABLE,0);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_INFCE_HUMIDITY_ENABLE,0);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_INFCE_HUMIDITY_ENABLE,0);
    #endif
    }
}

void RestoreDefaults(){
  uint32_t address;
  char ok[2 + 1];
  uint8_t buffer1[512];
 // Serial.print("SSID: ");Serial.println(ssid);
 // Serial.print("Password: ");Serial.println(password);
 // memcpy(buffer1, (void *)&ssid, sizeof(ssid));
  //String("user").toCharArray(dev_user, sizeof(dev_user));
  //String("password").toCharArray(dev_password, sizeof(dev_password));

  String("").toCharArray(WundergroundStationID,sizeof(WundergroundStationID));
  String("").toCharArray(WundergroundStationPassword,sizeof(WundergroundStationPassword));
  WundergroundInfceEnable=true;
  thermometer1Type = WU_S_TEMPF_T;
  humidity1_sensor_type = WU_S_HUMIDITY_T;
  QueueThermometerForInterfaces=true;
  QueueHumidityForInterfaces=true;
  SaveWundergroundCredentials();

  /*
  for(address = 0; address <sizeof(dev_user);address++){
    buffer1[address] = (uint8_t)dev_user[address];
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_DEV_USER+address,buffer1[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_DEV_USER+address,buffer1[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_DEV_USER+address,buffer1[address]);
    #endif 
    }
    */
    /*
    for(address = 0; address <sizeof(dev_password);address++){
    buffer1[address] = (uint8_t)dev_password[address];
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_DEV_PASSWORD+address,buffer1[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_DEV_PASSWORD+address,buffer1[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_DEV_PASSWORD+address,buffer1[address]);
    #endif 
    }
    */

}

// Save device user and device password to NVRAM
/*
void SaveDeviceCredentials(){
  uint32_t address;
  char ok[2 + 1];
  uint8_t buffer1[512];
  for(address = 0; address <sizeof(dev_user);address++){
    buffer1[address] = (uint8_t)dev_user[address];
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_DEV_USER+address,buffer1[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_DEV_USER+address,buffer1[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_DEV_USER+address,buffer1[address]);
    #endif 
    }

    for(address = 0; address <sizeof(dev_password);address++){
    buffer1[address] = (uint8_t)dev_password[address];
    #ifdef USE_SPI_FRAM
      fram.writeEnable(true);
      fram.write8(mem_DEV_PASSWORD+address,buffer1[address]);
      fram.writeEnable(false);
    #endif
    #ifdef USE_I2C_FRAM
      fram.write(mem_DEV_PASSWORD+address,buffer1[address]);
    #endif
    #ifdef USE_I2C_EEPROM
      fram.write(mem_DEV_PASSWORD+address,buffer1[address]);
    #endif 
    }
}

*/


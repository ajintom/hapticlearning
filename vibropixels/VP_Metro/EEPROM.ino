#define EEPROM_SERIAL_DEBUG 0
#define SERIAL_PRINT_PARAMETERS 0

/*
EEPROM ADDRESSES:
0-99 actuator
100-199 LED
200-399 accelerometer
400-499 system

999: if == 255 runs initilization routine

deviceID = eeprom 400
supergroup = eeprom 401
 */

#include <EEPROM.h>

byte eepromFlag=0; //gets set to indicate a parameter has been changed and needs to be saved to EEPROM;

struct defEeprom{
  byte flag;
  uint32_t timer;
  uint16_t interval;
  
  uint16_t index;
};

defEeprom eeprom;

/******************************
EEPROM setup
******************************/
void eepromSetup(){
  //initializeVariables();
  if(EEPROM_SERIAL_DEBUG) Serial.begin(57600);
  eeprom.interval=10000;
  if(EEPROM_SERIAL_DEBUG) Serial.println("Reading EEPROM settings");
  if(EEPROM.read(999)==255) initializeEepromVariables();
  readEepromSettings();
  //setBrakeInterval(75);
  if(EEPROM_SERIAL_DEBUG) Serial.println("EEPROM read complete");
};

void initializeEepromVariables(){
  //setBrakeInterval(75);
  setLedColor2(0,212); setLedColor2(1,154); setLedColor2(2,0);
  setLedColor1(0,0); setLedColor1(1,0); setLedColor1(2,0);
  setLedEnvelope(0,10); setLedEnvelope(1,0); setLedEnvelope(2,255); setLedEnvelope(3,5);
  setLedScalars(0,0); setLedScalars(1,255); setLedScalars(2,0); setLedScalars(3,0); setLedScalars(4,0);
  //setActuatorScalars(0,0); setActuatorScalars(1,0); setActuatorScalars(2,255);
  writeEepromSettings();
  safeWriteEeprom(999,0);
}

void initializeVariables(){
//  motor1.brake(75);
//  motor2.brake(75);
  setLedColor2(0,212); setLedColor2(1,154); setLedColor2(2,0);
  setLedColor1(0,0); setLedColor1(1,0); setLedColor1(2,0);
  setLedEnvelope(0,10); setLedEnvelope(1,0); setLedEnvelope(2,255); setLedEnvelope(3,5);
  setLedScalars(0,0); setLedScalars(1,0); setLedScalars(2,255); setLedScalars(3,255); setLedScalars(4,0);
  //setActuatorScalars(0,0); setActuatorScalars(1,0); setActuatorScalars(2,255);
}

/****************************************************************************************************************
*LOOP
****************************************************************************************************************/
void eepromLoop(){
  if(eeprom.flag==1){
    if(millis()>eeprom.timer+eeprom.interval) {
      writeEepromSettings();
      if(EEPROM_SERIAL_DEBUG) Serial.println("EEPROM settings written");
    }
  }
}

/****************************************************************************************************************
*READ and SET deviceID
****************************************************************************************************************/

void setDeviceID(byte val)
{
  newDataFlag(); 
  deviceID=val;
  setLED(deviceID);
}

void displayDeviceID(){
  byte fives = deviceID / 5;
  byte ones = deviceID % 5;
  for(byte i=0;i<fives;i++){
    processLed(40,40,0);
    wdt_reset();
    delay(500);
    processLed(0,0,0);
    wdt_reset();
    delay(500);
  }
  for(byte i=0;i<ones;i++){
    processLed(40,40,0);
    wdt_reset();
    delay(100);
    wdt_reset();
    processLed(0,0,0);
    delay(400);
  }
}

/****************************************************************************************************************
*READ and SET supergroup
****************************************************************************************************************/

void setSupergroup(byte val)
{
  newDataFlag(); 
  supergroup=val;
  setLED(deviceID);
}

void displaySupergroup(){
  byte fives = supergroup / 5;
  byte ones = supergroup % 5;
  for(byte i=0;i<fives;i++){
    processLed(0,0,50);
    wdt_reset();
    delay(500);
    wdt_reset();
    processLed(0,0,0);
    delay(500);
  }
  for(byte i=0;i<ones;i++){
    processLed(0,0,50);
    wdt_reset();
    delay(100);
    wdt_reset();
    processLed(0,0,0);
    delay(400);
  }
}

/****************************************************************************************************************
*EEPROM FLAG 
*This function serves to indicate new parameters have been set, and
*to initialize a timer to prepare for saving the new data to EEPROM.
*The point of the timer is to minimize extraneous EEPROM writes for
*when you are scrolling through parameter values to find the right one.
****************************************************************************************************************/

void newDataFlag(){
  eeprom.flag=1;
  eeprom.timer=millis();
  if(EEPROM_SERIAL_DEBUG) Serial.println("New data received");
}

/****************************************************************************************************************
*WRITE EEPROM SETTINGS 
****************************************************************************************************************/
void writeEepromSettings(){
  //writeEepromActuatorSettings();
  //writeEepromLedSettings();
  writeEepromAccelerometerSettings();
  writeEepromSystemSettings();
  eeprom.flag=0;
}
/*
void writeEepromActuatorSettings(){
  byte index=0;
  int val = getMotorParameters(0);
  while(val<256){
    if(EEPROM_SERIAL_DEBUG){
    Serial.print("Actuator write, index: ");
    Serial.print(index);
    Serial.print(", value: ");
    Serial.println(val);
    }
    
    safeWriteEeprom(index,(byte)val);
    index++;
    val = getMotorParameters(index);
    if(index>100) val=256;
  }
}*/

/*
void writeEepromLedSettings(){
  uint16_t start_index=100;
  
  byte index=0;
  int val = getLedParameters(0);
  while(val<256){
    if(EEPROM_SERIAL_DEBUG){
    Serial.print("LED write, index: ");
    Serial.print(index+start_index);
    Serial.print(", value: ");
    Serial.println(val);
    }
    
    safeWriteEeprom(index + start_index,(byte)val);
    index++;
    val = getLedParameters(index);
    if(index>100) val=256;
  }
}
*/

void writeEepromAccelerometerSettings(){
  uint16_t start_index=200;
  
  byte index=0;
  int val = getAccelParameters(0);
  while(val<256){ 
    if(EEPROM_SERIAL_DEBUG){
    Serial.print("Accel write, index: ");
    Serial.print(index+start_index);
    Serial.print(", value: ");
    Serial.println(val);
    }
    
    safeWriteEeprom(index + start_index,(byte)val);
    index++;
    val = getAccelParameters(index);
    if(index>100) val=256;
  }
}

void writeEepromSystemSettings(){
  uint16_t start_index=400;
  
  byte index=0;
  int val = getSystemParameters(0);
  while(val<256){
    if(EEPROM_SERIAL_DEBUG){
    Serial.print("system write, index: ");
    Serial.print(index+start_index);
    Serial.print(", value: ");
    Serial.println(val);
    } 
    
    safeWriteEeprom(index + start_index,(byte)val);
    index++;
    val = getSystemParameters(index);
    if(index>100) val=256; 
  }
}

int getSystemParameters(byte num){
  switch(num){
    case 0: return deviceID;
    case 1: return supergroup;
  }
  return 256;
}

/****************************************************************************************************************
*READ EEPROM SETTINGS 
****************************************************************************************************************/
void readEepromSettings(){
  //readEepromActuatorSettings();
 // if(EEPROM_SERIAL_DEBUG) Serial.println("Read motor settings");
 // readEepromLedSettings();
 // if(EEPROM_SERIAL_DEBUG) Serial.println("Read led settings");
  readEepromAccelerometerSettings();
 // if(EEPROM_SERIAL_DEBUG) Serial.println("Read accel settings");
  readEepromSystemSettings();
  //if(EEPROM_SERIAL_DEBUG) Serial.println("Read system settings");
  if(EEPROM_SERIAL_DEBUG) Serial.println("EEPROM settings read");
}
/*
void readEepromActuatorSettings(){
  uint16_t start_index=0;
  byte index=0;
  
  byte val = EEPROM.read(index + start_index);
  int write_status = setMotorParameters(0, val);
  while(write_status<256){
    if(EEPROM_SERIAL_DEBUG){
    Serial.print("Actuator read, index: ");
    Serial.print(index+start_index);
    Serial.print(", value: ");
    Serial.println(write_status);
    }
    
    index++;
    val = EEPROM.read(index + start_index);
    write_status = setMotorParameters(index, val);
    if(index>100) write_status=256;
  }
}*/

/*
void readEepromLedSettings(){
  uint16_t start_index=100;
  byte index=0;
  
  byte val = EEPROM.read(index + start_index);
  int write_status = setLedParameters(0, val);
  while(write_status<256){
    if(EEPROM_SERIAL_DEBUG){
    Serial.print("LED read, index: ");
    Serial.print(index+start_index);
    Serial.print(", value: ");
    Serial.println(write_status);
    }
    
    index++;
    val = EEPROM.read(index + start_index);
    write_status = setLedParameters(index, val);
    if(index>100) write_status=256;
  }
}
*/

void readEepromAccelerometerSettings(){
  uint16_t start_index=200;
  byte index=0;
  
  byte val = EEPROM.read(index + start_index);
  int write_status = setAccelParameters(0, val);
  while(write_status<256){
    if(EEPROM_SERIAL_DEBUG){
    Serial.print("Accel read, index: ");
    Serial.print(index+start_index);
    Serial.print(", value: ");
    Serial.println(write_status);
    }
    
    index++;
    val = EEPROM.read(index + start_index);
    write_status = setAccelParameters(index, val);
    if(index>100) write_status=256;
  }
}

void readEepromSystemSettings(){
  uint16_t start_index=400;
  byte index=0;
  
  byte val = EEPROM.read(index + start_index);
  int write_status = setSystemParameters(0, val);
  while(write_status<256){
    if(EEPROM_SERIAL_DEBUG){
    Serial.print("System read, index: ");
    Serial.print(index+start_index);
    Serial.print(", value: ");
    Serial.println(write_status);
    }
    
    index++;
    val = EEPROM.read(index + start_index);
    write_status = setSystemParameters(index, val);
    if(index>100) write_status=256;
  }
}

int setSystemParameters(byte num, byte val){
  switch(num){
    case 0:  deviceID=val; return 0; break;
    case 1: supergroup=val; return 0; break;
  }
  return 256;
}

/*

*/
void safeWriteEeprom(int index, byte val){
  if(EEPROM.read(index) != val) EEPROM.write(index, val);
}

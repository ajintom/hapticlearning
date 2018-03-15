#define ENABLE_SIMPLE_DIGITAL_OUT 0
#define ENABLE_SETUP_DEBUG 0
#include <SPI.h>
#include <avr/wdt.h>
#include "VPmotor.h"

const byte VP_VERSION = 2;
uint32_t loop_counter=0;
uint16_t rf_counter=10;
byte reset_flag=0;

byte SERIALOUT = 0; //send useful info over Serial
#define DEBUG 0 //for more messy debugging over serial

const byte NUMBER_OF_DEVICES = 9;

//function of this module - 0=RX only, 1=TX only, 2=both
const byte RX_MODE = 0;
const byte TX_MODE  = 1;
byte RxTxMode= RX_MODE; //change this flag before uploading to microcontroller

byte deviceID = 0; //change according to id startib ib 1 and up till 12 units. Only relevant for rx modules
byte supergroup=1;
byte testMode=0;

byte deviceMode=1; //0=setup, 1=active; 
//In setup mode, LEDs display deviceId color and motors are inactive; 
//in active mode LEDs and motors are activated by 

//byte powerState = 0; //0=OK, 1=warning, 2=turn power off
//byte powerOffset=0;


/*
WIRELESS PROTOCOL VARIABLES
*/

//constants for SLIP encoding 
const byte escByte = 219; //1101 1011
const byte beginMsg = 164; //1010 0100
const byte endMsg = 192; // 1100 0000

//message types
const byte sensors=11;
const byte errorMsg = 12;
const byte acknowledgement=13; 
const byte testMessage=254;

//variables for acknowledging packet receipt
byte recPktNum=0;
byte packetReceiveFlag=0;

//variables for polling enable
byte pollEnable=0;
uint32_t pollEnableTimer=0; //check to see if to continue polling
uint16_t pollEnableInterval = 5000; //rate of poll timeout
uint16_t poll_interval=100;

byte specialChar[] = {
  escByte, //escape character
  beginMsg, //begin message packet
  endMsg, //end character
};

//MOTORS
VPmotor motor1(3,4);
VPmotor motor2(5,6);

//SETUP
/*
 * 
 * 
 */
void setup() { 
  loop_counter=0;
  rf_counter=10;
  reset_flag=0;
  
  buttonSetup(); 
  delay(10);
  if(digitalRead(7) == 0) testMode=1;
  //Serial.begin(57600);
  if(ENABLE_SETUP_DEBUG) Serial.begin(57600);
  if (testMode) SERIALOUT = 1;
  if(SERIALOUT) Serial.begin(57600);
  if(testMode)  Serial.println("begin test!"); 
  
  accelSetup();
  eepromSetup();
  RFSetup();
  //actuatorSetup();
  ledSetup();
  testSetup();
  
  motor2.scalar(255); //limit max amplitude of motor2
  
  if(1){
    processLed(50,50,50);
    delay(500);
    if(ENABLE_SETUP_DEBUG) processLed(0,0,0);
    delay(200);
}
  processLed(0,0,0);
  powerSetup();
  
  if(deviceID > NUMBER_OF_DEVICES) deviceID=1;

  //disable test mode
  //if(!digitalRead(7)) testMode=1; //if button is held down on power up, enter test mode

  watchdogOn();

  if(ENABLE_SIMPLE_DIGITAL_OUT){
    pinMode(0,OUTPUT);
    pinMode(1,OUTPUT);
  }
  
  if(ENABLE_SETUP_DEBUG) Serial.println("Setup Complete");
}

/*MAIN LOOP
 * 
 * 
 * 
 */
void loop() {
  //simpleTestDigitalOut(); //toggles tx pin once per loop
  
  //resetTimer();
  accelLoop();
  testLoop();
  //if(testMode) testLoop();
  if(powerState>0){
    //checkButton();
    checkNRF24();
    //if(deviceMode==1) //actuatorLoop();
    //accelLoop();
    static uint32_t timer=0;
    byte interval=5;
    if(millis() - timer > interval){
      timer=millis();
    
    motor1.loop();
    motor2.loop();
    }
    //processOutput();
    ledLoop();
    eepromLoop();
  } 
  else powerOff();
  
  powerLoop();
  
  if(reset_flag==0) wdt_reset();
  loop_counter++;
  //motorReset(10);
  
}

void resetTimer(){
  static uint32_t timer=0;
  uint16_t interval=10000;
  int reset_interval = 60;
  static uint16_t minute_counter=0;

  if(millis()>timer+interval){
    timer=millis();
    minute_counter++;
    //Serial.println("minute");
  }

  if(minute_counter==reset_interval){
    //Serial.println("reset");
    reset_flag=1;
    minute_counter=0;
  }
}


void watchdogOn() {
  cli();
  wdt_reset();
// Clear the reset flag, the WDRF bit (bit 3) of MCUSR.
MCUSR = MCUSR & B11110111;
  
// Set the WDCE bit (bit 4) and the WDE bit (bit 3) 
// of WDTCSR. The WDCE bit must be set in order to 
// change WDE or the watchdog prescalers. Setting the 
// WDCE bit will allow updtaes to the prescalers and 
// WDE for 4 clock cycles then it will be reset by 
// hardware.
WDTCSR = WDTCSR | B00011000; 

// Set the watchdog timeout prescaler value to 1024 K 
// which will yeild a time-out interval of about 8.0 s.
WDTCSR = B00000110;

// Enable the watchdog timer interupt.
WDTCSR = WDTCSR | B01000000;
MCUSR = MCUSR & B11110111;

sei();
}



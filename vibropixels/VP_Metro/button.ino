#define BUTTON_SERIAL_DEBUG 0
byte buttonPin = 7;

typedef struct Button{
  byte pin;
  byte val;
  byte prevVal;
  byte timer;
  byte interval;
};

Button button = {7,0,0,0,100};

/*
 * SETUP
 * 
 */
void buttonSetup(){
  pinMode(button.pin, INPUT_PULLUP);
}

//CHECKBUTTON
void checkButton(){
  byte transition=buttonStatus();
  if(transition){
    if(deviceMode==1) {
      deviceMode=0;
      setDeviceID(deviceID);
    }
    else{
    deviceID = (deviceID+1) % NUMBER_OF_DEVICES;
    setDeviceID(deviceID);
    if (BUTTON_SERIAL_DEBUG) Serial.print("Device ID: ");
    if (BUTTON_SERIAL_DEBUG) Serial.println(deviceID);
    }
  }
}

//buttonStatus
byte buttonStatus(){

  //store current and previous button values
  static uint8_t buttonValue[]={1,1};
  static byte transition=0;
  
  uint32_t curMillis2 = millis(); //called 2 to avoid conflicts
  if(curMillis2>button.timer+button.interval){
    button.timer=curMillis2;
    button.val=digitalRead(button.pin);
    if ( button.val < button.prevVal ){
      transition=1;
      button.prevVal=button.val;
    } 
    else if ( button.val > button.prevVal ){
      button.prevVal=button.val;
      transition=0;
    }
    else if ( button.val == button.prevVal ){
      button.prevVal=button.val;
      transition=0;
    }
    if((transition==1) && (curMillis2<button.timer+100)) transition=0;
  }
  return transition;
}

//assigns a unique colour to each board ID
//0 = turn led off, device ID from 1-12
#define NUMBER_OF_LED_SUBGROUPS 4
#define LED_SERIAL_DEBUG 0

#include <Adafruit_NeoPixel.h>
#define NEOPIXEL_PIN 9
#define NUMPIXELS 4

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/* LED driver functions
 * 
 * call setLED(byte ID) to set LED color. The result is calculated by ledTimer(byte ID, byte subgroup).
 * leDTimer(255,0) is also called by ledLoop() - the ID 255 is used to signal processing the loop without changing ID or subgroup
 * 
 *   //within each group the LED blinks to indicat the number of the device within the group
  //i.e. red-1 blinks once, red-2 blinks twice, etc.
 * 
 */

byte device_color[4][3] = {
  {0,11,191},
  {6,120,0},
    {59,0,159},
  {159,50,0},
};

 struct defLed{
  byte pin[3];
  byte color1[3];
  byte color2[3];
  byte cur_color[3];
  
  //variable for scaling input sources
  //all values form 0-255, indicate amount of influence the input signal has on LED output
  byte accelScale;
  byte accelSource;
  byte motor;
  byte ADSR; //just for led envelope
  byte packet;
  
  byte newColor1[3];
  byte oldColor1[3];
  byte newColor1Flag;
  uint32_t color1_onset;
  byte powerVal;
  uint32_t pwrTimer;
  
  byte state; 
  uint16_t envelope[4];
  uint32_t onset;
  byte goal;
  byte curEnv;
};

defLed LED;


/*SETUP
 * 
 * 
 */
void ledSetup(){
  //setup for both versions of VP
  if(VP_VERSION==1){ //version 1
    LED.pin[0]=3; LED.pin[1]=5; LED.pin[2]=6;
    for (byte i=0;i<3;i++) pinMode(LED.pin[i],OUTPUT);
  } else if (VP_VERSION==2) { //version 2
    //pinMode(16,INPUT);
    pixels.begin();
  }
  for(byte i=0;i<3;i++) setLedColor1(0,i);
  LED.color2[0] =0; LED.color2[1] =0; LED.color2[2] =0; 
  LED.color1[0] =0; LED.color1[1] =0; LED.color1[2] =0; 
  LED.envelope[0]=0; LED.envelope[1]=200; LED.envelope[2]=255; LED.envelope[3]=500;
  LED.ADSR=200;
  LED.packet=55; 
  LED.powerVal=0;
  LED.state=0;
  for(byte i=0;i<3;i++) setLedColor1(0,i);
  if (LED_SERIAL_DEBUG) {Serial.begin(57600); delay(100);}
  if (LED_SERIAL_DEBUG) Serial.print("LED Debug On");   
} //end setup

/*LED LOOP
 * 
 * 
 */
 void ledLoop(){
  if(deviceMode==0) ledStandby(255,0);
  else ledMain();
 } //LED LOOP

/****************************************************************************************************************
*LEDPROCESS
*functions for active control of LED
****************************************************************************************************************/
void ledMain(){
  static uint32_t timer=0;
  byte interval=20;
  uint32_t cur_millis=millis();
  
  if(cur_millis>timer+interval){
    timer=cur_millis;
    calcLedEnv();
    calcLedValue();
    calcLedPowerVal();
    if(LED.newColor1Flag==1) calcColor1();
  }
}

/****************************************************************************************************************
*LED CALCULATIONS
****************************************************************************************************************/

void calcColor1(){
  uint32_t cur_position = millis() - LED.color1_onset;
  float led_scalar;

  if (cur_position <  LED.envelope[0])  {
    led_scalar = (float)cur_position/LED.envelope[0]; //attack
    for(byte i=0;i<3;i++){
      LED.color1[i] = (byte)(((float)LED.oldColor1[i]*(1-led_scalar)) + ((float)LED.newColor1[i]*(led_scalar)));
    }
  }
  else if (cur_position > LED.envelope[0]) {
   LED.newColor1Flag=0;
   for(int i=0;i<3;i++)LED.color1[i]=LED.newColor1[i];
  }
}//calcColor1

/*
 * calcLedEnv based on motor envelopes
 */
 
void calcLedEnv(){
    if (LED.state > 0 ) {
        uint32_t cur_position = millis() - LED.onset;
        byte led_scalar = (LED.envelope[2]);
        
        //attack
        if (cur_position <  LED.envelope[0])  LED.curEnv =  (((float)cur_position/LED.envelope[0]) * led_scalar);
        //sustain
        else if (cur_position < LED.envelope[0] + LED.envelope[1])   LED.curEnv = led_scalar;
        //decay phase
        else if (cur_position < LED.envelope[0] + LED.envelope[1] + LED.envelope[3]) { 
        LED.curEnv = ((1-((float)cur_position-LED.envelope[0]+LED.envelope[1])/LED.envelope[3]) * led_scalar) ;}
        //end of envelope
        else if (cur_position > LED.envelope[0] + LED.envelope[1] + LED.envelope[3] ) { 
        LED.curEnv = 0;
        LED.state=0;
      }
  }
}//calcLedEnv

/*
 * Generates output values for LEDs
 */
 
void calcLedValue(){
  uint8_t accel_scalar=0;
  uint8_t motor_scalar=0;
  uint8_t ADSR_scalar=0;
  uint8_t packet_scalar=0;
  uint16_t led_scalar=0;

  //calculate current LED value based on the values of the four scalars
  //if(LED.accelScale > 0)  accel_scalar = ((uint16_t)LED.accelScale*getAccelValue(LED.accelSource))>>8;

  //base LED lighting on whichever motor value is greatest
  uint8_t curMotorScalar = motor1.output();
  uint8_t curMotorScalar2 = motor2.output();
  if(curMotorScalar2 > curMotorScalar) curMotorScalar = curMotorScalar2;
  if(LED.motor > 0) motor_scalar = ((uint16_t)LED.motor*curMotorScalar2)>>8;
  
  if(LED.ADSR > 0) ADSR_scalar = ((uint16_t)LED.ADSR*LED.curEnv)>>8;
  //if(LED.packet > 0) packet_scalar = LED.packet;
  led_scalar = ADSR_scalar + motor_scalar + accel_scalar;
  if(led_scalar > 255) led_scalar=255;

  //calculate the current color as an interpolation of color1 and color2
  for(byte i=0;i<3;i++){ 
    int diff = (led_scalar*(LED.color2[i]-LED.color1[i]))/256;
    LED.cur_color[i] = LED.color1[i]+diff;
  }
  
  processLed(LED.cur_color[0],LED.cur_color[1],LED.cur_color[2]);
}//calcLedValue

//CALC LED POWER VAL
void calcLedPowerVal(){
  //for all states except for normal operation
  if(powerState() != 3){
    //interval scales with batt.volt = lower voltage equals higher rate
      int interval = ((getBattVolt() - 310 ) * 4 ) + 200; //ranges from 2200 to 200ms
    if(millis() > LED.pwrTimer + interval){
      LED.pwrTimer=millis();

      //brightness scales 
      //int val = 
      
      //uint16_t att=interval/3;
      //uint16_t dec=interval/3;
      uint16_t att=100;
      uint16_t dec=100;
    }
  }
}

/****************************************************************************************************************
*LED STANDBY
*
****************************************************************************************************************/

//setLED
void setLED(byte ID){
  byte num_in_subgroup = NUMBER_OF_DEVICES / NUMBER_OF_LED_SUBGROUPS;
  if(ID>NUMBER_OF_DEVICES){ID=0;} 

    byte subgroup= ID/num_in_subgroup;
    ID =   ID % num_in_subgroup;
    ledStandby(ID, subgroup);

} //setLED

void ledStandby(byte ID, byte subgroup){
  //private variables
  static byte led_flag=0;
  static uint32_t led_timer=0;
  uint16_t led_interval=100;
  static uint8_t cur_ID=0;
  static uint8_t cur_subgroup=0;
  static uint8_t led_loop_position = 0;

  //set new sequence
  if(ID!=255){
      cur_ID=ID;
      cur_subgroup=subgroup;
      led_flag=1;
      led_timer=millis();
      led_loop_position=0;
  } 

  //process sequence
  else{
    if(led_flag==1){
      if(millis() > led_timer + (led_interval*led_loop_position)){
        byte led_switch=led_loop_position%2;
        switch(led_switch){
          case 0:
          processLed(device_color[cur_subgroup][0],device_color[cur_subgroup][1],device_color[cur_subgroup][2]); //lights led
          break;

          case 1:
          setLedOff(); //turns off
          break;
        }
        led_loop_position++;
        if(led_loop_position > cur_ID*2 ) led_flag=0;
      }
    }
  }
} //ledTimer

  
//PROCESS LED

/* OLD VERSION
void processLed(byte r, byte g, byte b){
  uint8_t input[] = {r,g,b};
  for(byte i=0;i<3;i++){
    float out_color = 255 - input[i]; //invert brightness due to common anode
    analogWrite(LED.pin[i],(uint8_t)out_color);
    if(LED_SERIAL_DEBUG){
      Serial.print((uint8_t) input[i]);
      Serial.print("\t");
      } 
  } if(LED_SERIAL_DEBUG) Serial.println();
}//old processLED
*/

void processLed(byte r, byte g, byte b){
  static uint8_t prevColors[3];
  byte newColorFlag=0;
  uint8_t input[] = {r,g,b};

  //is there a new color?
  for(byte i=0;i<3;i++){
    if(input[i]!=prevColors[i]){
      prevColors[i]=input[i];
      newColorFlag=1;
    }
  }

  if(newColorFlag==1){
    if(VP_VERSION==1) {
      if(LED_SERIAL_DEBUG) Serial.println("Processing VP version 1");
      for(byte i=0;i<3;i++){
       float out_color = 255 - input[i]; //invert brightness due to common anode
       analogWrite(LED.pin[i],(uint8_t)out_color);
      }
    }
    else if(VP_VERSION==2){
      if(LED_SERIAL_DEBUG) Serial.println("Processing VP version 2");
      for(byte i=0;i<NUMPIXELS;i++) pixels.setPixelColor(i, pixels.Color(r,g,b)); 
      pixels.show();
    }
    
    if(LED_SERIAL_DEBUG){
      Serial.print("Process LEDs: ");
      for(byte i=0;i<3;i++){
        Serial.print((uint8_t) input[i]);
        Serial.print("\t");  
      } 
     Serial.println();
    }
  }
} //processLED

//setLedOff
void setLedOff(){
  processLed(0,0,0);
}


/****************************************************************************************************************
*SETTER AND GETTERS
*
****************************************************************************************************************/
void setLedEnvelope(byte val, byte index){
  if(index==2)LED.envelope[index]=val;
  else LED.envelope[index]=val*val;
}
void triggerLedEnvelope(){
  LED.onset=millis();
  LED.state=1;
}
void setLedColor1(byte val,byte index){
  LED.oldColor1[index]=LED.color1[index];
  LED.newColor1[index]=val;
  LED.newColor1Flag=1;
  LED.color1_onset=millis();
  if (LED_SERIAL_DEBUG) {Serial.print("Color 1: "); Serial.print(index);Serial.print(" ");Serial.println(val);}
}

void setLedColor2(byte val,byte index){
  LED.color2[index]=val;
  if (LED_SERIAL_DEBUG) {Serial.print("Color 2: "); Serial.print(index);Serial.print(" ");Serial.println(val);}
}

void setLedScalars(byte val, byte index){
  if (LED_SERIAL_DEBUG) {Serial.print("Scalar: ");Serial.print(index);Serial.print(" ");Serial.println(val);}
  switch(index){
    case 0:LED.accelSource=val; break;
    case 1:LED.accelScale=val; break;
    case 2:LED.motor=val; break;
    case 3:LED.ADSR=val; break;
    case 4: LED.packet=val; break;
  }
}

int getLedParameters(byte num){
    switch(num){
    case 0: return LED.color1[0]; break;
    case 1: return LED.color1[1]; break;
    case 2: return LED.color1[2]; break;
    case 3: return LED.color2[0]; break;
    case 4: return LED.color2[1]; break;
    case 5: return LED.color2[2]; break;
    
    case 6: return LED.accelScale; break;
    case 7: return LED.accelSource; break;
    case 8: return LED.motor; break;
    case 9: return LED.ADSR; break;
    case 10: return LED.packet; break;
    
    case 11: return sqrt(LED.envelope[0]); break;
    case 12: return sqrt(LED.envelope[1]); break;
    case 13: return sqrt(LED.envelope[2]); break;
    case 14: return sqrt(LED.envelope[3]); break;
    
  //  case 15: return LED.fadeMode; break;
    default: return 256;
  }
  return 256;
}

int setLedParameters(byte num, byte val){
    switch(num){
    case 0:  LED.color1[0] = val; return 0; break;
    case 1:  LED.color1[1] = val; return 0; break;
    case 2:  LED.color1[2] = val; return 0; break;
    case 3:  LED.color2[0] = val; return 0; break;
    case 4:  LED.color2[1] = val; return 0; break;
    case 5:  LED.color2[2] = val; return 0; break;
    
    case 6:  LED.accelScale = val; return 0; break;
    case 7:  LED.accelSource = val; return 0; break;
    case 8:  LED.motor = val; return 0; break;
    case 9:  LED.ADSR = val; return 0; break;
    case 10:  LED.packet = val; return 0; break;
    
    case 11:  LED.envelope[0] = pow(val,2); return 0;  break;
    case 12:  LED.envelope[1] = pow(val,2); return 0; break;
    case 13:  LED.envelope[2] = pow(val,2); return 0; break;
    case 14:  LED.envelope[3] = pow(val,2); return 0; break;
    
   // case 15:  LED.fadeMode = val; return 0; break;
  }
  return 256;
}

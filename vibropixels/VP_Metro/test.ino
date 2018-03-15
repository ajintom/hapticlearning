/*
Functions for generating signals for testing
Pins 0 and 1 are used for indicating  status
 
pin0  pin1  message
0     0     none
1     1     reset/poweron
1     0     message received, addressed to this device
0     1     message received, addressed to this device
*/
const byte onTime=100;

struct defTestPin{
  byte pin;
  byte state;
  uint32_t onset;
  uint32_t prev_onset;
};

defTestPin testPin[2];
uint32_t prevOnset=0;

void testSetup(){
  for(byte i=0;i<2;i++){
    testPin[i].pin=i;
    testPin[i].state=0;
   // pinMode(testPin[i].pin,OUTPUT);
    //set both pins high on reset
    testPin[i].state=1;
    testPin[i].onset=millis();
  }
}

void testLoop(){
  for(byte i=0;i<2;i++){
    if(testPin[i].state==1){
      if(millis()-testPin[i].onset > onTime) {
        testPin[i].state=0;
      }
    }
  }
}

void setTestPin(byte state, byte pktNum, byte countNum){
  Serial.print(pktNum);
  Serial.print(" ");
  Serial.print(state);
  Serial.print(" ");
  
  //Serial.print(countNum);
  //Serial.print(" ");
  //digitalWrite(testPin[pin].pin, HIGH);
  //calculate timing
  uint32_t curMillis = millis();
  int interOnsetInterval = curMillis - prevOnset;
  static int prev_interOnsetInterval = 0;
  prevOnset=curMillis;
  int jitter = interOnsetInterval - prev_interOnsetInterval;
  prev_interOnsetInterval = interOnsetInterval;
  Serial.println(jitter);
}


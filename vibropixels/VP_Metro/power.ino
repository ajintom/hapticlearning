#define POWER_SERIAL_DEBUG 0

/*
Measures battery power level
States:
4.2v-3.6v normal
3.6v-3.3v scale big motor amplitude, indicate low power
3.3v - no function, set LEDs red
3.1v - all off
*/

struct defBatt{
  uint16_t volt;
  uint16_t thresh[3];
  byte pin;
  byte state;
};
defBatt batt;

static uint32_t meanVcc;

void powerSetup(){
  batt.pin = 6;
  batt.thresh[0] = 360; //scale motor
  batt.thresh[1] = 347; //const red
  batt.thresh[2] = 344; //all off
  batt.volt = 370;
  
  if(POWER_SERIAL_DEBUG) Serial.begin(57600);
  if(POWER_SERIAL_DEBUG) Serial.print("Power debug enabled");
}

void powerLoop(){
  static uint32_t timer=0;
  uint16_t interval = 5000;
  
  if(millis() > timer+interval){
    timer=millis();
    
    readBattery();
    calcPowerState();
  /*
    if(batt.volt>360) processLed(0,100,0);
    else if (batt.volt>347) processLed(0,0,(batt.volt-330));
    else if (batt.volt>344) processLed((batt.volt-310+20),0,0);
    else processLed(1,1,1);
    */
 
  switch(batt.state){
    case 3: //normal operation
    break;
    
    case 2:
    break;
    
    case 1: //less than 3.3v
    processLed(10,0,0);
    delay(100);
    break;
    
    default:
    break;

  }
    static byte num_of_resets=1;
    if(POWER_SERIAL_DEBUG) Serial.print("Power State: ");
    if(POWER_SERIAL_DEBUG) Serial.println(powerState());
    if(POWER_SERIAL_DEBUG) Serial.print(rf_counter);
    if(POWER_SERIAL_DEBUG) Serial.print(", ");
    if(POWER_SERIAL_DEBUG) Serial.println(loop_counter);
    /*
    if(rf_counter==0) {
      if(ENABLE_SETUP_DEBUG) {
        Serial.print("SOFT RESET: ");
        Serial.println(num_of_resets);
        num_of_resets++;
      }
      reset_flag=1;
    }
    else reset_flag=0;
    */
    rf_counter=0;
    loop_counter=0;
      
  }

}

/*
 * readBattery
 * Ian Hattwick
 * July 4, 2016
 * converts ADC value to a voltage range expressed as an int, i.e. 3.3v = 330
 */
void readBattery() {
  uint32_t val = analogRead(batt.pin);
  if(POWER_SERIAL_DEBUG) Serial.print("raw: ");
  if(POWER_SERIAL_DEBUG) Serial.print(val);
  int vcc = 330;
  int adc_res = 1023;
  val = (val* vcc / adc_res);
  val = val + (val/2);
  
  batt.volt = ((((uint32_t)batt.volt*9) + val+9)/10);
  if(POWER_SERIAL_DEBUG) Serial.print(" scaled: ");
  if(POWER_SERIAL_DEBUG) Serial.print(val);
  if(POWER_SERIAL_DEBUG) Serial.print(" Battery Voltage: ");
  if(POWER_SERIAL_DEBUG) Serial.println(batt.volt);
}

/*
 * calcPowerState
 * Ian Hattwick
 * July 4, 2016
 * determines the current power state dependent on battery voltage
 */
void calcPowerState() {
  if(batt.volt>batt.thresh[0]) batt.state=3; //normal operation
  else if(batt.volt>batt.thresh[1]) batt.state=2;    //reduced motor, red flicker 
  else if(batt.volt>batt.thresh[2]) batt.state=1; //constant red
  else batt.state=0; //all off
}

/*
 * GETTERS AND SETTERS
 */

int getBattVolt(){
  return batt.volt;
}

int powerState(){
  return batt.state;
}

void powerOff(){
  
}


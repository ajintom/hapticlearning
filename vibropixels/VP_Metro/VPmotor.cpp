#include <Arduino.h>
#include "VPmotor.h"
#include <math.h>

#define EXP_FACTOR 1.025

//CONSTRUCTOR
VPmotor::VPmotor(uint8_t drivePin, uint8_t brakePin){
	_drive_pin=drivePin;
	_brake_pin=brakePin;
    pinMode(_drive_pin,OUTPUT);
    pinMode(_brake_pin,OUTPUT);
    analogWrite(_drive_pin,0);
    digitalWrite(_brake_pin,0);

    //Set default values
    _scalar=255;
    _brake=75;
    _debug=0;
    _ext_scalar=0;
    _ADSR_scalar=255;
    _ext_drive=0; 
    _expFlag = 0;

}

uint32_t _mtr_timer=0;
byte _mtr_interval = 2;

/*************PUBLIC METHODS*************

*******************************************/
void VPmotor::init(){
  setPwmFrequency();

 if(_debug) Serial.begin(57600);
 if(_debug) Serial.println("Motor Debug enabled");
 if(_debug) Serial.println("Oscillation Debug enabled");

} //actuatorSetup


/******************************************
trigEnvelope()
initiates envelope
*******************************************/

void VPmotor::trigEnvelope(){
  
  _state=1;
  _onset=millis();
  //_mtr_timer = millis() - _mtr_interval;
  
  if(_debug){//debug to see that values were set correctly
    Serial.print("Attack: ");
    Serial.println(_attack);
    Serial.print("envelope length: ");
    Serial.println(_envelopeLength);
    Serial.print("amp: ");
    Serial.println(_amp);
    Serial.print("decay: ");
    Serial.println(_decay);
  } 
} //end trigEnvelope

/******************************************
START OSCILLATION
*******************************************/

void VPmotor::trigOscillation(){
  if( ( _numOsc - _counter) > 0){
    switch(_oscState){
     case 1: //an oscillation in progress
      _state=1;
      _oscState=1;
      _counter=0;
      _oscOnset=millis();
      break;

      case 0: //no oscillation in progress
      _onset=millis();
      _state=1;
      _oscState=1;
      _counter=0;
      _oscOnset=millis();
      break;
    }

  if(_debug) {
        Serial.print("Start oscillation, number of oscillations: ");
        Serial.println(_numOsc);
      }
  }
  else { //a singeshot envelope happens if length of one envelope is greater than the length of the total oscillation
    _state=1;
    _onset=millis();
    _oscOnset = millis();
    _counter=0;
    if(_debug) Serial.println("Start single envelope");
  }
}//end start_oscillation

/******************************************
LOOP
generates PWM values for motor
*******************************************/

void VPmotor::loop()
{
  calcOscillation();
      
 if(_state==0 && _output!=0) {
  motorDrive();
 }
  if(_state > 0) motorDrive();
} //actuatorLoop

/****************************************************************************************************************
*SETTER AND GETTERS
*
****************************************************************************************************************/
uint8_t VPmotor::output(){
  return _output;
}

uint8_t VPmotor::state(){
  return _state;
}

void VPmotor::brake(uint8_t val){
  _brake=val;
}

void VPmotor::adsrScalar(uint8_t val){
  _ADSR_scalar=val;
}

void VPmotor::extScalar(uint8_t val){
  _ext_scalar=val;
}

void VPmotor::scalar(uint8_t val){
  _scalar=val;
}

void VPmotor::debug(uint8_t val){
  if(val==1) _debug=1;
  else _debug=0;
}

void VPmotor::setExpFlag(){
  _expFlag = !_expFlag;
}

void VPmotor::setBaseAmp(uint8_t val){
  _baseAmp = val;
}

void VPmotor::setEnvelope(uint8_t attackRatio, uint8_t envLenByteMultiple, uint8_t envLenByteRemainder, uint8_t amp, uint8_t decayRatio){
  
  _envelopeLength = (255*envLenByteMultiple)+envLenByteRemainder;
  if(_debug) Serial.print("envelope length: ");
  if(_debug) Serial.print(_envelopeLength);
  _attack = ((float) attackRatio/100)*_envelopeLength;
  if(_debug)Serial.print(", attack: ");
  if(_debug)Serial.print(_attack);
  _decay = ((float) decayRatio/100)*_envelopeLength; 
  if(_debug)Serial.print(", decay: ");
  if(_debug)Serial.print(_decay);
  _amp = amp;
  if(_debug) Serial.print(", amp: ");
  if(_debug) Serial.print(_amp);

}

void VPmotor::setOscillation(uint8_t frequency, uint8_t attack, uint8_t decay, uint8_t length){
  if(frequency<56) {
        _frequency = sqrt((float)frequency/55);
      }
      else _frequency = pow((uint32_t)frequency - 25 , 2)/800.; 
      _period = 1000/_frequency;
      
      _length = pow(length,2); //from 0 to 65 seconds
      if(_length > 30000) _length = 30000;
      if(_length >= 0){
         _numOsc = round((float) _length/_period)+1; //number of oscillations calc from length
      }
      else _state = 3;
      
      _oscAttack = pow(attack,2); 
      if(_oscAttack>(_length/2)) _oscAttack = (_length/2);
      
    _oscDecay = pow(decay,2);  
      if(_oscDecay>(_length/2)) _oscDecay = _length/2;
} //setOscEnv


/*************PRIVATE METHODS*************

*******************************************/
// Functions only available to other functions in this library

/******************************************
CALC OSCILLATION
*******************************************/

void VPmotor::calcOscillation()
{
  //Calculates when the oscillation should repeat
  uint32_t _cur_position = millis() - _onset;
  
  if(_counter >= _numOsc){
    //state = 0;
  } 
  else if(_cur_position >= _period) {
    trigEnvelope();
    _counter++;
    if(_debug) {Serial.print("oscillations left: "); Serial.println(_numOsc - _counter);}
  }  
} //end calcOscillations


/******************************************
//motorDrive
calculates value to drive motor
*******************************************/

void VPmotor::motorDrive()
{
  //if(millis() - _mtr_timer > _mtr_interval){
  if(1){
    _mtr_timer=millis();
    
    uint8_t ext_scaled=0;
    uint8_t ADSR_scaled=0;
    uint32_t motor_value=0;
    uint8_t out1,out2;
    
    if(_ext_scalar > 0) {
      ext_scaled = ((uint16_t)_ext_drive * _ext_scalar) >>8;
      _state=1;
    }
    
    uint8_t cur_ADSR = calcEnvelope();
    if(_ADSR_scalar > 0) ADSR_scaled = ((uint16_t)_ADSR_scalar * cur_ADSR) >>8;
    
    motor_value = ADSR_scaled + ext_scaled;
    if(motor_value > 255) motor_value=255;
    
    if(_numOsc>2) motor_value = calcOscEnvelope(motor_value);
    _output = motor_value;
    
    switch(_state){
      case 1:
      if(_scalar<255) analogWrite(_drive_pin,(motor_value * _scalar) >> 8 );
      else analogWrite(_drive_pin, motor_value);
    digitalWrite( _brake_pin, 0);
      break;
      
      case 2:
      analogWrite(_drive_pin,0);
      digitalWrite(_brake_pin, 1);
      break;
      
      case 3:
      analogWrite(_drive_pin,0);
      digitalWrite(_brake_pin, 0);
      _state=0;
      break;
    }
  }//timer
}//end motorDrive

/******************************************
CALC ENVELOPES
*******************************************/

uint8_t VPmotor::calcEnvelope()
{ 
  uint8_t MOTOR_ENVELOPE_DEBUG = 0;

  uint8_t env_output=0;
  if (_state > 0 ) {
    uint32_t _cur_position = millis() - _onset;
     
    //attack
    if (_cur_position <  _attack) {
      if(_expFlag){
         env_output = (((float)_cur_position/_attack) * (_amp-_baseAmp))+_baseAmp;
    }

    else{
        env_output = ((float)_cur_position/_attack) * _amp;//calc attack pwm
        
             }
      if(MOTOR_ENVELOPE_DEBUG)Serial.print("attack ");
    }
    
    //sustain
    else if (_cur_position < _envelopeLength - _decay) {//sustain
      env_output = _amp;
      if(MOTOR_ENVELOPE_DEBUG) Serial.print("sustain");
    }
    
    //decay
    else if (_cur_position < _envelopeLength ) { //decay phase
    float temp=0;
     if(_expFlag){
       env_output = _amp - ( ( (float)(_cur_position + _decay - _envelopeLength) /_decay) * (_amp-_baseAmp));
     }
     else{
      env_output = _amp - ( ( (float)(_cur_position + _decay - _envelopeLength) /_decay) * _amp);
     }
      if(MOTOR_ENVELOPE_DEBUG)Serial.print("decay ");
    }
    
    //brake
    else if (_cur_position < (_envelopeLength + _brake)) { //braking
     if(!_expFlag){ //No breaking when in exp metro mode
        if(MOTOR_ENVELOPE_DEBUG)Serial.print("brake ");
        env_output = 0;
        if (_state != 2)  _state=2;
      }
      else{
        env_output = _baseAmp;
      }
  
    }
    
    //disabled
    if (_cur_position > ( _envelopeLength + _brake)) // not active
    {
      if(!_expFlag){ //No breaking when in exp or sine metro mode
        if (_state != 3) {
         _state=3;
         env_output = 0;
        }
     }
      else{
        env_output = _baseAmp;
    }
    }
     if(MOTOR_ENVELOPE_DEBUG){
      Serial.print("Motor State: ");
      Serial.print(_state);
      Serial.print(" env_output: ");
      Serial.println(env_output);
      }
  }
  return env_output;
} //end calcEnvelopes

/******************************************
CALC OSCILLATION ENVELOPE
*******************************************/
uint8_t VPmotor::calcOscEnvelope( byte val){
    uint8_t OSC_ENVELOPE_DEBUG=0;
    uint32_t cur_position = millis() - _oscOnset;
    
    if (cur_position <  _oscAttack) {//attack
       if(_oscAttack == 0) val=val;
       else val = ((float)cur_position/_oscAttack) * val;//calc attack pwm
      if(OSC_ENVELOPE_DEBUG)Serial.print("Oscillator attack ");
      if(OSC_ENVELOPE_DEBUG) Serial.println((float)cur_position/_oscAttack);
    }
    
    else if (cur_position > _length - _oscDecay ) { //decay phase
      uint32_t time_remaining = 0;
      if(_length>cur_position) time_remaining = _length - cur_position;
      else time_remaining = 0;
      if(_oscDecay == 0) val=val;
      else val =  ( (float)time_remaining /_oscDecay) * val;
      if(OSC_ENVELOPE_DEBUG){
        Serial.print("motor ");
        Serial.print(cur_position);
        Serial.print(" time remainging:  ");
        Serial.print(time_remaining);
        Serial.print(" oscillator decay ");
        Serial.println((float)time_remaining /_oscDecay);
      }
    }
    //if(OSC_ENVELOPE_DEBUG)Serial.println(val);
    return (uint8_t) val;
} //end calcOscillationEnvelope

void VPmotor::setPwmFrequency(){
  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
 // TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
}

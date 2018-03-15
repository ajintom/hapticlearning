// ensure this library description is only included once

#ifndef VPmotor_h
#define VPmotor_h

#include <Arduino.h>

//define constants
//#define

class VPmotor
{
  public:
    void init();
    void trigEnvelope();
    void trigOscillation();
    void loop();
    //setters
    void setEnvelope(uint8_t attack, uint8_t envLenByteMultiple, uint8_t envLenByteRemainder, uint8_t amp, uint8_t decay);
    void setOscillation(uint8_t frequency, uint8_t attack, uint8_t decay, uint8_t length);
    void brake(uint8_t brake);
    void pwmFrequency();
    void actuatorScalar(uint8_t actuator_scalar);
    void drive(uint8_t external_drive);
    void adsrScalar(uint8_t val);
    void extScalar(uint8_t val);
    void scalar(uint8_t val);
    void debug(uint8_t val);
    void setExpFlag();
    void setBaseAmp(uint8_t val);
    //getters
    uint8_t output();
    uint8_t state();
    
    //constructor
    VPmotor(uint8_t drivePin, uint8_t brakePin);

/*
*
*/
  private:
    void setPwmFrequency();
    void calcOscillation();
    uint8_t calcEnvelope();
    uint8_t calcOscEnvelope(byte val);
    void motorDrive();
  
  uint8_t _drive_pin;
  uint8_t _brake_pin;
  uint8_t _scalar;
  //envelope  
  uint16_t _attack;
  uint16_t _envelopeLength;
  uint8_t _envLenByteMultiple; // multiple of 255 given by the envelope length
  uint8_t _envLenByteRemainder; //remainder of envelope length given by modulo with 255
  uint8_t _amp;
  uint16_t _decay;

  //oscillation envelope params
  uint16_t _oscAttack;
  uint16_t _oscDecay;
  uint32_t _oscOnset;
  uint32_t _envOnset;
  uint8_t _oscState;

  float _frequency; //frequency of oscillations
  uint16_t _period; //length of one single oscillations
  uint32_t _length; //overall duration of oscillations
  uint16_t _numOsc;
  uint16_t _counter; //number of oscillations since last oscillation msg
  uint16_t _prev_onset; 
  
  uint8_t _state;
  uint32_t _onset;
  uint8_t _PWM;
  uint8_t _output;
  
  //variable for scaling input sources
  //all values from 0-255, indicate amount of influence the input signal has on actuator output
  uint8_t _ext_scalar;
  uint8_t _ADSR_scalar;
  uint8_t _ext_drive; 

  uint8_t _brake; //duration of braking, default set to 75
  
  bool _debug;
  
  bool _expFlag;
  uint8_t _baseAmp;
  float lenSq;
    
};

#endif


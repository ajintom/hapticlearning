#define ACCEL_SERIAL_DEBUG 0 //for debugging in arduino serial monitor
#define ACCEL_STREAM_SERIAL_DATA 0  //for streaming data to max patch monitorAccelData

#include <Wire.h> 
#define SEND_ACCEL_RAW 0 //sends accel data to PC
#define ACCEL_DATA_RATE 2 //0=800Hz, 1=400Hz, 2=200 Hz, 3=100Hz, 4=50Hz, . . . 9=1.56Hz
/*
MMA8653 I2C accelerometer 10-bit
Actually we use MMA8652 I2C accelerometer 12-bit
ToDo:
-send data to PC for analysis
-create EEPROM functions
-accel outputs should be 12-bit ints to take advantage of resolution.
-should be scaled to 8-bit when sent to other devices
Enabling serial debugging will send accelerometer data over the serial bus for analysis using a simple SLIP encoding.
 */
 
/********************ACCELEROMETER DATA************/
// address of accelerometer
#define ACCEL_ADDRESS 0X1D // MMA8653FC and MMA8652FC
// address of registers for MMA8653FC
#define ACC_CTRL_REG1  0x2A
#define ACC_CTRL_REG4  0x2D
#define ACC_CTRL_REG5  0x2E
#define XYZ_DATA_CFG 0x0E

/********************PIN DEFINITIONS************/
#define ACC_INTPIN_1 14
#define ACC_INTPIN_2 15

/******ACCELEROMETER STRUCT**********/
typedef struct defAccel{
  // INPUTS
  byte enable;      // 0 = off; 1 = on;
  byte mode;        // 1 thru
  byte modeVar[8]; // Variables for the mode programs
  int  raw[3][16]; //raw input data
  byte index;
  int prevData[3]; //data from the previous pass of processing
  int prevData2[3];
  int prevTotal; //
  byte available;
  // OUTPUTS
  byte out[3];      // output data, scaled to 0-255
  byte total;       // "total" acceleration scaled to 0-255
  byte RMS;
};

static defAccel accel = {1,2};

/****************************************************************************************************************
*ACCEL SETUP
*
****************************************************************************************************************/
void accelSetup(){
 ACC_INIT();
 if(ACCEL_SERIAL_DEBUG || ACCEL_STREAM_SERIAL_DATA)Serial.begin(57600);
 accel.mode=6;
} //end setup

/****************************************************************************************************************
*ACCEL LOOP
*
****************************************************************************************************************/
void accelLoop(){
   if(accel.enable == 0)return;
   if(digitalRead(ACC_INTPIN_2)==0) { //pin 15 going LOW indicates we have new data - at data rate set by ACCEL_DATA_RATE.
     I2C_READ_ACCEL();
     calcAccelRMS(2);
     switch(accel.mode){
     case 1: leakyInt(); break;
     case 2: onePole(); break; // Works well with orientation (XYZ->RGB)
     case 3: rawOut();  break;
     case 4: calcVelocity(); break;
     case 5: variableLeakyInt(); break;
     case 6: calcOrientation(); break;
     }
   } //end of data input and processing
   
   static uint32_t timer=0;
   uint16_t interval=100;
   if(millis() > timer + interval){
     timer = millis();
     //if(ACCEL_SERIAL_DEBUG) SerialPrintAccelData();
     if(ACCEL_STREAM_SERIAL_DATA) outputSerialAccelDebug();
     if(ACCEL_STREAM_SERIAL_DATA) checkSerialAccelDebug();
   }
   
} // end Accel Loop

/****************************************************************************************************************
*ACCEL MODE FUNCTIONS
*
****************************************************************************************************************/
// Leaky Integrator
void leakyInt(){
  long velocity[3] = {0,0,0};
  uint8_t velocity_scale = accel.modeVar[0];
  uint16_t leak = accel.modeVar[1];
  uint8_t leak_interval=accel.modeVar[2];
  uint8_t output_scale=accel.modeVar[3];
  static uint32_t leak_timer=0;

  for(byte i=0;i<3;i++)if(accel.prevData[i] < 5000)  accel.prevData[i] += calcAccelPeak(i)/velocity_scale;
  
  if(millis()>leak_timer+leak_interval){
    leak_timer=millis();
    for(int i=0;i<3;i++) {
      if(accel.prevData[i]> leak) accel.prevData[i] -= leak;
      else accel.prevData[i]=0;
      uint16_t temp_out = accel.prevData[i]/output_scale;
      if(temp_out<255) accel.out[i]=temp_out;
      else accel.out[i]=255;
    }
  accel.total = constrain((uint16_t)accel.out[0]+accel.out[1]+accel.out[2],0,255);
  }
} //leakyInt()

// One-Pole FIR Filter
void onePole(){
  uint16_t feedback = accel.modeVar[1];
  uint8_t input = 255-feedback;
  
  for(int k=0;k<3;k++) accel.prevData[k] = (((long)calcAccelAvg(k) * input)/255) + (((long)accel.prevData[k] * feedback)/255);
  accel.prevTotal = sqrt( ( (pow((long)accel.prevData[0],2)) + (pow((long)accel.prevData[1],2)) + (pow((long)accel.prevData[2],2)) ) / 16 );
  
  //calc output value
  for(int i=0;i<3;i++) {
    accel.out[i]= uint16_t(accel.prevData[i] + 2048) >> 4;
}
 // Serial.println(" ");
  accel.total = uint16_t(accel.prevTotal + 2048) >> 4;
} //onePole

// XYZ Accel and Magnitude
void rawOut(){
  for(byte i=0;i<3;i++){
    accel.out[i] = (calcAccelAvg(i)+2048)>>4;
  }
  accel.total = sqrt( (((uint32_t)accel.out[0]*accel.out[0]) + ((uint32_t)accel.out[1]*accel.out[1]) + ((uint32_t)accel.out[2]*accel.out[2])) >> 4);

//bufferAccelData(); //sends data to PC. . . in progress
}

void calcVelocity(){
  uint8_t input_gain = accel.modeVar[0];
  uint8_t feedback = accel.modeVar[1];
  uint8_t output_gain = accel.modeVar[2];
  uint8_t total_output_gain = accel.modeVar[3];
  uint8_t input = 255-feedback;
  int velocity[3] = {0,0,0};
  int cur_acc[3];
  for(byte i=0;i<3;i++) {
    cur_acc[i]=calcAccelAvg(i);
    velocity[i] = abs(accel.prevData[i] - cur_acc[i]);
    velocity[i] = constrain( velocity[i] * input_gain, 0 , 255);
    accel.prevData[i]=cur_acc[i];
    accel.prevData2[i] = (((long)velocity[i] * input)/255) + (((long)accel.prevData2[i] * feedback)/255);
  }
  accel.prevTotal = sqrt( ( (pow((long)accel.prevData2[0],2)) + (pow((long)accel.prevData2[1],2)) + (pow((long)accel.prevData2[2],2)) ) / 16 );
  
  //calc output value
  for(int i=0;i<3;i++) accel.out[i]= constrain ( (uint32_t)accel.prevData2[i] * output_gain , 0,255);
  accel.total = constrain ( (uint32_t)accel.prevTotal * total_output_gain , 0,255);
} //calcVelocity

//LEAKY INTEGRATOR W/VARIABLE LEAK
// Leaky Integrator
void variableLeakyInt(){

  uint16_t leak = accel.modeVar[1]*1;
  uint8_t velocity_scale = accel.modeVar[0];
  uint8_t leak_interval=accel.modeVar[2];
  uint8_t output_scale=accel.modeVar[3];
  
  static uint32_t leak_timer=0;

  for(byte i=0;i<3;i++)if(accel.prevData[i] < 20000)  accel.prevData[i] += calcAccelPeak(i)/velocity_scale;
  
  if(millis()>leak_timer+leak_interval){
    leak_timer=millis();
    for(int i=0;i<3;i++) {
      uint16_t thisLeak = ((uint32_t)accel.prevData[i]*leak)>>8;
      if(accel.prevData[i]> thisLeak) accel.prevData[i] -= thisLeak;
      else accel.prevData[i]=0;
      
      uint16_t temp_out = accel.prevData[i]/output_scale;
      if(temp_out<255) accel.out[i]=temp_out;
      else accel.out[i]=255;
    }
  accel.total = constrain(((uint16_t)accel.out[0]+accel.out[1]+accel.out[2])/2,0,255);
  }
}//variableLeakyInt()


//Orientation filter
void calcOrientation(){
  uint16_t feedback = accel.modeVar[0];
  //feedback = 0;
  uint8_t input = 255-feedback;
  uint8_t top_thresh = accel.modeVar[1];
  uint8_t bottom_thresh = accel.modeVar[2];
  // top_thresh = 32;
  // bottom_thresh = 16;
  
  for(int k=0;k<3;k++){
  accel.prevData[k] = (((long)calcAccelAvg(k) * input)/255) + (((long)accel.prevData[k] * feedback)/255);
  }
  accel.prevTotal = sqrt( ( (pow((long)accel.prevData[0],2)) + (pow((long)accel.prevData[1],2)) + (pow((long)accel.prevData[2],2)) ) / 16 );
  
  //calc output value
  for(int i=0;i<3;i++) {
    int scale_out= accel.prevData[i]/4;
    //int scale_out2= (scale_out * (top_thresh + bottom_thresh));
   if      ( scale_out > 0 + top_thresh) accel.out[i]=255;
   else if (scale_out < 0 - top_thresh ) accel.out[i] = 0;
   else if (scale_out > 0 + bottom_thresh ) accel.out[i] = 127 + bottom_thresh;
   else if (scale_out < 0 - bottom_thresh ) accel.out[i] = 127 - bottom_thresh;
   else accel.out[i]=127;
    
//  Serial.print(scale_out);
//  Serial.print(" ");
//  Serial.print(accel.out[i]);
//  Serial.println(" ");
  }
  //Serial.println(" ");
  accel.total = uint16_t(accel.prevTotal + 2048) >> 4;
  //delay(250);
} //calcOrientation

//CALC AVG
int calcAccelAvg(byte axis){
  long average=0;
  for(byte i=0;i<16;i++) average += accel.raw[axis][i];
  average = average/16;
  return average;
}

//CALC PEAK
uint16_t calcAccelPeak(byte axis){
  uint16_t peak = abs(accel.raw[axis][0]-accel.raw[axis][15]);
  for(int i=1;i<16;i++) {
    uint16_t temp = abs(accel.raw[axis][i] - accel.raw[axis][i-1]);
    if(temp> peak) peak = temp;
  }
  return peak;
}

//CALC RMS
uint16_t calcAccelRMS(byte axis){
  float IIR = 0.01;
  long val = pow((int) accel.out[axis]-127,2);
  static long prevVal = 0;
  val = (float)val * IIR +  (float)prevVal* (1.-IIR);
  prevVal = val;

  val = sqrt(val);
  accel.RMS = val;
}

//TODO
//int calcAccelMedian(byte axis, byte range){
//  //set range to safe value
//  if(range<3) range = 3; 
//  else if (range>15) range=15;
//  if(range%2 == 0) range--; 
//  
//  int sorted_list[range]; //variable to store sorted list of size range
//  byte deviation = (range/2)-1; //the number of values on each side of the cur index to sort
//  
//  byte begin_index = (accel.index+deviation) % 16; //location of first value to evaluate
//  byte cur_index = (begin_index+15) % 16; 
//  byte prev_index=begin_index;
//  byte  end_index = ((accel.index + 14) - deviation) % 16; //location of second value to evaluate
//  
//  sorted_list[0]=accel.raw[axis][begin_index]; //store first value
//  while (cur_index > end_index){
//    int swap;
//    if(accel.raw[axis][curIndex]<sorted_list[prev_index]
//    
//  }
//}

/****************************************************************************************************************
*ACCEL INIT
****************************************************************************************************************/
void ACC_INIT()
{
  Wire.begin(); // start of the i2c protocol
  I2C_SEND(ACC_CTRL_REG1 ,0X00); // set to standby mode to be able to configure
  I2C_SEND(XYZ_DATA_CFG ,B00000000); // B00000000: 2G, B00000001:4G, B00000010: 8G
  I2C_SEND(ACC_CTRL_REG4 , B00000001); // Enables data ready interrupt
  I2C_SEND(ACC_CTRL_REG5 , B00000000); // Routes interrupt to interupt pin 2 (ACCEL_INTPIN_2)
  
  //enable accelerometer and set output data rate
  const int accelDataRateHertz[] = {800,400,200,100,50,25,12,6,1};
  
  //if(accel.enable==1)  I2C_SEND(ACC_CTRL_REG1 ,(ACCEL_DATA_RATE<<3)+1);
  I2C_SEND(ACC_CTRL_REG1 ,(ACCEL_DATA_RATE<<3)+1);
  // Output data rate: B00000001:800Hz, B00001001:400Hz, B00010001:200Hz, B00011001: 100Hz
  // auto wake B0:B1, fast read mode B6, active B7
  if(ACCEL_SERIAL_DEBUG){
    Serial.print("Accelerometer data rate is ");
    Serial.print(accelDataRateHertz[ACCEL_DATA_RATE],DEC);
    Serial.println(" Hertz.");
  }

}

/****************************************************************************************************************
*READ ACCEL
****************************************************************************************************************/
void I2C_READ_ACCEL() 
{
  byte inc_data[7];

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x00);  
  Wire.requestFrom(ACCEL_ADDRESS,7); 
  
  for(int i=0; i<7; i++)inc_data[i]=Wire.read(); 
  for (int i=1;i<7;i=i+2){
    int accel_temp = ( inc_data[i+1]  |  ((int)inc_data[i]<<8)) >> 4;
    if (accel_temp>0x1FFF) accel_temp = (( (~accel_temp)+1) - 0xFC00 ); //calc two's complement
    
    switch(i){
    case 1: accel.raw[0][accel.index]=accel_temp; break;
    case 3: accel.raw[1][accel.index]=accel_temp; break;
    case 5: accel.raw[2][accel.index]=accel_temp; break;
      }
   }
   accel.index=(accel.index+1)%16;
}

/****************************************************************************************************************
*I2C FUNCTIONS
****************************************************************************************************************/
byte I2C_READ_REG(int ctrlreg_address) 
{
  byte input=123; 
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(ctrlreg_address); 
  Wire.endTransmission();
  
  if(Wire.requestFrom(ACCEL_ADDRESS,1)){
    input = Wire.read();
    //Serial.print(ctrlreg_address, HEX);
    //Serial.print(" read successfully, value: ");
    //Serial.println(input,BIN);
  }
   return input;
} //I2C_READ_REG

void I2C_SEND(unsigned char REG_ADDRESS, unsigned  char DATA) 
{
  byte stat=255;
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(REG_ADDRESS);
  Wire.write(DATA);
  Wire.endTransmission ();
}//I2C_SEND


/****************************************************************************************************************
*SETTER AND GETTERS
*
****************************************************************************************************************/
void setAccelEnable(byte val){
  accel.enable=val;
}
void setAccelMode(byte val){
  accel.mode=val;
}
void setAccelModeVar(byte val, byte index){
  accel.modeVar[index]=val;
}

byte getAccelTotal(){
  return accel.total;
}
//getAccelValue 0=total, 1=X 2=Y 3=Z
byte getAccelValue(byte num){
  if( num == 0 ) return accel.total;
  else if ( (num>0) && (num<4) )return accel.out[num-1];
}

int getAccelParameters(byte num){
   if(num==0) return accel.mode;
   else if(num<9) return accel.modeVar[num-1];
   else return 256;
   return 256;
}

int setAccelParameters(byte num, byte val){
  int write_status=256;
   if(num==0)  { accel.mode=val; write_status=0;}
   else if(num<9) { accel.modeVar[num-1]=val; write_status=0; }
   else write_status=256;
   return write_status;
}

/****************************************************************
slipEncodeData()

, and then SLIP encodes it and
sends it out the serial port
****************************************************************/

void outputSerialAccelDebug(){
  for(byte i=0;i<3;i++)slipOutAccel(accel.out[i]);
  //slipOutAccel(accel.total);
  slipOutAccel(accel.RMS);
  Serial.write(175); //delimByte
}

void slipOutAccel(int data){
  byte delimByte=175;
  byte escByte=236;
  
  if(data==delimByte){
    Serial.write(escByte);
    Serial.write(delimByte);
  } else if(data==escByte){
    Serial.write(escByte);
    Serial.write(escByte);
  } else Serial.write(data);
}


//


//
void checkSerialAccelDebug(){
  byte delimByte=175;
  byte escByte=236;
  
  static byte inputDataIndex=0;
  byte inputData[16];
  
  while(Serial.available()){
     byte temp = Serial.read();
    
    //check for escape message.
    //if we find one the next byte is stored in our inputData array.
    if( temp == escByte ) {
      inputData[ inputDataIndex ] = Serial.read();
      inputDataIndex++;
    }
    //check for the end message
    else if (temp == delimByte ) {
      processSerialAccelData( inputData[0], inputData[1], inputData[2], inputData[3], inputData[4], inputData[5],inputData[6], inputData[7], inputData[8]  ); //process data however you want
      inputDataIndex=0; //get ready for the next incoming packet
    }
    //store data in inputData array
    else {
      inputData[ inputDataIndex ] = temp;
      inputDataIndex++;
    }
  }
}

void processSerialAccelData(byte mode, byte var1, byte var2, byte var3, byte var4, byte var5, byte var6, byte var7, byte var8 ){
  processLed(0,0,0);
  accel.mode=mode;
  accel.modeVar[0]=var1;
  accel.modeVar[1]=var2;
  accel.modeVar[2]=var3;
  accel.modeVar[3]=var4;
  accel.modeVar[4]=var5;
  accel.modeVar[5]=var6;
  accel.modeVar[6]=var7;
  accel.modeVar[7]=var8;
}

void SerialPrintAccelData(){
  for(byte i=0;i<3;i++){
  //Serial.print(accel.raw[i][0]);
  //Serial.print("\t");
  }
  //Serial.println();
}


/*
This code contains all of the serial sending functions
the most important are:

prepStreamingData() - prepares data for streaming.
outputSerialData() - actually sends the data. 

data sent format:
begin msg / device ID / packet number / message type + data packets / packet size /   packet timer  / end msg
*/
  static byte serialOutputBuffer[64]; //buffer for serial messages
  static byte dataToSend[64];
  static byte dataIndex=0;
  byte buffer_index=0;
uint32_t streamTimer=0;
uint16_t streamInterval=10;
  
void processOutput(){
  static uint32_t timer=0;
  uint32_t cur_millis = millis();
  
  if(cur_millis > timer + poll_interval){
    timer=cur_millis;
    prepStreamingData();
    //if(buffer_index>0)outputSerialData(); //or put in prepStreamingData
  }
}

void prepStreamingData()
{
    //write functions here to buffer data
  if( accel.enable == 1) bufferAccelData(); //example
//  ackFlag=0; //no acknowledgement needed
  if( buffer_index > 0 ) outputSerialData(); //this function trasmits buffered data to wireless device after all data has been buffered
  /*
  Polling means the device will send data continuously at the <pollInterval> ms rate. 
  If a message from the host device is not received after a delay of <pollEnableInterval> ms then polling is disabled.
  This protects the host device from a flood of serial data when the host is not ready to receive it.
  Sometimes flooding the host device will cause your PC to freeze. . . 
  */
    if(millis()>(pollEnableTimer+pollEnableInterval)){
      //bufferSensors();
      pollEnable=0;
    }
}

/*
An example of buffering data
*/

void bufferAccelData(){
  for(int i=0;i<3;i++){
  //bufferInt (constrain(accel.magnitude,0,60000));
//  Serial.print(accel.raw[i]);
//  Serial.print("\t");

  }
//    Serial.println();
  
//  Serial.print("shifted val 1: ");
//  Serial.print(accel.raw[1]>>8,DEC);
//  Serial.print(", ");
//  Serial.print((accel.raw[1]>>8)&0xFF,BIN);
//  Serial.print(", ");
//  Serial.print(accel.raw[1]&0xFF,DEC);
//  Serial.print(", ");
//  Serial.println((char)accel.raw[1]&0xFF,BIN);
//  Serial.print(" processed accel value: ");
//  Serial.print(accel.out[0]);
//  Serial.print(", ");
//  Serial.print(accel.out[0]);
//  Serial.print(", ");
//  Serial.print(accel.out[0]);
//  Serial.print(", ");
//  Serial.print(accel.magnitude);
//  Serial.print(", ");
//  Serial.print(accel.modeVar[0]);
//  Serial.print(", ");
//  Serial.println(accel.modeVar[1]);
}



void sendErrorMessage(byte error){
  ackFlag=0;
  bufferByte(errorMsg);
  bufferByte(error);
 // outputSerialData();
}

void sendAckMessage(byte packetNumber){
  bufferByte(acknowledgement);
  bufferByte(packetNumber);
  ackFlag=0;
  //outputSerialData();
}



/*
functions for writing data to the serial output buffer <serialOutputBuffer[]>
*/
void bufferByte(byte input)
{
  serialOutputBuffer[buffer_index]=input;
  buffer_index+=1;
}

void bufferInt(int output)
{
  serialOutputBuffer[buffer_index]=((byte)(output >> 8));
  buffer_index=buffer_index+1;
  serialOutputBuffer[buffer_index]=((byte)(output & 0xFF));
  buffer_index=buffer_index+1;
}



/***************************************************************************
outputSerialData is the function that actually transmits the serial data.

#TODO device ID, msgType and pktNumber are not SLIP encoded
****************************************************************************/

/*
 * for RHDATAGRAM library send functions data should be configured in a buffer and sent as an array.
 * For this reason the all output of serial data is sent to a function to store wrap it correctly.
 */
void outputSerialData()
{
  uint16_t pktTime=0; //timestamp - unimplemented
  
  if(0) Serial.print("outputSerialData: ");
  packageOutput(beginMsg); //SLIP start message
  
  slipOut(deviceID); //device ID
  byte byte2 = (ackFlag<<7) | pktNumber; //2nd byte - acknowledge flag and number
  pktNumber=(pktNumber+1)%128;
  slipOut(byte2);

  for(int i=0;i<buffer_index;i++) {      //message type + data. . . 
    slipOut(serialOutputBuffer[i]);
  }

  slipOut(buffer_index);
  buffer_index=0; //reset buffer to have 0 elements

  //set time elapsed since last packet was sent. Seems redundant here. . . .
  pktTime=  (millis()-streamTimer);
  slipOut(pktTime);
  streamTimer=millis();

  packageOutput(endMsg); //SLIP end message

  if(0) Serial.print("Bytes sent: ");
  if(0) Serial.println(dataIndex);
  manager.sendto((uint8_t *)dataToSend,dataIndex,SERVER_ADDRESS);
  dataIndex=0;
}

/*
escapes special characters
*/
void slipOut(byte input){
    //check to see if input is a special character
    for(int i=0;i<(sizeof(specialChar));i++) {
      if(input==specialChar[i]) { //if it is, escape it
        packageOutput(escByte);
      }
    }
   packageOutput(input); //write data byte
}

void packageOutput(byte val){ 
   dataToSend[dataIndex] = val;
   if(0) Serial.print(val);
   if(0) Serial.print(" ");
   dataIndex++;
}


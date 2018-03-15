#define NRF_RECEIVE_DEBUG 0
#define SIMPLE_NRF_RECEIVE_DEBUG 0

#include <RHDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

#define BROADCAST_MESSAGE 15
#define RAW_BUFFER_SIZE 128

#define PACKET_BEGIN_BYTE 164
#define PACKET_END_BYTE 192
#define PACKET_ESC_BYTE 219
#define PACKET_TRANS_BEGIN_BYTE 65
#define PACKET_TRANS_END_BYTE 12
#define PACKET_TRANS_ESC_BYTE 37

#define MESSAGE_END_BYTE 175
#define MESSAGE_ESC_BYTE 236
#define TRANS_MESSAGE_END_BYTE 72
#define TRANS_MESSAGE_ESC_BYTE 245

const byte NUM_DEVICE_IDS = 9; //the number of available device IDs
//number of bits available for supergroups = 16-NUM_DEVICE_IDS
const byte NUM_SUPERGROUPS = 16 - NUM_DEVICE_IDS;

byte ackFlag=0; //0:no acknowledgement needed, 1:ask for acknowledgement
byte pktNumber=0; //0-127 sequence number of packet

byte countPacket = 0; //counts the number of incoming packets

uint32_t lastMsg = 0;
uint32_t inMsg = 0;
uint32_t timeSinceLastMsg = 0;
unsigned int msgCount = 0;
bool latencyTest = false;


/*
 * NRF24 radio transceiver
 * 
 * transmitter address:
 * 
 */

///////////////////////////////////NRF24///////////////////////////////////
//setup
#define CLIENT_ADDRESS RH_BROADCAST_ADDRESS
#define SERVER_ADDRESS 123 //using broadcast addresses instead of client/server IDs

RH_NRF24 driver; // Singleton instance of the radio driver
RHDatagram manager(driver, CLIENT_ADDRESS); // Class to manage message delivery and receipt, using the driver declared above

void RFSetup(){
  if(NRF_RECEIVE_DEBUG) Serial.begin(57600);
  // if (RxTxMode == 0) {pinLed=5;} else {pinLed=3;} // if tx red if rx green
  // pinMode(pinLed, OUTPUT);  
  byte init_status = manager.init(); //initiate message manager
 if(NRF_RECEIVE_DEBUG){
  Serial.print("Radio initialized: ");
  Serial.println(init_status);
 }
 if(latencyTest){
  Serial.begin(57600);
 }
}

/*
This code does the following:
1) Removes SLIP encoding from incoming serial stream
2) Places de-encoded serial data in a buffer
3) Processes data according to the data protocol, does rudimentary error checking, sends acknowledgement and error messages
4) Processes the incoming messages

Serial input data format:
 device ID / packet number / #bytes /timing / data 
 */
 
// <static> variables declared outside a function are limited in scope to this file
 byte serialInputBuffer[64];

/************************************************************************************************************************************************************/
/*
 * This functions checks the radio for available input
 * then removes the top layer of SLIP encoding and assembles the serial data into packets
 * then calls checkPktForErrors, and if no error calls processMessages
 */
void checkNRF24()
{
  while (manager.available()) { //has the radio received any data

    //static variables declared in a function are function-scope, but persistent. 
    static int inputIndex=0;
    byte rawInput[RAW_BUFFER_SIZE]={0}; //holds the raw data from radio
    byte rawInputLength=sizeof(rawInput);

    //this function takes data stored in the radio and copies it to rawInput
    byte goodMessage = manager.recvfrom(rawInput, &rawInputLength);
    
    if(NRF_RECEIVE_DEBUG){
      Serial.print("raw: ");
      byte i=0;
      while (rawInput[i]!= PACKET_END_BYTE) {  //limit should be incoming length
        Serial.print(rawInput[i]);
        Serial.print(", ");
        i++;
      }
      Serial.println();
    }

    //this removes SLIP encoding
    for(int i=0;i<RAW_BUFFER_SIZE;i++){  //limit should be incoming length
      switch(rawInput[i]){
        case PACKET_BEGIN_BYTE:
        inputIndex=0;
        break;
        
        case PACKET_ESC_BYTE:
        switch(rawInput[i+1]){
          case PACKET_TRANS_BEGIN_BYTE: serialInputBuffer[inputIndex] = PACKET_BEGIN_BYTE; break;
          case PACKET_TRANS_END_BYTE: serialInputBuffer[inputIndex] = PACKET_END_BYTE; break;
          case PACKET_TRANS_ESC_BYTE: serialInputBuffer[inputIndex] = PACKET_ESC_BYTE; break;
        }
        inputIndex++;
        i++;
        break;
        
        case PACKET_END_BYTE:
        { 
          digitalWrite(1,1); //toggles tx pin at successful reception of ctrl msg
          countPacket++;
          i=RAW_BUFFER_SIZE;
          // AN> This part implements the new header of messages including two bytes for deviceID
          // Still motors are identified from 0 - 11 (integers) but packet comes with two bytes addressing
          // each motor as a bit so ...0001 is motor 0, ...0010 is motor 1, ...0100 is motor 2 and so on. 
          // 1 = motor ON, 0 = motor OFF.
           
          //combine device ID bytes into one variable
          uint16_t inc_device_id = (serialInputBuffer[0]<<8) + serialInputBuffer[1];
          //calc supergroup
          uint16_t inc_supergroup = serialInputBuffer[0]>>1;
          if(NRF_RECEIVE_DEBUG) {Serial.print("Inc 0: "); Serial.println(serialInputBuffer[0]);}
          if(NRF_RECEIVE_DEBUG) {Serial.print("Inc 1: "); Serial.println(serialInputBuffer[1]);}
          
          //mask out the supergroup bits
          if(NRF_RECEIVE_DEBUG) {Serial.print("Inc Supergroup: "); Serial.println(inc_supergroup);}
          uint16_t device_id_mask= 0;
          for(byte k= 0 ; k< NUM_DEVICE_IDS; k++) device_id_mask += (1<<k) ;
          inc_device_id &= device_id_mask; //final incoming device_id
          if(NRF_RECEIVE_DEBUG) {Serial.print("Inc DeviceID: "); Serial.println(inc_device_id, BIN);}
          
          //if((inc_supergroup == 11)) { //manual set
          //if(( inc_supergroup == 1 || inc_supergroup == 0 )) { //manual set
          
          if(( inc_supergroup == supergroup || inc_supergroup == 0 )) {
            if(NRF_RECEIVE_DEBUG) {Serial.print("Supergroup OK "); Serial.println(inc_supergroup);}
            if( (inc_device_id >> (deviceID-1)) & 1 ) {
              
            //if( (inc_device_id >> (1-1)) & 1 ) { //manual set
          
               byte receiveError=checkPktForErrors(inputIndex);
               if(receiveError==0) {
                processSerialInput(inputIndex);
               }
               else {}
               if(NRF_RECEIVE_DEBUG) {Serial.print("Message received: "); Serial.println(serialInputBuffer[5]);}
               inputIndex=0;
            } else {}//if deviceID 
          } else {}//if supergroup
        }//if end byte
        break;
    
      default:
        serialInputBuffer[inputIndex]=rawInput[i];
        inputIndex++;
        break;
      }
      if(inputIndex>63)inputIndex=0;
    }
  }
  digitalWrite(1,0);
}

/************************************************************************************************************************************************************/
/*
processSerialInput
Processes incoming serial packets according to their message type
Sept 23 updated for use with VPmotor library
*/
void processSerialInput(byte msgLength)
{
  byte msg_data_byte[16]; //array for storing data bytes in switch statement

  byte messageType = 255; //255 indicates start of message, reset at end of this function
  byte messageIndex=0;
  byte current_data=0;
  byte end_message_flag=0;
  
  //check each byte of data packet individually
  for (int i=5;i<msgLength;i++) { 
    
    /*
    REMOVE SLIP ENCODING
    */
    if (serialInputBuffer[i]==MESSAGE_ESC_BYTE){ //message esc byte
      switch (serialInputBuffer[i+1]){
        case TRANS_MESSAGE_ESC_BYTE: current_data=MESSAGE_ESC_BYTE; break;
        case TRANS_MESSAGE_END_BYTE: current_data=MESSAGE_END_BYTE; break;
        default: current_data = serialInputBuffer[i+1];
      }
      i++;
    } 
    else if (serialInputBuffer[i]==MESSAGE_END_BYTE) end_message_flag=1; //end of message
    else current_data = serialInputBuffer[i];
    /*
    ENDREMOVE SLIP ENCODING
    */
 
    if(messageType==255){ //beginning of new message
      messageType=current_data;
      messageIndex=0;
      if(NRF_RECEIVE_DEBUG) {Serial.print("Message Type: "); Serial.println(messageType);}
    } 
    
    //the main switch is contained within this else:
    else{
     if(NRF_RECEIVE_DEBUG) {
      if (end_message_flag==1) Serial.println("end");
      else{
        Serial.print("Current data: ");
        Serial.println(current_data);
        }
      }
//begin switch
//
//
// 

    switch (messageType) { //serialInputBuffer[i] is the message type

      /*
       * MOTOR COMMAND MESSAGE 0-19
       */
      case 0: //set and trigger a single envelope for mtr1
      if(end_message_flag == 1) { 
        motor1.setEnvelope(msg_data_byte[0],msg_data_byte[1],msg_data_byte[2],msg_data_byte[3],msg_data_byte[4]);
        motor1.trigEnvelope();
        break;
        }
      else if (messageIndex>3) {break;}
      else msg_data_byte[messageIndex] = current_data; //square time values
      break;

      case 1: //set and trigger a single envelope for mtr2
      if(end_message_flag == 1) { 
        motor2.setEnvelope(msg_data_byte[0],msg_data_byte[1],msg_data_byte[2],msg_data_byte[3],msg_data_byte[4]);
        motor2.trigEnvelope();
        break;
        }
      else if (messageIndex>3) {break;}
      else msg_data_byte[messageIndex] = current_data; //square time values
      break;

      case 2: //set and trigger a single envelope for both motors
      if(end_message_flag == 1) { 
        motor1.setEnvelope(msg_data_byte[0],msg_data_byte[1],msg_data_byte[2],msg_data_byte[3],msg_data_byte[4]);
        motor2.setEnvelope(msg_data_byte[0],msg_data_byte[1],msg_data_byte[2],msg_data_byte[3],msg_data_byte[4]);
        motor1.trigEnvelope();
        motor2.trigEnvelope();
        break;
        }
      else if (messageIndex>3) {break;}
      else msg_data_byte[messageIndex] = current_data; //square time values
      break;
    
      case 14: //trig oscillation for mtr1, small motor
      if(end_message_flag == 1) { 
         motor1.setEnvelope(msg_data_byte[4],msg_data_byte[2],msg_data_byte[3], msg_data_byte[1],msg_data_byte[5]); //attack sustain amp decay
         motor1.setOscillation(msg_data_byte[0],msg_data_byte[7],msg_data_byte[8],msg_data_byte[6] ); //frequency attack decay length
         setLedColor2(msg_data_byte[10],0);
         setLedColor2(msg_data_byte[11],1);
         setLedColor2(msg_data_byte[12],2);//rgb
         setLedParameters(8,msg_data_byte[9]); 
         motor1.setBaseAmp(msg_data_byte[13]);
         if(latencyTest){
          inMsg = millis();
          timeSinceLastMsg = inMsg - lastMsg;
          lastMsg = inMsg;
          msgCount++;
          Serial.print(msgCount);
          Serial.print(" , ");
          Serial.println(timeSinceLastMsg);
         }
         motor1.trigOscillation();
        break;
        }
      else if (messageIndex>13) {break;}
      else msg_data_byte[messageIndex] = current_data; //square time values
      break;

      case 15: //trig oscillation for mtr2, big motor
      if(end_message_flag == 1) { 
         motor2.setEnvelope(msg_data_byte[4],msg_data_byte[2],msg_data_byte[3], msg_data_byte[1],msg_data_byte[5]); //attack sustain amp decay
         motor2.setOscillation(msg_data_byte[0],msg_data_byte[7],msg_data_byte[8],msg_data_byte[6] ); //frequency attack decay length
         setLedColor2(msg_data_byte[10],0);
         setLedColor2(msg_data_byte[11],1);
         setLedColor2(msg_data_byte[12],2);//rgb
         setLedParameters(8,msg_data_byte[9]); 
         if(latencyTest){
          inMsg = millis();
          timeSinceLastMsg = inMsg - lastMsg;
          lastMsg = inMsg;
          msgCount++;
          Serial.print(msgCount);
          Serial.print(" , ");
          Serial.println(timeSinceLastMsg);
         }
         motor2.trigOscillation();
        break;
        }
      else if (messageIndex>12) {break;}
      else msg_data_byte[messageIndex] = current_data; //square time values
      break;

      case 16: //trig oscillation for both motors
      if(end_message_flag == 1) { 
         motor1.setEnvelope(msg_data_byte[4],msg_data_byte[2],msg_data_byte[3], msg_data_byte[1],msg_data_byte[5]); //attack sustain amp decay
         motor1.setOscillation(msg_data_byte[0],msg_data_byte[7],msg_data_byte[8],msg_data_byte[6] ); //frequency attack decay length
         motor2.setEnvelope(msg_data_byte[4],msg_data_byte[2],msg_data_byte[3], msg_data_byte[1],msg_data_byte[5]); //attack sustain amp decay
         motor2.setOscillation(msg_data_byte[0],msg_data_byte[7],msg_data_byte[8],msg_data_byte[6] ); //frequency attack decay length
         setLedColor2(msg_data_byte[10],0);
         setLedColor2(msg_data_byte[11],1);
         setLedColor2(msg_data_byte[12],2);//rgb
         setLedParameters(8,msg_data_byte[9]); 
         motor1.setBaseAmp(msg_data_byte[13]);
         motor1.trigOscillation();
         motor2.trigOscillation();
        break;
        }
      else if (messageIndex>13) {break;}
      else msg_data_byte[messageIndex] = current_data; //square time values
      break;

      case 17: //trig metronome for one motor using exp model
        motor1.setExpFlag();
        if(latencyTest){
          if(msgCount > 0){
            inMsg = millis();
            timeSinceLastMsg = inMsg - lastMsg;
            lastMsg = inMsg;
            msgCount++;
            Serial.print(msgCount);
            Serial.print(" , ");
            Serial.println(timeSinceLastMsg);
            msgCount = 0;
          }
          else{
            lastMsg = millis();
            //Serial.println("0, 0");
          }
          
        }
        break;
      
      /*
      *LED COMMAND MESSAGES 21-39
      */
      case 20: //LED envelope message      
      if(end_message_flag) { triggerLedEnvelope(); break;}
      if (messageIndex>3) break;
      setLedEnvelope(current_data,messageIndex); 
      break;

      case 21: //LED color1
      if(end_message_flag) {newDataFlag(); break;}
      if (messageIndex>2) break; //test to make sure incoming index is not out of range
      else setLedColor1(current_data,messageIndex);
      break;

      case 22: //LED color2
      if(end_message_flag) {triggerLedEnvelope();break;}
      if (messageIndex>2) break; //test to make sure incoming index is not out of range
      else setLedColor2(current_data,messageIndex);
      newDataFlag();
      break;

      case 23:
      //LED static color message
      break;

      case 24:
      if(end_message_flag)  break;
      if (messageIndex>4) break;
      else setLedScalars(current_data,messageIndex);
      newDataFlag();
      break;

      //Metronome cases with fewer bytes transmitted
      case 30: //trig oscillation for mtr1, small motor
      if(end_message_flag == 1) { 
         motor1.setEnvelope(msg_data_byte[4],msg_data_byte[2],msg_data_byte[3], msg_data_byte[1],msg_data_byte[5]); //attack sustain amp decay
         motor1.setOscillation(msg_data_byte[0], 0, 0, 0 ); //frequency attack decay length
         motor1.setBaseAmp(msg_data_byte[6]);
         if(latencyTest){
          inMsg = millis();
          timeSinceLastMsg = inMsg - lastMsg;
          lastMsg = inMsg;
          msgCount++;
          Serial.print(msgCount);
          Serial.print(" , ");
          Serial.println(timeSinceLastMsg);
         }
         motor1.trigOscillation();
        break;
        }
      else if (messageIndex>6) {break;}
      else msg_data_byte[messageIndex] = current_data; //square time values
      break;

      case 31: //trig oscillation for mtr2, big motor
      if(end_message_flag == 1) { 
         motor2.setEnvelope(msg_data_byte[4],msg_data_byte[2],msg_data_byte[3], msg_data_byte[1],msg_data_byte[5]); //attack sustain amp decay
         motor2.setOscillation(msg_data_byte[0],0, 0, 0 ); //frequency attack decay length
         if(latencyTest){
          inMsg = millis();
          timeSinceLastMsg = inMsg - lastMsg;
          lastMsg = inMsg;
          msgCount++;
          Serial.print(msgCount);
          Serial.print(" , ");
          Serial.println(timeSinceLastMsg);
         }
         motor2.trigOscillation();
        break;
        }
      else if (messageIndex>5) {break;}
      else msg_data_byte[messageIndex] = current_data; //square time values
      break;

      case 32: //trig oscillation for both motors
      if(end_message_flag == 1) { 
         motor1.setEnvelope(msg_data_byte[4],msg_data_byte[2],msg_data_byte[3], msg_data_byte[1],msg_data_byte[5]); //attack sustain amp decay
         motor1.setOscillation(msg_data_byte[0],0, 0, 0 ); //frequency attack decay length
         motor2.setEnvelope(msg_data_byte[4],msg_data_byte[2],msg_data_byte[3], msg_data_byte[1],msg_data_byte[5]); //attack sustain amp decay
         motor2.setOscillation(msg_data_byte[0],0, 0, 0 ); //frequency attack decay length
         motor1.setBaseAmp(msg_data_byte[6]);
         motor1.trigOscillation();
         motor2.trigOscillation();
        break;
        }
      else if (messageIndex>6) {break;}
      else msg_data_byte[messageIndex] = current_data; //square time values
      break;

      case 33: //trig metronome for one motor using exp model
        motor1.setExpFlag();
        if(latencyTest){
          if(msgCount > 0){
            inMsg = millis();
            timeSinceLastMsg = inMsg - lastMsg;
            lastMsg = inMsg;
            msgCount++;
            Serial.print(msgCount);
            Serial.print(" , ");
            Serial.println(timeSinceLastMsg);
            msgCount = 0;
          }
          else{
            lastMsg = millis();
            //Serial.println("0, 0");
          }
          
        }
        break;

      //other functions here

      case 50: //set accel mode and variables    
      if(end_message_flag) { break;}
      if (messageIndex>9) break;
      if(messageIndex==0) setAccelMode(current_data);
      else setAccelModeVar(current_data,messageIndex-1);
      newDataFlag();
      break;
      
      case 51: //set accel enable   
      if(end_message_flag) { break;}
      if (messageIndex==0) {
        if(current_data<2) setAccelEnable(current_data);
        newDataFlag();
      }
      break;

      case 100: //set deviceMode
      if(end_message_flag) break;
      else if (messageIndex>1) break;
      else deviceMode = current_data;
      if(deviceMode==0)  setLED(deviceID);
      break;
    
      case 121: //ascii 'y', set deviceID remotely from host computer
      
      if( digitalRead(7) == 1) break; //if button is not pushed, break

      if(end_message_flag) {
        writeEepromSystemSettings();
        if(NRF_RECEIVE_DEBUG) {
          Serial.print("deviceID: ");
          Serial.println(deviceID);
          Serial.print("supergroup: ");
          Serial.println(supergroup);
        }
      while(digitalRead(7) == 0) processLed(0,100,0);
      processLed(0,0,0);
      break;
      }
      if (messageIndex==0) setDeviceID(current_data); 
      if (messageIndex==1) setSupergroup(current_data); 
      break;
    
      case 122: //ascii 'z', turn power off from host computer
        if (serialInputBuffer[i+1]==97) {
          if (serialInputBuffer[i+2]==105) {
            if (serialInputBuffer[i+3]==111) {
              if (serialInputBuffer[i+4]==97) {
                //write function to turn power off here
              }
            }
          }
        }
        i+=4;
        break;

        case 123:
        while(digitalRead(7) == 0){
          displaySupergroup();
          delay(500);
          displayDeviceID();
          delay(500);
        }
        if(end_message_flag) break;
        break;

        //set brake interval
        case 252:
        if(end_message_flag) break;
        if (messageIndex>0) break;
        motor1.brake(current_data); 
        newDataFlag();
        break;
        
        case 253:
        for(int j=0;j<10;j++) bufferByte(j);
        outputSerialData();
        break;
        
        case 254: //test case, mirrors input back to sender
          bufferByte(254);
          while(i<msgLength) 
          {
            i+=1;
            bufferByte(serialInputBuffer[i]);
          }
          outputSerialData();
        break;
        
        case 255:  //reserved for indicating new message
        break;
        } //end switch
        
        messageIndex++;
        
        if(end_message_flag){
          messageIndex=0;
          end_message_flag=0;
          messageType=255; //reserved to indicate new msg
        }
       // if(NRF_RECEIVE_DEBUG) Serial.println();
    }//end else 
  }
    
}
  
/************************************************************************************************************************************************************/

/*
checkPktForErrors()
Checks to see if the package is either not the expected size or is a duplicate. 
If either case is true the received data is not processed.
*/
byte checkPktForErrors(byte msgLength)
{
  byte incomingAckFlag=serialInputBuffer[2]>>7; //MSB is ackFlag
  byte recPktNum=serialInputBuffer[2] & 127; //packet number filters out MSB
  byte packetSize=serialInputBuffer[3]; //packet size
  static byte prevPktNum=255;
  packetReceiveFlag=1;

  if(msgLength != packetSize) { //packet not expected size
    if(NRF_RECEIVE_DEBUG) {Serial.print("Incorrect Packet Size"); Serial.println(1);}
    return(1);
   
  } else if (prevPktNum==recPktNum) { //packet is a duplicate
    if(NRF_RECEIVE_DEBUG) {Serial.print("Duplicate Packet:"); Serial.println(2);}
    return(2);
  } else { //packet is expected size and not a duplicate
    prevPktNum=recPktNum;
    if(incomingAckFlag==1) sendAckMessage(recPktNum); //send acknowledgement of packet number, if required
    return(0);
  }
}

/*
 * TEST RF testing function
 */
void testRF(byte val){
  byte dataToSend[]={val};
  manager.sendto((uint8_t *)dataToSend,1,SERVER_ADDRESS);
}



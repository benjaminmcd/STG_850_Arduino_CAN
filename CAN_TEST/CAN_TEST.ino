#include "STG-CAN.h"

uint8_t counter = 0;
uint8_t frameLength = 0;
unsigned long previousMillis = 0;     // stores last time output was updated
const long interval = 1000;           // transmission interval (milliseconds)

STG_CAN Can;  //Create CAN object

void setup() {
  Serial.begin(115200);
  bool ret = Can.CANInit(CAN_500KBPS);  //Initialize CAN at selected bit rate, options are in STG-CAN.h
  if (!ret) while(true);
}

void loop() {
  CAN_msg_t CAN_TX_msg;
  CAN_msg_t CAN_RX_msg;
   
  CAN_TX_msg.data[0] = 0x01;
  CAN_TX_msg.data[1] = 0x01;
  CAN_TX_msg.data[2] = 0x02;
  CAN_TX_msg.data[3] = 0x03;
  CAN_TX_msg.data[4] = 0x04;
  CAN_TX_msg.data[5] = 0x05;
  CAN_TX_msg.data[6] = 0x06;
  CAN_TX_msg.data[7] = 0x07;
  CAN_TX_msg.len = 8;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if ( ( counter % 2) == 0) {
      CAN_TX_msg.type = DATA_FRAME;
      CAN_TX_msg.format = EXTENDED_FORMAT;
      CAN_TX_msg.id = 0x072;
      Serial.println("extended");
    } else {
      CAN_TX_msg.type = DATA_FRAME;
      CAN_TX_msg.format = STANDARD_FORMAT;
      CAN_TX_msg.id = 0x401;
      Serial.println("standard");
    }
    Can.CANSend(&CAN_TX_msg);\
    counter++;
    Serial.println(counter);
  }
  
  if(Can.CANMsgAvail()) {
  
    Can.CANReceive(&CAN_RX_msg);

    if (CAN_RX_msg.format == EXTENDED_FORMAT) {
      Serial.print("Extended ID: 0x");
      if (CAN_RX_msg.id < 0x10000000) Serial.print("0");
      if (CAN_RX_msg.id < 0x1000000) Serial.print("0");
      if (CAN_RX_msg.id < 0x100000) Serial.print("0");
      if (CAN_RX_msg.id < 0x10000) Serial.print("0");
      if (CAN_RX_msg.id < 0x1000) Serial.print("0");
      if (CAN_RX_msg.id < 0x100) Serial.print("0");
      if (CAN_RX_msg.id < 0x10) Serial.print("0");
      Serial.print(CAN_RX_msg.id, HEX);
    } else {
      Serial.print("Standard ID: 0x");
      if (CAN_RX_msg.id < 0x100) Serial.print("0");
      if (CAN_RX_msg.id < 0x10) Serial.print("0");
      Serial.print(CAN_RX_msg.id, HEX);
      Serial.print("     ");
    }

    Serial.print(" DLC: ");
    Serial.print(CAN_RX_msg.len);
    if (CAN_RX_msg.type == DATA_FRAME) {
      Serial.print(" Data: ");
      for(int i=0; i<CAN_RX_msg.len; i++) {
        Serial.print("0x"); 
        Serial.print(CAN_RX_msg.data[i], HEX); 
        if (i != (CAN_RX_msg.len-1))  Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println(" Data: REMOTE REQUEST FRAME");
    }
  }
}
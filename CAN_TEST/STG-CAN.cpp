#include "STG-CAN.h"

void STG_CAN::CANSetGpio(GPIO_TypeDef* addr, uint8_t index, uint8_t afry, uint8_t speed = 3) {
  uint8_t _index2 = index * 2;
  uint8_t _index4 = index * 4;
  uint8_t ofs = 0;
  uint8_t setting;

  if (index > 7) {
    _index4 = (index - 8) * 4;
    ofs = 1;
  }

  uint32_t mask;
  printRegister("GPIO_AFR(b)=", addr->AFR[1]);
  mask = 0xF << _index4;
  addr->AFR[ofs] &= ~mask;  // Reset alternate function
  //setting = 0x9;                    // AF9
  setting = afry;  // Alternative function selection
  mask = setting << _index4;
  addr->AFR[ofs] |= mask;  // Set alternate function
  printRegister("GPIO_AFR(a)=", addr->AFR[1]);

  printRegister("GPIO_MODER(b)=", addr->MODER);
  mask = 0x3 << _index2;
  addr->MODER &= ~mask;  // Reset mode
  setting = 0x2;         // Alternate function mode
  mask = setting << _index2;
  addr->MODER |= mask;  // Set mode
  printRegister("GPIO_MODER(a)=", addr->MODER);

  printRegister("GPIO_OSPEEDR(b)=", addr->OSPEEDR);
  mask = 0x3 << _index2;
  addr->OSPEEDR &= ~mask;  // Reset speed
  setting = speed;
  mask = setting << _index2;
  addr->OSPEEDR |= mask;  // Set speed
  printRegister("GPIO_OSPEEDR(a)=", addr->OSPEEDR);

  printRegister("GPIO_OTYPER(b)=", addr->OTYPER);
  mask = 0x1 << index;
  addr->OTYPER &= ~mask;  // Reset Output push-pull
  printRegister("GPIO_OTYPER(a)=", addr->OTYPER);

  printRegister("GPIO_PUPDR(b)=", addr->PUPDR);
  mask = 0x3 << _index2;
  addr->PUPDR &= ~mask;  // Reset port pull-up/pull-down
  printRegister("GPIO_PUPDR(a)=", addr->PUPDR);
}


void STG_CAN::printRegister(char* buf, uint32_t reg) {
#ifdef DEBUG
  return;
#endif
  Serial.print(buf);
  Serial.print("0x");
  Serial.print(reg, HEX);
  Serial.println();
}

void STG_CAN::CANSetFilter(uint8_t index, uint8_t scale, uint8_t mode, uint8_t fifo, uint32_t bank1, uint32_t bank2) {
  if (index > 13) return;

  CAN->FA1R &= ~(0x1UL << index);  // Deactivate filter

  if (scale == 0) {
    CAN->FS1R &= ~(0x1UL << index);  // Set filter to Dual 16-bit scale configuration
  } else {
    CAN->FS1R |= (0x1UL << index);  // Set filter to single 32 bit configuration
  }
  if (mode == 0) {
    CAN->FM1R &= ~(0x1UL << index);  // Set filter to Mask mode
  } else {
    CAN->FM1R |= (0x1UL << index);  // Set filter to List mode
  }

  if (fifo == 0) {
    CAN->FFA1R &= ~(0x1UL << index);  // Set filter assigned to FIFO 0
  } else {
    CAN->FFA1R |= (0x1UL << index);  // Set filter assigned to FIFO 1
  }

  CAN->sFilterRegister[index].FR1 = bank1;  // Set filter bank registers1
  CAN->sFilterRegister[index].FR2 = bank2;  // Set filter bank registers2

  CAN->FA1R |= (0x1UL << index);  // Activate filter
}

bool STG_CAN::CANInit(BITRATE bitrate) {

  //Pull CAN-S Pin low to enable tranceiver
  pinMode(14, OUTPUT);

  digitalWrite(14, LOW);

  // Reference manual
  // https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

  RCC->APB1ENR |= 0x2000000UL;  // Enable CAN clock

  RCC->AHBENR |= 0x40000UL;   // Enable GPIOB clock
  CANSetGpio(GPIOB, 8, AF4);  // Set PB8 to AF4
  CANSetGpio(GPIOB, 9, AF4);  // Set PB9 to AF4

  CAN->MCR |= 0x1UL;  // Set CAN to Initialization mode
  while (!(CAN->MSR & 0x1UL))
    ;  // Wait for Initialization mode

  //CAN->MCR = 0x51UL;                 // Hardware initialization(No automatic retransmission)
  CAN->MCR = 0x41UL;  // Hardware initialization(With automatic retransmission)

  // Set bit rates
  //CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF));
  //CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);
  CAN->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x3FF));
  CAN->BTR |= (((can_configs[bitrate].TS2 - 1) & 0x07) << 20) | (((can_configs[bitrate].TS1 - 1) & 0x0F) << 16) | ((can_configs[bitrate].BRP - 1) & 0x3FF);
  printRegister("CAN->BTR=", CAN->BTR);

  // Configure Filters to default values
  CAN->FMR |= 0x1UL;  // Set to filter initialization mode

  // Set fileter 0
  // Single 32-bit scale configuration
  // Two 32-bit registers of filter bank x are in Identifier Mask mode
  // Filter assigned to FIFO 0
  // Filter bank register to all 0
  CANSetFilter(0, 1, 0, 0, 0x0UL, 0x0UL);

  CAN->FMR &= ~(0x1UL);  // Deactivate initialization mode

  uint16_t TimeoutMilliseconds = 1000;
  bool can1 = false;
  CAN->MCR &= ~(0x1UL);  // Require CAN1 to normal mode

  // Wait for normal mode
  // If the connection is not correct, it will not return to normal mode.
  for (uint16_t wait_ack = 0; wait_ack < TimeoutMilliseconds; wait_ack++) {
    if ((CAN->MSR & 0x1UL) == 0) {
      can1 = true;
      break;
    }
    delayMicroseconds(1000);
  }
  //Serial.print("can1=");
  //Serial.println(can1);
  if (can1) {
    Serial.println("CAN1 initialize ok");
  } else {
    Serial.println("CAN1 initialize fail!!");
    return false;
  }
  return true;
}

void STG_CAN::CANReceive(CAN_msg_t* CAN_rx_msg)
{
  uint32_t id = CAN->sFIFOMailBox[0].RIR;
  if ((id & STM32_CAN_RIR_IDE) == 0) { // Standard frame format
      CAN_rx_msg->format = STANDARD_FORMAT;;
      CAN_rx_msg->id = (CAN_STD_ID_MASK & (id >> 21U));
  } 
  else {                               // Extended frame format
      CAN_rx_msg->format = EXTENDED_FORMAT;;
      CAN_rx_msg->id = (CAN_EXT_ID_MASK & (id >> 3U));
  }

  if ((id & STM32_CAN_RIR_RTR) == 0) { // Data frame
      CAN_rx_msg->type = DATA_FRAME;
  }
  else {                               // Remote frame
      CAN_rx_msg->type = REMOTE_FRAME;
  }

  
  CAN_rx_msg->len = (CAN->sFIFOMailBox[0].RDTR) & 0xFUL;
  
  CAN_rx_msg->data[0] = 0xFFUL &  CAN->sFIFOMailBox[0].RDLR;
  CAN_rx_msg->data[1] = 0xFFUL & (CAN->sFIFOMailBox[0].RDLR >> 8);
  CAN_rx_msg->data[2] = 0xFFUL & (CAN->sFIFOMailBox[0].RDLR >> 16);
  CAN_rx_msg->data[3] = 0xFFUL & (CAN->sFIFOMailBox[0].RDLR >> 24);
  CAN_rx_msg->data[4] = 0xFFUL &  CAN->sFIFOMailBox[0].RDHR;
  CAN_rx_msg->data[5] = 0xFFUL & (CAN->sFIFOMailBox[0].RDHR >> 8);
  CAN_rx_msg->data[6] = 0xFFUL & (CAN->sFIFOMailBox[0].RDHR >> 16);
  CAN_rx_msg->data[7] = 0xFFUL & (CAN->sFIFOMailBox[0].RDHR >> 24);

  // Release FIFO 0 output mailbox.
  // Make the next incoming message available.
  CAN->RF0R |= 0x20UL;
}

void STG_CAN::CANSend(CAN_msg_t* CAN_tx_msg)
{
  volatile int count = 0;

  uint32_t out = 0;
  if (CAN_tx_msg->format == EXTENDED_FORMAT) { // Extended frame format
      out = ((CAN_tx_msg->id & CAN_EXT_ID_MASK) << 3U) | STM32_CAN_TIR_IDE;
  }
  else {                                  // Standard frame format
      out = ((CAN_tx_msg->id & CAN_STD_ID_MASK) << 21U);
  }

  // Remote frame
  if (CAN_tx_msg->type == REMOTE_FRAME) {
      out |= STM32_CAN_TIR_RTR;
  }

  CAN->sTxMailBox[0].TDTR &= ~(0xF);
  CAN->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;
  
  CAN->sTxMailBox[0].TDLR  = (((uint32_t) CAN_tx_msg->data[3] << 24) |
                               ((uint32_t) CAN_tx_msg->data[2] << 16) |
                               ((uint32_t) CAN_tx_msg->data[1] <<  8) |
                               ((uint32_t) CAN_tx_msg->data[0]      ));
  CAN->sTxMailBox[0].TDHR  = (((uint32_t) CAN_tx_msg->data[7] << 24) |
                               ((uint32_t) CAN_tx_msg->data[6] << 16) |
                               ((uint32_t) CAN_tx_msg->data[5] <<  8) |
                               ((uint32_t) CAN_tx_msg->data[4]      ));

  // Send Go
  CAN->sTxMailBox[0].TIR = out | STM32_CAN_TIR_TXRQ;

  // Wait until the mailbox is empty
  while(CAN->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000);
   
  // The mailbox don't becomes empty while loop
  if (CAN->sTxMailBox[0].TIR & 0x1UL) {
#ifdef DEBUG
    Serial.println("Send Fail");
    Serial.println(CAN->ESR);
    Serial.println(CAN->MSR);
    Serial.println(CAN->TSR);
#endif
  }
}

uint8_t STG_CAN::CANMsgAvail(void)
{
  // Check for pending FIFO 0 messages
  return CAN->RF0R & 0x3UL;
}
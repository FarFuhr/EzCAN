#include "EzCAN.h"

bool messageReceived = false;
EzCAN* can;

union {
  int32_t num;
  uint8_t arr[4];
} RxConvert;

void onMessage(const FDCAN_RxHeaderTypeDef& rxHeader, uint8_t* rxData) {
  memcpy(RxConvert.arr, rxData, 4);
  messageReceived = true;
}

void button_interrupt() {
  if (can->sendInt(0, 0x1) != EZ_CAN_OK) {
    SerialUSB.println("Error sending int");
  }
}

void setup() {
  SerialUSB.begin();
  delay(500);
  pinMode(PB13, OUTPUT);
  pinMode(PA15, INPUT_PULLUP);
  attachInterrupt(PA15, button_interrupt, RISING);
  can = EzCAN::getInstance();
  can->init(EzCAN::Speed::Kbps1000);
  can->configFilterAcceptAll();
  can->configGlobalFilter();
  can->start();
  can->activateNotification(onMessage);
  SerialUSB.println("CAN initialized");
}

void loop() {
  if (messageReceived) {
    digitalToggle(PB13);
    SerialUSB.printf("Message received: %d\n", (uint32_t)RxConvert.num);
    messageReceived = false;
    if (can->sendInt(RxConvert.num + 1, 0x1) != HAL_OK) {
      SerialUSB.println("Error sending CAN message");
    }
  }
}

#include <Arduino.h>
#include "EzCAN.h"

FDCAN_HandleTypeDef EzCAN::hfdcan1;

EzCAN::EzCAN()
  : RxHeader({})
  , RxData({0, })
  , _onReceive(nullptr)
  , _initialized(false) {
    hfdcan1.Instance = FDCAN1;
}

EzCANStatus EzCAN::init(Speed speed) {
    if (_initialized)
        return EZ_CAN_ERROR;
    hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = ENABLE;
    hfdcan1.Init.ProtocolException = DISABLE;

    hfdcan1.Init.NominalPrescaler = static_cast<uint16_t>(speed);
    hfdcan1.Init.NominalSyncJumpWidth = 4;
    hfdcan1.Init.NominalTimeSeg1 = 12;
    hfdcan1.Init.NominalTimeSeg2 = 4;

    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 4;
    hfdcan1.Init.DataTimeSeg1 = 5;
    hfdcan1.Init.DataTimeSeg2 = 4;
    hfdcan1.Init.StdFiltersNbr = 1;
    hfdcan1.Init.ExtFiltersNbr = 1;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    auto status = CONVERT_STATUS(HAL_FDCAN_Init(&hfdcan1));
    if (status == EZ_CAN_OK)
        _initialized = true;
    return status;
}

EzCANStatus EzCAN::start() {
    return CONVERT_STATUS(HAL_FDCAN_Start(&hfdcan1));
}

EzCANStatus EzCAN::configFilter(FDCAN_FilterTypeDef* filter) {
    return CONVERT_STATUS(HAL_FDCAN_ConfigFilter(&hfdcan1, filter));
}

EzCANStatus EzCAN::configFilter(uint32_t filterID1, uint32_t filterID2) {
    FDCAN_FilterTypeDef filterConfig;
    filterConfig.IdType = FDCAN_STANDARD_ID;
    filterConfig.FilterIndex = 0;
    filterConfig.FilterType = FDCAN_FILTER_MASK;
    filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filterConfig.FilterID1 = filterID1;
    filterConfig.FilterID2 = filterID2;
    return configFilter(&filterConfig);
}

EzCANStatus EzCAN::configFilterAcceptAll() {
    FDCAN_FilterTypeDef filterConfig;
    filterConfig.IdType = FDCAN_STANDARD_ID;
    filterConfig.FilterIndex = 0;
    filterConfig.FilterType = FDCAN_FILTER_MASK;
    filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filterConfig.FilterID1 = 0x0;
    filterConfig.FilterID2 = 0x0;
    return configFilter(&filterConfig);
}

EzCANStatus EzCAN::configFilterDenyAll() {
    FDCAN_FilterTypeDef filterConfig;
    filterConfig.IdType = FDCAN_STANDARD_ID;
    filterConfig.FilterIndex = 0;
    filterConfig.FilterType = FDCAN_FILTER_MASK;
    filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filterConfig.FilterID1 = 0x7FF;
    filterConfig.FilterID2 = 0x7FF;
    return configFilter(&filterConfig);
}

EzCANStatus EzCAN::configGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt) {
    return CONVERT_STATUS(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, nonMatchingStd, nonMatchingExt, rejectRemoteStd, rejectRemoteExt));
}

EzCANStatus EzCAN::activateNotification(ReceiveCallback onReceive) {
    if (_onReceive != nullptr)
        return EZ_CAN_ERROR;
    _onReceive = onReceive;
    return CONVERT_STATUS(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0));
}

EzCANStatus EzCAN::deactivateNotification() {
    if (!_onReceive)
        return EZ_CAN_OK;
    _onReceive = nullptr;
    return CONVERT_STATUS(HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
}

void EzCAN::notify() {
    _onReceive(RxHeader, RxData);
}

EzCANStatus EzCAN::sendMessage(uint8_t* data, uint8_t messageSize, uint32_t canID) {
    if (messageSize > 8)
        return EZ_CAN_ERROR;

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = canID;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = messageSize << 16;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    return CONVERT_STATUS(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data));
}

EzCANStatus EzCAN::sendInt(int32_t intVal, uint32_t canID) {
    return sendMessage((uint8_t*)&intVal, sizeof(int32_t), canID);
}

EzCANStatus EzCAN::sendInt(int32_t intVal1, int32_t intVal2, uint32_t canID) {
    uint8_t buf[sizeof(int32_t) * 2];
    memcpy(buf, &intVal1, sizeof(int32_t));
    memcpy(buf + sizeof(int32_t), &intVal2, sizeof(int32_t));
    return sendMessage(buf, sizeof(int32_t) * 2, canID);
}

EzCANStatus EzCAN::sendFloat(float floatVal, uint32_t canID) {
    return sendMessage((uint8_t*)&floatVal, sizeof(float), canID);    
}

EzCANStatus EzCAN::sendFloat(float floatVal1, float floatVal2, uint32_t canID) {
    uint8_t buf[sizeof(float) * 2];
    memcpy(buf, &floatVal1, sizeof(float));
    memcpy(buf + sizeof(float), &floatVal2, sizeof(float));
    return sendMessage(buf, sizeof(float) * 2, canID);    
}

EzCANStatus EzCAN::sendText(const char* text, uint32_t canID, bool split) {
    if (strlen(text) > 8 && !split)
        return EZ_CAN_ERROR;
    char buf[8];
    for (size_t i = 0; i < strlen(text); i += 8) {
        memcpy(buf, text + i, 8);
        EzCANStatus sendStatus = sendMessage((uint8_t*)buf, strlen(buf), canID);
    }
}

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    if(hfdcan->Instance==FDCAN1) {
        /** Initializes the peripherals clocks
         */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        }

        /* Peripheral clock enable */
        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /**FDCAN1 GPIO Configuration
        PB8-BOOT0     ------> FDCAN1_RX
        PB9     ------> FDCAN1_TX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // Enable interrupts
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
}

extern "C" void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan) {
  if(hfdcan->Instance==FDCAN1) {
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PB8-BOOT0     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
    
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  }
}

extern "C" void FDCAN1_IT0_IRQHandler(void) {
  HAL_FDCAN_IRQHandler(&EzCAN::hfdcan1);
}


extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (HAL_FDCAN_GetRxFifoFillLevel(&EzCAN::hfdcan1, FDCAN_RX_FIFO0) > 0) {
        EzCAN* ezcan = EzCAN::getInstance();
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &(ezcan->RxHeader), ezcan->RxData) == HAL_OK) {
            ezcan->notify();
        }
    }
}

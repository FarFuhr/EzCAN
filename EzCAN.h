#pragma once

#include <stm32g4xx_hal_fdcan.h>

enum EzCANStatus {
    EZ_CAN_OK = 0x00U,
    EZ_CAN_ERROR = 0x01U,
    EZ_CAN_BUSY = 0x02U,
    EZ_CAN_TIMEOUT = 0x03U
};

#define CONVERT_STATUS(x) (static_cast<EzCANStatus>(x))

typedef void(*ReceiveCallback)(const FDCAN_RxHeaderTypeDef& rxHeader, uint8_t* rxData);

class EzCAN {
public:
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    enum class Speed {
        Kbps1000 = 10,
        Kbps500 = 20,
        Kbps250 = 40,
        Kbps100 = 100
    };

    static EzCAN* getInstance() {
        static EzCAN* instance = nullptr;
        if (!instance)
            instance = new EzCAN();
        return instance;
    }

    EzCANStatus init(Speed speed);
    EzCANStatus start();

    EzCANStatus configFilter(FDCAN_FilterTypeDef* filter);
    EzCANStatus configFilter(uint32_t filterID1 = 0x1, uint32_t filterID2 = 0x7FF);
    EzCANStatus configFilterAcceptAll();
    EzCANStatus configFilterDenyAll();
    EzCANStatus configGlobalFilter(uint32_t nonMatchingStd = FDCAN_REJECT,
                                   uint32_t nonMatchingExt = FDCAN_REJECT,
                                   uint32_t rejectRemoteStd = FDCAN_REJECT_REMOTE,
                                   uint32_t rejectRemoteExt = FDCAN_REJECT_REMOTE);
    EzCANStatus activateNotification(ReceiveCallback callback);
    EzCANStatus deactivateNotification();

    EzCANStatus sendMessage(uint8_t* data, uint8_t messageSize, uint32_t canID);
    EzCANStatus sendInt(int32_t intVal, uint32_t canID);
    EzCANStatus sendInt(int32_t intVal1, int32_t intVal2, uint32_t canID);
    EzCANStatus sendFloat(float floatVal, uint32_t canID);
    EzCANStatus sendFloat(float floatVal1, float floatVal2, uint32_t canID);
    EzCANStatus sendText(const char* text, uint32_t canID, bool split = true);
    
    void notify();

    static FDCAN_HandleTypeDef hfdcan1;
private:
    EzCAN();

    ReceiveCallback _onReceive;
    bool _initialized;
};

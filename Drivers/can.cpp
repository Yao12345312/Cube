#include "can.hpp"


static CanDriver* s_can1_instance = nullptr;

// ================= 全局接收缓存 =================
static FDCAN_RxHeaderTypeDef g_rxHeader;
static uint8_t g_rxData[8];

CanDriver::CanDriver()
{
    hfdcan_ = nullptr;
    rx_ch2_speed_ = 0.0f;
    rx_ch3_speed_ = 0.0f;
    rx_frame_count_ = 0;
    is_online_ = false;
}

// ================= 初始化 =================
void CanDriver::Init(FDCAN_HandleTypeDef *hfdcan)
{
    hfdcan_ = hfdcan;
	
	s_can1_instance = this;
	
    // 启动 FDCAN
    HAL_FDCAN_Start(hfdcan_);

    // 使能接收中断
    HAL_FDCAN_ActivateNotification(hfdcan_,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0);
}

// ================= 发送电机速度 =================
void CanDriver::SendMotorSpeed(float ch2_speed, float ch3_speed)
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];

    int32_t ch2_i = (int32_t)(ch2_speed * 1000.0f);
    int32_t ch3_i = (int32_t)(ch3_speed * 1000.0f);

    txHeader.Identifier = eCAN_SEND_MOTOR_SPEED_FRAME;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    txData[0] = ch2_i & 0xFF;
    txData[1] = (ch2_i >> 8) & 0xFF;
    txData[2] = (ch2_i >> 16) & 0xFF;
    txData[3] = (ch2_i >> 24) & 0xFF;

    txData[4] = ch3_i & 0xFF;
    txData[5] = (ch3_i >> 8) & 0xFF;
    txData[6] = (ch3_i >> 16) & 0xFF;
    txData[7] = (ch3_i >> 24) & 0xFF;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData);
}

// ================= 接收回调处理 =================
void CanDriver::RxCallback()
{
    HAL_FDCAN_GetRxMessage(hfdcan_,
        FDCAN_RX_FIFO0,
        &g_rxHeader,
        g_rxData);

    // 标准帧 + 数据帧
    if (g_rxHeader.IdType == FDCAN_STANDARD_ID &&
        g_rxHeader.RxFrameType == FDCAN_DATA_FRAME)
    {
        if (g_rxHeader.Identifier == eCAN_GET_MOTOR_SPEED_FRAME)
        {
            int32_t ch2_i =
                g_rxData[0] |
                (g_rxData[1] << 8) |
                (g_rxData[2] << 16) |
                (g_rxData[3] << 24);

            int32_t ch3_i =
                g_rxData[4] |
                (g_rxData[5] << 8) |
                (g_rxData[6] << 16) |
                (g_rxData[7] << 24);

            rx_ch2_speed_ = (float)ch2_i / 1000.0f;
            rx_ch3_speed_ = (float)ch3_i / 1000.0f;

            rx_frame_count_ = 0;
            is_online_ = true;
        }
    }
}

// ================= 获取速度 =================
void CanDriver::GetMotorSpeed(float &ch2_speed, float &ch3_speed)
{
    ch2_speed = rx_ch2_speed_;
    ch3_speed = rx_ch3_speed_;
}

// ================= 在线检测 =================
bool CanDriver::IsOnline()
{
    rx_frame_count_++;

    if (rx_frame_count_ > CAN_OFFLINE_COUNT)
    {
        is_online_ = false;
    }

    return is_online_;
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                          uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == FDCAN1)
    {
        if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        {
            if (s_can1_instance != nullptr)
            {
                s_can1_instance->RxCallback();
            }
        }
    }
}
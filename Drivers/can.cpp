#include "can.hpp"
#include <string.h>
#include "uart3Driver.hpp"

UavcanCanDriver::UavcanCanDriver(FDCAN_HandleTypeDef* hfdcan)
    : hfdcan_(hfdcan), canard_(nullptr)
{
}

void UavcanCanDriver::DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
//获取系统时间（us）
uint64_t UavcanCanDriver::micros64()
{
    static uint32_t last = 0;
    static uint64_t high = 0;

    uint32_t now = DWT->CYCCNT;

    if(now < last) //溢出检测
    {
        high += (1ULL << 32);
    }

    last = now;

    uint64_t full = high | now;

    return full / (SystemCoreClock / 1000000);
}

bool UavcanCanDriver::init()
{
	
    if (HAL_FDCAN_Start(hfdcan_) != HAL_OK)
        return false;

    DWT_Init();

    return true;
}

void UavcanCanDriver::attach_canard(CanardInstance* canard)
{
    canard_ = canard;
}


void UavcanCanDriver::process_tx(uint8_t max_frams)
{
    const CanardCANFrame* txf = NULL;
	uint8_t count = 0;
	
    while((txf = canardPeekTxQueue(canard_)) != NULL)
    {	
		
		if(count++ >= max_frams){break;}
		
        FDCAN_TxHeaderTypeDef txHeader = {0};

        txHeader.Identifier = txf->id & CANARD_CAN_EXT_ID_MASK;
        txHeader.IdType = FDCAN_EXTENDED_ID;
		//解析数据长度
        uint32_t dlc;
        switch (txf->data_len)
        {
            case 0: dlc = FDCAN_DLC_BYTES_0; break;
            case 1: dlc = FDCAN_DLC_BYTES_1; break;
            case 2: dlc = FDCAN_DLC_BYTES_2; break;
            case 3: dlc = FDCAN_DLC_BYTES_3; break;
            case 4: dlc = FDCAN_DLC_BYTES_4; break;
            case 5: dlc = FDCAN_DLC_BYTES_5; break;
            case 6: dlc = FDCAN_DLC_BYTES_6; break;
            case 7: dlc = FDCAN_DLC_BYTES_7; break;
            case 8: dlc = FDCAN_DLC_BYTES_8; break;
            default: dlc = FDCAN_DLC_BYTES_8; break;
        }
		//配置发送帧头
        txHeader.DataLength = dlc;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
		//送入发送队列
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txf->data) == HAL_OK)
        {
            canardPopTxQueue(canard_);
        }
        else
        {
            break; // FIFO满
        }
    }
}

void UavcanCanDriver::process_rx(uint8_t max_frams)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];
	uint8_t count = 0;
	
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0) > 0)
    {	
		if(count++ >= max_frams){break;}
		
        HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rxHeader, data);

        CanardCANFrame rx_frame;
		//处理扩展帧
        rx_frame.id = rxHeader.Identifier | CANARD_CAN_FRAME_EFF;
		
        uint8_t len;
        switch (rxHeader.DataLength)
        {
            case FDCAN_DLC_BYTES_0: len = 0; break;
            case FDCAN_DLC_BYTES_1: len = 1; break;
            case FDCAN_DLC_BYTES_2: len = 2; break;
            case FDCAN_DLC_BYTES_3: len = 3; break;
            case FDCAN_DLC_BYTES_4: len = 4; break;
            case FDCAN_DLC_BYTES_5: len = 5; break;
            case FDCAN_DLC_BYTES_6: len = 6; break;
            case FDCAN_DLC_BYTES_7: len = 7; break;
            case FDCAN_DLC_BYTES_8: len = 8; break;
            default: len = 8; break;
        }

        rx_frame.data_len = len;
		
	    memcpy(rx_frame.data, data, len);
			
		//if(((rx_frame.id >> 7)&0x1U) ? ((rx_frame.id>>8)& 0x7FU) == canardGetLocalNodeID(canard_) : 1){
		canardHandleRxFrame(canard_, &rx_frame, micros64());
		//}
        

    }
}
//HAL库发送测试函数
void UavcanCanDriver::CAN_Send_Test()
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    uint8_t txData[8] = {1,1,2,4,5,6,7,8};

    txHeader.Identifier = 0x123;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;

	if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan_) > 0)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) == HAL_OK)
		{
			printf("TX success\r\n");
		}
	}
	else
	{
		printf("TX FIFO FULL\r\n");
	}
}
//HAL库接收测试函数
void UavcanCanDriver::CAN_Receive_Test()
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
	
				FDCAN_ProtocolStatusTypeDef status;
	HAL_FDCAN_GetProtocolStatus(hfdcan_, &status);

	printf("LEC=%d, ACT=%d, EP=%d, BO=%d\r\n",
        status.LastErrorCode,
        status.Activity,
        status.ErrorPassive,
        status.BusOff);	
	
	printf("RX FIFO level: %d\r\n",
    HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0));
	
    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0) > 0)
    {
        if ( HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            printf("RX ID: 0x%lx | Data: ", rxHeader.Identifier);

            for (int i = 0; i < 8; i++)
            {
                printf("%d ", rxData[i]);
            }
            printf("\r\n");
        }
    }
}
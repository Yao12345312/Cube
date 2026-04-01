#pragma once
#include "stm32h7xx_hal.h"
#include "canard.h"
#include "can.hpp"
#include <string.h>

class ESCNode
{
public:
    ESCNode(UavcanCanDriver& can_driver);

    void init();
    void spin_once();
	
	void send_node_status();
	void send_esc_raw(uint8_t esc_index, float throttle);
private:
    // 回调
    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);
	
	
    void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer);

private:
    UavcanCanDriver& can_driver_;

    CanardInstance canard_;
	//初始化内存池
    uint8_t memory_pool_[2048];

};
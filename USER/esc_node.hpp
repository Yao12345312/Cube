#pragma once

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "canard.h"
#include "can.hpp"
#include <string.h>

#define ESC1_Index 0
#define ESC2_Index 1
#define ESC3_Index 2
#define Max_ESC_Num 3

class ESCNode
{
public:
    ESCNode(UavcanCanDriver& can_driver);
	
	struct ESCStatusCache
	{
		int32_t rpm;
		float voltage;
		float current;
		float temperature;
		volatile bool calib_flag;
	};
	
    void init();
    void spin_once();
	
	void send_node_status();
	
	void set_esc_index_command(uint8_t target_esc_index);
	void calib_esc_command(uint8_t target_esc_index);
	void send_esc_rpm_commmand(uint8_t esc_index, int32_t rpm);
	
	bool get_esc_status(uint8_t esc_index, ESCStatusCache& out);
	
	
private:
    // 回调
    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);
	
	
    void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer);
	
	void handle_esc_status(CanardInstance *ins, CanardRxTransfer* transfer);

private:
    UavcanCanDriver& can_driver_;

    CanardInstance canard_;
	//初始化内存池
    uint8_t memory_pool_[2048];
	
	//电调状态结构体
	ESCStatusCache esc_status_[Max_ESC_Num];
	
	//互斥锁定义
	osMutexId_t m_send_mutex;

	osMutexId_t m_esc_get_staus_mutex;
	
	//transfer ID定义
	uint8_t node_status_transfer_id_;

	uint8_t esc_index_transfer_id_;
	
	uint8_t calib_esc_transfer_id_;

	uint8_t esc_rpm_commmand_transfer_id_;
	
	volatile bool esc_arm_flag ;
};
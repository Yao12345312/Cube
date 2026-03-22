#include "mavlink_bridge.hpp"
#include "board.hpp"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

namespace {
	//配置此设备系统ID和组件ID
	static uint8_t mav_sysid = 1;
    static uint8_t mav_compid = 1;
	//配置发送缓冲区
	static uint8_t mav_tx_buf[MAVLINK_TX_BUF_LEN];
	
	volatile bool mavlink_connected=false;
}


namespace MAVLink {
	
	void set_mavlink_connect_status(bool status){ mavlink_connected=status; }
	
	bool get_mavlink_connect_status(void){return mavlink_connected;}
	
	
	void SendHeartbeat(void)
	{	
	//获取串口实例
	auto& uart = Board::getUart1();
	
    // 创建 MAVLink 消息结构体
    mavlink_message_t msg;

    // MAV_TYPE_QUADROTOR表示飞行器类型
    uint8_t type = MAV_TYPE_QUADROTOR;      

    // MAV_AUTOPILOT_ARDUPILOTMEGA
    uint8_t autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;

    // base_mode 和 custom_mode 通常为 0
    uint8_t base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t custom_mode = 0;

    // system_status：一般发 MAV_STATE_ACTIVE
    uint8_t system_status = MAV_STATE_ACTIVE;

    // 打包 HEARTBEAT 消息
    mavlink_msg_heartbeat_pack(mav_sysid,
                               mav_compid,
                               &msg,
                               type,
                               autopilot,
                               base_mode,
                               custom_mode,
                               system_status);

    // 序列化成字节流
    uint16_t len = mavlink_msg_to_send_buffer(mav_tx_buf, &msg);

    // 通过 DMA 发送
	uart.send(mav_tx_buf,len);
	}
	
}






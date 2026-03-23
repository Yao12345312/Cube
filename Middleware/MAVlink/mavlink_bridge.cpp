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
	//volatile修饰防止编译优化，确保每次读取真实状态
	static volatile bool mavlink_connected = false;
	
	 // MAVLink解析器状态
    static mavlink_status_t mav_status;
    static mavlink_message_t mav_msg;
    
    // MAVLink状态信息
    static struct {
        uint32_t last_hb_time;      // 最后收到心跳的时间
        uint8_t link_active;        // 链路是否激活 (0:断开, 1:连接)
    } mav_status_info = {
        .last_hb_time = 0,
        .link_active = 0
    };
    
    // 互斥锁保护状态变量
    static osMutexId_t mav_status_mutex = NULL;
	
}


namespace MAVLink {
	
	void Init(void) {
        // 创建互斥锁
        mav_status_mutex = osMutexNew(NULL);
        if (mav_status_mutex == NULL) {
            printf("MAVLink: Failed to create mutex\r\n");
        } else {
            printf("MAVLink: Initialized successfully\r\n");
        }
        
        // 初始化MAVLink解析器状态
        memset(&mav_status, 0, sizeof(mav_status));
        memset(&mav_msg, 0, sizeof(mav_msg));
        
        // 初始化状态信息
        mav_status_info.last_hb_time = osKernelGetTickCount();
        mav_status_info.link_active = 0;
        mavlink_connected = false;
    }
	
	//设置mavlink连接状态
	void set_mavlink_connect_status(bool status){ mavlink_connected=status; }
	//获取mavlink连接状态
	bool get_mavlink_connect_status(void){return mavlink_connected;}
	
	//MAVlink协议解析器
	void ParseData(const uint8_t* data, uint16_t len) {
        if (data == NULL || len == 0) {return;}
    
        // 逐字节喂给MAVLink解析器
        for (uint16_t i = 0; i < len; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &mav_msg, &mav_status) == MAVLINK_FRAMING_OK) {
                // 解析成功，根据消息ID处理
                switch (mav_msg.msgid) {
					//解析地面站心跳包
                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        // 解码心跳消息
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&mav_msg, &hb);
                        
                        // 使用互斥锁保护状态更新
                        if (mav_status_mutex != NULL) {
                            osMutexAcquire(mav_status_mutex, osWaitForever);
                        }
                        
                        mav_status_info.last_hb_time = osKernelGetTickCount();
                        //如果之前不在连接状态，修改连接状态为1（已连接）
                        if (mav_status_info.link_active != 1) {
                            mav_status_info.link_active = 1;
                            set_mavlink_connect_status(true);
                            printf("MAVLink: Heartbeat received from system %d, comp %d\r\n", 
                                   mav_msg.sysid, mav_msg.compid);
                        }
                        
                        if (mav_status_mutex != NULL) {
                            osMutexRelease(mav_status_mutex);
                        }
                        break;
                    }
                    
				   
                    default:
                        // 其他消息可以忽略或根据需要添加处理
                        break;
                }
            }
        }
    }


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






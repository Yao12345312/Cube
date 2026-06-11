#pragma once

#ifdef __cplusplus
extern "C" {
#endif
	
#include "commonACFly/mavlink.h"
	
#ifdef __cplusplus
}
#endif

//发送缓冲区大小
#define MAVLINK_TX_BUF_LEN  128

namespace MAVLink {
	
	void Init(void);
		
	void SendHeartbeat(void);
	
	void SendBatteryStatus(float voltage_v,
                       float current_a,
                       int8_t battery_remaining);
					   
	void SendAttitude(float roll,
                  float pitch,
                  float yaw,
                  float rollspeed,
                  float pitchspeed,
                  float yawspeed);
	
	bool get_mavlink_connect_status(void);
	
	void set_mavlink_connect_status(bool status);
	
	void ParseData(const uint8_t* data, uint16_t len);
}

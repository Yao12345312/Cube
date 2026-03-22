#pragma once

#ifdef __cplusplus
extern "C" {
#endif
	
#include "mavlink.h"   	
	
#ifdef __cplusplus
}
#endif

//发送缓冲区大小
#define MAVLINK_TX_BUF_LEN  128

namespace MAVLink {
	
	void SendHeartbeat(void);
	
	bool get_mavlink_connect_status(void);
	
	void set_mavlink_connect_status(bool status);

}

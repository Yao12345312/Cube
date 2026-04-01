#include "esc_node.hpp"
#include "dronecan_msgs.h"
#include "uavcan.equipment.esc.RawCommand.h" 

static struct uavcan_protocol_NodeStatus node_status;
	
//初始化时先实例化can_driver,再传入can_driver实例化ESCNode
ESCNode::ESCNode(UavcanCanDriver& can_driver)
    : can_driver_(can_driver)
{
}

void ESCNode::init()
{
    canardInit(&canard_,
               memory_pool_,
               sizeof(memory_pool_),
               onTransferReceived, //提供静态函数
               shouldAcceptTransfer,
               this);
			   
	//设置当前节点ID
    canardSetLocalNodeID(&canard_,10);

    // 绑定底层can驱动
    can_driver_.attach_canard(&canard_);
}

void ESCNode::spin_once()
{	
	//每次最多发送10帧
    can_driver_.process_tx(1);
    can_driver_.process_rx(1);
}
//处理接收到的信息
void ESCNode::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{	
	ESCNode* self = (ESCNode*)ins->user_reference;
	
	//处理请求
    if (transfer->transfer_type == CanardTransferTypeRequest) 
	{
        switch (transfer->data_type_id) 
		{
			case UAVCAN_PROTOCOL_GETNODEINFO_ID:
			self->handle_GetNodeInfo(ins, transfer);
				break;

        }
    }
}
//检查我们是否需要这条消息
bool ESCNode::shouldAcceptTransfer(const CanardInstance* ins,
                                   uint64_t* out_data_type_signature,
                                   uint16_t data_type_id,
                                   CanardTransferType transfer_type,
                                   uint8_t source_node_id)
{	
	ESCNode* self = (ESCNode*)ins->user_reference;
	
    if (transfer_type == CanardTransferTypeRequest) 
	{
        switch (data_type_id) 
		{
			case UAVCAN_PROTOCOL_GETNODEINFO_ID:
				*out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
				return true;
			
		}
	}
	
	return false;
	
}

void ESCNode::handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{	
	ESCNode* self = (ESCNode*)ins->user_reference;
	
	uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;
	
	memset(&pkt,0,sizeof(pkt));
	
	node_status.uptime_sec = self->can_driver_.micros64()/1000000;
	pkt.status = node_status;
	pkt.software_version.major = 1;
    pkt.software_version.minor = 1;
    pkt.software_version.optional_field_flags = 1;
    pkt.software_version.vcs_commit = 1; // should put git hash in here
	
	uint32_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);
	
	canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
	
}
	
//广播当前节点状态
void ESCNode::send_node_status()
{
    uavcan_protocol_NodeStatus msg;
	//获取系统时间（s）
    msg.uptime_sec = can_driver_.micros64() / 1000000;
    msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.mode   = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    msg.sub_mode = 0;
    msg.vendor_specific_status_code = 0;

	//存放序列化后的数据，传入Broadcast函数
    uint8_t buffer[32];
	//序列化数据并获取有效长度
    uint32_t size = uavcan_protocol_NodeStatus_encode(&msg, buffer);

    static uint8_t transfer_id = 0;
	/*
    * param1:Canard库实例指针，包含了内存池、节点ID等状态信息
    * param2:数据类型签名：由DSDL定义计算出的64位哈希值，用于唯一标识消息类型。接收端通过此值验证消息格式正确性
    * param3:消息主题ID：0-65535之间的数值，标识消息类别
    * param4:传输ID指针：每次发送同类型广播消息时自增1（0-31循环），用于接收端区分消息先后顺序和检测丢帧
    * param5:优先级：0（最高）到31（最低），CAN总线仲裁时高优先级消息优先传输
    * param6:有效负载数据
    * param7:有效数据负载长度
    */
    canardBroadcast(&canard_,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    size);
					
	transfer_id++;
}

//广播电调油门值，传入指定电调序号和油门值[-1~1],负值表示反转
void ESCNode::send_esc_raw(uint8_t esc_index, float throttle)
{
    // 油门限幅
    if (throttle > 1.0f) throttle = 1.0f;
    if (throttle < -1.0f) throttle = -1.0f;
	
    uavcan_equipment_esc_RawCommand msg;
	//设置命令数量
    msg.cmd.len = esc_index + 1;
	
	//设置单个电调油门，其他通道设置0
    for (uint8_t i = 0; i < msg.cmd.len; i++)
    {
        if (i == esc_index)
        {	
			//油门值映射为-8191~8191
            msg.cmd.data[i] = (int16_t)(throttle * 8192);
        }
        else
        {
            msg.cmd.data[i] = 0;
        }
    }

    //存放序列化后的数据
    uint8_t buffer[64];
	//序列化数据并获取有效长度
    uint32_t size = uavcan_equipment_esc_RawCommand_encode(&msg, buffer);
	
	//序列化失败
	if(size == 0){return;}

    // 发送
    static uint8_t transfer_id = 0;
	
    canardBroadcast(&canard_,
                    UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_HIGH, //控制指令设置高优先级
                    buffer,
                    size);
	
	transfer_id++;
}

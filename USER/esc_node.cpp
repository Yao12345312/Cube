#include "esc_node.hpp"
#include "dronecan_msgs.h"
#include "uart3Driver.hpp"

static struct uavcan_protocol_NodeStatus node_status;

//初始化时先实例化can_driver,再传入can_driver实例化ESCNode
ESCNode::ESCNode(UavcanCanDriver& can_driver)
    : can_driver_(can_driver)
	, node_status_transfer_id_(0)      // 构造函数中初始化transfer ID为0
    , esc_index_transfer_id_(0)
    , calib_esc_transfer_id_(0)
    , esc_rpm_commmand_transfer_id_(0)
	, esc_arm_flag(false)
{	
	//发送指令互斥锁
	m_send_mutex = osMutexNew(NULL);
	//获取状态互斥锁
	m_esc_get_staus_mutex = osMutexNew(NULL);
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
	//每次最多发送1帧
    can_driver_.process_tx(5);
    can_driver_.process_rx(64);
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
	//处理广播类型
	if (transfer->transfer_type == CanardTransferTypeBroadcast)
    {
        switch (transfer->data_type_id)
        {
            case UAVCAN_EQUIPMENT_ESC_CUBESTATUS_ID:
                self->handle_esc_status(ins, transfer);
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
	if (transfer_type == CanardTransferTypeBroadcast)
    {
        switch (data_type_id)
        {
            case UAVCAN_EQUIPMENT_ESC_CUBESTATUS_ID:
                *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_CUBESTATUS_SIGNATURE;
                return true;
			
        }
    }
	
	return false;
	
}
//解码接收到的节点信息
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
//处理接收到的数据
void ESCNode::handle_esc_status(CanardInstance *ins, CanardRxTransfer* transfer)
{	
	ESCNode* self = (ESCNode*)ins->user_reference;
	
    struct uavcan_equipment_esc_CubeStatus msg;

    if (!uavcan_equipment_esc_CubeStatus_decode(transfer, &msg))
    {
        // 解析成功,电调ID从[1,3]映射为[0,2]
        uint8_t esc_id = (msg.esc_index - 1);

        float voltage = msg.voltage;
        float current = msg.current;
        float temperature = msg.temperature;
		//获取电调实际转速
        int32_t rpm = msg.rpm;
		//获取电调控制指令（字段名仍为target_rpm）
		int32_t target_rpm = msg.target_rpm;
		
		bool calib_flag = msg.calib_done;
		
//      printf("ESC[%d]: rpm=%ld, V=%.2f, I=%.2f, T=%.2f\n",
//               esc_id, rpm, voltage, current, temperature);

        //存储电调状态
        if (esc_id < Max_ESC_Num)
		{	
			osMutexAcquire(self->m_esc_get_staus_mutex, osWaitForever);
			
			self->esc_status_[esc_id].rpm = rpm;
			self->esc_status_[esc_id].target_rpm = target_rpm;
			self->esc_status_[esc_id].voltage = voltage;
			self->esc_status_[esc_id].current = current;
			self->esc_status_[esc_id].temperature = (temperature-273.15f); //开尔文转换为摄氏度
			self->esc_status_[esc_id].calib_flag=calib_flag;
			
			osMutexRelease(self->m_esc_get_staus_mutex);
		}
    }
	
}

	
//广播当前节点状态
void ESCNode::send_node_status()
{	

    struct uavcan_protocol_NodeStatus msg;
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

	/*
    * param1:Canard库实例指针，包含了内存池、节点ID等状态信息
    * param2:数据类型签名：由DSDL定义计算出的64位哈希值，用于唯一标识消息类型。接收端通过此值验证消息格式正确性
    * param3:消息主题ID：0-65535之间的数值，标识消息类别
    * param4:传输ID指针：每次发送同类型广播消息时自增1（0-31循环），用于接收端区分消息先后顺序和检测丢帧
    * param5:优先级：0（最高）到31（最低），CAN总线仲裁时高优先级消息优先传输
    * param6:有效负载数据
    * param7:有效数据负载长度
    */
	osMutexAcquire(m_send_mutex, osWaitForever);
	
    canardBroadcast(&canard_,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &node_status_transfer_id_,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    size);
					
	node_status_transfer_id_++;
	
	osMutexRelease(m_send_mutex);
}

//设置电调编号
void ESCNode::set_esc_index_command(uint8_t target_esc_index)
{

    struct uavcan_equipment_esc_CubeSetID  msg = {0};
	//传入目标电调ID序号
	msg.esc_index=target_esc_index;

    //存放序列化后的数据
    uint8_t buffer[32];
	//序列化数据并获取有效长度
    uint32_t size = uavcan_equipment_esc_CubeSetID_encode(&msg, buffer);
	
	//序列化失败
	if(size == 0){return;}
	
	osMutexAcquire(m_send_mutex, osWaitForever);
	
    canardBroadcast(&canard_,
                    UAVCAN_EQUIPMENT_ESC_CUBESETID_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_CUBESETID_ID,
                    &esc_index_transfer_id_,
                    CANARD_TRANSFER_PRIORITY_MEDIUM, 
                    buffer,
                    size);
	
	esc_index_transfer_id_++;
	
	osMutexRelease(m_send_mutex);
}
//电调校准命令
void ESCNode::calib_esc_command(uint8_t target_esc_index)
{

    struct uavcan_equipment_esc_CubeCalibCommand msg = {0};
	//传入需要校准的目标电调ID序号
	msg.esc_index=target_esc_index;

    //存放序列化后的数据
    uint8_t buffer[32];
	//序列化数据并获取有效长度
    uint32_t size = uavcan_equipment_esc_CubeCalibCommand_encode(&msg, buffer);
	
	
	//序列化失败
	if(size == 0){return;}
	
	osMutexAcquire(m_send_mutex, osWaitForever);
	
    canardBroadcast(&canard_,
                    UAVCAN_EQUIPMENT_ESC_CUBECALIBCOMMAND_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_CUBECALIBCOMMAND_ID,
                    &calib_esc_transfer_id_,
                    CANARD_TRANSFER_PRIORITY_MEDIUM, 
                    buffer,
                    size);
	
	calib_esc_transfer_id_++;
	
	osMutexRelease(m_send_mutex);
}


// 广播电调电流
void ESCNode::send_esc_current_commands(const int32_t *cmd_array, uint8_t len)
{
    struct uavcan_equipment_esc_CubeIqCommand msg = {0};

    // 长度保护
    if (len > Max_ESC_Num) len = Max_ESC_Num;

    msg.Iq.len = len + 1;
    msg.arm = 1;

    for (uint8_t i = 0; i < len; i++)
    {
        int32_t Iq = cmd_array[i];

        msg.Iq.data[i] = Iq;
    }

    uint8_t buffer[64];
    uint32_t size = uavcan_equipment_esc_CubeIqCommand_encode(&msg, buffer);

    if (size == 0) return;

    osMutexAcquire(m_send_mutex, osWaitForever);

    canardBroadcast(&canard_,
                    UAVCAN_EQUIPMENT_ESC_CUBEIQCOMMAND_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_CUBEIQCOMMAND_ID,
                    &esc_rpm_commmand_transfer_id_,
                    CANARD_TRANSFER_PRIORITY_HIGH,
                    buffer,
                    size);

    esc_rpm_commmand_transfer_id_++;

    osMutexRelease(m_send_mutex);
}
//广播转速值
void ESCNode::send_esc_rpm_commands(const int32_t *cmd_array, uint8_t len)
{
    struct uavcan_equipment_esc_CubeRPMCommand msg = {0};

    // 长度保护
    if (len > Max_ESC_Num) len = Max_ESC_Num;

    msg.rpm.len = len + 1;
    msg.arm = 1;

    for (uint8_t i = 0; i < len; i++)
    {
        int32_t rpm = cmd_array[i];

        // 限幅
        if (rpm >  8000) rpm = 8000;
        if (rpm < -8000) rpm = -8000;

        msg.rpm.data[i] = rpm;
    }

    uint8_t buffer[64];
    uint32_t size = uavcan_equipment_esc_CubeRPMCommand_encode(&msg, buffer);

    if (size == 0) return;

    osMutexAcquire(m_send_mutex, osWaitForever);

    canardBroadcast(&canard_,
                    UAVCAN_EQUIPMENT_ESC_CUBERPMCOMMAND_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_CUBERPMCOMMAND_ID,
                    &esc_rpm_commmand_transfer_id_,
                    CANARD_TRANSFER_PRIORITY_HIGH,
                    buffer,
                    size);

    esc_rpm_commmand_transfer_id_++;

    osMutexRelease(m_send_mutex);
}

//获取电调状态
bool ESCNode::get_esc_status(uint8_t esc_index, ESCStatusCache& out)
{	
	if (esc_index >= Max_ESC_Num)
      return false;
	
	if(osMutexAcquire(m_esc_get_staus_mutex, 0) == osOK)
	{
	out = esc_status_[esc_index];
	
	osMutexRelease(m_esc_get_staus_mutex);
	}


	
    return true;
	
}


#include "can.h"
#include "motor.h"

// 发送一帧标准帧
void CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len)
{

	CAN_TxHeaderTypeDef tx_header = {0};
	uint32_t tx_mailbox;

	tx_header.StdId = id;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = len;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &tx_mailbox);

}

// CAN2 接收滤波器
void CAN_Filter_init(void)
{
	CAN_FilterTypeDef filter;

	filter.SlaveStartFilterBank  = 14;                    // CAN2 从第 14 组起
	filter.FilterBank            = 14;                    // 配置第 14 组
	filter.FilterMode            = CAN_FILTERMODE_IDMASK; // 掩码模式
	filter.FilterScale           = CAN_FILTERSCALE_32BIT; // 32 位

	filter.FilterIdHigh          = 0x0000;
	filter.FilterIdLow           = 0x0000;
	filter.FilterMaskIdHigh      = 0x0000;                // 掩码全 0，先全放行
	filter.FilterMaskIdLow       = 0x0000;

	filter.FilterFIFOAssignment  = CAN_FILTER_FIFO0;
	filter.FilterActivation      = CAN_FILTER_ENABLE;

	if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
	{
		Error_Handler();
	}
}

// 启动 CAN2 并开启接收中断
void CAN_Start(void)
{
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// 接收中断回调：只认 Motor_1 的反馈 ID，解析工作交给 get_motor_value
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
		return;

	if (rx_header.StdId != Motor_1_ID)
		return;

	get_motor_value(rx_data); //把接收到的数据传递给电机值解析函数
}

// 解析 RM3508 反馈报文（8 字节大端）：
//   [0..1] 机械角度 0~8191  [2..3] 转速 rpm  [4..5] 实际转矩电流
// 角度是单圈值，所以这里维护 PosPre/PosNow 两帧历史，
// 用增量累加到 PositionMeasure 上得到多圈累计角度。
void get_motor_value(uint8_t *rxData)
{
	// 转速原始单位是 rpm，除以 19 换算到输出轴转速
	Motor_1.CurrentMeasure = (float)(short)((rxData[4] << 8) | rxData[5]);
	Motor_1.SpeedMeasure   = (float)(short)((rxData[2] << 8) | rxData[3]) / 19.0f;

	// 首帧只做初始化，此时没有上一帧可比较，增量记 0
	if (Motor_1.PosPre == 0 && Motor_1.PosNow == 0)
	{
		Motor_1.PosPre = Motor_1.PosNow = (short)((rxData[0] << 8) | rxData[1]);
	}
	else
	{
		Motor_1.PosPre = Motor_1.PosNow;
		Motor_1.PosNow = (short)((rxData[0] << 8) | rxData[1]);
	}

	Motor_1.PositionMeasure += Get_RM3508_Distance(Motor_1);
}

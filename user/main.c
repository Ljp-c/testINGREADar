#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stdint.h>

#define radar_ADDRESS 0x57
#define radar_WREG 0xAE
#define radar_RREG 0xAF

static uint8_t radar_REGS[3]
={radar_ADDRESS ,
 radar_WREG,
 radar_RREG };

static uint8_t radar_wREG=0x01;

typedef struct 
{
	uint8_t BYte_H;
	uint8_t BYte_M;
	uint8_t BYte_L;
}radar_data ;

static radar_data radarData;

static uint32_t distance;

typedef enum
{
	I2C_STATUS_OK = 0,
	I2C_STATUS_BUSY,
	I2C_STATUS_START_TIMEOUT,
	I2C_STATUS_ADDR_NACK,
	I2C_STATUS_ADDR_TIMEOUT
} I2C_Status;

#define I2C_WAIT_TIMEOUT 60000U

/**
  * @brief  微秒级延迟
  */
void Delay_us(uint32_t us)
{
	SysTick->LOAD = 72 * us;				  // 设置重装载值 (HCLK=72MHz)
	SysTick->VAL = 0x00;					  // 清空计数器
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk; // 使能计数器
	while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // 等待计数完成
	SysTick->CTRL = 0x00;					 // 关闭计数器
}

/**
  * @brief  毫秒级延迟
  */
void Delay_ms(uint32_t ms)
{
	while (ms--)
	{
		Delay_us(1000);
	}
}

//gpio配置
void GPIO_INitation(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// 启用 GPIOB 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	// PB10 (Trig): 推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// PB11 (Echo): 下拉输入 (或浮空输入)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 初始状态拉低 Trig
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
}

/*
void I2C_Initation(void)
{
	I2C_InitTypeDef I2C_InitStructure;
	
	// 首先复位 I2C2 以确保状态清洁
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C2, &I2C_InitStructure);

	I2C_Cmd(I2C2, ENABLE);
}

void I2c_sendMEssage(uint8_t *slaveAddress,	uint8_t *rEGdata , uint8_t* data, uint16_t size)
{
while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET);

I2C_GenerateSTART(I2C2, ENABLE);
while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(I2C2, (*slaveAddress) << 1, I2C_Direction_Transmitter);
while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//I2c已经认定是主动发送模式

I2C_SendData(I2C2, *rEGdata);
while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

for (uint16_t i = 0; i < size; i++)
{
I2C_SendData(I2C2, *(data+i));
while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));	
}
I2C_GenerateSTOP(I2C2, ENABLE);

}

void I2creceivemessage(uint8_t *slaveAddress, uint8_t *rEGdata, uint8_t* datarevive, uint16_t size)
{
while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET);

I2C_GenerateSTART(I2C2, ENABLE);
while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(I2C2, (*slaveAddress) << 1, I2C_Direction_Transmitter);
while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//I2c已经认定是主动发送模式

I2C_SendData(I2C2, *(rEGdata));
while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));	

I2C_GenerateSTART(I2C2, ENABLE);
while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(I2C2, (*slaveAddress) << 1, I2C_Direction_Receiver);
while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//I2c已经认定是主动接收模式

for(uint16_t i = 0; i < size; i++)
{
if(i==size-1)
{
I2C_AcknowledgeConfig(I2C2, DISABLE);
}
while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
*(datarevive+i) = I2C_ReceiveData(I2C2);
}
I2C_GenerateSTOP(I2C2, ENABLE);

I2C_AcknowledgeConfig(I2C2, ENABLE);

}
//还可以使用I2c_registerread和I2c_registerwrite函数来简化代码
//现在基本完成函数封装 明天实现实战芯片读取 
void read_radar(void)
{
I2c_sendMEssage(radar_REGS, radar_REGS+1, &radar_wREG, 1);
I2creceivemessage(radar_REGS, radar_REGS+2, (uint8_t *)&radarData, sizeof(radarData));
}
*/

void GPIO_Read_Radar(void)
{
	uint32_t count = 0;
	
	// 1. 发送 15us 触发信号 (PB10 = Trig)
	GPIO_SetBits(GPIOB, GPIO_Pin_10);
	Delay_us(15);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);
	
	// 2. 等待 Echo (PB11) 变高，设置超时防止卡死
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
	{
		count++;
		if (count > 2000000) return; // 超时退出
	}
	
	// 3. 开始计时 (利用 SysTick 测量脉宽)
	SysTick->LOAD = 0xFFFFFF; // 设置最大重装载值
	SysTick->VAL = 0x00;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
	
	// 4. 等待 Echo 变低
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 1);
	
	// 5. 停止计时并获取计数值
	SysTick->CTRL = 0x00;
	uint32_t ticks = 0xFFFFFF - SysTick->VAL;
	
	// 6. 计算距离 (T = ticks/72 MHz)
	// Distance = (T * 34000) / 2  (单位为 cm)
	// Distance = (ticks / 72 * 0.034) / 2
	distance = (uint32_t)((float)ticks / 72.0 * 0.034 / 2.0);
}

void data_process(void)
{
	// distance 已经在 GPIO_Read_Radar 中直接计算
}

/*
static I2C_Status I2C_CheckRadarStatus(void)
{
	uint32_t timeout = I2C_WAIT_TIMEOUT;

	// 等待 I2C2 闲置
	while ((I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == SET) && timeout--);
	if (timeout == 0) return I2C_STATUS_BUSY;

	// 生成起始信号
	I2C_GenerateSTART(I2C2, ENABLE);
	timeout = I2C_WAIT_TIMEOUT;
	while ((!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) && timeout--);
	if (timeout == 0) {
		I2C_GenerateSTOP(I2C2, ENABLE);
		return I2C_STATUS_START_TIMEOUT;
	}

	// 发送目标从机地址，这里要左移 1 位，或者 radar_ADDRESS 已经包含最低位方向。
	// 注意：radar_ADDRESS 定义成了 0x57，一般是 7 位地址。I2C_Send7bitAddress 会自动处理最低位。
	I2C_Send7bitAddress(I2C2, radar_ADDRESS << 1, I2C_Direction_Transmitter);
	
	timeout = I2C_WAIT_TIMEOUT;
	while (timeout--) {
		// 检查 ADDR 是否成功 (EVENT_MASTER_TRANSMITTER_MODE_SELECTED 包含 ADDR 置位)
		if (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			// 一旦确认地址被 ACK，立即发送停止信号，结束这个检查过程。
			I2C_GenerateSTOP(I2C2, ENABLE);
			return I2C_STATUS_OK;
		}
		// 检查 NACK (AF 标志)
		if (I2C_GetFlagStatus(I2C2, I2C_FLAG_AF) == SET) {
			I2C_ClearFlag(I2C2, I2C_FLAG_AF);
			I2C_GenerateSTOP(I2C2, ENABLE);
			return I2C_STATUS_ADDR_NACK;
		}
	}

	I2C_GenerateSTOP(I2C2, ENABLE);
	return I2C_STATUS_ADDR_TIMEOUT;
}

static void OLED_ShowI2CStatus(I2C_Status status)
{
	OLED_ShowString(1, 1, "I2C2 RADAR:");

	switch (status)
	{
	case I2C_STATUS_OK:
		OLED_ShowString(1, 12, "OK   ");
		break;
	case I2C_STATUS_BUSY:
		OLED_ShowString(1, 12, "BUSY ");
		break;
	case I2C_STATUS_START_TIMEOUT:
		OLED_ShowString(1, 12, "STTO ");
		break;
	case I2C_STATUS_ADDR_NACK:
		OLED_ShowString(1, 12, "NACK ");
		break;
	default:
		OLED_ShowString(1, 12, "ATMO ");
		break;
	}
	OLED_ShowString(4, 1, "S1:");
	OLED_ShowHexNum(4, 4, I2C2->SR1, 4);
	OLED_ShowString(4, 9, "S2:");
	OLED_ShowHexNum(4, 12, I2C2->SR2, 4);
}
*/

int main(void)
{
	GPIO_INitation();
	// I2C_Initation(); // 已注释
	OLED_Init();
	OLED_Clear();
	
	OLED_ShowString(1, 1, "Radar GPIO Mode");
	
	while(1)
	{
		GPIO_Read_Radar(); // 测量距离
		
		OLED_ShowString(2, 1, "Dist: ");
		OLED_ShowNum(2, 7, distance, 5);
		OLED_ShowString(2, 12, " cm");
		
		// 适当延时，防止刷新过快
		Delay_ms(100);
	}
}

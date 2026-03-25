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
#define RADAR_TIMEOUT_US 30000U
#define ENABLE_LOOPBACK_TEST 0U  // 1: 使能PA1->PA0环回测试; 0: 正常测距

#define US_GPIO_PORT GPIOA
#define US_GPIO_CLK RCC_APB2Periph_GPIOA
#define US_TRIG_PIN GPIO_Pin_1
#define US_ECHO_PIN GPIO_Pin_0

static uint8_t radar_measure_ok;
static uint32_t pulse_width_us;
static uint8_t pin_check_ok;
static uint8_t loopback_ok;
static uint8_t loopback_err;
static uint32_t g_sysclk_mhz = 72U;

typedef enum
{
	RADAR_STATUS_OK = 0,
	RADAR_STATUS_WAIT_HIGH_TIMEOUT,
	RADAR_STATUS_WAIT_LOW_TIMEOUT
} RadarStatus;

static RadarStatus g_radar_status = RADAR_STATUS_WAIT_HIGH_TIMEOUT;

/**
  * @brief  微秒级延迟
  */
void Delay_us(uint32_t us)
{
	if (us == 0)
	{
		return;
	}

	SysTick->LOAD = (g_sysclk_mhz * us) - 1U;	  // 设置重装载值
	SysTick->VAL = 0x00;					  // 清空计数器
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // HCLK,使能计数器
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
	
	// 启用 GPIOA 时钟
	RCC_APB2PeriphClockCmd(US_GPIO_CLK, ENABLE);
	
	// PA1 (Trig): 推挽输出
	GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);
	
	// PA0 (Echo): 正常测距用浮空输入；环回测试用下拉输入稳定基线
	GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
	#if ENABLE_LOOPBACK_TEST
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	#else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	#endif
	GPIO_Init(US_GPIO_PORT, &GPIO_InitStructure);

	// 初始状态拉低 Trig
	GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
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
	uint32_t timeout_ticks = RADAR_TIMEOUT_US * g_sysclk_mhz;
	uint32_t ticks;
	
	// 1. 发送 15us 触发信号 (PA1 = Trig)
	GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
	Delay_us(2);
	GPIO_SetBits(US_GPIO_PORT, US_TRIG_PIN);
	Delay_us(15);
	GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
	
	// 2. 等待 Echo (PA0) 变高，带超时防止卡死
	while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == 0)
	{
		if (timeout_ticks-- == 0)
		{
			radar_measure_ok = 0;
			g_radar_status = RADAR_STATUS_WAIT_HIGH_TIMEOUT;
			distance = 0;
			pulse_width_us = 0;
			return;
		}
	}
	
	// 3. 开始计时 (利用 SysTick 测量脉宽)
	SysTick->LOAD = 0x00FFFFFF; // 设置最大重装载值
	SysTick->VAL = 0x00;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	
	// 4. 等待 Echo 变低，带超时防止卡死
	timeout_ticks = RADAR_TIMEOUT_US * g_sysclk_mhz;
	while (GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == 1)
	{
		if (timeout_ticks-- == 0)
		{
			SysTick->CTRL = 0x00;
			radar_measure_ok = 0;
			g_radar_status = RADAR_STATUS_WAIT_LOW_TIMEOUT;
			distance = 0;
			pulse_width_us = 0;
			return;
		}
	}
	
	// 5. 停止计时并获取计数值
	SysTick->CTRL = 0x00;
	ticks = 0x00FFFFFF - SysTick->VAL;
	pulse_width_us = ticks / g_sysclk_mhz;
	
	// 6. 计算距离，单位 cm: distance = pulse_us * 0.0343 / 2
	distance = (pulse_width_us * 1715U + 50000U) / 100000U;
	radar_measure_ok = 1;
	g_radar_status = RADAR_STATUS_OK;
}

static uint8_t GPIO_PinSelfCheck(void)
{
	uint8_t ok = 1;
	
	// 检查 PA1(Trig) 输出是否可控
	GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
	Delay_us(2);
	if (GPIO_ReadOutputDataBit(US_GPIO_PORT, US_TRIG_PIN) != Bit_RESET)
	{
		ok = 0;
	}

	GPIO_SetBits(US_GPIO_PORT, US_TRIG_PIN);
	Delay_us(2);
	if (GPIO_ReadOutputDataBit(US_GPIO_PORT, US_TRIG_PIN) != Bit_SET)
	{
		ok = 0;
	}

	GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);

	// Echo 可能因模块状态或外部电路在空闲时高/低，
	// 这里只检查 Trig 口可控性，不把 Echo 静态电平作为 FAIL 条件。

	pin_check_ok = ok;
	return ok;
}

static uint8_t GPIO_LoopbackTest(void)
{
	uint32_t timeout_ticks;
	loopback_err = 0;

	// 该测试需要将 PA1 和 PA0 用跳线短接
	GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
	Delay_us(2);

	// 基线应为低，避免在高电平状态下误判
	timeout_ticks = g_sysclk_mhz * 300U; // 300us
	while ((GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == Bit_SET) && timeout_ticks--);
	if (timeout_ticks == 0U)
	{
		loopback_ok = 0;
		loopback_err = 1; // E1: Echo 基线无法回到低电平
		return 0;
	}

	// 输出一个短高电平，检查 PA0 能否同步感知到
	GPIO_SetBits(US_GPIO_PORT, US_TRIG_PIN);
	timeout_ticks = g_sysclk_mhz * 300U; // 最多等待约300us
	while ((GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == Bit_RESET) && timeout_ticks--);
	if (timeout_ticks == 0U)
	{
		GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);
		loopback_ok = 0;
		loopback_err = 2; // E2: Trig 拉高后 Echo 未检测到高电平
		return 0;
	}

	GPIO_ResetBits(US_GPIO_PORT, US_TRIG_PIN);

	// 拉低后 PA0 应回到低电平
	timeout_ticks = g_sysclk_mhz * 300U;
	while ((GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN) == Bit_SET) && timeout_ticks--);
	if (timeout_ticks == 0U)
	{
		loopback_ok = 0;
		loopback_err = 3; // E3: Trig 拉低后 Echo 未回落
		return 0;
	}

	loopback_ok = 1;
	loopback_err = 0;
	return 1;
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
	SystemCoreClockUpdate();
	if (SystemCoreClock >= 1000000U)
	{
		g_sysclk_mhz = SystemCoreClock / 1000000U;
	}

	GPIO_INitation();
	// I2C_Initation(); // 已注释
	OLED_Init();
	OLED_Clear();
	
	OLED_ShowString(1, 1, "Radar GPIO");
	
	while(1)
	{
		GPIO_PinSelfCheck();

#if ENABLE_LOOPBACK_TEST
		GPIO_LoopbackTest();
		g_radar_status = RADAR_STATUS_OK;
		distance = 0;
		pulse_width_us = 0;
#else
		GPIO_Read_Radar(); // 测量距离
#endif
		
		OLED_ShowString(2, 1, "Dist: ");
		OLED_ShowNum(2, 7, distance, 5);
		OLED_ShowString(2, 12, " cm");

		OLED_ShowString(3, 1, "St:");
		if (g_radar_status == RADAR_STATUS_OK)
		{
			OLED_ShowString(3, 4, "OK ");
		}
		else if (g_radar_status == RADAR_STATUS_WAIT_HIGH_TIMEOUT)
		{
			OLED_ShowString(3, 4, "THI");
		}
		else
		{
			OLED_ShowString(3, 4, "TLO");
		}

		OLED_ShowString(3, 8, "E:");
		OLED_ShowNum(3, 10, GPIO_ReadInputDataBit(US_GPIO_PORT, US_ECHO_PIN), 1);

		OLED_ShowString(4, 1, "Pin:");
		if (pin_check_ok)
		{
			OLED_ShowString(4, 5, "OK  ");
		}
		else
		{
			OLED_ShowString(4, 5, "FAIL");
		}

		OLED_ShowString(4, 10, "U:");
		OLED_ShowNum(4, 12, pulse_width_us, 5);

		OLED_ShowString(1, 11, "L:");
#if ENABLE_LOOPBACK_TEST
		if (loopback_ok)
		{
			OLED_ShowString(1, 13, "OK");
		}
		else
		{
			if (loopback_err == 1)
			{
				OLED_ShowString(1, 13, "E1");
			}
			else if (loopback_err == 2)
			{
				OLED_ShowString(1, 13, "E2");
			}
			else
			{
				OLED_ShowString(1, 13, "E3");
			}
		}
#else
		OLED_ShowString(1, 13, "--");
#endif
		
		// 适当延时，防止刷新过快
		Delay_ms(250);
	}
}

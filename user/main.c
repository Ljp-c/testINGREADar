#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
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

//gpio配置
void GPIO_INitation(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);			
RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);		
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void I2C_Initation(void)
{

I2C_InitTypeDef I2C_InitStructure;
I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
I2C_InitStructure.I2C_OwnAddress1 = 0x00;
I2C_InitStructure.I2C_ClockSpeed = 100000;
I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
I2C_Init(I2C1, &I2C_InitStructure);

I2C_Cmd(I2C1, ENABLE);

}

void I2c_sendMEssage(uint8_t *slaveAddress,	uint8_t *rEGdata , uint8_t* data, uint16_t size)
{
while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));

I2C_GenerateSTART(I2C1, ENABLE);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(I2C1, *slaveAddress, I2C_Direction_Transmitter);
while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//I2c已经认定是主动发送模式

I2C_SendData(I2C1, *rEGdata);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

for (uint16_t i = 0; i < size; i++)
{
I2C_SendData(I2C1, *(data+i));
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));	
}
I2C_GenerateSTOP(I2C1, ENABLE);

}

void I2creceivemessage(uint8_t *slaveAddress, uint8_t *rEGdata, uint8_t* datarevive, uint16_t size)
{
while(!I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));

I2C_GenerateSTART(I2C1, ENABLE);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(I2C1, *slaveAddress, I2C_Direction_Transmitter);
while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//I2c已经认定是主动发送模式

I2C_SendData(I2C1, *(rEGdata));
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));	

I2C_GenerateSTART(I2C1, ENABLE);
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

I2C_Send7bitAddress(I2C1, *slaveAddress, I2C_Direction_Receiver);
while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));//I2c已经认定是主动接收模式

for(uint16_t i = 0; i < size; i++)
{
if(i==size-1)
{
I2C_AcknowledgeConfig(I2C1, DISABLE);
}
while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
*(datarevive+i) = I2C_ReceiveData(I2C1);
}
I2C_GenerateSTOP(I2C1, ENABLE);

I2C_AcknowledgeConfig(I2C1, ENABLE);

}
//还可以使用I2c_registerread和I2c_registerwrite函数来简化代码
//现在基本完成函数封装 明天实现实战芯片读取 
void read_radar(void)
{
I2c_sendMEssage(radar_REGS, radar_REGS+1, &radar_wREG, 1);
I2creceivemessage(radar_REGS, radar_REGS+2, (uint8_t *)&radarData, sizeof(radarData));
}

void data_process(void)
{
	distance = ((uint32_t)radarData.BYte_H << 16) | ((uint32_t)radarData.BYte_M << 8) | (uint32_t)radarData.BYte_L;
}

void main(void)
{
	GPIO_INitation();
	OLED_Init();
	OLED_ShowHexNum(1, 1, 0x12, 6);
	//开始写I2c硬件通信	
	while(1){
	I2C_Initation();
	read_radar();
	data_process();
	OLED_ShowHexNum(2, 1, distance, 6);
	}
}
#include "system_stm32f4xx.h"
#include "timer.h"
#include "eventman.h"
#include "led.h"
#include "melody.h"
#include "lightsensor.h"
#include "temhumsensor.h"
#include "eventbutton.h"
#include "button.h"
#include "buzzer.h"
#include "Ucglib.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32f401re_syscfg.h>
#include <stm32f401re_exti.h>
#include <stm32f401re_i2c.h>
#include <stm32f401re_spi.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_rcc.h>
#include <misc.h>

#define GPIO_PIN_LOW						0
#define GPIO_PIN_HIGH						1

#define CYCLE_SEND_DATA_1    				1000 // ms
#define CYCLE_SEND_DATA_2 					2000 // ms

/*Defined SPI **********************************************************************/

#define SPI1_CS_PORT           				 GPIOB
#define SPI1_CS_PIN            				 GPIO_Pin_6

#define SPI1_RST_PORT          				 GPIOC
#define SPI1_RST_PIN           				 GPIO_Pin_7

#define SPI1_MOSI_PORT         				 GPIOA
#define SPI1_MOSI_PIN          				 GPIO_Pin_7

#define SPI1_SCK_PORT          				 GPIOA
#define SPI1_SCK_PIN           				 GPIO_Pin_5

#define SPI1_RS_PORT           				 GPIOA
#define SPI1_RS_PIN            				 GPIO_Pin_9

#define SPI1_ENABLE_PORT       				 GPIOB
#define SPI1_ENABLE_PIN        				 GPIO_Pin_10

#define SPI1_MODE_PORT          			 GPIOA
#define SPI1_MODE_PIN           			 GPIO_Pin_8

/*Defined I2C ******************************************************************/

#define I2Cx_RCC							 RCC_APB1Periph_I2C1
#define I2Cx_SENSOR							 I2C1
#define I2C_GPIO_RCC		    			 RCC_AHB1Periph_GPIOB
#define I2C_GPIO							 GPIOB
#define I2C_PIN_SDA			    		 	 GPIO_Pin_9
#define I2C_PIN_SCL			    			 GPIO_Pin_8

/*Defined SENSOR ***************************************************************/

#define SI7020_ADDR                          0x40
#define CMDR_MEASURE_VALUE                   0xE0

typedef enum {
	EVENT_EMPTY,
	EVENT_APP_INIT,
	EVENT_APP_FLUSHMEM_READY,
	EVENT_BUTTON_PRESSED_5_TIMES,
} event_api_t, *event_api_p;

typedef enum {
	STATE_APP_STARTUP,
	STATE_APP_IDLE,
	STATE_APP_RESET,
} state_app_t;
state_app_t eCurrentState;

/*Privated Function ************************************************************/
void AppInitCommon(void);
static void SetStateApp(state_app_t state);
static state_app_t GetStateApp(void);
static void AppStateManager(uint8_t event);

static void LoadConfiguration(void);
void DeviceStateMachine(uint8_t event);
void IncreaseBrightness();
void DecreaseBrightness();
void MultiSensorScan();

static void SPI1_Init(void);

static void I2C_Init_temphumi(void);
void I2C_start(void);
void I2C_address_direction(uint8_t address, uint8_t direction);
void I2C_transmit(uint8_t byte);
void I2C_stop(void);
uint8_t I2C_receive_nack(void);
uint8_t I2C_receive_ack(void);

static void TemHumSensor_readRegister(
		uint8_t address,
	    uint8_t* pAddressRegister,
	    uint8_t* pDataRegister,
	    uint8_t Length_Data,
	    uint16_t delay);
uint32_t GetTemp_Sensor(void);
uint32_t GetHumi_Sensor(void);

void delay_ms(uint32_t ms);
uint32_t CalculatorTime(uint32_t dwTimeInit, uint32_t dwTimeCurrent);

/* Private variables ---------------------------------------------------------*/
int brightness = 0;
static ucg_t ucg;
static char srcTemp[20] = "";
static char srcHumi[20] = "";
static char srcLight[20] = "";
static SSwTimer idTimerScan = NO_TIMER;

int main(void)
{
	AppInitCommon();
	SetStateApp(STATE_APP_STARTUP);
	EventSchedulerAdd(EVENT_APP_INIT);
	while(1)
	{
		processTimerScheduler();
		processEventScheduler();
	}
}

void AppInitCommon(void) {
	SystemCoreClockUpdate();
	TimerInit();
	EventButton_Init();
	LedControl_Init();
	LightSensor_Init(ADC_READ_MODE_DMA);
	SPI1_Init();
	I2C_Init_temphumi();
	TemHumSensor_Init();
	BuzzerControl_Init();
	EventSchedulerInit(AppStateManager);
}

static state_app_t GetStateApp(void) {
	return eCurrentState;
}

static void SetStateApp(state_app_t state) {
	eCurrentState = state;
}

static void AppStateManager(uint8_t event) {
	switch(GetStateApp())
	{
		case STATE_APP_STARTUP:
			if(event == EVENT_APP_INIT)
			{
				LoadConfiguration();
				SetStateApp(STATE_APP_IDLE);
			}
			break;
		case STATE_APP_IDLE:
			DeviceStateMachine(event);
			break;
		case STATE_APP_RESET:
			break;
		default:
			break;
	}
}

static void LoadConfiguration(void) {
	Ucglib4WireSWSPI_begin(&ucg, UCG_FONT_MODE_SOLID);
	int width = ucg_GetWidth(&ucg);
	int heightRemain = (ucg_GetHeight(&ucg) - 3)/2;
	ucg_ClearScreen(&ucg);
	ucg_SetFont(&ucg, ucg_font_helvR08_tr);
	ucg_SetRotate180(&ucg);
	ucg_DrawString(&ucg, (width - ucg_GetStrWidth(&ucg, "IOT"))/2, heightRemain - 14, 0, "IOT");
	ucg_DrawString(&ucg, (width - ucg_GetStrWidth(&ucg, "Programming by"))/2, heightRemain, 0, "Programming by");
	ucg_DrawString(&ucg, (width - ucg_GetStrWidth(&ucg, "Lumi Smarthome"))/2, heightRemain + 14, 0, "Lumi Smarthome");
}

void IncreaseBrightness() {
    if(brightness <= 98) {
        brightness += 2;
        LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_GREEN, brightness);
        LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_GREEN, brightness);
    }
}

void DecreaseBrightness() {
    if(brightness >= 2) {
    	brightness -= 2;
        LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_GREEN, brightness);
        LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_GREEN, brightness);
    }
}

void MultiSensorScan() {
	uint8_t temp = (uint8_t) GetTemp_Sensor();
    uint8_t humi = (uint8_t) GetHumi_Sensor();
    uint32_t light = LightSensor_MeasureUseDMAMode();

    ucg_SetFont(&ucg, ucg_font_helvR10_hr);
    ucg_SetRotate180(&ucg);
    memset(srcTemp, 0, sizeof(srcTemp));
	sprintf(srcTemp, " Temp = %d oC", temp);
	ucg_DrawString(&ucg, 0, 52, 0, srcTemp);

	memset(srcHumi, 0, sizeof(srcHumi));
	sprintf(srcHumi, " Humi = %3d %%", humi);
	ucg_DrawString(&ucg, 0, 72, 0, srcHumi);

	memset(srcLight, 0, sizeof(srcLight));
	sprintf(srcLight, " Light = %lu lx", light);
	ucg_DrawString(&ucg, 0, 92, 0, srcLight);
}

void DeviceStateMachine(uint8_t event) {
	switch(event)
	{
		// Bai 2
		case EVENT_OF_BUTTON_3_PRESS_5_TIMES:
		{
			if (idTimerScan != NO_TIMER)
			{
				TimerStop(idTimerScan);
				idTimerScan = NO_TIMER;
			}
			int height = ucg_GetHeight(&ucg)/10;
			char *string[] = {	"Device:",
								"Board STM32 NUCLEO",
								"Code:",
								"STM32F401RE NUCLEO",
								"Manufacturer:",
								"STMicroelectronics",
								"Kit expansion:",
								"Lumi Smarthome",
								"Project:",
								"Simulator touch switch" };
			LedControl_BlinkStart(LED_ALL_ID, BLINK_GREEN, 10, 1000, GPIO_PIN_LOW);
			ucg_ClearScreen(&ucg);
			ucg_SetFont(&ucg, ucg_font_helvR08_tr);
			ucg_SetRotate180(&ucg);
			for(int i = 0; i < 5; i++) {
				int j = 2*i;
				ucg_DrawString(&ucg, 0, ((j+1) * height), 0, string[j]);
				ucg_DrawString(&ucg, 0, ((j+2) * height), 0, string[j + 1]);
			}
		}	break;

		// Bai 3
		case EVENT_OF_BUTTON_1_PRESS_LOGIC:
		{
			LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_RED, 50);
			LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_RED, 50);
			BuzzerControl_SetMelody(pbeep);
		}	break;
		case EVENT_OF_BUTTON_2_PRESS_LOGIC:
		{
			LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_GREEN, 50);
			LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_GREEN, 50);
			BuzzerControl_SetMelody(pbeep);
		}	break;
		case EVENT_OF_BUTTON_4_PRESS_LOGIC:
		{
			LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_BLUE, 50);
			LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_BLUE, 50);
			BuzzerControl_SetMelody(pbeep);
		}	break;
		case EVENT_OF_BUTTON_5_PRESS_LOGIC:
		{
			LedControl_SetColorGeneral(LED_KIT_ID0, LED_COLOR_WHITE, 50);
			LedControl_SetColorGeneral(LED_KIT_ID1, LED_COLOR_WHITE, 0);
			BuzzerControl_SetMelody(pbeep);
		}	break;

		//Bai 4
		case EVENT_OF_BUTTON_1_HOLD_1S:
		{
			idTimerScan = TimerStart("IncreaseBrightness", \
			                         200, \
									 TIMER_REPEAT_FOREVER, \
			                         IncreaseBrightness, \
			                         NULL);
		}	break;
		case EVENT_OF_BUTTON_5_HOLD_1S:
		{
			idTimerScan = TimerStart("DecreaseBrightness", \
									 200, \
									 TIMER_REPEAT_FOREVER, \
			                         DecreaseBrightness, \
			                         NULL);
		}	break;
		case EVENT_OF_BUTTON_1_RELEASED_1S:
		case EVENT_OF_BUTTON_5_RELEASED_1S:
		{
			if (idTimerScan != NO_TIMER)
			{
				TimerStop(idTimerScan);
				idTimerScan = NO_TIMER;
			}
		}	break;

		//Bai 5:
		case EVENT_OF_BUTTON_3_PRESS_2_TIMES:
		{
			ucg_ClearScreen(&ucg);
			MultiSensorScan();

		    if(idTimerScan != NO_TIMER) {
		        TimerStop(idTimerScan);
		        idTimerScan = NO_TIMER;
		    }
			idTimerScan = TimerStart("MultiSensorScan", \
									 2000, \
									 TIMER_REPEAT_FOREVER, \
									 MultiSensorScan, \
			                         NULL);
		}
		default:
			break;
	}
}

uint32_t CalculatorTime(uint32_t dwTimeInit, uint32_t dwTimeCurrent) {
	uint32_t dwTimeTotal;
	if (dwTimeCurrent >= dwTimeInit) {
		dwTimeTotal = dwTimeCurrent - dwTimeInit;
	} else {
		dwTimeTotal = 0xFFFFFFFFU + dwTimeCurrent - dwTimeInit;
	}
	return dwTimeTotal;

}

void delay_ms(uint32_t ms) {

	uint32_t startTime = GetMilSecTick();
	while (CalculatorTime(startTime, GetMilSecTick()) <= ms)
		;
}

static void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA, GPIOB and GPIOC Clocks enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_RS_PIN | SPI1_MODE_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI1_CS_PIN | SPI1_ENABLE_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI1_RST_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void I2C_Init_temphumi(void)
{
// Initialization struct
	I2C_InitTypeDef I2C_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	// Step 1: Initialize I2C
	RCC_APB1PeriphClockCmd(I2Cx_RCC, ENABLE);
	I2C_InitStruct.I2C_ClockSpeed = 400000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2Cx_SENSOR, &I2C_InitStruct);
	I2C_Cmd(I2Cx_SENSOR, ENABLE);

	// Step 2: Initialize GPIO as open drain alternate function
	RCC_AHB1PeriphClockCmd(I2C_GPIO_RCC, ENABLE);
	GPIO_InitStruct.GPIO_Pin = I2C_PIN_SCL | I2C_PIN_SDA;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(I2C_GPIO, &GPIO_InitStruct);

	/* Connect PXx to I2C_SCL */
	GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource8, GPIO_AF_I2C1);

	/* Connect PXx to I2C_SDA */
	GPIO_PinAFConfig(I2C_GPIO, GPIO_PinSource9, GPIO_AF_I2C1);
}

void I2C_start(void)
{
	// Wait until I2Cx is not busy anymore
	while (I2C_GetFlagStatus(I2Cx_SENSOR, I2C_FLAG_BUSY));

	// Generate start condition
	I2C_GenerateSTART(I2Cx_SENSOR, ENABLE);

	// Wait for I2C EV5.
	// It means that the start condition has been correctly released
	// on the I2C bus (the bus is free, no other devices is communicating))
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_MODE_SELECT));
}

void I2C_address_direction(uint8_t address, uint8_t direction)
{
	// Send slave address
	I2C_Send7bitAddress(I2Cx_SENSOR, address, direction);

	// Wait for I2C EV6
	// It means that a slave acknowledges his address
	if (direction == I2C_Direction_Transmitter)		// truyền
	{
		while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if (direction == I2C_Direction_Receiver)  // nhận
	{
		while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

void I2C_transmit(uint8_t byte)
{
	// Send data byte
	I2C_SendData(I2Cx_SENSOR, byte);
	// Wait for I2C EV8_2.
	// It means that the data has been physically shifted out and
	// output on the bus)
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

void I2C_stop(void)
{
	// Generate I2C stop condition
	I2C_GenerateSTOP(I2Cx_SENSOR, ENABLE);
}

uint8_t I2C_receive_nack(void)
{
	// Disable ACK of received data
	I2C_AcknowledgeConfig(I2Cx_SENSOR, DISABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_BYTE_RECEIVED));

	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2Cx_SENSOR);
}

uint8_t I2C_receive_ack(void)
{
	// Enable ACK of received data
	I2C_AcknowledgeConfig(I2Cx_SENSOR, ENABLE);
	// Wait for I2C EV7
	// It means that the data has been received in I2C data register
	while (!I2C_CheckEvent(I2Cx_SENSOR, I2C_EVENT_MASTER_BYTE_RECEIVED));

	// Read and return data byte from I2C data register
	return I2C_ReceiveData(I2Cx_SENSOR);
}

static void TemHumSensor_readRegister(
		uint8_t address,	// Địa chỉ cảm biến.
	    uint8_t* pAddressRegister, // Địa chỉ của thanh ghi chứa dữ liệu nhiệt độ, độ ẩm.
	    uint8_t* pDataRegister, // Dữ liệu đọc được từ thanh ghi tương ứng.
	    uint8_t Length_Data, //Độ dài Dữ liệu đọc được từ thanh ghi tương ứng.
	    uint16_t delay)	//Dữ liệu đọc được từ thanh ghi tương ứng.
{
	uint8_t LengthCmd = pAddressRegister[0];

	I2C_start();	// conditon start
	I2C_address_direction(address << 1, I2C_Direction_Transmitter); // send slave address - Transmit

	for (uint8_t i = 1; i < LengthCmd; i++) {
		I2C_transmit(pAddressRegister[i]);		// Send Data from register
	}

    if (delay > 0) {
        delay_ms(delay);
    }

	I2C_stop();// condition stop

	I2C_start(); // condition start
	I2C_address_direction(address << 1, I2C_Direction_Receiver); // send slave address - Received

	for (uint8_t i = 0; i < Length_Data; i++)
	{
		if (i == (Length_Data - 1))
		{
			pDataRegister[i] = I2C_receive_nack();  // 	NA
		}
		else
		{
			pDataRegister[i] = I2C_receive_ack();   // A
		}
	}
	I2C_stop();

}

uint32_t GetTemp_Sensor(void)
{
	uint32_t RT;
    uint8_t pRT[3] = { 0 };	// pRT[0]: MSB		pRT[1]: LSB

    uint8_t CMD_MEASURE_TEMP[2] =  { 2, 0xE3 }; // gửi độ dài byte cần truyền và CMD_MEASURE chế độ HOLD MASTER MODE

    TemHumSensor_readRegister(SI7020_ADDR, CMD_MEASURE_TEMP, pRT, 3, 4);

    RT = (pRT[0] << 8) + pRT[1];			// RT_CODE (MSB << 8) + LSB		// Temp = ((17572* RT)/(0xFFu +1) - 4685)/100
    RT = ((RT * 17572) >> 16) - 4685;
    RT = RT/100;
    return RT;
}

uint32_t GetHumi_Sensor(void)
{
	uint32_t RH;
	uint8_t pRH[3] = { 0 };


	uint8_t CMD_MEASURE_HUMI[2] =  { 2, 0xE5 }; // gửi độ dài byte cần truyền và CMD_MEASURE chế độ HOLD MASTER MODE

    TemHumSensor_readRegister(SI7020_ADDR, CMD_MEASURE_HUMI, pRH, 3, 8);

    RH = (pRH[0] << 8) + pRH[1]; // RH_CODE (MSB << 8) + LSB
    RH = ((RH * 12500) >> 16) - 600;
    RH = RH/100;

    return RH;
}

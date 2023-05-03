#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "radioenge_modem.h"
#include "main.h"

extern osTimerId_t PeriodicSendTimerHandle;
extern osThreadId_t AppSendTaskHandle;
extern ADC_HandleTypeDef hadc1;
extern osEventFlagsId_t ModemStatusFlagsHandle;
extern TIM_HandleTypeDef htim1, htim3;
extern osMessageQueueId_t setPWMDutyQueueHandle;

void LoRaWAN_RxEventCallback(uint8_t *data, uint32_t length, uint32_t port, int32_t rssi, int32_t snr)
{
    osMessageQueuePut(setPWMDutyQueueHandle, &data, 0U, 0U);
}

void PeriodicSendTimerCallback(void *argument)
{
    osThreadFlagsSet(AppSendTaskHandle, 0x01);
}

void AppSendTaskCode(void *argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    uint32_t read;
    TEMPERATURE_OBJ_t temp;
    uint32_t modemflags;
    osTimerStart(PeriodicSendTimerHandle, 30000U);
    temp.seq_no = 0;

    while (1)
    {
        LoRaWaitDutyCycle();
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        read = HAL_ADC_GetValue(&hadc1);
        temp.temp_oCx100 = (int32_t)(330 * ((float)read / 4096));
        LoRaSendBNow(2, (uint8_t *)&temp, sizeof(TEMPERATURE_OBJ_t));
        temp.seq_no++;
        osThreadFlagsClear(0x01);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    }
}

void setPWMDutyTaskCode(void *argument)
{
    AC_CONTROLLER_OBJ_t *data,pwmData;
    osStatus_t pwmEvent; 
    while(1)
    {
        pwmEvent = osMessageQueueGet(setPWMDutyQueueHandle, &data, NULL, osWaitForever);   // wait for message
        memcpy(&pwmData,data,sizeof(AC_CONTROLLER_OBJ_t));
        if (pwmEvent == osOK)
        {
            //memcpy(&ocPWM_data,&data,sizeof(AC_CONTROLLER_OBJ_t));
            htim1.Instance->CCR1 = (htim1.Instance->ARR*(pwmData.compressor_power))/100;
            htim3.Instance->CCR2 = (htim3.Instance->ARR*(pwmData.compressor_power))/100;
        }
    }
}
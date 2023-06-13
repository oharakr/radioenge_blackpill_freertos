#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "radioenge_modem.h"
#include "main.h"
#include <math.h>
#include <string.h>

extern osTimerId_t PeriodicSendTimerHandle;
extern osThreadId_t AppSendTaskHandle;
extern ADC_HandleTypeDef hadc1;
extern osEventFlagsId_t ModemStatusFlagsHandle;
extern TIM_HandleTypeDef htim1, htim3;
extern osMessageQueueId_t setPWMDutyQueueHandle;

AC_CONTROLLER_OBJ_t gLastPWMStatus;

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
    float R1 = 10000, logR2;
    float tempThermistor;
    const float c1 = 0.001129148;
    const float c2 = 0.000234125;
    const float c3 = 0.0000000876741;
    uint16_t mV;               // read voltage
    const uint16_t mV_KY_013 = 3300; // KY-013 power voltage

    uint32_t read;
    TEMPERATURE_OBJ_t temp;
    uint32_t modemflags;
    osTimerStart(PeriodicSendTimerHandle, 15000U);
    temp.seq_no = 0;
    while (1)
    {
        LoRaWaitDutyCycle();
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        read = HAL_ADC_GetValue(&hadc1);       

        mV = 3300 * read / 4095.0;
        logR2 = log(R1 * mV / (mV_KY_013 - mV));  // calculate resistance on thermistor               
        tempThermistor = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2)); // temperature in Kelvin
        tempThermistor = tempThermistor - 273.15; // convert Kelvin to Celsius
        temp.temp_oCx100 = (int32_t)(tempThermistor*100);
        temp.compressor_power = gLastPWMStatus.compressor_power;
        LoRaSendBNow(2, (uint8_t *)&temp, sizeof(TEMPERATURE_OBJ_t));
        temp.seq_no++;
        osThreadFlagsClear(0x01);
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    }
}

void HeaterPowerTaskCode(void *argument)
{

    while(1)
    {
        HAL_GPIO_WritePin(KIT_LED_GPIO_Port, KIT_LED_Pin,1);
        osDelay(gLastPWMStatus.compressor_power);
        HAL_GPIO_WritePin(KIT_LED_GPIO_Port, KIT_LED_Pin,0);
        osDelay(100-gLastPWMStatus.compressor_power);
    }
}

void setPWMDutyTaskCode(void *argument)
{
    AC_CONTROLLER_OBJ_t *data;
    osStatus_t pwmEvent;
    while (1)
    {
        pwmEvent = osMessageQueueGet(setPWMDutyQueueHandle, &data, NULL, osWaitForever); // wait for message
        memcpy(&gLastPWMStatus, &gLastPWMStatus, sizeof(AC_CONTROLLER_OBJ_t));
        if (pwmEvent == osOK)
        {            
            // memcpy(&ocPWM_data,&data,sizeof(AC_CONTROLLER_OBJ_t));
            htim1.Instance->CCR1 = (htim1.Instance->ARR * (gLastPWMStatus.compressor_power)) / 100;
            htim3.Instance->CCR2 = (htim3.Instance->ARR * (gLastPWMStatus.compressor_power)) / 100;
        }

    }
}
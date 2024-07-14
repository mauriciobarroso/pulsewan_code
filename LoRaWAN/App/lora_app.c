/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */
#include "shtc3.h"
#include "i2c.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern LPTIM_HandleTypeDef hlptim1;
//I2C_HandleTypeDef i2c2;
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * LEDs period value of the timer in ms
  */
#define LED_PERIOD_TIME 500

/**
  * Join switch period value of the timer in ms
  */
#define JOIN_TIME 2000

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */
#define PULSES_KWH_FACTOR	0.000625F
#define MULT_FACTOR				1000000
static const char *slotStrings[] = { "1", "2", "C", "C_MC", "P", "P_MC" };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief callback when LoRaWAN application has sent a frame
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRaWAN application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
  * @brief callback when LoRaWAN Beacon status is updated
  * @param params status of Last Beacon
  */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
  * @brief callback when system time has been updated
  */
static void OnSysTimeUpdate(void);

/**
  * @brief callback when LoRaWAN application Class is changed
  * @param deviceClass new class
  */
static void OnClassChange(DeviceClass_t deviceClass);

/**
  * @brief  LoRa store context in Non Volatile Memory
  */
static void StoreContext(void);

/**
  * @brief  stop current LoRa execution to switch into non default Activation mode
  */
static void StopJoin(void);

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */
static void OnStopJoinTimerEvent(void *context);

/**
  * @brief  Notifies the upper layer that the NVM context has changed
  * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
  */
static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
  * @brief  Store the NVM Data context to the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were stored
  */
static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * @brief  Restore the NVM Data context from the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were restored
  */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * Will be called each time a Radio IRQ is handled by the MAC layer
  *
  */
static void OnMacProcessNotify(void);

/**
  * @brief Change the periodicity of the uplink frames
  * @param periodicity uplink frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnTxPeriodicityChanged(uint32_t periodicity);

/**
  * @brief Change the confirmation control of the uplink frames
  * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
  * @note Compliance test protocol callbacks
  */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
  * @brief Change the periodicity of the ping slot frames
  * @param pingSlotPeriodicity ping slot frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/**
  * @brief Will be called to reset the system
  * @note Compliance test protocol callbacks
  */
static void OnSystemReset(void);

/* USER CODE BEGIN PFP */
/**
  * @brief  LED Tx timer callback function
  * @param  context ptr of LED context
  */
static void pulses_calc(void);

/**
  * @brief  LED Tx timer callback function
  * @param  context ptr of LED context
  */
static void on_pulses_counter_timer_event(void *context);

/**
  * @brief  LED Tx timer callback function
  * @param  context ptr of LED context
  */
static void OnTxTimerLedEvent(void *context);

/**
  * @brief  LED Join timer callback function
  * @param  context ptr of LED context
  */
static void OnJoinTimerLedEvent(void *context);

/**
  * @brief  LED Join timer callback function
  * @param  context ptr of LED context
  */
static void JoinProcess(void);

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */
static void OnJoinProcessTimerEvent(void *context);

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief LoRaWAN default activation type
  */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN force rejoin even if the NVM context is restored
  */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =              GetBatteryLevel,
  .GetTemperature =               GetTemperatureLevel,
  .GetUniqueId =                  GetUniqueId,
  .GetDevAddr =                   GetDevAddr,
  .OnRestoreContextRequest =      OnRestoreContextRequest,
  .OnStoreContextRequest =        OnStoreContextRequest,
  .OnMacProcess =                 OnMacProcessNotify,
  .OnNvmDataChange =              OnNvmDataChange,
  .OnJoinRequest =                OnJoinRequest,
  .OnTxData =                     OnTxData,
  .OnRxData =                     OnRxData,
  .OnBeaconStatusChange =         OnBeaconStatusChange,
  .OnSysTimeUpdate =              OnSysTimeUpdate,
  .OnClassChange =                OnClassChange,
  .OnTxPeriodicityChanged =       OnTxPeriodicityChanged,
  .OnTxFrameCtrlChanged =         OnTxFrameCtrlChanged,
  .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
  .OnSystemReset =                OnSystemReset,
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .TxPower =                  LORAWAN_DEFAULT_TX_POWER,
  .PingSlotPeriodicity =      LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
  .RxBCTimeout =              LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT
};

/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_EVENT;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/**
  * @brief Tx Timer period
  */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
  * @brief Join Timer period
  */
static UTIL_TIMER_Object_t StopJoinTimer;

/* USER CODE BEGIN PV */
/**
  * @brief User application buffer
  */
static uint16_t pulses_count = 0;
static uint16_t pulses_count_curr = 0;
static uint16_t pulses_count_prev = 0;
static uint32_t pulses_count_total = 0;
static uint32_t pulses_count_acum = 0;

/**
  * @brief Timer to handle the application Tx Led to toggle
  */
static UTIL_TIMER_Object_t pulses_counter_timer;

/**
  * @brief Timer to handle the application Tx Led to toggle
  */
static UTIL_TIMER_Object_t TxLedTimer;

/**
  * @brief Timer to handle the application Rx Led to toggle
  */
static UTIL_TIMER_Object_t JoinLedTimer;

/**
  * @brief Timer to handle the application Join Led to toggle
  */
static UTIL_TIMER_Object_t JoinProcessTimer;

/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_LV */
  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */
	UTIL_TIMER_Create(&TxLedTimer, 200, UTIL_TIMER_ONESHOT, OnTxTimerLedEvent, NULL);
	UTIL_TIMER_Create(&JoinLedTimer, 200, UTIL_TIMER_ONESHOT, OnJoinTimerLedEvent, NULL);
	UTIL_TIMER_Create(&pulses_counter_timer, 30000, UTIL_TIMER_PERIODIC, on_pulses_counter_timer_event, NULL);

  if (FLASH_IF_Init(NULL) != FLASH_IF_OK)
  {
    Error_Handler();
  }
  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT, OnStopJoinTimerEvent, NULL);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU, StopJoin);

  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  UTIL_TIMER_Start(&pulses_counter_timer);
  /* USER CODE END LoRaWAN_Init_2 */

  LmHandlerJoin(ActivationType, ForceRejoin);

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */
  UTIL_SEQ_RegTask(1 << CFG_SEQ_Task_JoinProcess, UTIL_SEQ_RFU, JoinProcess);
  UTIL_TIMER_Create(&JoinProcessTimer, 10000, UTIL_TIMER_PERIODIC, OnJoinProcessTimerEvent, NULL);
	UTIL_TIMER_Start(&JoinProcessTimer);
  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case  EWDT_WAKE_Pin:
      /* Note: when "EventType == TX_ON_TIMER" this GPIO is not initialized */
      if (EventType == TX_ON_EVENT)
      {
        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
      }
      break;
    default:
      break;
  }
}

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
	switch (appData->Port) {
		case LORAWAN_USER_APP_PORT:
			switch (appData->Buffer[0]) {
				case 'A':
					APP_LOG(TS_ON, VLEVEL_L, "First option\r\n");
					break;
				case 'B':
					APP_LOG(TS_ON, VLEVEL_L, "Second option\r\n");
					break;
				case 'C':
					APP_LOG(TS_ON, VLEVEL_L, "Third option\r\n");
					break;
				default:
					APP_LOG(TS_ON, VLEVEL_L, "%Invalid data\r\n");
			}
		default:
			break;
	}
  /* USER CODE END OnRxData_1 */
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
  /* Reset TPL5010 counter */
  HAL_GPIO_WritePin(EWDT_DONE_GPIO_Port, EWDT_DONE_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(EWDT_DONE_GPIO_Port, EWDT_DONE_Pin, GPIO_PIN_RESET);

	LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
	uint8_t battery = GetBatteryLevel();
	sensor_t sensor_data;
	UTIL_TIMER_Time_t nextTxIn = 0;

	if (LmHandlerIsBusy() == false)
	{
		pulses_calc();

		/* Calculate kwh with the pulses not transmitted */
		uint32_t kwh = pulses_count_acum * PULSES_KWH_FACTOR * MULT_FACTOR;
		int32_t temperature = 0;
    int32_t humidity = 0;
		uint32_t i = 0;
		int32_t latitude = 0;
		int32_t longitude = 0;
		uint16_t altitudeGps = 0;

		/* Read temperature and humidity values from SHTC3 */
	  MX_I2C2_Init();
	  HAL_I2C_MspInit(&hi2c2);

	  if (shtc3_perform_measurements(&hi2c2, &temperature, &humidity)) {
	  	APP_LOG(TS_OFF, VLEVEL_M, "Temperature: %d\tHumidity: %d\r\n",
	  			temperature, humidity);
	  }

	  shtc3_sleep(&hi2c2);

	  /* Read internal temperature and battery values */
		EnvSensors_Read(&sensor_data);
		APP_LOG(TS_OFF, VLEVEL_M, "VDDA: %d\tCPU temperature: %d\r\n",
				battery, (int16_t)(sensor_data.temperature));

		AppData.Port = LORAWAN_USER_APP_PORT;

		/* Set LoRaWAN payload buffer */
		AppData.Buffer[i++] = (uint8_t)((temperature >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((temperature >> 0) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((humidity >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((humidity >> 0) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((kwh >> 24) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((kwh >> 16) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((kwh >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((kwh >> 0) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((MULT_FACTOR >> 24) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((MULT_FACTOR >> 16) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((MULT_FACTOR >> 8) & 0xFF);
		AppData.Buffer[i++] = (uint8_t)((MULT_FACTOR >> 0) & 0xFF);
		AppData.Buffer[i++] = battery;        /* 1 (very low) to 254 (fully charged) */

		if((LmHandlerParams.ActiveRegion == LORAMAC_REGION_US915) ||
				(LmHandlerParams.ActiveRegion == LORAMAC_REGION_AU915) ||
				(LmHandlerParams.ActiveRegion == LORAMAC_REGION_AS923)) {
			AppData.Buffer[i++] = 0;
			AppData.Buffer[i++] = 0;
			AppData.Buffer[i++] = 0;
			AppData.Buffer[i++] = 0;
		}
		else {
			latitude = sensor_data.latitude;
			longitude = sensor_data.longitude;

			AppData.Buffer[i++] = (uint8_t)((latitude >> 16) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((latitude >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(latitude & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((longitude >> 16) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((longitude >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(longitude & 0xFF);
			AppData.Buffer[i++] = (uint8_t)((altitudeGps >> 8) & 0xFF);
			AppData.Buffer[i++] = (uint8_t)(altitudeGps & 0xFF);
		}

		AppData.BufferSize = i;

    if ((JoinLedTimer.IsRunning) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
    {
      UTIL_TIMER_Stop(&JoinLedTimer);
      HAL_GPIO_WritePin(JOIN_LED_GPIO_Port, JOIN_LED_Pin, GPIO_PIN_RESET);
    }

		status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
		if (LORAMAC_HANDLER_SUCCESS == status)
		{
			APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");

			/* Clear the pulses counter variables */
			pulses_count_acum = 0;
			pulses_count = 0;
		}
		else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
		{
			nextTxIn = LmHandlerGetDutyCycleWaitTime();
			if (nextTxIn > 0)
			{
				APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
			}
		}
	}

	if (EventType == TX_ON_TIMER)
	{
		UTIL_TIMER_Stop(&TxTimer);
		UTIL_TIMER_SetPeriod(&TxTimer, MAX(nextTxIn, TxPeriodicity));
		UTIL_TIMER_Start(&TxTimer);
	}
  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */
static void pulses_calc(void) {
	pulses_count_curr = HAL_LPTIM_ReadCounter(&hlptim1);

	/* Calculate the pulses counted between transmissions */
	if (pulses_count_curr < pulses_count_prev) {
		pulses_count = 65536 - pulses_count_prev + pulses_count_curr;
	}
	else {
		pulses_count = pulses_count_curr - pulses_count_prev;
	}

	/* Accumulate the pulses not transmitted */
	pulses_count_acum += pulses_count;
	APP_LOG(TS_OFF, VLEVEL_M, "Pulses counter accumulate: %d\r\n", pulses_count_acum);

	/* Accumulate the pulses */
	pulses_count_total += pulses_count;
	APP_LOG(TS_OFF, VLEVEL_M, "Pulses counter: %d\r\n", pulses_count);

	/* Update the previous pulses counted */
	APP_LOG(TS_OFF, VLEVEL_M, "Pulses counter total: %d\r\n", pulses_count_total);
	pulses_count_prev = pulses_count_curr;
}

static void on_pulses_counter_timer_event(void *context)
{
	pulses_calc();
}

static void OnTxTimerLedEvent(void *context)
{
	HAL_GPIO_WritePin(PULSES_LED_GPIO_Port, PULSES_LED_Pin, GPIO_PIN_RESET);
}

static void OnJoinTimerLedEvent(void *context)
{
	HAL_GPIO_WritePin(JOIN_LED_GPIO_Port, JOIN_LED_Pin, GPIO_PIN_RESET);
}

static void JoinProcess(void) {
  HAL_GPIO_WritePin(JOIN_LED_GPIO_Port, JOIN_LED_Pin, GPIO_PIN_SET);
  UTIL_TIMER_Start(&JoinLedTimer);

  APP_LOG(TS_OFF, VLEVEL_M, "Restarting join process...\r\n");
  LmHandlerJoin(ActivationType, ForceRejoin);
}

static void OnJoinProcessTimerEvent(void *context) {
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_JoinProcess), CFG_SEQ_Prio_0);
}
/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  if ((params != NULL))
  {
    /* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
    if (params->IsMcpsConfirm != 0)
    {
    	HAL_GPIO_WritePin(PULSES_LED_GPIO_Port, PULSES_LED_Pin, GPIO_PIN_SET);
    	UTIL_TIMER_Start(&TxLedTimer);

      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
      APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
              params->AppData.Port, params->Datarate, params->TxPower);

      APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
      if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
      {
        APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
      }
    }
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  if (joinParams != NULL)
  {
    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
    {
    	UTIL_TIMER_Stop(&JoinProcessTimer);
      UTIL_TIMER_Stop(&JoinLedTimer);
      HAL_GPIO_WritePin(JOIN_LED_GPIO_Port, JOIN_LED_Pin, GPIO_PIN_RESET); /* LED_RED */

      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
      {
        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
      }

      /* Store context */
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
    }

    APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:JOIN | DR:%d | PWR:%d\r\n", joinParams->Datarate, joinParams->TxPower);
  }
  /* USER CODE END OnJoinRequest_1 */
}

static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params)
{
  /* USER CODE BEGIN OnBeaconStatusChange_1 */
  /* USER CODE END OnBeaconStatusChange_1 */
}

static void OnSysTimeUpdate(void)
{
  /* USER CODE BEGIN OnSysTimeUpdate_1 */

  /* USER CODE END OnSysTimeUpdate_1 */
}

static void OnClassChange(DeviceClass_t deviceClass)
{
  /* USER CODE BEGIN OnClassChange_1 */
  /* USER CODE END OnClassChange_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

static void OnTxPeriodicityChanged(uint32_t periodicity)
{
  /* USER CODE BEGIN OnTxPeriodicityChanged_1 */

  /* USER CODE END OnTxPeriodicityChanged_1 */
  TxPeriodicity = periodicity;

  if (TxPeriodicity == 0)
  {
    /* Revert to application default periodicity */
    TxPeriodicity = APP_TX_DUTYCYCLE;
  }

  /* Update timer periodicity */
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, TxPeriodicity);
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxPeriodicityChanged_2 */

  /* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
  /* USER CODE BEGIN OnTxFrameCtrlChanged_1 */

  /* USER CODE END OnTxFrameCtrlChanged_1 */
  LmHandlerParams.IsTxConfirmed = isTxConfirmed;
  /* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

  /* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */

  /* USER CODE END OnPingSlotPeriodicityChanged_1 */
  LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

  /* USER CODE END OnPingSlotPeriodicityChanged_2 */
}

static void OnSystemReset(void)
{
  /* USER CODE BEGIN OnSystemReset_1 */

  /* USER CODE END OnSystemReset_1 */
  if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt()) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
  {
    NVIC_SystemReset();
  }
  /* USER CODE BEGIN OnSystemReset_Last */

  /* USER CODE END OnSystemReset_Last */
}

static void StopJoin(void)
{
  /* USER CODE BEGIN StopJoin_1 */

  /* USER CODE END StopJoin_1 */

  UTIL_TIMER_Stop(&TxTimer);

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop())
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
    if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP)
    {
      ActivationType = ACTIVATION_TYPE_OTAA;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
    }
    else
    {
      ActivationType = ACTIVATION_TYPE_ABP;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
    }
    LmHandlerConfigure(&LmHandlerParams);
    LmHandlerJoin(ActivationType, true);
    UTIL_TIMER_Start(&TxTimer);
  }
  UTIL_TIMER_Start(&StopJoinTimer);
  /* USER CODE BEGIN StopJoin_Last */

  /* USER CODE END StopJoin_Last */
}

static void OnStopJoinTimerEvent(void *context)
{
  /* USER CODE BEGIN OnStopJoinTimerEvent_1 */

  /* USER CODE END OnStopJoinTimerEvent_1 */
  if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE)
  {
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
  }
  /* USER CODE BEGIN OnStopJoinTimerEvent_Last */

  /* USER CODE END OnStopJoinTimerEvent_Last */
}

static void StoreContext(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  /* USER CODE BEGIN StoreContext_1 */

  /* USER CODE END StoreContext_1 */
  status = LmHandlerNvmDataStore();

  if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
  }
  else if (status == LORAMAC_HANDLER_ERROR)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
  }
  /* USER CODE BEGIN StoreContext_Last */

  /* USER CODE END StoreContext_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
  /* USER CODE BEGIN OnNvmDataChange_1 */

  /* USER CODE END OnNvmDataChange_1 */
  if (state == LORAMAC_HANDLER_NVM_STORE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
  }
  /* USER CODE BEGIN OnNvmDataChange_Last */

  /* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */

  /* USER CODE END OnStoreContextRequest_1 */
  /* store nvm in flash */
  if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
  {
    FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (const void *)nvm, nvm_size);
  }
  /* USER CODE BEGIN OnStoreContextRequest_Last */

  /* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnRestoreContextRequest_1 */

  /* USER CODE END OnRestoreContextRequest_1 */
  FLASH_IF_Read(nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size);
  /* USER CODE BEGIN OnRestoreContextRequest_Last */

  /* USER CODE END OnRestoreContextRequest_Last */
}


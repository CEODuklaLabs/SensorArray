/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
  COMMAND_GET_CU_CALIB = 0x01,
  COMMAND_SET_CU_CALIB,
  COMMAND_GET_MEAS,
  COMMAND_GET_STATUS,
  COMMAND_MEAS,
  COMMAND_SET_USER_ID,
  COMMAND_GET_USER_ID,
  COMMAND_GET_UID,
  COMMAND_SET_CYCLE,
  COMMAND_GET_CYCLE,
  COMMAND_GET_NMBR_MEAS,
  COMMAND_SET_NMBR_MEAS,
  COMMAND_START_MEASUREMENT,
  COMMAND_STOP_MEASUREMENT,

  COMMAND_SEND_DATA = 0xFF
} SPI_COMANDS;

typedef enum {
  CUx_STATUS = 0x01,
  CUx_UID,
  CUx_USER_ID,
  CUx_CAL,
  CUx_ERROR,
  CUx_MEAS
} CUx_DESCRIPTOR_TYPES;

typedef enum {
  ERROR_UNKNOWN_COMMAND,
  ERROR_EEPROM_WRITE,
  ERROR_USER_ID
} CUx_ERROR_TYPES;

typedef enum {
  CUx_OK = 0x00,
  CUx_READY = 0x01,
  CUx_BUSY,
  CUx_MEASUREMENT,
  #if defined(ENABLE_UART)
  CUx_MEASUREMENT_DONE
  #endif
} CUx_STATUS_TYPES;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
 * @brief  Funkce pro zpracování dat z ADC
 * @retval None
 * @note   Funkce je volána z funkce HAL_ADC_ConvCpltCallback
 * @note   Funkce vypočítá průměry a uloží je do pole channel_averages
 */
void StartADCConversion(void);

/** 
 * @brief  Funkce pro zpracování Spuštění zpracování dat po konverzi ADC
 * @param hadc Ukazatel na ADC, který dokončil konverzi
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

/**
 * @brief  Funkce pro zpracování dat z ADC
 * @retval None
 * @note   Funkce je volána z funkce HAL_ADC_ConvCpltCallback
 */
void Process_ADC_Data(void);

/**
 * @brief  Funkce pro zpracování příkazu obdrženého přes SPI
 * @param  command: Příkaz, který má být zpracován
 * @retval None
 * @note   Funkce je volána z funkce HAL_SPI_RxCpltCallback
 */
void handleCommand(SPI_COMANDS command);

/**
 * @brief  Funkce pro vykonání příkazu GETCALIB přes SPI
 * @retval None
 * @note   Funkce odesílá kalibrační hodnoty přes SPI
 */
void getCUCalib();

/**
 * @brief  Funkce pro vykonání příkazu SETCALIB přes SPI
 * @retval None
 * @note   Funkce nastaví kalibrační hodnoty z přijatých dat
 */
void setCUCalib();

/**
 * @brief  Funkce pro přepočet ADC hodnoty na teplotu
 * @param  adcValue: ADC hodnota
 * @retval Teplota 
 */
float adcValueToTemperature(uint32_t adcValue);

/**
 * @brief  Funkce pro získání měření
 * @retval None
 * @note   Funkce neodesílá naměřené hodnoty přes SPI pouze spoští ADC konverzi
 */
void measurement();

/**
 * @brief  Funkce pro získání měření
 * @retval None
 * @note   Funkce odesílá naměřené hodnoty teplot přes SPI
 */
void getMeasurement();

/**
 * @brief  Funkce pro získání stavu zařízení
 * @retval None

 */
void getStatus();

/**
 * @brief  Funkce pro zpracování neznámého příkazu
 * @retval None
 */
void handleUnknownCommand(); 

/**
 * @brief  Funkce pro přidání dat do SPI bufferu
 * @param  data: Data, která mají být přidána
 * @param  size: Velikost dat
 * @param  descriptor: Descriptor dat
 * @retval None
 */
void addDataToSPIBuffer(uint8_t* data, size_t size, CUx_DESCRIPTOR_TYPES descriptor);

/**
 * @brief  Funkce pro odeslání SPI zprávy
 * @retval None
 * @note   Funkce odesílá data z bufferu SPITxBuffer přes SPI
 */
void sendSPIMasssage();

/**
 * @brief  Funkce pro získání unikátního ID
 * @retval None
 * @note   Funkce odesílá unikátní ID přes SPI
 */
void getUID();

/**
 * @brief  Funkce pro získání uživatelského ID
 * @retval None
 * @note   Funkce odesílá uživatelské ID přes SPI
 */
void set_user_ID();

/**
 * @brief  Funkce pro nastavení uživatelského ID
 * @retval None
 * @note   Funkce nastaví uživatelské ID z přijatých dat
 */
void get_user_ID();

/**
 * @brief  Funkce pro zpracování dat z ADC
 * @retval None
 * @note   Funkce je volána z funkce HAL_ADC_ConvCpltCallback
 * @note   Funkce vypočítá průměry a uloží je do pole channel_averages 
 */
void ProcessAveregeData(void);

/**
 * @brief  Funkce na rekci na chybné uživatelské ID
 * @retval None
 */
void wrongUserID(void);

/**
 * @brief  Funkce pro vynulování průměrů
 * @retval None
 */
void ZeroChannelAverages(void);

/**
 * @brief  Funkce pro přidání hodnot do průměrů
 * @retval None
 */
void AddToChannelAverages(void);

/**
 * @brief  Funkce pro kontrolu UserID příkazu obdrženého přes SPI
 * 
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

/**
 * @brief  Funkce pro zpracování Přerušení časovače typicky spoštení ADC převodu
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/**
 * @brief  Funkce pro nastavení cyklu měření
 * @retval None
 
 */
void setCycle(void);

/**
 * @brief  Funkce pro získání cyklu měření
 * @retval None
*/
void getCycle(void);

/**
 * @brief  Funkce pro nastavení počtu vzorků
 * @retval None
 */
void setNumSamples(void);

/**
 * @brief  Funkce pro získání počtu vzorků
 * @retval None
 */ 
void getNumSamples(void);

/**
 * @brief  Funkce pro spuštění měření
 * @retval None
 */
void StartCycle(void);

/**
 * @brief  Funkce pro zastavení měření
 * @retval None
 */
void StopCycle(void);



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN3_Pin GPIO_PIN_0
#define AIN3_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_2
#define AIN1_GPIO_Port GPIOA
#define ANI0_Pin GPIO_PIN_3
#define ANI0_GPIO_Port GPIOA
#define AIN4_Pin GPIO_PIN_4
#define AIN4_GPIO_Port GPIOA
#define AIN5_Pin GPIO_PIN_5
#define AIN5_GPIO_Port GPIOA
#define AIN6_Pin GPIO_PIN_6
#define AIN6_GPIO_Port GPIOA
#define AIN7_Pin GPIO_PIN_7
#define AIN7_GPIO_Port GPIOA
#define AIN8_Pin GPIO_PIN_0
#define AIN8_GPIO_Port GPIOB
#define AIN9_Pin GPIO_PIN_1
#define AIN9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ADC_RESOLUTION 4096 // 12bitová ADC hodnota (2^12)
#define VREF 3.3 // Referenční napětí
#define TEMP_MIN 0.0 // Minimální teplota odpovídající 0V
#define TEMP_MAX 100.0 // Maximální teplota odpovídající 3.3V
#define NUM_CHANNELS 10 // Počet kanálů a kalibračních hodnot

#define EEPROM_USER_ID_ADDRESS 0x08080000 // Adresa pro uživatelské ID
#define EEPROM_NUM_SAMPLES_ADDRESS 0x08080004 // Adresa pro počet vzorků
#define EEPROM_CYCLE_ADDRESS 0x08080008 // Adresa pro cyklus
#define EEPROM_CALIB_ADDRESS 0x0808000C // Adresa pro kalibrační hodnoty
#define APP_TX_DATA_SIZE 64
#define APP_RX_DATA_SIZE 64
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

#define ENABLE_UART
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#if defined(ENABLE_UART)
  #include "usart.h"
#endif
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t SPIRxBuffer[APP_RX_DATA_SIZE];
uint8_t SPITxBuffer[APP_TX_DATA_SIZE];

uint16_t TxBuffer_pointer;
CUx_STATUS_TYPES STATUS;

uint16_t adc_values[NUM_CHANNELS];
uint32_t channel_averages[NUM_CHANNELS];

float calibValues[NUM_CHANNELS];
float temperatureValues[NUM_CHANNELS];

uint8_t Actual_cycle;
uint8_t User_ID;

uint32_t NUM_SAMPLES; // Počet vzorků pro průměrování
uint32_t CYCLE_TIME; // Čas cyklu pro měření

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	TxBuffer_pointer = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  User_ID = *(uint32_t*)EEPROM_USER_ID_ADDRESS;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  //MX_RTC_Init();
  //MX_TIM2_Init();
  #if defined(ENABLE_UART)
    MX_USART2_UART_Init();
  #endif
  /* USER CODE BEGIN 2 */
  #ifdef ENABLE_UART
    HAL_UART_Transmit(&huart2, (uint8_t *)"UART Enabled\r\n", 14, HAL_MAX_DELAY);
  #endif
  getCUCalib();
  getCycle();
  getNumSamples();
  
  NUM_SAMPLES <= 0 ? NUM_SAMPLES = 25 : NUM_SAMPLES;
  Actual_cycle= 0;

  HAL_SPI_Receive_DMA(&hspi1, SPIRxBuffer, APP_RX_DATA_SIZE);
  HAL_SPI_Transmit_DMA(&hspi1, SPITxBuffer, APP_TX_DATA_SIZE);
  STATUS = CUx_READY; //READY
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 /* HAL_UART_Transmit(&huart2, (uint8_t *)"Hello from STM32!\r\n", 20, HAL_MAX_DELAY);
  HAL_Delay(1000);
  ZeroChannelAverages();
  StartADCConversion(); // Spustit ADC s DMA, uložit hodnoty do pole adc_values*/
  #if defined(ENABLE_UART)
    HAL_UART_Transmit(&huart2, (uint8_t *)"UART Initialized\r\n", 19, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)"Hello from STM32!\r\n", 20, HAL_MAX_DELAY);
  #endif
  while (1)
  {
    HAL_Delay(1000);
    #if defined(ENABLE_UART)
      
      if(STATUS == CUx_READY) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"Start Measurement\r\n", 20, HAL_MAX_DELAY);
        StartADCConversion();
      }
      else if (STATUS == CUx_MEASUREMENT) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"Measurement in progress\r\n", 27, HAL_MAX_DELAY);
      } else if (STATUS == CUx_MEASUREMENT_DONE) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"Measurement done\r\n", 18, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t *)"Temperature values: ", 20, HAL_MAX_DELAY); 
        HAL_UART_Transmit(&huart2, (uint8_t *)temperatureValues, sizeof(temperatureValues), HAL_MAX_DELAY);
      }
    #endif


    /* USER CODE END WHILE */



    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  handleCommand(SPITxBuffer[1]);
}

void StartADCConversion(void)
{ 

    // Spustit ADC s DMA, uložit hodnoty do pole adc_values
    HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_values, NUM_CHANNELS);
    STATUS = CUx_MEASUREMENT; //MEASURING
}

void Process_ADC_Data(void) {
  ProcessAveregeData();
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    temperatureValues[i] = adcValueToTemperature(channel_averages[i]);
    temperatureValues[i] += calibValues[i];
  }
  ZeroChannelAverages();
}

void ProcessAveregeData(void) {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      channel_averages[i] /= NUM_SAMPLES;
  } 
}

void ZeroChannelAverages(void) {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
      channel_averages[ch] = 0;
  }
}

void AddToChannelAverages(void) {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) { 
      channel_averages[ch] += (uint32_t)adc_values[ch];
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Stop_DMA(hadc); // Zastavit DMA
    if (Actual_cycle < NUM_SAMPLES) {
        Actual_cycle++;
        AddToChannelAverages();
        StartADCConversion();
    } else {
        Actual_cycle = 0;
        Process_ADC_Data();
        addDataToSPIBuffer((uint8_t*)temperatureValues, sizeof(temperatureValues), CUx_MEAS);
        STATUS = CUx_READY; //READY
        #if defined(ENABLE_UART)
          HAL_UART_Transmit(&huart2, (uint8_t *)"Measurement complete\r\n", 22, HAL_MAX_DELAY);
          HAL_UART_Transmit(&huart2, (uint8_t *)temperatureValues, sizeof(temperatureValues), HAL_MAX_DELAY);
        #endif
    }
}

void handleCommand(SPI_COMANDS command) {
  switch (command) {
    case COMMAND_GET_CU_CALIB:
      getCUCalib();
      break;
    case COMMAND_SET_CU_CALIB:
      setCUCalib();
      break;
    case COMMAND_GET_MEAS:
      getMeasurement();
      break;
      case COMMAND_MEAS:
      measurement();
      break;
    case COMMAND_GET_STATUS:
      getStatus();
      break;
    case COMMAND_SET_USER_ID:
      set_user_ID(SPIRxBuffer[2]);
      break;
    case COMMAND_GET_USER_ID:
      get_user_ID();
      break;
    case COMMAND_GET_UID:
      getUID();
      break;
    case COMMAND_SET_CYCLE:
      setCycle();
      break;
    case COMMAND_GET_CYCLE:
      getCycle();
      break;
    case COMMAND_GET_NMBR_MEAS:
      getNumSamples();
      break;
    case COMMAND_SET_NMBR_MEAS: 
      setNumSamples();
      break;
    case COMMAND_START_MEASUREMENT:
      StartCycle();
      break;
    case COMMAND_STOP_MEASUREMENT:
      StopCycle();
      break;

    default:
  }
  sendSPIMasssage();
}

void StartCycle() {
  HAL_TIM_Base_Start_IT(&htim2); // Spustit časovač
  addDataToSPIBuffer(CUx_OK, 1, CUx_STATUS);
}

void StopCycle() {
  HAL_TIM_Base_Stop_IT(&htim2); // Zastavit časovač
  addDataToSPIBuffer(CUx_OK, 1, CUx_STATUS);
}

void setCycle() {
  HAL_FLASHEx_DATAEEPROM_Unlock();
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_CYCLE_ADDRESS, *(uint32_t*)&SPIRxBuffer[2]);
  HAL_FLASHEx_DATAEEPROM_Lock();
  addDataToSPIBuffer(CUx_OK, 1, CUx_STATUS);

  // Načtení nové hodnoty cyklu z EEPROM
  CYCLE_TIME = *(uint32_t*)EEPROM_CYCLE_ADDRESS;

  // Přenastavení časovače (např. TIM2)
  if (CYCLE_TIME > 0) {
    HAL_TIM_Base_Stop_IT(&htim2); // Zastavení časovače
    __HAL_TIM_SET_AUTORELOAD(&htim2, CYCLE_TIME - 1); // Nastavení nové periody
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) { 
    // Zde se provede měření ADC a zpracování dat
    StartADCConversion();
  }
}

void getCycle() {
  uint32_t cycle = *(uint32_t*)EEPROM_CYCLE_ADDRESS;
  addDataToSPIBuffer((uint8_t*)&cycle, sizeof(cycle), CUx_STATUS);
}

void getNumSamples() {
  uint32_t numSamples = *(uint32_t*)EEPROM_NUM_SAMPLES_ADDRESS;
  addDataToSPIBuffer((uint8_t*)&numSamples, sizeof(numSamples), CUx_STATUS);
}

void setNumSamples() {
  HAL_FLASHEx_DATAEEPROM_Unlock();
  HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_NUM_SAMPLES_ADDRESS, *(uint32_t*)&SPIRxBuffer[2]);
  HAL_FLASHEx_DATAEEPROM_Lock();
  addDataToSPIBuffer(CUx_OK, 1, CUx_STATUS);
  sendSPIMasssage();
}

void wrongUserID() {
  addDataToSPIBuffer((uint8_t *)ERROR_USER_ID, 1, CUx_ERROR);
  sendSPIMasssage();
}

void set_user_ID(uint32_t userID) {
  HAL_FLASHEx_DATAEEPROM_Unlock();
  if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_USER_ID_ADDRESS, userID) != HAL_OK) {
    HAL_FLASHEx_DATAEEPROM_Lock();
    addDataToSPIBuffer((uint8_t *)CUx_ERROR, 1, CUx_ERROR);
  }
  HAL_FLASHEx_DATAEEPROM_Lock();
  addDataToSPIBuffer((uint8_t *)ERROR_EEPROM_WRITE, 1, CUx_STATUS);
}

void get_user_ID(){
  uint32_t userID = *(uint32_t*)EEPROM_USER_ID_ADDRESS;
  addDataToSPIBuffer((uint8_t*)&userID, sizeof(userID), CUx_USER_ID);
}

void getCUCalib() {
  for (uint32_t i = 0; i < NUM_CHANNELS; i++) {
    uint32_t address = EEPROM_CALIB_ADDRESS + (i * sizeof(float));
    calibValues[i] = *(float*)address;
  }
  addDataToSPIBuffer((uint8_t*)calibValues, sizeof(calibValues), CUx_CAL); 
}

void setCUCalib() {
  float calibValues[NUM_CHANNELS];
  memcpy(calibValues, SPIRxBuffer + 2, sizeof(calibValues)); // +1 kvůli userid, command 
  HAL_FLASHEx_DATAEEPROM_Unlock();
  for (uint32_t i = 0; i < NUM_CHANNELS; i++) {
    uint32_t address = EEPROM_CALIB_ADDRESS + (i * sizeof(float));
    HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address, *(uint32_t*)&calibValues[i]);
  } 
  HAL_FLASHEx_DATAEEPROM_Lock();
  addDataToSPIBuffer(CUx_OK, 1, CUx_STATUS);
  sendSPIMasssage();
}

float adcValueToTemperature(uint32_t adcValue) {
    // Přepočet ADC hodnoty na napětí
    float voltage = (adcValue / (float)ADC_RESOLUTION) * VREF * 1000.0f;
    // Přepočet napětí na teplotu
    //float temperature = ((voltage - 0.0) * (TEMP_MAX - TEMP_MIN)) / (VREF - 0.0) + TEMP_MIN;
    float temperature = (voltage - 500.0f)/10.0f;
    return temperature;
}

void measurement() {
  // Implementace pro získání měření
  StartADCConversion();
}

void getMeasurement() {
  // Implementace pro získání měření
  addDataToSPIBuffer((uint8_t*)temperatureValues, sizeof(temperatureValues), CUx_MEAS);
  STATUS = CUx_READY; //READY
}

void getStatus() {
  // Implementace pro získání stavu zařízení
  addDataToSPIBuffer(&STATUS, sizeof(STATUS), CUx_STATUS);
}

void getUID() {
  // Odeslání unikátního ID
  uint32_t uid0 = *(uint32_t*)0x1FFF7590; // První část unikátního ID
  uint32_t uid1 = *(uint32_t*)0x1FFF7594; // Druhá část unikátního ID
  uint32_t uid2 = *(uint32_t*)0x1FFF7598; // Třetí část unikátního ID
  addDataToSPIBuffer((uint8_t*)&uid0, sizeof(uid0), CUx_UID);
  addDataToSPIBuffer((uint8_t*)&uid1, sizeof(uid1), 0);
  addDataToSPIBuffer((uint8_t*)&uid2, sizeof(uid2), 0);
}

void handleUnknownCommand() {
  //odeslání chybové hlášky přes přes SPI
  addDataToSPIBuffer((uint8_t *)ERROR_UNKNOWN_COMMAND, 1, CUx_ERROR);
}

void addDataToSPIBuffer(uint8_t* data, size_t size, CUx_DESCRIPTOR_TYPES descriptor) {
    // Přidání descriptoru na aktuální pozici
    if (TxBuffer_pointer ==0) {
      SPITxBuffer[TxBuffer_pointer++] = User_ID;
    }
    if (descriptor != 0) {
      SPITxBuffer[TxBuffer_pointer++] = descriptor;
    }
    // Kopírování dat za descriptor
    memcpy(SPITxBuffer + TxBuffer_pointer, data, size);
    // Aktualizace aktuální pozice
    TxBuffer_pointer += size;
}

void sendSPIMasssage() {
    HAL_SPI_Transmit_DMA(&hspi1, SPITxBuffer, TxBuffer_pointer);
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    TxBuffer_pointer = 0;
    memset(SPITxBuffer, 0x00, APP_TX_DATA_SIZE);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

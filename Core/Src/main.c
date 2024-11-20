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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bsec_integration.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "bsec_selectivity.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BME68X_I2C_ADDRESS BME68X_I2C_ADDR_HIGH

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Global sensor APIs data structure */
uint8_t	bsec_mem_block[NUM_OF_SENS][BSEC_INSTANCE_SIZE];
uint8_t *bsecInstance[NUM_OF_SENS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief           调试打印函数
 *
 * @param[in]       fmt               格式化字符串
 * @param[in]       ...               可变参数列表
 *
 * @return          �?
 */
void debug_print(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

	char buffer[512] = {0};
    vsprintf(buffer, fmt, args);

	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 1000);
    va_end(args);
}

/*!
 * @brief           在Wire或SPI中进行写操作
 *
 * @param[in]        reg_addr        寄存器地�?
 * @param[in]        reg_data_ptr    指向要写入数据的指针
 * @param[in]        data_len        要写入的字节�?
 * @param[in]        intf_ptr        接口指针
 *
 * @return          总线通信函数的结�?
 */
int8_t bus_write(uint8_t reg_addr, const uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, (BME68X_I2C_ADDRESS << 1), reg_addr, 
												I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data_ptr, data_len, 
												1000);
    return (status == HAL_OK) ? 0 : -1;
}

/*!
 * @brief           在Wire或SPI中进行读操作
 *
 * @param[in]        reg_addr        寄存器地�?
 * @param[out]       reg_data_ptr    指向用于存储读取数据的内存的指针
 * @param[in]        data_len        要读取的字节�?
 * @param[in]        intf_ptr        接口指针
 * 
 * @return          总线通信函数的结�?
 */
int8_t bus_read(uint8_t reg_addr, uint8_t *reg_data_ptr, uint32_t data_len, void *intf_ptr)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, ((BME68X_I2C_ADDRESS << 1) | 0x01), reg_addr, 
												I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data_ptr, data_len, 
												1000);
    return (status == HAL_OK) ? 0 : -1;
}

/*!
 * @brief           系统特定的睡眠函数实�?
 *
 * @param[in]       t_us     微秒为单位的时间
 * @param[in]       intf_ptr 接口描述符的指针
 * 
 * @return          �?
 */
void sleep_n(uint32_t t_us, void *intf_ptr)
{
	if (t_us % 1000 > 500) {
		HAL_Delay(t_us / 1000 + 1);
	} else {
		HAL_Delay(t_us / 1000);
	}
}

static uint32_t overflowCounter;
static uint32_t lastTimeMS;

/*!
 * @brief           获取系统时间（以微秒为单位）
 *
 * @return          system_current_time    当前系统时间戳（以微秒为单位�?
 */
int64_t get_timestamp_us()
{
    int64_t timeMs = HAL_GetTick() * 1000;

    if (lastTimeMS > timeMs) /* An overflow occurred */
    { 
        overflowCounter++;
    }
    lastTimeMS = timeMs;
    
    return timeMs + (overflowCounter * INT64_C(0xFFFFFFFF));
}

/*!
 * @brief           处理准备就绪的输�?
 *
 * @param[in]       outputs               	output_t结构�?
 * @param[in]       bsec_status             bsec_do_steps()调用返回的�??
 *
 * @return          �?
 */
void output_ready(output_t *outputs, bsec_library_return_t bsec_status)
{
	float timestamp_ms = outputs->timestamp / 1e6;

	debug_print("%d,", outputs->sens_no);
	debug_print("%d,", (int)timestamp_ms);

	#if (OUTPUT_MODE == CLASSIFICATION || OUTPUT_MODE == REGRESSION)
	debug_print("%f,", outputs->gas_estimate_1);
	debug_print("%f,", outputs->gas_estimate_2);
	debug_print("%f,", outputs->gas_estimate_3);
	debug_print("%f,", outputs->gas_estimate_4);
	debug_print("%d,", outputs->gas_accuracy_1);
	debug_print("%d,", outputs->gas_accuracy_2);
	debug_print("%d,", outputs->gas_accuracy_3);
	debug_print("%d,", outputs->gas_accuracy_4);
	debug_print("%f,", outputs->raw_pressure);
	debug_print("%f,", outputs->raw_temp);
	debug_print("%f,", outputs->raw_humidity);
	debug_print("%f,", outputs->raw_gas);
	debug_print("%d,", outputs->raw_gas_index);
	#elif (OUTPUT_MODE == IAQ)
	debug_print("%f,", outputs->iaq);
	output += String(outputs->iaq_accuracy) + ", ";
	output += String(outputs->static_iaq) + ", ";
	output += String(outputs->raw_temp) + ", ";
	output += String(outputs->raw_humidity) + ", ";
	output += String(outputs->temperature) + ", ";
	output += String(outputs->humidity) + ", ";
	output += String(outputs->raw_pressure) + ", "; 
	output += String(outputs->raw_gas) + ", ";
	output += String(outputs->gas_percentage) + ", ";
	output += String(outputs->co2_equivalent) + ", ";
	output += String(outputs->breath_voc_equivalent) + ", ";
	output += String(outputs->stabStatus) + ", ";
	output += String(outputs->runInStatus) + ", ";
	#endif

	debug_print("%d,", bsec_status);
	debug_print("\r\n");
}

/*!
 * @brief           从非易失性存储器加载先前的库状�??
 *
 * @param[in,out]   state_buffer    用于保存加载状�?�字符串的缓冲区
 * @param[in]       n_buffer        分配的状态缓冲区大小
 *
 * @return          复制到state_buffer的字节数
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // 如果可用,从非易失性存储器加载先前的库状�?��??
    //
    // 如果加载失败或没有可用状态则返回�?,
    // 否则返回加载的状态字符串长度�?
    // ...
    return 0;
}

/*!
 * @brief           将库状�?�保存到非易失�?�存储器
 *
 * @param[in]       state_buffer    保存要存储状态的缓冲�?
 * @param[in]       length          要存储的状�?�字符串长度
 *
 * @return          �?
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // 如果可能,将字符串保存到某种形式的非易失�?�存储器�?
    // ...
}
 
/*!
 * @brief           从非易失性存储器加载库配�?
 *
 * @param[in,out]   config_buffer    用于保存加载状�?�字符串的缓冲区
 * @param[in]       n_buffer        分配的状态缓冲区大小
 *
 * @return          复制到config_buffer的字节数
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
	memcpy(config_buffer, bsec_config_selectivity, n_buffer);
    return n_buffer;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(1000);
	debug_print("Starting...\r\n");

    struct bme68x_dev bme_dev;
	memset(&bme_dev, 0, sizeof(bme_dev)); 
	bme_dev.intf = BME68X_I2C_INTF;
	bme_dev.write = bus_write;
	bme_dev.read = bus_read;
	bme_dev.delay_us = sleep_n;
	bme_dev.intf_ptr = NULL;
	bme_dev.amb_temp = 25;

	/* 分配内存块给bsecInstance */
	allocateMemory(bsec_mem_block[0], 0);

    /* 调用初始化BSEC库的函数
     * �?启低功�?�模式并且不提供温度偏移 */
    return_values_init ret = bsec_iot_init(SAMPLE_RATE, 0.0f, bus_write, bus_read, sleep_n, state_load, config_load, bme_dev, 0);

    if (ret.bme68x_status) {
        /* 无法初始化BME68x */
		debug_print("ERROR while initializing BME68x: %d\r\n", ret.bme68x_status);
        return (int)ret.bme68x_status;
    } else if (ret.bsec_status < BSEC_OK) {
        /* 无法初始化BSEC库 */
		debug_print("ERROR while initializing BSEC library: %d\r\n", ret.bsec_status);
        return (int)ret.bsec_status;
    } else if (ret.bsec_status > BSEC_OK) {
		debug_print("WARNING while initializing BSEC library: %d\r\n", ret.bsec_status);
        return (int)ret.bsec_status;
    }

    bsec_version_t version;
	bsec_get_version_m(bsecInstance, &version);
	debug_print("BSEC library version %d.%d.%d.%d\r\n", version.major, version.minor, version.major_bugfix, version.minor_bugfix);

	#if (OUTPUT_MODE == CLASSIFICATION || OUTPUT_MODE == REGRESSION)
		debug_print("Sensor_No, Time(ms), Class/Target_1_prediction, Class/Target_2_prediction,"
					 "Class/Target_3_prediction, Class/Target_4_prediction, Prediction_accuracy_1,"
					 "Prediction_accuracy_2, Prediction_accuracy_3, Prediction_accuracy_4,"
					 "Raw_pressure(Pa), Raw_Temperature(degC),  Raw_Humidity(%rH), Raw_Gas(ohm),"
					 "Raw_Gas_Index(num), Bsec_status\r\n");
	#elif (OUTPUT_MODE == IAQ)
		debug_print("Sensor_No, Time(ms), IAQ, IAQ_accuracy, Static_IAQ, Raw_Temperature(degC),"
					 "Raw_Humidity(%rH), Comp_Temperature(degC),  Comp_Humidity(%rH),"
					 "Raw_pressure(Pa), Raw_Gas(ohms), Gas_percentage, CO2, bVOC,"
					 "Stabilization_status, Run_in_status, Bsec_status\r\n");
	#endif
  
    /* 调用无限循环函数，基于传感器设置读取和处理数�? */
    /* �?10,000个样本保存一次状态，即每10,000 * 3�? = 500分钟保存�?�? */
    bsec_iot_loop(sleep_n, get_timestamp_us, output_ready, state_save, 10000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

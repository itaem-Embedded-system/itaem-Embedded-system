/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "QMI8658A.h"
#include "platfrom_qmi8658a.h"
#include "MahonyAHRS.h"
#include <math.h>
#include "vofa.h"
#include "imu.h"
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
mahony_t mahony;
float g_roll, g_pitch, g_yaw;
VOFA_HandleTypeDef hvofa;
uint8_t rx_byte;

qmi8658_handle_t qmi_handle;
qmi_raw_data_t qmi_raw;
qmi_physical_data_t qmi_phys;

uint8_t no_motion_detected = 0;
uint32_t no_motion_count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
bool STM32_UART_Send(void *serial_handle,
                     const uint8_t *data,
                     uint16_t len)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)serial_handle;
    
    if (huart == NULL) {
        return false;
    }
    if (data == NULL) {
        return false;
    }
    if (len == 0) {
        return false;
    }

    HAL_StatusTypeDef ret = HAL_UART_Transmit(
        huart,
        (uint8_t *)data,
        len,
        1000
    );
    
    return (ret == HAL_OK);

}

void qmi8658_init(void) {
    if (IMU_Init() != IMU_OK) {
        // 初始化失败，后续由 read_task 中的重试机制处理
    }
}


void qmi8658_read_task(void) {
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    if (dt > 0.1f) dt = 0.01f;
    
    // 直接从寄存器读取数据（不使用 FIFO）
    float send_data[8] = {0};
    VOFA_Data_TypeDef_Enum data_types[8] = {
        VOFA_DATA_FLOAT, VOFA_DATA_FLOAT, VOFA_DATA_FLOAT,
        VOFA_DATA_FLOAT, VOFA_DATA_FLOAT, VOFA_DATA_FLOAT,
        VOFA_DATA_FLOAT, VOFA_DATA_FLOAT,
    };

    imu_data_t imu_data;
    if (IMU_GetData(&imu_data) == IMU_OK) {
        send_data[0] = imu_data.acc_x;
        send_data[1] = imu_data.acc_y;
        send_data[2] = imu_data.acc_z;
        send_data[3] = imu_data.gyro_x;
        send_data[4] = imu_data.gyro_y;
        send_data[5] = imu_data.gyro_z;
        
        imu_status_t status;
        if (IMU_GetStatus(&status) == IMU_OK) {
            no_motion_detected = status.no_motion_detected ? 1 : 0;
            no_motion_count = status.no_motion_count;
        }
    } else {
        // 读取失败或未初始化，IMU_GetData 内部已处理自动重试
        no_motion_detected = 0;
    }

    send_data[6] = (float)no_motion_detected;
    send_data[7] = (float)now;
    VOFA_SendData(&hvofa, send_data, 8, data_types);
    
    last_time = now;
}
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	VOFA_Init(&hvofa,

          &huart1,

          STM32_UART_Send,

          VOFA_FMT_JUSTFLOAT,

          VOFA_BAUD_115200);
		  
	qmi8658_init();
	mahony_init(&mahony, 1.0f, 0.0f);
	
	qmi8658_lpf_enable(&qmi_handle, 0.1f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  qmi8658_read_task();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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

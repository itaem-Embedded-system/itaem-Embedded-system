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
#include "TAM.h"
#include "tam_console.h"
#include "QMC5883P.h"
#include "vofa.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_VOFA_UART_TIMEOUT_MS 100U
#define MAIN_VOFA_POLL_PERIOD_MS 100U
#define MAIN_STAGE_SLOT_MS 1800U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static VOFA_HandleTypeDef main_vofa_handle;
static VOFA_Data_TypeDef_Enum main_vofa_types[7] = {
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT
};
static VOFA_Data_TypeDef_Enum main_vofa_raw_types[8] = {
  VOFA_DATA_UCHAR,
  VOFA_DATA_UCHAR,
  VOFA_DATA_UCHAR,
  VOFA_DATA_UCHAR,
  VOFA_DATA_UCHAR,
  VOFA_DATA_UCHAR,
  VOFA_DATA_UCHAR,
  VOFA_DATA_UCHAR
};
static VOFA_Data_TypeDef_Enum main_vofa_types5[5] = {
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT,
  VOFA_DATA_FLOAT
};

typedef enum {
  MAIN_VOFA_STAGE_MCU_JUSTFLOAT = 0,
  MAIN_VOFA_STAGE_MCU_RAWDATA,
  MAIN_VOFA_STAGE_MCU_UART,
  MAIN_VOFA_STAGE_INIT_JUSTFLOAT,
  MAIN_VOFA_STAGE_INIT_RAWDATA,
  MAIN_VOFA_STAGE_INIT_UART,
  MAIN_VOFA_STAGE_DATA_JUSTFLOAT,
  MAIN_VOFA_STAGE_DATA_RAWDATA,
  MAIN_VOFA_STAGE_DATA_UART,
  MAIN_VOFA_STAGE_SENSOR_STREAM
} main_vofa_stage_t;

/* 运行状态量 */
static volatile uint8_t main_vofa_poll_armed = 0U;

/* 时间戳状态量 */
static volatile uint32_t main_vofa_last_poll_tick = 0U;
static volatile uint32_t main_vofa_stage_start_tick = 0U;
static volatile uint32_t main_vofa_last_text_tick = 0U;
static volatile uint16_t main_vofa_test_seq = 0U;
static volatile uint16_t main_vofa_send_error_count = 0U;
static volatile main_vofa_stage_t main_vofa_stage = MAIN_VOFA_STAGE_MCU_JUSTFLOAT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static bool Main_SendText(const char *msg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static bool Main_VofaUartSend(void *serial_handle, const uint8_t *data, uint16_t len)
{
  UART_HandleTypeDef *uart = (UART_HandleTypeDef *)serial_handle;

  if ((uart == NULL) || (data == NULL) || (len == 0U)) return false;
  if (uart->Instance != USART2) return false;
  return (HAL_UART_Transmit(uart, (uint8_t *)data, len, MAIN_VOFA_UART_TIMEOUT_MS) == HAL_OK) ? true : false;
}

static bool Main_SendText(const char *msg)
{
  if (msg == NULL) return false;
  return Main_VofaUartSend(&huart2, (const uint8_t *)msg, (uint16_t)strlen(msg));
}

static void Main_RecordSendResult(bool ok)
{
  if (!ok && (main_vofa_send_error_count < 0xFFFFU)) {
    main_vofa_send_error_count++;
  }
}

static void Main_StageAnnounce(main_vofa_stage_t stage)
{
  switch (stage) {
  case MAIN_VOFA_STAGE_MCU_JUSTFLOAT:
    (void)Main_SendText("[DIAG] STAGE MCU JUSTFLOAT\r\n");
    break;
  case MAIN_VOFA_STAGE_MCU_RAWDATA:
    (void)Main_SendText("[DIAG] STAGE MCU RAWDATA\r\n");
    break;
  case MAIN_VOFA_STAGE_MCU_UART:
    (void)Main_SendText("[DIAG] STAGE MCU UART\r\n");
    break;
  case MAIN_VOFA_STAGE_INIT_JUSTFLOAT:
    (void)Main_SendText("[DIAG] STAGE INIT JUSTFLOAT\r\n");
    break;
  case MAIN_VOFA_STAGE_INIT_RAWDATA:
    (void)Main_SendText("[DIAG] STAGE INIT RAWDATA\r\n");
    break;
  case MAIN_VOFA_STAGE_INIT_UART:
    (void)Main_SendText("[DIAG] STAGE INIT UART\r\n");
    break;
  case MAIN_VOFA_STAGE_DATA_JUSTFLOAT:
    (void)Main_SendText("[DIAG] STAGE DATA JUSTFLOAT\r\n");
    break;
  case MAIN_VOFA_STAGE_DATA_RAWDATA:
    (void)Main_SendText("[DIAG] STAGE DATA RAWDATA\r\n");
    break;
  case MAIN_VOFA_STAGE_DATA_UART:
    (void)Main_SendText("[DIAG] STAGE DATA UART\r\n");
    break;
  case MAIN_VOFA_STAGE_SENSOR_STREAM:
    (void)Main_SendText("[DIAG] STAGE SENSOR STREAM JUSTFLOAT\r\n");
    break;
  default:
    break;
  }
}

static void Main_VofaAdvanceStage(uint32_t now_tick)
{
  if (main_vofa_stage == MAIN_VOFA_STAGE_SENSOR_STREAM) return;
  if ((uint32_t)(now_tick - main_vofa_stage_start_tick) < MAIN_STAGE_SLOT_MS) return;

  main_vofa_stage = (main_vofa_stage_t)((uint32_t)main_vofa_stage + 1U);
  main_vofa_stage_start_tick = now_tick;
  Main_StageAnnounce(main_vofa_stage);
}

static void Main_VofaInit(void)
{
  VOFA_Init(&main_vofa_handle,
            &huart2,
            Main_VofaUartSend,
            VOFA_FMT_JUSTFLOAT,
            VOFA_BAUD_115200);
  main_vofa_stage = MAIN_VOFA_STAGE_MCU_JUSTFLOAT;
  main_vofa_stage_start_tick = HAL_GetTick();
  main_vofa_last_text_tick = 0U;
  main_vofa_test_seq = 0U;
  main_vofa_send_error_count = 0U;
  (void)Main_SendText("[DIAG] START MCU/INIT/DATA each JUSTFLOAT->RAWDATA->UART, then JUSTFLOAT stream\r\n");
  Main_StageAnnounce(main_vofa_stage);
}

static void Main_VofaLoop(void)
{
  uint32_t now_tick = HAL_GetTick();
  if (main_vofa_poll_armed == 0U) {
    main_vofa_last_poll_tick = now_tick - MAIN_VOFA_POLL_PERIOD_MS;
    main_vofa_poll_armed = 1U;
  }
  if ((uint32_t)(now_tick - main_vofa_last_poll_tick) < MAIN_VOFA_POLL_PERIOD_MS) return;
  main_vofa_last_poll_tick = now_tick;

  Main_VofaAdvanceStage(now_tick);

  TAM_DiagSnapshot_t snapshot;
  int16_t mag[3] = {0};
  uint8_t has_new_data = 0U;
  float jf_payload5[5] = {0.0f};
  uint8_t raw_payload[8] = {0};
  char text_line[160] = {0};
  bool send_ok = false;

  TAM_GetDiagSnapshot(&snapshot);
  TAM_GetLatestMag(mag, &has_new_data);

  if (has_new_data == 0U) {
    mag[0] = snapshot.mag[0];
    mag[1] = snapshot.mag[1];
    mag[2] = snapshot.mag[2];
    has_new_data = snapshot.mag_valid;
  }

  if (main_vofa_stage == MAIN_VOFA_STAGE_MCU_JUSTFLOAT) {
    jf_payload5[0] = 1001.0f;
    jf_payload5[1] = (float)main_vofa_test_seq;
    jf_payload5[2] = (float)now_tick;
    jf_payload5[3] = (float)(SystemCoreClock / 1000000U);
    jf_payload5[4] = (float)main_vofa_send_error_count;
    main_vofa_handle.fmt = VOFA_FMT_JUSTFLOAT;
    send_ok = VOFA_SendData(&main_vofa_handle, jf_payload5, 5U, main_vofa_types5);
    Main_RecordSendResult(send_ok);
  } else if (main_vofa_stage == MAIN_VOFA_STAGE_MCU_RAWDATA) {
    raw_payload[0] = 0xA1U;
    raw_payload[1] = 0x01U;
    raw_payload[2] = (uint8_t)(main_vofa_test_seq & 0xFFU);
    raw_payload[3] = (uint8_t)((main_vofa_test_seq >> 8) & 0xFFU);
    raw_payload[4] = (uint8_t)(now_tick & 0xFFU);
    raw_payload[5] = (uint8_t)((now_tick >> 8) & 0xFFU);
    raw_payload[6] = (uint8_t)(main_vofa_send_error_count & 0xFFU);
    raw_payload[7] = 0x5AU;
    main_vofa_handle.fmt = VOFA_FMT_RAWDATA;
    send_ok = VOFA_SendData(&main_vofa_handle, raw_payload, 8U, main_vofa_raw_types);
    Main_RecordSendResult(send_ok);
  } else if (main_vofa_stage == MAIN_VOFA_STAGE_MCU_UART) {
    (void)snprintf(text_line,
                   sizeof(text_line),
                   "[MCU] seq=%u tick=%lu sysclk=%luMHz tx_err=%u\r\n",
                   main_vofa_test_seq,
                   (unsigned long)now_tick,
                   (unsigned long)(SystemCoreClock / 1000000U),
                   main_vofa_send_error_count);
    send_ok = Main_SendText(text_line);
    Main_RecordSendResult(send_ok);
  } else if (main_vofa_stage == MAIN_VOFA_STAGE_INIT_JUSTFLOAT) {
    jf_payload5[0] = 2001.0f;
    jf_payload5[1] = (float)snapshot.sensor_ready;
    jf_payload5[2] = (float)snapshot.i2c_online;
    jf_payload5[3] = (float)snapshot.chip_id;
    jf_payload5[4] = (float)snapshot.reinit_active;
    main_vofa_handle.fmt = VOFA_FMT_JUSTFLOAT;
    send_ok = VOFA_SendData(&main_vofa_handle, jf_payload5, 5U, main_vofa_types5);
    Main_RecordSendResult(send_ok);
  } else if (main_vofa_stage == MAIN_VOFA_STAGE_INIT_RAWDATA) {
    raw_payload[0] = 0xA2U;
    raw_payload[1] = snapshot.sensor_ready;
    raw_payload[2] = snapshot.i2c_online;
    raw_payload[3] = snapshot.chip_id;
    raw_payload[4] = snapshot.reg_status;
    raw_payload[5] = snapshot.reg_ctrl1;
    raw_payload[6] = snapshot.reg_ctrl2;
    raw_payload[7] = snapshot.reinit_active;
    main_vofa_handle.fmt = VOFA_FMT_RAWDATA;
    send_ok = VOFA_SendData(&main_vofa_handle, raw_payload, 8U, main_vofa_raw_types);
    Main_RecordSendResult(send_ok);
  } else if (main_vofa_stage == MAIN_VOFA_STAGE_INIT_UART) {
    (void)snprintf(text_line,
                   sizeof(text_line),
                   "[INIT] ready=%u i2c=%u id=0x%02X st=0x%02X c1=0x%02X c2=0x%02X reinit=%u try=%u\r\n",
                   snapshot.sensor_ready,
                   snapshot.i2c_online,
                   snapshot.chip_id,
                   snapshot.reg_status,
                   snapshot.reg_ctrl1,
                   snapshot.reg_ctrl2,
                   snapshot.reinit_active,
                   snapshot.reinit_try_count);
    send_ok = Main_SendText(text_line);
    Main_RecordSendResult(send_ok);
  } else if (main_vofa_stage == MAIN_VOFA_STAGE_DATA_JUSTFLOAT) {
    jf_payload5[0] = (float)mag[0];
    jf_payload5[1] = (float)mag[1];
    jf_payload5[2] = (float)mag[2];
    jf_payload5[3] = (float)has_new_data;
    jf_payload5[4] = (float)main_vofa_send_error_count;
    main_vofa_handle.fmt = VOFA_FMT_JUSTFLOAT;
    send_ok = VOFA_SendData(&main_vofa_handle, jf_payload5, 5U, main_vofa_types5);
    Main_RecordSendResult(send_ok);
  } else if (main_vofa_stage == MAIN_VOFA_STAGE_DATA_RAWDATA) {
    raw_payload[0] = 0xA3U;
    raw_payload[1] = has_new_data;
    raw_payload[2] = (uint8_t)(mag[0] & 0xFF);
    raw_payload[3] = (uint8_t)((mag[0] >> 8) & 0xFF);
    raw_payload[4] = (uint8_t)(mag[1] & 0xFF);
    raw_payload[5] = (uint8_t)((mag[1] >> 8) & 0xFF);
    raw_payload[6] = (uint8_t)(mag[2] & 0xFF);
    raw_payload[7] = (uint8_t)((mag[2] >> 8) & 0xFF);
    main_vofa_handle.fmt = VOFA_FMT_RAWDATA;
    send_ok = VOFA_SendData(&main_vofa_handle, raw_payload, 8U, main_vofa_raw_types);
    Main_RecordSendResult(send_ok);
  } else if (main_vofa_stage == MAIN_VOFA_STAGE_DATA_UART) {
    (void)snprintf(text_line,
                   sizeof(text_line),
                   "[DATA] valid=%u mx=%d my=%d mz=%d tx_err=%u\r\n",
                   has_new_data,
                   mag[0],
                   mag[1],
                   mag[2],
                   main_vofa_send_error_count);
    send_ok = Main_SendText(text_line);
    Main_RecordSendResult(send_ok);
  } else {
    if (has_new_data == 0U) {
      if ((uint32_t)(now_tick - main_vofa_last_text_tick) >= 1000U) {
        main_vofa_last_text_tick = now_tick;
        if (TAM_IsSensorReady() == 0U) {
          send_ok = Main_SendText("[DIAG] SENSOR_NOT_READY\r\n");
        } else {
          send_ok = Main_SendText("[DIAG] SENSOR_READY_NO_DATA\r\n");
        }
        Main_RecordSendResult(send_ok);
      }
      main_vofa_test_seq++;
      return;
    }

    float vofa_channels[7];
    vofa_channels[0] = (float)mag[0];
    vofa_channels[1] = (float)mag[1];
    vofa_channels[2] = (float)mag[2];
    vofa_channels[3] = qmc5883p_from_fs8_to_gauss(mag[0]);
    vofa_channels[4] = qmc5883p_from_fs8_to_gauss(mag[1]);
    vofa_channels[5] = qmc5883p_from_fs8_to_gauss(mag[2]);
    vofa_channels[6] = (float)now_tick;

    main_vofa_handle.fmt = VOFA_FMT_JUSTFLOAT;
    send_ok = VOFA_SendData(&main_vofa_handle, vofa_channels, 7U, main_vofa_types);
    Main_RecordSendResult(send_ok);
  }

  main_vofa_test_seq++;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  TAM_Init();
  TAM_ConsoleInit();
  Main_VofaInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    TAM_Loop();
    TAM_ConsoleLoop();
    Main_VofaLoop();
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK1_CIRCLE_TEST_MODE   0   /* 1=任务1同心圆对准专项测试 */
#define SERVO_TEST_MODE          0   /* 1=CH2专项测试: 电机禁用, 仅CH2步进旋转; 0=正式比赛 */
#define QR_SCAN_TEST_MODE        0   /* 1=原地扫码专项测试: 按钮触发 + LCD显示原始字节 */
#define SERVO_TEST_CH2_INIT_US   500U  /* 测试初始值 */
#define SERVO_TEST_CH2_STEP_US   400U  /* 每按一次 START_BTN 步进量 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#include "HWT101.h"
#include "LCD.h"
#include "ChassisMotor.h"
#include "LineFollow.h"
#include "CompetitionTask.h"
#include "ProjectService.h"
#include "Servo4.h"
#include "VofaTune.h"
#include "GM65Scanner.h"

int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_FSMC_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM8_Init();
    MX_TIM7_Init();

    /* Init LCD */
    LCD_Init();
    LCD_FillScreen(LCD_BLACK);
    HAL_Delay(1000);

    /* Init Servo (TIM9 software PWM) */
    Servo4_Init();
    /* CH2 上电立即回零: Servo4_Init() 使所有通道 PWM 为 0 (无保持力矩),
     * 不归零则转盘初始槽位物理上不确定. */
    Servo4_RotationHome();
    /* CH3..CH7 must be open as soon as the controller powers up, including
     * while waiting for the start button.  Task2 changes to its calibrated
     * open value at entry and closes only after all trophies are collected. */
    Servo4_LittleAllOpen();

    /* Init Motor & Encoder */
    ChassisMotor_Init(0.62f, 0.06f, 0.00f);

#if TASK1_CIRCLE_TEST_MODE
    /* 使用任务1正式的 VisionAlignment 配置和底盘控制。
     * 上电自动开始一次；对准成功或超时后停车，按键可重新测试。 */
    LCD_FillScreen(LCD_BLACK);
    LCD_DrawString_NoDMA(1, 10, "TASK1 CIRCLE TEST", LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 220, "BTN: RESTART", LCD_GREEN, LCD_BLACK);
    CompetitionTask_Init();
    ProjectService_StartTask1CircleTest();
#if VOFA_TUNE_ENABLE
    VofaTune_Init();
#endif
    {
        uint8_t last_pressed = 0U;
        for (;;)
        {
            uint8_t pressed = (HAL_GPIO_ReadPin(START_BTN_GPIO_Port,
                                                START_BTN_Pin) == GPIO_PIN_RESET);
            if (pressed && !last_pressed)
            {
                HAL_Delay(50U);
                ProjectService_StartTask1CircleTest();
            }
            last_pressed = pressed;
            CompetitionTask_Process();
#if VOFA_TUNE_ENABLE
            VofaTune_Process();
#endif
            HAL_Delay(10U);
        }
    }
#elif QR_SCAN_TEST_MODE
    /* ---- 原地扫码专项测试 ----
     * 每按 START_BTN 触发一次扫码, LCD 显示:
     * ST 状态 (0=IDLE 1=WAITING 2=READY 3=TIMEOUT 4=OVERFLOW)
     * RX 原始字节数, 最近8字节 HEX (按时间顺序), 解析文本.
     * 现象: 扫码器滴滴响但 MCU 未收到 → RX 不涨/字节乱. */
    LCD_FillScreen(LCD_BLACK);
    LCD_DrawString_NoDMA(1, 10, "QR SCAN TEST", LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 220, "BTN: SCAN", LCD_GREEN, LCD_BLACK);
    GM65_Init(&huart1);
    (void)GM65_StartReceiveIT();
    {
        uint8_t last_pressed = 0U;
        char buf[24];
        for (;;)
        {
            uint8_t pressed = (HAL_GPIO_ReadPin(START_BTN_GPIO_Port,
                                                START_BTN_Pin) == GPIO_PIN_RESET);
            if (pressed && !last_pressed)
            {
                HAL_Delay(50);
                (void)GM65_StartScan(3000U);
            }
            last_pressed = pressed;

            GM65_Process();

            sprintf(buf, "ST:%d RX:%lu", (int)GM65_GetStatus(),
                    (unsigned long)gm65_diag_rx_bytes);
            LCD_DrawString_NoDMA(1, 40, buf, LCD_WHITE, LCD_BLACK);
            {
                uint8_t i;
                uint8_t idx = (uint8_t)((gm65_diag_last_index + 1U) & 7U);
                char *p = buf;
                for (i = 0U; i < 8U; i++)
                {
                    p += sprintf(p, "%02X ", gm65_diag_last_bytes[idx]);
                    idx = (uint8_t)((idx + 1U) & 7U);
                }
                LCD_DrawString_NoDMA(1, 60, buf, LCD_WHITE, LCD_BLACK);
            }
            if (GM65_GetStatus() == GM65_READY)
            {
                char text[GM65_TEXT_CAPACITY];
                if (GM65_TakeText(text, sizeof(text)))
                {
                    LCD_DrawString_NoDMA(1, 80, "TXT:", LCD_WHITE, LCD_BLACK);
                    LCD_DrawString_NoDMA(1, 100, text, LCD_GREEN, LCD_BLACK);
                }
            }
            HAL_Delay(50);
        }
    }
#elif SERVO_TEST_MODE
    /* ---- CH2 专项测试状态: 电机禁用, 仅 CH2 旋转 ----
     * 上电 CH2=500; 每按 START_BTN +400, 2100 后回到 500 循环.
     * 不启动 TIM7/ProjectService, 四轮保持 0 PWM 禁用. */
    LCD_FillScreen(LCD_BLACK);
    LCD_DrawString_NoDMA(1, 40, "SERVO CH2 TEST", LCD_WHITE, LCD_BLACK);
    {
        uint8_t step = 0U;
        uint8_t last_pressed = 0U;
        char buf[32];

        Servo4_SetPulse(2U, SERVO_TEST_CH2_INIT_US);
        for (;;)
        {
            uint8_t pressed = (HAL_GPIO_ReadPin(START_BTN_GPIO_Port,
                                                START_BTN_Pin) == GPIO_PIN_RESET);

            if (pressed && !last_pressed)
            {
                HAL_Delay(50);  /* debounce */
                step = (step >= 4U) ? 0U : (uint8_t)(step + 1U);
                Servo4_SetPulse(2U, (uint16_t)(SERVO_TEST_CH2_INIT_US
                                + (uint16_t)SERVO_TEST_CH2_STEP_US * step));
            }
            last_pressed = pressed;

            sprintf(buf, "STEP:%d PWM:%u   ", step, (unsigned)Servo4_GetPulse(2U));
            LCD_DrawString_NoDMA(1, 80, buf, LCD_WHITE, LCD_BLACK);
            HAL_Delay(10);
        }
    }
#else
    /* Wait for START_BTN (PG10, active-low) press before running. */
    LCD_DrawString_NoDMA(1, 120, "Press BTN to Start", LCD_WHITE, LCD_BLACK);
    while (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) != GPIO_PIN_RESET)
    {
        HAL_Delay(10);
    }
    HAL_Delay(50);  /* debounce */
    LCD_FillScreen(LCD_BLACK);

    CompetitionTask_Init();

#if VOFA_TUNE_ENABLE
    VofaTune_Init();
#endif

    while (1)
    {
        CompetitionTask_Process();
#if VOFA_TUNE_ENABLE
        VofaTune_Process();
#endif
        HAL_Delay(10);
    }
#endif
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  else if (htim->Instance == TIM7)
  {
    ProjectService_OnTimerElapsed(htim);
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

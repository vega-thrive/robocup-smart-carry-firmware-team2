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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#include <stdio.h>
#include "TestMode.h"
#include "HWT101.h"
#include "LCD.h"
#include "ChassisMotor.h"
#include "LineFollow.h"
#include "CompetitionTask.h"
#include "ProjectService.h"
#include "Servo4.h"
#include "Task1Mechanism.h"
#include "TrophyMechanism.h"
#include "MaxiCam.h"
#include "Sensor.h"

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

#if ARM_SERVO_TEST_MODE
    /* 大臂+小臂专项测试: 仅初始化舵机, 从 Task0 缓降至 FLAT.
     * 底盘、编码器、巡线、IMU、摄像头均不启动. */
    Servo4_Init();
    {
        const Task1ArmPose *t0 = Task1Mechanism_GetTask0Pose();
        const Task1ArmPose *work = Task1Mechanism_GetWorkingPose();
        Servo4_SetPulse(0U, t0->big_arm_us);
        Servo4_SetPulse(1U, t0->small_arm_us);

        LCD_DrawString_NoDMA(1, 10, "ARM SERVO TEST", LCD_WHITE, LCD_BLACK);
        LCD_DrawString_NoDMA(1, 40, "CH0: Big Arm", LCD_GREEN, LCD_BLACK);
        LCD_DrawString_NoDMA(1, 60, "CH1: Small Arm", LCD_GREEN, LCD_BLACK);
        LCD_DrawString_NoDMA(1, 100, "POSE: FLAT         ", LCD_WHITE, LCD_BLACK);
        LCD_DrawString_NoDMA(1, 200, "CHASSIS DISABLED", LCD_RED, LCD_BLACK);

        Servo4_ArmRampTo(work->big_arm_us, work->small_arm_us);

        while (1)
        {
            char arm_buf[32];
            (void)sprintf(arm_buf, "CH0:%u CH1:%u    ",
                          (unsigned)work->big_arm_us,
                          (unsigned)work->small_arm_us);
            LCD_DrawString_NoDMA(1, 140, arm_buf, LCD_GREEN, LCD_BLACK);
            HAL_Delay(500U);
        }
    }
#elif CIRCLE_UART_TEST_MODE
    MaxiCam_Frame circle_frame;
    HAL_StatusTypeDef circle_rx_status;
    uint32_t circle_next_command_ms;
    char circle_text[40];

    /* Dedicated receive-only diagnostics. Do not initialize servos, chassis,
     * encoders, line following, IMU or the competition state machine. */
    MaxiCam_Init(&huart4);
    circle_rx_status = MaxiCam_StartReceiveIT();

    LCD_DrawString_NoDMA(1, 10, "CIRCLE UART RX TEST",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 40,
                         (circle_rx_status == HAL_OK) ? "RX IT: OK" : "RX IT: ERROR",
                         (circle_rx_status == HAL_OK) ? LCD_GREEN : LCD_RED,
                         LCD_BLACK);
    LCD_DrawString_NoDMA(1, 70, "CAMERA BOOT 3S",
                         LCD_GREEN, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 220, "MOTOR/SERVO DISABLED",
                         LCD_RED, LCD_BLACK);

    HAL_Delay(3000U);
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    HAL_Delay(100U);
    (void)MaxiCam_SetMode(MAXICAM_MODE_CIRCLE);
    circle_next_command_ms = HAL_GetTick() + 1000U;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (MaxiCam_GetLatest(&circle_frame, 1000U))
        {
            uint32_t age = now - circle_frame.received_at_ms;

            (void)sprintf(circle_text, "M:%u S:%u X:%u Y:%u       ",
                          (unsigned int)circle_frame.data[0],
                          (unsigned int)circle_frame.data[1],
                          (unsigned int)circle_frame.data[2],
                          (unsigned int)circle_frame.data[3]);
            LCD_DrawString_NoDMA(1, 70, circle_text, LCD_WHITE, LCD_BLACK);

            (void)sprintf(circle_text, "SEQ:%lu AGE:%lums         ",
                          (unsigned long)circle_frame.sequence,
                          (unsigned long)age);
            LCD_DrawString_NoDMA(1, 100, circle_text, LCD_GREEN, LCD_BLACK);

            (void)sprintf(circle_text, "D4:%u D5:%u               ",
                          (unsigned int)circle_frame.data[4],
                          (unsigned int)circle_frame.data[5]);
            LCD_DrawString_NoDMA(1, 130, circle_text, LCD_WHITE, LCD_BLACK);

            if (circle_frame.data[0] != 1U)
            {
                LCD_DrawString_NoDMA(1, 160, "BAD MODE                  ",
                                     LCD_RED, LCD_BLACK);
            }
            else if ((circle_frame.data[2] >= 160U) ||
                     (circle_frame.data[3] >= 120U))
            {
                LCD_DrawString_NoDMA(1, 160, "BAD XY                    ",
                                     LCD_RED, LCD_BLACK);
            }
            else
            {
                LCD_DrawString_NoDMA(1, 160, "PACKET + XY OK            ",
                                     LCD_GREEN, LCD_BLACK);
            }

            if ((circle_frame.data[2] >= 75U) &&
 
 (circle_frame.data[2] <= 85U) &&
                (circle_frame.data[3] >= 55U) &&
                (circle_frame.data[3] <= 65U))
            {
                LCD_DrawString_NoDMA(1, 190, "NEAR CENTER (80,60)       ",
                                     LCD_GREEN, LCD_BLACK);
            }
            else
            {
                LCD_DrawString_NoDMA(1, 190, "MOVE TARGET TO CENTER     ",
                                     LCD_WHITE, LCD_BLACK);
            }
        }
        else
        {
            LCD_DrawString_NoDMA(1, 70, "M:- S:- X:- Y:-           ",
                                 LCD_WHITE, LCD_BLACK);
            LCD_DrawString_NoDMA(1, 100, "SEQ:0 / FRAME STALE       ",
                                 LCD_RED, LCD_BLACK);
            LCD_DrawString_NoDMA(1, 130, "WAIT FF AA ... FE BB      ",
                                 LCD_WHITE, LCD_BLACK);
            LCD_DrawString_NoDMA(1, 160, "NO VALID PACKET           ",
                                 LCD_RED, LCD_BLACK);
            LCD_DrawString_NoDMA(1, 190, "CHECK CAMERA TX -> PC11   ",
                                 LCD_WHITE, LCD_BLACK);

            /* Retry the mode command only while no fresh packet is available. */
            if ((int32_t)(now - circle_next_command_ms) >= 0)
            {
                (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
                HAL_Delay(20U);
                (void)MaxiCam_SetMode(MAXICAM_MODE_CIRCLE);
                circle_next_command_ms = HAL_GetTick() + 1000U;
            }
        }

        HAL_Delay(50U);
    }
#elif SERVO_RANGE_TEST_MODE
    static const uint16_t range_test_pulse[] =
    {
        1500U, 1750U, 2000U, 2250U, 2500U,
        1500U, 1250U, 1000U, 750U, 500U, 1500U
    };
    uint8_t range_index;

    /* Only Gray2 CH3 / PA4 is enabled.  All chassis and vision functions are
     * deliberately excluded from this test branch. */
    Servo4_Init();
    LCD_DrawString_NoDMA(1, 40, "CH3 RANGE TEST",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 80, "CURRENT PWM:",
                         LCD_GREEN, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 160, "CHASSIS DISABLED",
                         LCD_RED, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 200, "POWER OFF IF HIT",
                         LCD_RED, LCD_BLACK);

    for (range_index = 0U;
         range_index < (uint8_t)(sizeof(range_test_pulse) /
                                 sizeof(range_test_pulse[0]));
         range_index++)
    {
        Servo4_SetPulse(2U, range_test_pulse[range_index]);
        LCD_DrawNumber((int32_t)range_test_pulse[range_index],
                       120U, 80U, LCD_GREEN, LCD_BLACK, ALIGN_LEFT);
        HAL_Delay((range_index == 0U) ? 5000U : 3000U);
    }

    LCD_DrawString_NoDMA(1, 120, "DONE: HOLD 1500",
                         LCD_WHITE, LCD_BLACK);
    while (1)
    {
        Servo4_SetPulse(2U, 1500U);
        HAL_Delay(1000U);
    }
#elif COLOR_SERVO_TEST_MODE
    MaxiCam_Frame color_frame;
    uint32_t color_last_sequence = 0U;
    uint8_t color_count = 0U;
    /* uint8_t color_seen_mask = 0U;  原逻辑使用, 已注释 */
    /* uint8_t color_finished = 0U;   原逻辑使用, 已注释 */
    char color_count_text[] = "COLOR COUNT: 0";

    /* The chassis, QR scanner and complete race state machine are deliberately
     * not initialized in this test branch. */
    Servo4_Init();
    Servo4_SetPulse(COLOR_SERVO_CHANNEL, COLOR_SERVO_HOME_US);
    MaxiCam_Init(&huart4);
    (void)MaxiCam_StartReceiveIT();

    LCD_DrawString_NoDMA(1, 40, "COLOR SERVO TEST",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 80, "CAMERA BOOT 4S  ",
                         LCD_GREEN, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 120, "CH2: 850 us",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 200, color_count_text,
                         LCD_GREEN, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 160, "CHASSIS DISABLED",
                         LCD_RED, LCD_BLACK);

    /* MaxiCam may still be booting or may already be inside its circle-mode
     * loop.  Exit that loop first, then request COLOR from the outer loop. */
    HAL_Delay(COLOR_CAMERA_BOOT_MS);
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    HAL_Delay(200U);
    (void)MaxiCam_SetMode(MAXICAM_MODE_COLOR);
    LCD_DrawString_NoDMA(1, 80, "WAIT COLOR 21-25",
                         LCD_GREEN, LCD_BLACK);

    /* ---- 简化测试: 识别到任意颜色(21-25) → CH2 转一步 ----
     * 原逻辑(5色去重计数/5色齐切回CIRCLE)已注释保留, 需要时恢复. */
    while (1)
    {
        if (MaxiCam_GetLatest(&color_frame, 250U)
            && (color_frame.sequence != color_last_sequence))
        {
            color_last_sequence = color_frame.sequence;
            if ((color_frame.data[0] >= 21U)
                && (color_frame.data[0] <= 25U))
            {
                /* 每识别到一次(允许重复), 转盘前进一档, 超过第4档回零点. */
                color_count = (color_count >= 4U) ? 0U : (uint8_t)(color_count + 1U);
                Servo4_SetPulse(COLOR_SERVO_CHANNEL,
                                (uint16_t)(COLOR_SERVO_HOME_US
                                           + COLOR_SERVO_STEP_US * color_count));
                color_count_text[13] = (char)('0' + color_count);
                LCD_DrawString_NoDMA(1, 200, color_count_text,
                                     LCD_GREEN, LCD_BLACK);

                /* ==== 原逻辑 (注释保留) ====
                uint8_t color_bit = (uint8_t)(1U << (color_frame.data[0] - 21U));
                if ((color_seen_mask & color_bit) != 0U)
                {
                    HAL_Delay(10);
                    continue;
                }
                color_seen_mask |= color_bit;
                color_count++;
                if (color_count < 5U)
                {
                    Servo4_SetPulse(COLOR_SERVO_CHANNEL,
                                    (uint16_t)(COLOR_SERVO_HOME_US
                                               + COLOR_SERVO_STEP_US * color_count));
                }
                if (color_count >= 5U)
                {
                    color_finished = 1U;
                    (void)MaxiCam_SetMode(MAXICAM_MODE_CIRCLE);
                    LCD_DrawString_NoDMA(1, 80, "5 COLORS -> CIRCLE",
                                         LCD_GREEN, LCD_BLACK);
                }
                else
                {
                    LCD_DrawString_NoDMA(1, 80, "WAIT NEXT COLOR ",
                                         LCD_GREEN, LCD_BLACK);
                }
                ==== 原逻辑结束 ==== */
            }
        }
        HAL_Delay(10);
    }
#elif SERVO_CHANNEL_TEST_MODE
    /* 仅 CH2/PA4 输出固定脉宽, 其余 7 通道 PWM off, 底盘不初始化. */
    Servo4_Init();
    Servo4_SetPulse(2U, SERVO_TEST_LOW_US);
    LCD_DrawString_NoDMA(1, 40, "ROTATE SERVO CH2",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 80, "PA4: 500us FIXED",
                         LCD_GREEN, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 120, "OTHER PWM OFF",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 160, "CHASSIS DISABLED",
                         LCD_RED, LCD_BLACK);
    while (1)
    {
        Servo4_SetPulse(2U, SERVO_TEST_LOW_US);
        HAL_Delay(10);
    }
#elif ENCODER_ROUTE_TEST_MODE
    /* 编码器定距专项测试: 从 ROTATE_90 开始, 跑完所有编码器运动段
     * 到 FINAL_LEFT_40 停车. 跳过视觉对准/扫码/颜色识别/巡线,
     * 摄像头在充电时可用. */
    Servo4_Init();
    ChassisMotor_Init(0.62f, 0.06f, 0.00f);
    LineFollow_Init();
    HWT101_UART_Init();
    HeadingHold_Init();
    HWT101_getAngle_Reset();
    HeadingHold_Reset();
    ChassisPos_ResetEncoder();
    ChassisPos_ResetVelocityPid();

    LCD_DrawString_NoDMA(1, 40, "ENCODER ROUTE TEST",
                         LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 80, "ROTATE90->FINAL LEFT40",
                         LCD_GREEN, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 120, "VISION SKIP ON",
                         LCD_GREEN, LCD_BLACK);
    LCD_DrawString_NoDMA(1, 160, "Press BTN to Start",
                         LCD_WHITE, LCD_BLACK);

    while (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) != GPIO_PIN_RESET)
    {
        HAL_Delay(10);
    }
    HAL_Delay(50);
    LCD_FillScreen(LCD_BLACK);

    /* 等待陀螺仪稳定后记录初始航向 (供 FINAL_ROTATE_ALIGN 回中用) */
    HAL_Delay(200);
    ProjectService_SetInitialHeading();
    ProjectService_SetRouteState(ROUTE_ROTATE_90);
    ProjectService_SetVisionSkip(1U);
    ChassisPos_DrawStatus(1, "Rotate 90");

    HAL_TIM_Base_Start_IT(&htim7);

    while (1)
    {
        ProjectService_Run();
        HAL_Delay(10);
    }
#else
    /* Init Servo (TIM9 software PWM) */
    Servo4_Init();
    /* CH3..CH7 must be open as soon as the controller powers up, including
     * while waiting for the start button.  Task2 changes to its calibrated
     * open value at entry and closes only after all trophies are collected. */
    Servo4_LittleAllOpen();

    /* Init Motor & Encoder */
    ChassisMotor_Init(0.62f, 0.06f, 0.00f);

#if HARDWARE_DEBUG_MODE
    /* 硬件调试: 上电直接舒展舵机, 不跑路线, 不等按键 */
    {
        const Task1ArmPose *pose = Task1Mechanism_GetWorkingPose();
        Servo4_SetPulse(0U, pose->big_arm_us);
        Servo4_SetPulse(1U, pose->small_arm_us);
        Servo4_LittleAllOpen();   /* CH3~CH7 小爪子张开 */
    }
    LCD_DrawString_NoDMA(1, 120, "HW DEBUG", LCD_WHITE, LCD_BLACK);
    while (1)
    {
        HAL_Delay(10);
    }
#else
    /* Wait for START_BTN (PG10, active-low) press before running */
    LCD_DrawString_NoDMA(1, 120, "Press BTN to Start", LCD_WHITE, LCD_BLACK);
    while (HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin) != GPIO_PIN_RESET)
    {
        HAL_Delay(10);
    }
    HAL_Delay(50);  /* debounce */
    LCD_FillScreen(LCD_BLACK);

    CompetitionTask_Init();

    while (1)
    {
        CompetitionTask_Process();
        HAL_Delay(10);
    }
#endif
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

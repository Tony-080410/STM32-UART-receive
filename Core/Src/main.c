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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 和 A 板约定的帧格式：[0]=0xAA 帧头  [1]=高字节  [2]=低字节  [3]=异或校验 */
#define FRAME_HEAD 0xAAU
/* 舵机脉宽范围（微秒）。500~2500 对应 0~180 度，实际以你手上的舵机实测为准 */
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U
/* 舵机回中位 */
#define SERVO_CENTER_US 1500U
/* A 板发过来的是 12 位 ADC 值 */
#define ADC_FULL_SCALE 4095U
/* PWM 更新周期，20ms = 50Hz */
#define APPLY_PERIOD_MS 20U
/* 超过这么久没收到合法帧，判定为链路断开 */
#define LINK_TIMEOUT_MS 500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---- 接收状态机，只在中断里使用 ---- */
static uint8_t rxByte;       /* HAL 逐字节接收的落点 */
static uint8_t rxState = 0U; /* 0=等帧头 1=等高字节 2=等低字节 3=等校验 */
static uint8_t rxHigh;
static uint8_t rxLow;

/* ---- 中断和主循环共享，必须加 volatile ---- */
static volatile uint16_t frameValue = 2048U; /* 最近收到的电位器值 */
static volatile uint8_t frameReady = 0U;     /* 1 = 有新数据待应用 */
static volatile uint32_t lastFrameTick = 0U; /* 最近收到合法帧的时刻 */
static volatile uint8_t linkLost = 0U;       /* 1 = 链路断了 */

/* ---- 只在主循环里使用 ---- */
static uint32_t lastApplyTick = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  把 0~4095 线性映射成 500~2500 的定时器比较值
 *         （TIM3 已配成 1 个计数 = 1us，所以数值直接就是微秒）
 */
static uint32_t Servo_MapToPulse(uint16_t adc) {
    return (uint32_t)SERVO_MIN_US
           + ((uint32_t)adc * (uint32_t)(SERVO_MAX_US - SERVO_MIN_US)) / (uint32_t)ADC_FULL_SCALE;
}

/**
 * @brief  每收到一个字节进一次（中断上下文，越短越好）
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    uint8_t b;

    if (huart->Instance != USART1) {
        return;
    }

    b = rxByte;

    switch (rxState) {
    case 0: /* 等帧头 */
        if (b == FRAME_HEAD) {
            rxState = 1U;
        }
        break;

    case 1: /* 高字节 */
        rxHigh = b;
        rxState = 2U;
        break;

    case 2: /* 低字节 */
        rxLow = b;
        rxState = 3U;
        break;

    case 3: /* 校验字节 */
        /* 校验：异或要对得上，而且高字节必须 <= 0x0F（值只有 12 位） */
        if ((rxHigh <= 0x0FU) && ((uint8_t)(rxHigh ^ rxLow) == b)) {
            frameValue = (uint16_t)(((uint16_t)rxHigh << 8) | (uint16_t)rxLow);
            lastFrameTick = HAL_GetTick();
            frameReady = 1U;
        }
        rxState = 0U; /* 不管对错，都回去重新找帧头 */
        break;

    default:
        rxState = 0U;
        break;
    }

    /* 关键：重新武装，否则只能收到这一个字节 */
    (void)HAL_UART_Receive_IT(&huart1, &rxByte, 1U);
}

/**
 * @brief  串口出错时调用。本版本 HAL 遇到 ORE 会中止接收，必须在这里重新武装
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart); /* 清溢出标志 */
    rxState = 0U;                    /* 丢掉半截帧，重新同步 */
    (void)HAL_UART_Receive_IT(&huart1, &rxByte, 1U);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    /* 先把比较值设成中位，再启动 PWM，避免上电瞬间舵机乱甩 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_CENTER_US);
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    /* 记两个时间戳，避免上电后立刻误判成链路断开 */
    lastFrameTick = HAL_GetTick();
    lastApplyTick = HAL_GetTick();

    /* 武装串口接收中断，开始收第一个字节 */
    if (HAL_UART_Receive_IT(&huart1, &rxByte, 1U) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if ((HAL_GetTick() - lastApplyTick) >= APPLY_PERIOD_MS) {
            lastApplyTick = HAL_GetTick();

            /* 收到新数据就更新舵机 */
            if (frameReady != 0U) {
                frameReady = 0U;
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Servo_MapToPulse(frameValue));
            }

            /* 链路检测：超时就置标志。舵机保持原位不动，不做任何动作 */
            linkLost = ((HAL_GetTick() - lastFrameTick) > LINK_TIMEOUT_MS) ? 1U : 0U;
        }
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

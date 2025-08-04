/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body adapted for TCD1304 Linear CCD driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed AS-IS per original reference implementation.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "adc.h"
#include "dma.h"
#include "usbd_cdc_if.h" // for CDC_Transmit_FS

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define CCDBUFFER_SIZE 6000
volatile uint16_t CCDPixelBuffer[CCDBUFFER_SIZE]; // buffer for ADC samples
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* none */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* none beyond buffer size */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* none */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* none additional */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* none */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Conversion complete callback: send raw buffer via USB CDC */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Transmit entire buffer as bytes. The PC side reconstructs 12-bit samples from the 16-bit words. */
  CDC_Transmit_FS((uint8_t*) CCDPixelBuffer, CCDBUFFER_SIZE * sizeof(uint16_t));
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* nothing here */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* nothing */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* nothing */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();     // USB depends on clocks; CubeMX-generated
  MX_TIM2_Init();           // ICG (master, 32-bit)
  MX_TIM3_Init();           // ϕM (master clock)
  MX_TIM4_Init();           // ADC trigger
  MX_DMA_Init();            // DMA must come before ADC init. Important. :contentReference[oaicite:13]{index=13}
  MX_ADC1_Init();           // ADC1 with external trigger from TIM4 CC4 and DMA continuous requests
  MX_TIM5_Init();           // SH (slave to TIM2, with polarity/inversion as required)

  /* USER CODE BEGIN 2 */
  /* Start all required PWM/timer signals in proper order with delay to create 600ns offset between ICG and SH */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // ϕM master clock. :contentReference[oaicite:14]{index=14}
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // ADC trigger (internal, no external output needed). :contentReference[oaicite:15]{index=15}
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // ICG. :contentReference[oaicite:16]{index=16}

  /* Insert 600ns delay between ICG and SH by offsetting TIM2 counter before SH start */
  __HAL_TIM_SET_COUNTER(&htim2, 66); // approximate 600ns at 84MHz clock (demonstrated value from PDF). :contentReference[oaicite:17]{index=17}

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3); // SH (slave to TIM2 via ITR0, with appropriate polarity). :contentReference[oaicite:18]{index=18}

  /* Start ADC DMA acquisition */
  /* Note: ADC is triggered by TIM4 CC4; its DMA will fill CCDPixelBuffer when conversions complete. */
  // actual start of ADC via DMA happens inside loop to ensure continuous reading as needed
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)CCDPixelBuffer, CCDBUFFER_SIZE); // kick off DMA+ADC. :contentReference[oaicite:19]{index=19}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // The ADC conversion complete callback will fire when the buffer fills and will transmit over USB.
    // Optionally add a small delay or polling if needed to control acquisition repetition rate.
    HAL_Delay(1); // small slack to avoid hammering if desired
    /* USER CODE END 3 */
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
/* (Additional user-space routines could go here if you expand functionality) */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    // optional: blink an LED to signal fatal error
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  // User can implement reporting here
}
#endif


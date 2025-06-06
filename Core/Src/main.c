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
#include "adc.h"
#include "comp.h"
#include "cordic.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define __FPU_PRESENT 1U
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
// void SET_ADC_TIM_FREQ(TIM_HandleTypeDef *htim, uint16_t wavelen, uint32_t
// freq);
void SET_DAC_TIM_FREQ(TIM_HandleTypeDef *htim, uint16_t wavelen, uint32_t freq,
                      uint16_t timclk_Mhz);
void SET_DAC_TIM_PHASE(TIM_HandleTypeDef *htim, uint16_t wavelen,
                       float32_t phase);
void ADD_DAC_TIM(TIM_HandleTypeDef *htim);
void DEC_DAC_TIM(TIM_HandleTypeDef *htim);

void hamming_window(q15_t *pWindow);
uint16_t *generate_sine_wave_table(uint32_t table_size, uint8_t periods,
                                   uint16_t amplitude);
uint16_t *generate_cosine_wave_table(uint32_t table_size, uint8_t periods,
                                     uint16_t amplitude);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// #define SAMPLE_LENGTH 1024
#define FFT_LENGTH 1024
#define SAMPLE_RATES 1000000
#define TABLE_LENGTH 100
// #define TABLE_PERIODS 5

// q15_t cmplx_ang_out[FFT_LENGTH * 2] = {0};
uint8_t adc_conv_finished = 0;
uint8_t fft_finished = 0;
uint8_t cal_phase_err_finished = 1;
uint8_t start_adc = 0;

// DDS *stm32_dds;

// AD983X ad9834;
uint8_t comp_triggered = 0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  uint16_t adc_value[FFT_LENGTH] = {0};
  uint32_t max_freq = 100;
  uint32_t sec_freq = 100;
  uint16_t table_periods_dac1;
  uint16_t table_periods_dac2;
  uint16_t *sintable_dac1 = NULL;
  uint16_t *sintable_dac2 = NULL;
  uint16_t *costable = NULL;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC4_Init();
  MX_CORDIC_Init();
  MX_OPAMP4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_DAC2_Init();
  MX_TIM8_Init();
  MX_COMP1_Init();
  MX_TIM4_Init();
  MX_COMP2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t str[] = "\r\n-------------USART_Sending------------------\r\n";
  HAL_UART_Transmit(&hlpuart1, str, sizeof(str) / sizeof(str[0]), 40);
  HAL_OPAMP_Start(&hopamp4);
  // HAL_OPAMP_Start(&hopamp5);
  HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);

  // HAL_DMAEx_EnableMuxRequestGenerator(&hdma);
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float adc_dc_offset = 0;

  arm_rfft_instance_q15 forwardS;
  // arm_rfft_instance_q15 backwardS;
  // arm_rfft_init_q15(&forwardS, FFT_LENGTH, 0, 1);
  arm_rfft_init_1024_q15(&forwardS, 0, 1);
  // arm_rfft_init_q15(&backwardS, FFT_LENGTH, 1, 0);
  // arm_rfft_init_1024_q15(&S, 0, 0);

  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, FFT_LENGTH);
  while (1) {
    if (!fft_finished && adc_conv_finished) {
      HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
      for (int i = 0; i < FFT_LENGTH; i++) {
        uint8_t adc_fmt_tmp_str[20] = {0};
        sprintf((char *)adc_fmt_tmp_str, "adc: %d\n", adc_value[i]);
        HAL_UART_Transmit(&hlpuart1, adc_fmt_tmp_str,
                          strlen((char *)adc_fmt_tmp_str), 40);
      }

      // 计算adc_value的直流偏置
      for (uint8_t i = 0; i < 100; i++) {
        adc_dc_offset += (adc_value[i] >> 4);
      }
      adc_dc_offset /= 100.0f;

      q15_t fft_in[FFT_LENGTH] = {0};
      q15_t fft_out[FFT_LENGTH * 2] = {0};
      q15_t cmplx_mag_out[FFT_LENGTH * 2] = {0};
      // memcpy(fft_in, adc_value, FFT_LENGTH);
      for (size_t i = 0; i < FFT_LENGTH; ++i) {
        fft_in[i] = (int16_t)((int32_t)adc_value[i] - 32768);
      }

      float32_t h_window[FFT_LENGTH] = {0};
      q15_t h_window_q15[FFT_LENGTH] = {0};
      // hamming_window(h_window);
      arm_hanning_f32(h_window, FFT_LENGTH);
      arm_float_to_q15(h_window, h_window_q15, FFT_LENGTH);
      arm_mult_q15(fft_in, h_window_q15, fft_in, FFT_LENGTH);
      arm_rfft_q15(&forwardS, fft_in, fft_out);
      arm_cmplx_mag_q15(fft_out, cmplx_mag_out, FFT_LENGTH / 2 + 1);

      uint8_t fft_banner_str[] =
          "\r\n-------------FFT_Result_Sending------------------\r\n";
      HAL_UART_Transmit(&hlpuart1, fft_banner_str,
                        strlen((char *)fft_banner_str), 40);
      for (int i = 0; i < FFT_LENGTH / 4; i++) {
        uint8_t fft_fmt_tmp_str[50] = {0};
        sprintf((char *)fft_fmt_tmp_str, "fft(%dk) : %d\n", i,
                cmplx_mag_out[i] * 10);
        HAL_UART_Transmit(&hlpuart1, fft_fmt_tmp_str,
                          strlen((char *)fft_fmt_tmp_str), 40);
      }

      // 去除直流分量
      cmplx_mag_out[0] = 0;
      cmplx_mag_out[1] = 0;

      // 查找频谱的第一个峰值
      q15_t max_result = 0;
      uint32_t max_index = 0;
      arm_max_q15(cmplx_mag_out, FFT_LENGTH / 2 + 1, &max_result, &max_index);
      cmplx_mag_out[max_index] = 0;

      // 查找频谱的第二个峰值
      q15_t sec_result = 0;
      uint32_t sec_index = 0;
      arm_max_q15(cmplx_mag_out, FFT_LENGTH / 2 + 1, &sec_result, &sec_index);

      // 将频率近似到5的整数倍
      max_index = ((max_index + 2) / 5) * 5;
      sec_index = ((sec_index + 2) / 5) * 5;

      max_freq = max_index * 1000;
      sec_freq = sec_index * 1000;

      uint8_t cmp_str_buf[50] = {0};
      sprintf((char *)cmp_str_buf, "max: %uk, sec: %uk\r\n",
              (unsigned int)max_index, (unsigned int)sec_index);
      HAL_UART_Transmit(&hlpuart1, cmp_str_buf, strlen((char *)cmp_str_buf),
                        40);

      table_periods_dac1 = max_index / 10;
      table_periods_dac2 = sec_index / 10;
      sintable_dac1 =
          generate_sine_wave_table(TABLE_LENGTH, table_periods_dac1, 1024);
      sintable_dac2 =
          generate_sine_wave_table(TABLE_LENGTH, table_periods_dac2, 1024);
      // costable = generate_cosine_wave_table(TABLE_LENGTH, 1, 1024);

      uint32_t trg_freq = TABLE_LENGTH * sec_freq / table_periods_dac1;
      uint8_t Trigger_UART_buf[50] = {0};
      sprintf((char *)Trigger_UART_buf, "Trigger freq: %d\n", (int)trg_freq);
      HAL_UART_Transmit(&hlpuart1, Trigger_UART_buf,
                        strlen((char *)Trigger_UART_buf), 40);

      HAL_COMP_Start(&hcomp2);
      SET_DAC_TIM_FREQ(&htim4, TABLE_LENGTH / table_periods_dac1, max_freq,
                       160);
      SET_DAC_TIM_FREQ(&htim7, TABLE_LENGTH / table_periods_dac2, sec_freq,
                       160);
      HAL_TIM_Base_Start(&htim4);
      HAL_TIM_Base_Start(&htim7);

      // uint8_t dactable_str_buf[50] = {0};
      // for (uint16_t i = 0; i < TABLE_LENGTH; i++) {
      //   sprintf((char *)dactable_str_buf, "dactable[%d]: %d\n", i,
      //   sintable[i]); HAL_UART_Transmit(&hlpuart1, dactable_str_buf,
      //                     strlen((char *)dactable_str_buf), 40);
      // }

      HAL_DAC_Start_DMA(&hdac4, DAC_CHANNEL_1, (uint32_t *)sintable_dac1,
                        TABLE_LENGTH, DAC_ALIGN_12B_R);
      HAL_DAC_Start_DMA(&hdac4, DAC_CHANNEL_2, (uint32_t *)sintable_dac2,
                        TABLE_LENGTH, DAC_ALIGN_12B_R);

      HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, FFT_LENGTH);

      q15_t adc_val_cmp_q15[FFT_LENGTH] = {0};
      for (size_t i = 0; i < FFT_LENGTH; ++i) {
        adc_val_cmp_q15[i] = (int16_t)((int32_t)adc_value[i] - 32768);
      }
      q15_t adc_val_min_q15 = 0;
      float32_t adc_val_max_float = 0;
      arm_min_no_idx_q15(adc_val_cmp_q15, 1000, &adc_val_min_q15);
      arm_q15_to_float(&adc_val_min_q15, &adc_val_max_float, 1);

      uint8_t adc_val_max_cmp_buf[40] = {0};
      sprintf((char *)adc_val_max_cmp_buf, "cmp_val: %d",
              (int)adc_val_max_float * 1000);
      HAL_UART_Transmit(&hlpuart1, adc_val_max_cmp_buf,
                        strlen((char *)adc_val_max_cmp_buf), HAL_MAX_DELAY);
      HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_L,
                       adc_val_min_q15 + 35768);

      // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,
      //                       __HAL_TIM_GET_AUTORELOAD(&htim4) / 2);
      // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
      // HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
      // HAL_COMP_Start(&hcomp1);
      adc_conv_finished = 0;
      fft_finished = 1;
    }
    /*if (fft_finished && !cal_phase_err_finished) {
          // HAL_COMP_Start(&hcomp1);
          start_adc = 0;
          while (!start_adc) // 等待 dac dma 完成一次传输到达零点
            ;

          // SET_ADC_TIM_FREQ(&htim3, 100, max_freq * 1000);
          HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, TABLE_LENGTH);
          adc_conv_finished = 0;
          while (!adc_conv_finished) // 等待 adc 采样完成
            ;

          // for (uint16_t i = 0; i < 1000; i++) {
          //   uint8_t adc_val_buf[30] = {0};
          //   int16_t adc_val = ((adc_value[i] >> 4) - (int16_t)adc_dc_offset);
          //   sprintf((char *)adc_val_buf, "adc_val: %d\n", adc_val);
          //   HAL_UART_Transmit(&hlpuart1, adc_val_buf, strlen((char
          //   *)adc_val_buf),
          //                     40);
          // }
          // for (uint16_t i = 0; i < 1000; i++) {
          //   uint8_t sin_buf[30] = {0};
          //   int16_t sin_val = (sintable[i] - 512);
          //   sprintf((char *)sin_buf, "sin_val: %d\n", sin_val);
          //   HAL_UART_Transmit(&hlpuart1, sin_buf, strlen((char *)sin_buf),
       40);
          // }

          // uint16_t adc_value_test[100] = {0};
          // for (uint32_t i = 0; i < 100; i++) {
          //   float value = 1024 * (sin(2 * PI * i / 100 + PI / 4) + 1.0f)
       / 2.0f;
          //   // 将值映射到 [100, amplitude]
          //   adc_value_test[i] = (uint16_t)value;
          // }
          ;

          /**
          // 计算与参考波形的相关性
          float sum_sin = 0;
          float sum_cos = 0;
          for (uint16_t i = 0; i < 1000; i++) {
            // 转换为有符号值并计算相关性
            int16_t adc_val = ((adc_value[i] >> 4) - (int16_t)adc_dc_offset);
            // int16_t adc_val = ((adc_value_test[i] >> 4) - 512);
            int16_t sin_val = (sintable[i] - 512);
            int16_t cos_val = (costable[i] - 512);

            sum_sin += adc_val * sin_val;
            sum_cos += adc_val * cos_val;
          }

          float32_t phase_err = 0;
          float32_t delay_phase = 0;
          // 计算相位误差
          arm_atan2_f32(sum_sin, sum_cos, &phase_err);
          // phase_err = atan2(sum_sin, sum_cos);

          // if (phase_err > 0) {
          //   DEC_DAC_TIM(&htim6);
          // } else {
          //   ADD_DAC_TIM(&htim6);
          // }
          phase_err = phase_err - PI / 2;
          if (phase_err < 0) {
            delay_phase = -phase_err;
          } else {
            delay_phase = 2 * PI - phase_err;
          }

          // if (phase_err < 0) {
          //   phase_err += 2 * PI;
          // }

          // uint8_t phase_err_buf[50] = {0};
          // sprintf((char *)phase_err_buf, "phase_err: %d\n",
          //         (uint16_t)(phase_err * 1000));
          // HAL_UART_Transmit(&hlpuart1, phase_err_buf, strlen((char
          // *)phase_err_buf),
          //                   50);

          // 立即应用相位调整
          // SET_DAC_TIM_PHASE(&htim6, TABLE_LENGTH / 10, delay_phase);
          // HAL_Delay(1);
          cal_phase_err_finished = 1;
         }*/
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_value, SAMPLE_TIMES);
    if (fft_finished && comp_triggered) {
      HAL_DAC_Stop_DMA(&hdac4, DAC_CHANNEL_1);
      HAL_DAC_Stop_DMA(&hdac4, DAC_CHANNEL_2);
      HAL_DAC_Start_DMA(&hdac4, DAC_CHANNEL_1, (uint32_t *)sintable_dac1,
                        TABLE_LENGTH, DAC_ALIGN_12B_R);
      HAL_DAC_Start_DMA(&hdac4, DAC_CHANNEL_2, (uint32_t *)sintable_dac2,
                        TABLE_LENGTH, DAC_ALIGN_12B_R);

      for (int16_t i = 0; i < 10; i++) {
        start_adc = 0;
        while (!start_adc)
          ; // 等待 dac dma 完成一次传输到达零点
      }
      comp_triggered = 0;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  // dds_free(stm32_dds);
  free(sintable_dac1);
  free(sintable_dac2);
  free(costable);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
//   if (hcomp == &hcomp1) {
//     if (fft_finished) {
//       HAL_DAC_Stop_DMA(&hdac4, DAC_CHANNEL_1);
//       HAL_DAC_Stop_DMA(&hdac4, DAC_CHANNEL_2);
//       HAL_DAC_Start_DMA(&hdac4, DAC_CHANNEL_1, (uint32_t *)sintable,
//                         TABLE_LENGTH, DAC_ALIGN_12B_R);
//       HAL_DAC_Start_DMA(&hdac4, DAC_CHANNEL_2, (uint32_t *)sintable,
//                         TABLE_LENGTH, DAC_ALIGN_12B_R);
//       // HAL_GPIO_TogglePin(COMP_OUT_TEST_GPIO_Port, COMP_OUT_TEST_Pin);
//     }
//   }
// }
//

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
  if (hcomp == &hcomp1) {
    if (fft_finished && !comp_triggered) {
      comp_triggered = 1;
    }
  }
}

// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
//   if (htim == &htim4) {
//     __HAL_TIM_SetCounter(&htim4, 0); // 10kHz信号每来一个上升沿重新对齐
//   }
// }

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc1) {
    adc_conv_finished = 1;
  }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
  if (hdac == &hdac4) {
    start_adc = 1;
    if (cal_phase_err_finished) {
      cal_phase_err_finished = 0;
    }
  }
}

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//   if (GPIO_Pin == GPIO_PIN_3) {
//     if (fft_finished) {
//       // HAL_DAC_Stop_DMA(&hdac4, DAC_CHANNEL_1);
//       // HAL_DAC_Stop_DMA(&hdac4, DAC_CHANNEL_2);
//       HAL_DAC_Start_DMA(&hdac4, DAC_CHANNEL_1, (uint32_t *)sintable,
//                         TABLE_LENGTH - 2, DAC_ALIGN_12B_R);
//       HAL_DAC_Start_DMA(&hdac4, DAC_CHANNEL_2, (uint32_t *)sintable,
//                         TABLE_LENGTH - 2, DAC_ALIGN_12B_R);
//       HAL_GPIO_TogglePin(COMP_OUT_TEST_GPIO_Port, COMP_OUT_TEST_Pin);
//     }
//   }
// }

// void SET_ADC_TIM_FREQ(TIM_HandleTypeDef *htim, uint16_t wavelen,
//                       uint32_t freq) {
//   uint16_t psc = __HAL_TIM_GET_CLOCKDIVISION(htim);
//   uint16_t arr = 160000000 / (psc + 1) / freq / wavelen - 1;
//   __HAL_TIM_SET_AUTORELOAD(htim, arr);
// }

void SET_DAC_TIM_FREQ(TIM_HandleTypeDef *htim, uint16_t wavelen, uint32_t freq,
                      uint16_t timclk_Mhz) {
  uint32_t arr = timclk_Mhz * 1000000 / (wavelen * freq) - 1;
  __HAL_TIM_SET_AUTORELOAD(htim, arr);
}

void SET_DAC_TIM_PHASE(TIM_HandleTypeDef *htim, uint16_t wavelen,
                       float32_t phase) {
  uint16_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1;
  uint16_t phase_cnt = wavelen * phase * period / (2 * PI);
  uint32_t cnt = __HAL_TIM_GET_COUNTER(htim) + 65535 - phase_cnt;
  // HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, phase_cnt / 10);
  __HAL_TIM_SetCounter(htim, cnt);
  // __HAL_TIM_SetCounter(htim, __HAL_TIM_GetCounter(htim) + 65535 -
  //                                16000 * phase / 2 / PI);
}

uint16_t *generate_sine_wave_table(uint32_t table_size, uint8_t periods,
                                   uint16_t amplitude) {
  // 分配波表内存
  uint16_t *wave_table = (uint16_t *)malloc(table_size * sizeof(uint16_t));
  if (wave_table == NULL) {
    return NULL; // 内存分配失败
  }

  // 计算角频率
  float omega = 2.0f * M_PI * periods / table_size;

  // 填充波表
  for (uint32_t i = 0; i < table_size; i++) {
    float value = amplitude * (sin(omega * i) + 1.0f) / 2.0f;
    // 将值映射到 [100, amplitude]
    wave_table[i] = (uint16_t)value;
  }

  return wave_table;
}

uint16_t *generate_cosine_wave_table(uint32_t table_size, uint8_t periods,
                                     uint16_t amplitude) {
  // 分配波表内存
  uint16_t *wave_table = (uint16_t *)malloc(table_size * sizeof(uint16_t));
  if (wave_table == NULL) {
    return NULL; // 内存分配失败
  }

  // 计算角频率
  float omega = 2.0f * M_PI * periods / table_size;

  // 填充波表
  for (uint32_t i = 0; i < table_size; i++) {
    float value = amplitude * (cos(omega * i) + 1.0f) / 2.0f;
    // 将值映射到 [100, amplitude]
    wave_table[i] = (uint16_t)value;
  }

  return wave_table;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state
   */
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

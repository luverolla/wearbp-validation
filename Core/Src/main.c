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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dsp/filtering_functions.h"
#include "dsp/basic_math_functions.h"

#include "filter_coefs.h"
#include "model_coefs.h"
#include "signal_utils.h"
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
mch_state state = MCH_STATE_WAIT_NWIN;

uint32_t uart_rxbuf[CFG_CHUNKLEN];
uint8_t uart_txbuf[4]; // window number and predicted value

float sigbuf[CFG_SIGLEN];
float signorm[CFG_SIGLEN];

uint32_t chunk_count = 0;
uint32_t curr_nwin = 0;
float curr_pred = 0;

arm_fir_instance_f32 fir;
float fir_state[CFG_SIGLEN+FILT_FIR_NTAPS-1];

sig_fiducials fid;
float features[66];
float tmp[1000];
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  arm_fir_init_f32(&fir, FILT_FIR_NTAPS, FILT_FIR_COEFS, fir_state, CFG_SIGLEN);
  HAL_UART_Receive_DMA(&huart2, (uint8_t*)uart_rxbuf, 1*sizeof(uint32_t));
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {

		if (state == MCH_STATE_WAIT_NWIN) {
			curr_nwin += 1;
			uart_txbuf[0] = 0x06;
			HAL_UART_Transmit_DMA(&huart2, uart_txbuf, 4);
			state = MCH_STATE_WAIT_DEMOG;
			// 5 demographic features
			HAL_UART_Receive_DMA(&huart2, (uint8_t*)features, 5*sizeof(float));
		}

		else if (state == MCH_STATE_WAIT_DEMOG) {
			state = MCH_STATE_WAIT_DATA;
			HAL_UART_Receive_DMA(&huart2, (uint8_t*)uart_rxbuf, CFG_CHUNKLEN*sizeof(uint32_t));

		}

		else if(state == MCH_STATE_WAIT_DATA) {
			if (chunk_count >= CFG_WINSIZE - 1) {
				state = MCH_STATE_VALIDAT;

				arm_fir_f32(&fir, sigbuf, sigbuf, CFG_SIGLEN);
				sig_norm(sigbuf, CFG_SIGLEN, signorm, SIG_NORM_RANGE);
				sig_get_fiducials(signorm, CFG_SIGLEN, &fid);

				curr_pred = 0;
				if (fid.n_speaks >= 2 && fid.n_valleys >= 2 && fid.n_dpeaks >= 1) {
					// start filling features from position 5 (after demographic features)
					sig_featex(sigbuf, CFG_SIGLEN, fid, tmp, features+5);

					arm_sub_f32(features, SVM_DBP_SMIN, features, 66);
					for (size_t i = 0; i < 66; i++) {
						features[i] /= SVM_DBP_SDIF[i];
					}

					arm_dot_prod_f32(features, SVM_DBP_BETA, 66, &curr_pred);
					curr_pred += SVM_DBP_BIAS;
				}

				memcpy(uart_txbuf, &curr_pred, 4);
				HAL_UART_Transmit_DMA(&huart2, uart_txbuf, 4);
				chunk_count = 0;
				state = MCH_STATE_WAIT_NWIN;
				HAL_UART_Receive_DMA(&huart2, (uint8_t*)uart_rxbuf, 1*sizeof(uint32_t));
			}
			else {
				float* bufcell = sigbuf+chunk_count*CFG_CHUNKLEN;
				memcpy(bufcell, uart_rxbuf, CFG_CHUNKLEN*sizeof(uint32_t));
				chunk_count += 1;
				HAL_UART_Receive_DMA(&huart2, (uint8_t*)uart_rxbuf, CFG_CHUNKLEN*sizeof(float));
			}
		}
	}
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

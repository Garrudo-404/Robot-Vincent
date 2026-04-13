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
#include "cmsis_os.h"

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
/* Definitions for task_Comm */
//Escucha UART (nueva coordenada) y controla el gripper
osThreadId_t task_CommHandle;
const osThreadAttr_t task_Comm_attributes = {
  .name = "task_Comm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_PID */
//PID de servos realimentados, gestiona compare register para ir modificando el PWM de los servos realimentados
osThreadId_t task_PIDHandle;
const osThreadAttr_t task_PID_attributes = {
  .name = "task_PID",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for task_IK */
//Se encarga del cálculo de la cinemática inversa: Recibe coordenadas, reenvía el ángulo a task_PID y actualiza el PWM de los servos de posición
osThreadId_t task_IKHandle;
const osThreadAttr_t task_IK_attributes = {
  .name = "task_IK",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for task_Planner */
//Descompone trayectoria en puntos y se lo pasa a task_IK. Además se implementará una máquina de estados (espera, coger_color, pintando)
osThreadId_t task_PlannerHandle;
const osThreadAttr_t task_Planner_attributes = {
  .name = "task_Planner",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Queue_commands */
//Se encarga de pasar los comandos de task_Comm a task_Planer
osMessageQueueId_t Queue_commandsHandle;
const osMessageQueueAttr_t Queue_commands_attributes = {
  .name = "Queue_commands"
};
/* Definitions for Queue_points */
//Se encarga de pasar la trayectoria descompuesta en puntos de task_Planner a task_IK
osMessageQueueId_t Queue_pointsHandle;
const osMessageQueueAttr_t Queue_points_attributes = {
  .name = "Queue_points"
};
/* Definitions for Queue_angles */
//Se encarga de pasar los ángulos que deberían tener los servos realimentados de task_IK a task_PID
osMessageQueueId_t Queue_anglesHandle;
const osMessageQueueAttr_t Queue_angles_attributes = {
  .name = "Queue_angles"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void StartDefaultTask(void *argument);
void Start_task_PID(void *argument);
void Start_task_IK(void *argument);
void Start_task_Planner(void *argument);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue_commands */
  Queue_commandsHandle = osMessageQueueNew (5, sizeof(Command_t), &Queue_commands_attributes);

  /* creation of Queue_points */
  Queue_pointsHandle = osMessageQueueNew (10, sizeof(Point3D_t), &Queue_points_attributes);

  /* creation of Queue_angles */
  Queue_anglesHandle = osMessageQueueNew (2, sizeof(Angles_t), &Queue_angles_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task_Comm */
  task_CommHandle = osThreadNew(StartDefaultTask, NULL, &task_Comm_attributes);

  /* creation of task_PID */
  task_PIDHandle = osThreadNew(Start_task_PID, NULL, &task_PID_attributes);

  /* creation of task_IK */
  task_IKHandle = osThreadNew(Start_task_IK, NULL, &task_IK_attributes);

  /* creation of task_Planner */
  task_PlannerHandle = osThreadNew(Start_task_Planner, NULL, &task_Planner_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.PLL.PLLN = 50;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_task_PID */
/**
* @brief Function implementing the task_PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_task_PID */
/* Esta función se encarga de recibir los ángulos (tipo definido Angle_t) que deberían tener los servomotores de realimentación e implementa un
   PID para ajustar la salida PWM. Para ello se hará uso de una lectura ADC (potenciómetro del servomotor), que servirá como referencia
   para caracterizar la realimentación */
void Start_task_PID(void *argument)
{
  /* USER CODE BEGIN Start_task_PID */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_task_PID */
}

/* USER CODE BEGIN Header_Start_task_IK */
/**
* @brief Function implementing the task_IK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_task_IK */
/* Esta función recibe las coordenadas (tipo Point3D_t) y se encarga de calcular la cinemática inversa del robot (los ángulos requeridos por los actuadores)
 * También actualiza el PWM de los servomotores de posición, y envía el ángulo (tipo Angles_t) a task_PID*/
void Start_task_IK(void *argument)
{
  /* USER CODE BEGIN Start_task_IK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_task_IK */
}

/* USER CODE BEGIN Header_Start_task_Planner */
/**
* @brief Function implementing the task_Planner thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_task_Planner */
/* Recibe los comandos (tipo Command_t) y se encarga de descomponer la trayectoria en puntos que task_IK pueda calcular (tipo Point3D_t)
 * Además tiene una máquina de estados (espera, coger_color y pintando) .*/
void Start_task_Planner(void *argument)
{
  /* USER CODE BEGIN Start_task_Planner */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_task_Planner */
}

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

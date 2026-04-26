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
#include <math.h> // Para relaciones trigonométricas
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Definiciones de la geometría del brazo [mm]
#define L1 40.0f  // Altura desde la base hasta la articulación del hombro
#define L2 184.56f  // Longitud del eslabón del hombro
#define L3 114.5f  // Longitud del eslabón del codo
#define L4 110.0f   // Longitud desde la muñeca hasta la punta del rotulador
#define L4ini 110.07f // Longitud cuando no tiene el rotulador, desde la muñeca hasta la punta del gripper
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Definitions for task_Comm */
osThreadId_t task_CommHandle;
const osThreadAttr_t task_Comm_attributes = {
  .name = "task_Comm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_PID */
osThreadId_t task_PIDHandle;
const osThreadAttr_t task_PID_attributes = {
  .name = "task_PID",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for task_IK */
osThreadId_t task_IKHandle;
const osThreadAttr_t task_IK_attributes = {
  .name = "task_IK",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for task_Planner */
osThreadId_t task_PlannerHandle;
const osThreadAttr_t task_Planner_attributes = {
  .name = "task_Planner",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Queue_commands */
osMessageQueueId_t Queue_commandsHandle;
const osMessageQueueAttr_t Queue_commands_attributes = {
  .name = "Queue_commands"
};
/* Definitions for Queue_points */
osMessageQueueId_t Queue_pointsHandle;
const osMessageQueueAttr_t Queue_points_attributes = {
  .name = "Queue_points"
};
/* Definitions for Queue_angles */
osMessageQueueId_t Queue_anglesHandle;
const osMessageQueueAttr_t Queue_angles_attributes = {
  .name = "Queue_angles"
};
/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void Start_task_Comm(void *argument);
void Start_task_PID(void *argument);
void Start_task_IK(void *argument);
void Start_task_Planner(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t adc_val_codo = 0;
static int calculo_angulo_base = 0;
static int calculo_angulo_hombro = 0;
static int calculo_angulo_codo = 0;
static int calculo_angulo_muñeca = 0;

static int seguimiento_rutina = 0;
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
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
  task_CommHandle = osThreadNew(Start_task_Comm, NULL, &task_Comm_attributes);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 799;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOT_C_IN2_GPIO_Port, MOT_C_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOT_C_IN1_Pin|MOT_H_IN2_Pin|MOT_H_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MOT_C_IN2_Pin */
  GPIO_InitStruct.Pin = MOT_C_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOT_C_IN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOT_C_IN1_Pin MOT_H_IN2_Pin MOT_H_IN1_Pin */
  GPIO_InitStruct.Pin = MOT_C_IN1_Pin|MOT_H_IN2_Pin|MOT_H_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_task_Comm */
/**
  * @brief  Function implementing the task_Comm thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_task_Comm */
void Start_task_Comm(void *argument)
{
  /* USER CODE BEGIN 5 */
  Command_t test_command;

  // Pequeña pausa inicial para que todo el sistema arranque
  osDelay(2000);

  for(;;)
  {

	  //1. PUNTO INICIAL (En el aire, lado IZQUIERDO)
	      // Posicionamos el brazo antes de tocar el lienzo
	      test_command.type = CMD_MOVE_LINEAR;
	      test_command.x = 300.0f;
	      test_command.y = -100.0f; // Izquierda
	      test_command.z = 100.0f;  // Alto (fuera del lienzo)
	      seguimiento_rutina = 1;
	      osMessageQueuePut(Queue_commandsHandle, &test_command, 0, osWaitForever);

	      // Esperamos a que llegue. (Si el trayecto es largo, damos tiempo de sobra)
	      osDelay(8000);

	      //2. ACERCAMIENTO AL LIENZO (Bajar en Z)
	      // Bajamos el rotulador suavemente hasta la superficie de dibujo
	      test_command.type = CMD_MOVE_LINEAR;
	      test_command.x = 330.0f;
	      test_command.y = -100.0f;
	      test_command.z = 50.0f;   // Altura de contacto con el lienzo
	      seguimiento_rutina = 2;
	      osMessageQueuePut(Queue_commandsHandle, &test_command, 0, osWaitForever);

	      osDelay(5000); // Tiempo para bajar con calma

	      // 3. LÍNEA DE IZQUIERDA A DERECHA
	      // Desplazamiento horizontal manteniendo la punta apoyada
	      test_command.type = CMD_MOVE_LINEAR;
	      test_command.x = 330.0f;
	      test_command.y = 100.0f;  // Derecha
	      test_command.z = 50.0f;   // Mantiene la altura de contacto
	      seguimiento_rutina = 3;
	      osMessageQueuePut(Queue_commandsHandle, &test_command, 0, osWaitForever);

	      // Esta es la línea principal. Como son 200 pasos * 30ms = 6 segundos de movimiento.
	      osDelay(8000);

	      // 4. RETIRARSE (Subir en Z) ---
	      // Para no emborronar al terminar, subimos el brazo
	      test_command.type = CMD_MOVE_LINEAR;
	      test_command.x = 300.0f;
	      test_command.y = 100.0f;
	      test_command.z = 60.0f;
	      seguimiento_rutina = 4;
	      osMessageQueuePut(Queue_commandsHandle, &test_command, 0, osWaitForever);

	      osDelay(8000); // Pausa antes de repetir el ciclo completo
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_task_PID */
/**
Esta función se encarga de recibir los ángulos (tipo definido Angle_t) que deberían tener los servomotores de realimentación e implementa un
PID para ajustar la salida PWM. Para ello se hará uso de una lectura ADC (potenciómetro del servomotor), que servirá como referencia
para caracterizar la realimentación @brief Function implementing the task_PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_task_PID */
void Start_task_PID(void *argument)
{ /* USER CODE BEGIN Start_task_PID */
	  // ARRANCAR SEÑALES PWM
	    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Hombro ENA
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Codo ENB
	    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Base Servo
	    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Muñeca Servo


  Angles_t angulos_recibidos;
  static uint32_t last_pulse_q4 = 0; // Para evitar temblores guardamos el estado anterior

  // --- Variables para Motores con Realimentación (q2, q3) ---
  int posicionDeseada[2] = {1160, 2048};
  float errorAcumulado[2] = {0, 0};
  uint32_t tiempoAnterior = HAL_GetTick();
  float Kp = 2.2f; // Ajustado para mayor respuesta de corrección
  float Ki = 0.8f; // Ajustado para vencer la gravedad en el hombro

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  posicionDeseada[0] = HAL_ADC_GetValue(&hadc1); // El hombro se queda donde está
  HAL_ADC_PollForConversion(&hadc1, 10);
  posicionDeseada[1] = HAL_ADC_GetValue(&hadc1); // El codo se queda donde está
  HAL_ADC_Stop(&hadc1);

  for(;;)
  {
    // RECIBIR ÓRDENES Y ACTUALIZAR SETPOINTS
    if (osMessageQueueGet(Queue_anglesHandle, &angulos_recibidos, NULL, 0) == osOK)
    {
    	// --- SERVOS LAZO ABIERTO (q1 y q4) ---
    	// 2000 us/180º = 11.11 de pendiente
    	uint32_t pulse_q1 = (uint32_t)(angulos_recibidos.angles[0] * 11.11f + 500);

    	// --- NUEVA CALIBRACIÓN MUÑECA (q4) ---
    	// Referencia: 0º Reales = 25º PWM.
    	// Comportamiento: Al subir ángulos en código, baja físicamente (Inversión).
    	// Fórmula: (Referencia - Ángulo_Deseado)
    	float angulo_calibrado_q4 = 6.0f - angulos_recibidos.angles[3];

    	// Convertimos a microsegundos (PWM)
    	float calc_q4 = (angulo_calibrado_q4 * 11.11f) + 500.0f;

    	// --- PROTECCIÓN (SATURACIÓN) ---
    	// Ajustamos límites para permitir el rango de -130º si es necesario
    	if (calc_q4 < 400.0f)  calc_q4 = 400.0f;  // Límite mínimo de seguridad
    	if (calc_q4 > 2600.0f) calc_q4 = 2600.0f; // Límite máximo de seguridad

    	uint32_t pulse_q4 = (uint32_t)calc_q4;

    	// --- ENVÍO A TIMERS CON FILTRO ANTI-TEMBLOR ---
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_q1); // Base

    	if(abs((int)pulse_q4 - (int)last_pulse_q4) > 2) {
    	    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse_q4); // Muñeca
    	    last_pulse_q4 = pulse_q4;
    	}

      // --- B. SETPOINTS PARA PI (q2 y q3) ---
      // Nueva fórmula calibrada: Base 2080 + (ángulo * sensibilidad)
      //0º se corresponde con 2080, y 90º con 3700 de la lectura realizada con el ADC
      //Rango: 3700 - 2080 = 1620 puntos de ADC por cada 90 grados.

      posicionDeseada[0] = (int)(2080.0f + (angulos_recibidos.angles[1] * (1620.0f / 90.0f)));
      // Limitar el objetivo entre el tope físico inferior y el superior detectado
      if (posicionDeseada[0] < 1810) posicionDeseada[0] = 1810;
      if (posicionDeseada[0] > 3750) posicionDeseada[0] = 3750;

      // Nueva fórmula calibrada para el codo (Nuevo Servo)
      posicionDeseada[1] = (int)(1830.0f + (angulos_recibidos.angles[2] * (1740.0f / 90.0f)));

      // --- SEGURIDAD: LÍMITES MECÁNICOS ---
      if (posicionDeseada[1] < 1800) posicionDeseada[1] = 1800;
      if (posicionDeseada[1] > 3600) posicionDeseada[1] = 3600;
    }

    // 2. LECTURA ADC
    int valorPotenciometro[2];
    HAL_ADC_Start(&hadc1);

    // Esperar y leer la primera conversión (Hombro - Rank 1)
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            valorPotenciometro[0] = HAL_ADC_GetValue(&hadc1);
        }

        // Esperar y leer la segunda conversión (Codo - Rank 2)
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            valorPotenciometro[1] = HAL_ADC_GetValue(&hadc1);
            adc_val_codo = valorPotenciometro[1];
        }
    HAL_ADC_Stop(&hadc1);

    // 3. CÁLCULO DEL TIEMPO (dt)
    uint32_t tiempoActual = HAL_GetTick();
    float dt = (float)(tiempoActual - tiempoAnterior) / 1000.0f;
    if (dt <= 0) dt = 0.01f;
    tiempoAnterior = tiempoActual;

    // 4. BUCLE DE CONTROL PI
    for (int i = 0; i < 2; i++)
    {
      float error = (float)posicionDeseada[i] - (float)valorPotenciometro[i];

      // Zona muerta
      if (error > -10 && error < 10) {
          error = 0;
      }

      // Anti-windup (Corregido para permitir corrección en ambos sentidos)
      if (fabsf(error) < 400.0f) {
          errorAcumulado[i] += (error * dt);
      } else {
          errorAcumulado[i] = 0;
      }

      // Limitación de la integral para evitar saturación excesiva
      if (errorAcumulado[i] > 500.0f) errorAcumulado[i] = 500.0f;
      if (errorAcumulado[i] < -500.0f) errorAcumulado[i] = -500.0f;

      float pwm_calc = (Kp * error) + (Ki * errorAcumulado[i]);

      // La velocidad es siempre el valor absoluto del cálculo
      uint32_t velocidad = (uint32_t)fabsf(pwm_calc);
      if (velocidad > 700) velocidad = 700; // Aumentado para vencer carga mecánica

      // 5. SALIDA AL L298N (IN1, IN2 y ENA)
      if (i == 0) // MOTOR HOMBRO (Joint 2)
      {
          if (error == 0) {
              HAL_GPIO_WritePin(MOT_H_IN1_GPIO_Port, MOT_H_IN1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(MOT_H_IN2_GPIO_Port, MOT_H_IN2_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
          }
          else if (pwm_calc > 0) { // Sentido A
              HAL_GPIO_WritePin(MOT_H_IN1_GPIO_Port, MOT_H_IN1_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(MOT_H_IN2_GPIO_Port, MOT_H_IN2_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, velocidad);
          }
          else { // Sentido B
              HAL_GPIO_WritePin(MOT_H_IN1_GPIO_Port, MOT_H_IN1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(MOT_H_IN2_GPIO_Port, MOT_H_IN2_Pin, GPIO_PIN_SET);
              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, velocidad);
          }
      }
      else // MOTOR CODO (Joint 3)
      {
          if (error == 0) {
              HAL_GPIO_WritePin(MOT_C_IN1_GPIO_Port, MOT_C_IN1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(MOT_C_IN2_GPIO_Port, MOT_C_IN2_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
          }
          else if (pwm_calc > 0) { // Sentido A
              HAL_GPIO_WritePin(MOT_C_IN1_GPIO_Port, MOT_C_IN1_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(MOT_C_IN2_GPIO_Port, MOT_C_IN2_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, velocidad);
          }
          else { // Sentido B
              HAL_GPIO_WritePin(MOT_C_IN1_GPIO_Port, MOT_C_IN1_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(MOT_C_IN2_GPIO_Port, MOT_C_IN2_Pin, GPIO_PIN_SET);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, velocidad);
          }
      }
    }
    osDelay(10);
  }
  /* USER CODE END Start_task_PID */
}


/* USER CODE BEGIN Header_Start_task_IK */
/**
* Esta función recibe las coordenadas (tipo Point3D_t) y se encarga de calcular la cinemática inversa del robot (los ángulos requeridos por los actuadores)
* También actualiza el PWM de los servomotores de posición, y envía el ángulo (tipo Angles_t) a task_PID
*/
/* USER CODE END Header_Start_task_IK */
void Start_task_IK(void *argument)
{
  /* USER CODE BEGIN Start_task_IK */
  Point3D_t p;
  Angles_t a;

  // Ángulo deseado de ataque del pincel (A 45º para pintar en el plano del lienzo
  float phi_deg = -45.0f;
  float phi = phi_deg * (M_PI / 180.0f);//Pasamos a radianes

  for(;;)
  {
    // 1. Esperamos a recibir un punto desde el Planner
    if (osMessageQueueGet(Queue_pointsHandle, &p, NULL, osWaitForever) == osOK)
    {
        // 2. Base (q1) -> Asignado a angles[0]
        a.angles[0] = atan2f(p.y, p.x) * (180.0f / M_PI);

        // 3. Radio y posición de muñeca
        float r = sqrtf(p.x * p.x + p.y * p.y);
        float rw = r - L4 * cosf(phi);
        float zw = (p.z - L1) - L4 * sinf(phi);

        // 4. Codo (q3) - Ley de Cosenos
        float dist_sq = rw * rw + zw * zw;
        float cos_q3 = (dist_sq - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);

        // Verificar si el punto es físicamente alcanzable
        if (cos_q3 > 1.0f || cos_q3 < -1.0f) {
            // El punto está fuera de alcance.
            // Usamos continue para ignorar este punto y esperar el siguiente,
            // evitando que el robot se rompa o mande ángulos nulos al PID.
            continue;
        }

        float q3_rad = acosf(cos_q3);

        // 5. Hombro (q2)
        float alpha = atan2f(zw, rw);
        float beta = atan2f(L3 * sinf(q3_rad), L2 + L3 * cosf(q3_rad));
        float q2_rad = alpha + beta;

        // 6. Muñeca (q4)
        float q4_rad = phi - q2_rad - q3_rad;

        // 7. Guardar resultados para el PID (Hombro y Codo)
        a.angles[1] = q2_rad * (180.0f / M_PI);
        a.angles[2] = q3_rad * (180.0f / M_PI);
        a.angles[3] = q4_rad * (180.0f / M_PI);

        calculo_angulo_base = (int) a.angles[0];
        calculo_angulo_hombro = (int) a.angles[1];
        calculo_angulo_codo = (int) a.angles[2];
        calculo_angulo_muñeca = (int) a.angles[3];


        // 8. Gripper y estado del pincel
        //a.angles[4] = p.pen_down ? 45.0f : 0.0f; // 45º cerrado, 0º abierto
        //a.pen_state = p.pen_down;

        // 9. Enviar a la cola de la tarea PID
        osMessageQueuePut(Queue_anglesHandle, &a, 0, osWaitForever);
    }
  }
  /* USER CODE END Start_task_IK */
}

/* USER CODE BEGIN Header_Start_task_Planner */
/**
Recibe los comandos (tipo Command_t) y se encarga de descomponer la trayectoria en puntos que task_IK pueda calcular (tipo Point3D_t)
Además tiene una máquina de estados (espera, coger_color y pintando)
*/
/* USER CODE END Header_Start_task_Planner */
void Start_task_Planner(void *argument)
{
  /* USER CODE BEGIN Start_task_Planner */
	/*  .*/
  Command_t cmd;
  Point3D_t point;

  // Posición actual del robot (inicializada en Home al arrancar)
  float current_x = 350.0f, current_y = 0.0f, current_z = 100.0f;
  const int pasos = 200; // Número de segmentos en los que dividimos la línea
  point.pen_down = 1;
  for(;;)
  {
    if (osMessageQueueGet(Queue_commandsHandle, &cmd, NULL, osWaitForever) == osOK)
    {
      if (cmd.type == CMD_MOVE_LINEAR)
      {
        // Calculamos la distancia a recorrer en cada eje
        float diff_x = (cmd.x - current_x) / pasos;
        float diff_y = (cmd.y - current_y) / pasos;
        float diff_z = (cmd.z - current_z) / pasos;

        // Hacemos un bucle simple para descomponer la trayectoria en el número de pasos que hemos definido
        for (int i = 1; i <= pasos; i++)
        {
          point.x = current_x + (diff_x * i);
          point.y = current_y + (diff_y * i);
          point.z = current_z + (diff_z * i);
          //point.speed = 10.0f; // El control de velocidad lo dejaremos para más adelante, pero damos un valor para verificar que se pasa correctamente el dato
          point.pen_down = 1;

          osMessageQueuePut(Queue_pointsHandle, &point, 0, osWaitForever);
          osDelay(20); // Velocidad de generación de puntos
        }

        // Actualizamos la posición actual al finalizar la línea
        current_x = cmd.x;
        current_y = cmd.y;
        current_z = cmd.z;
      }
      else if (cmd.type == CMD_HOME)
      {
        // En Home, mandamos un punto directo (Posibilidad de interpolarlo si vemos que se mueve muy bruscamente desde la posición en la que pinta)
        point.x = 350.0f; point.y = 0.0f; point.z = 100.0f;
        //point.pen_down = 0;
        osMessageQueuePut(Queue_pointsHandle, &point, 0, osWaitForever);

        current_x = 350.0f; current_y = 0.0f; current_z = 100.0f;
      }
      else if (cmd.type == CMD_GRIPPER)
      {
    	  uint8_t anguloTest = 0;
    	  uint32_t pulse_test = (uint32_t)(anguloTest * 11.11f + 500.0f);

    	  // 2. Saturación de seguridad para no forzar el servo
    	  if (pulse_test < 500)  pulse_test = 500;
    	  if (pulse_test > 2500) pulse_test = 2500;

    	  // 3. Enviamos al Timer (HTIM4 es el de tus servos)
    	  // Prueba primero el Canal 1 (Base)
    	   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pulse_test);

    	    HAL_Delay(100); // Pequeña pausa para estabilidad
    	  if (point.pen_down == 1){
    		  uint8_t anguloTest = 50;
    		  uint32_t pulse_test = (uint32_t)(anguloTest * 11.11f + 500.0f);
    		  point.pen_down = 0;
    	  }
    	  else if (point.pen_down == 0){
    		  uint8_t anguloTest = 180;
    		  uint32_t pulse_test = (uint32_t)(anguloTest * 11.11f + 500.0f);
    		  point.pen_down = 1;
    	  }
      }
    }
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

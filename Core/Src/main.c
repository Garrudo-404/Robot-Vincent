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
#define L4 206.0f   // Longitud desde la muñeca hasta la punta del rotulador
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

// Variables del Controlador PI
int posicionDeseada = 2048; // El objetivo en puntos de ADC (0 a 4095)
float Kp = 2.0;             // Fuerza Proporcional: Reacciona al error instantáneo
float Ki = 0.5;             // Fuerza Integral: Vence la fricción acumulando el error
int error = 0;              // Distancia entre donde estoy y donde quiero ir
float errorAcumulado = 0;   // Suma de los errores pasados (memoria)
int salidaPWM = 0;          // La "fuerza" final de -1000 a 1000 que le damos al motor

// Variables para medir el tiempo (necesario para la Integral)
uint32_t tiempoAnterior = 0; // El instante en el que miramos el reloj por última vez
uint32_t tiempoActual = 0;   // La hora actual
float dt = 0;                // Diferencia de tiempo (Delta T) transcurrido

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
  hadc1.Init.NbrOfConversion = 1;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
	  // Test 1: Dibujar una línea recta desde donde esté hasta el destino
	test_command.type = CMD_MOVE_LINEAR;
	test_command.x = 200.0f;    // Destino X (mm)
	test_command.y = 50.0f;     // Destino Y (mm)
	test_command.z = 20.0f;     // Altura de dibujo (mm)
	test_command.param = 0.0f;  // No se usa en líneas

	osMessageQueuePut(Queue_commandsHandle, &test_command, 0, osWaitForever);

	 osDelay(10000); // Espera 10s para ver el movimiento

	 // Test 2: Volver a Home
	 test_command.type = CMD_HOME;
	 osMessageQueuePut(Queue_commandsHandle, &test_command, 0, osWaitForever);

	 //Damos 20 segundos antes de la siguiente instrucción
	 osDelay(20000);
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
{
  /* USER CODE BEGIN Start_task_PID */
  Angles_t angulos_recibidos;

  // ARRANCAR SEÑALES PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Hombro ENA
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Codo ENB
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Base Servo
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Muñeca Servo
  // --- Variables para Motores con Realimentación (q2, q3) ---
  int posicionDeseada[2] = {1160, 2048};
  float errorAcumulado[2] = {0, 0};
  uint32_t tiempoAnterior = HAL_GetTick();
  float Kp = 1.6f, Ki = 0.5f;

  for(;;)
  {
    // 1. RECIBIR ÓRDENES Y ACTUALIZAR SETPOINTS
    if (osMessageQueueGet(Queue_anglesHandle, &angulos_recibidos, NULL, 0) == osOK)
    {
      // --- A. SERVOS LAZO ABIERTO (q1 y q4) ---
      uint32_t pulse_q1 = (uint32_t)(angulos_recibidos.angles[0] * 11.11f + 500);
      uint32_t pulse_q4 = (uint32_t)(angulos_recibidos.angles[3] * 11.11f + 500);

      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_q1); // Base
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, pulse_q4); // Muñeca

      // --- B. SETPOINTS PARA PI (q2 y q3) ---
      posicionDeseada[0] = (int)(2327 - (angulos_recibidos.angles[1] * (2327.0f / 90.0f)));
      posicionDeseada[1] = (int)((angulos_recibidos.angles[2] * 4095.0f) / 180.0f);
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
        }
    HAL_ADC_Stop(&hadc1);

    // 3. CÁLCULO DEL TIEMPO (dt)
    uint32_t tiempoActual = HAL_GetTick();
    float dt = (tiempoActual - tiempoAnterior) / 1000.0f;
    if (dt <= 0) dt = 0.01f;
    tiempoAnterior = tiempoActual;

    // 4. BUCLE DE CONTROL PI
    for (int i = 0; i < 2; i++)
    {
      float error = posicionDeseada[i] - valorPotenciometro[i];

      // Zona muerta
      if (error > -15 && error < 15) {
          error = 0;
          errorAcumulado[i] = 0;
      }

      // Anti-windup
      if (abs((int)error) < 150) errorAcumulado[i] += (error * dt);
      else errorAcumulado[i] = 0;

      float pwm_calc = (Kp * error) + (Ki * errorAcumulado[i]);

      // La velocidad es siempre el valor absoluto del cálculo
      uint32_t velocidad = (uint32_t)fabsf(pwm_calc);
      if (velocidad > 799) velocidad = 799;
      if (salidaPWM < -799) salidaPWM = -799;

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

        // 8. Gripper y estado del pincel
        a.angles[4] = p.pen_down ? 45.0f : 0.0f; // 45º cerrado, 0º abierto (ajústalo a tus servos)
        a.pen_state = p.pen_down;

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
  float current_x = 100.0f, current_y = 0.0f, current_z = 100.0f;
  const int pasos = 50; // Número de segmentos en los que dividimos la línea

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
          point.speed = 10.0f; // El control de velocidad lo dejaremos para más adelante, pero damos un valor para verificar que se pasa correctamente el dato
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
        point.x = 100.0f; point.y = 0.0f; point.z = 100.0f;
        point.pen_down = 0;
        osMessageQueuePut(Queue_pointsHandle, &point, 0, osWaitForever);

        current_x = 100.0f; current_y = 0.0f; current_z = 100.0f;
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

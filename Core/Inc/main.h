/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    CMD_MOVE_LINEAR,    // Movimiento en línea recta
    CMD_DRAW_CIRCLE,    // Ejecutar rutina de círculo
    CMD_CHANGE_COLOR,   // Ir a la estación de pintura
    CMD_GRIPPER,        // Abrir/Cerrar manualmente
    CMD_HOME            // Volver a posición de reposo
} CommandType_t;

typedef struct {
    CommandType_t type; // Qué queremos hacer
    float x, y, z;      // Coordenadas destino (si aplica)
    uint8_t color_id;   // ID del bote de pintura (0-5)
    float param;        // Parámetro extra (ej: radio del círculo)
} Command_t;

typedef struct {
    float x;            // Posición X en mm
    float y;            // Posición Y en mm
    float z;            // Posición Z en mm
    float speed;        // Velocidad del trazo para este segmento
    uint8_t pen_down;   // 1: Rotulador tocando el lienzo, 0: Movimiento en aire; esto se implementará posteriormente con un sensor IR
} Point3D_t;

typedef struct {
    float angles[5];    // 0:Base, 1:Hombro, 2:Codo, 3:Muñeca, 4:Gripper; Tener en cuenta estos valores para la elección de los actuadores realimentados
    uint8_t pen_state;  // 1: Pintando, 0: Aire
} Angles_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

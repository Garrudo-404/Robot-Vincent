/* USER CODE BEGIN PV */
#include "math.h"

#define M_PI 3.14159265358979323846f

// --- PARÁMETROS FÍSICOS DEL ROBOT (mm) ---
const float L1 = ;   // Base a Hombro
const float L2 = ;   // Hombro a Codo 
const float L3 = ;  // Codo a Muñeca 
const float L4 = ;    // Muñeca a Punta

// Estructura para manejar los ángulos del robot
typedef struct {
    float q1, q2, q3, q4;  // En grados (0-180)
    uint8_t error;         // 0: OK, 1: Fuera de alcance
} Robot_State_t;

Robot_State_t miRobot;
/* USER CODE END PV */



/* USER CODE BEGIN 4 */
/**
 * @brief Calcula los ángulos necesarios para llegar a (x, y, z)
 * @param phi: Ángulo de aproximación final en grados (ej: -90 es vertical abajo)
 */
Robot_State_t IK_Compute(float x, float y, float z, float phi_deg) {
    Robot_State_t res = {0, 0, 0, 0, 0};
    float phi = phi_deg * (M_PI / 180.0f);

    // 1. Base
    res.q1 = atan2f(y, x);

    // 2. Radio y posición de muñeca
    float r = sqrtf(x*x + y*y);
    float rw = r - L4 * cosf(phi);
    float zw = (z - L1) - L4 * sinf(phi);

    // 3. Codo (q3) - Ley de Cosenos
    float dist_sq = rw*rw + zw*zw;
    float cos_q3 = (dist_sq - L2*L2 - L3*L3) / (2.0f * L2 * L3);

    // Verificar si el punto es alcanzable
    if (cos_q3 > 1.0f || cos_q3 < -1.0f) {
        res.error = 1;
        return res; 
    }

    res.q3 = acosf(cos_q3); 

    // 4. Hombro (q2)
    float alpha = atan2f(zw, rw);
    float beta = atan2f(L3 * sinf(res.q3), L2 + L3 * cosf(res.q3));
    res.q2 = alpha + beta;

    // 5. Muñeca (q4)
    res.q4 = phi - res.q2 - res.q3;

    // Conversión a Grados para Servos
    res.q1 *= (180.0f / M_PI);
    res.q2 *= (180.0f / M_PI);
    res.q3 *= (180.0f / M_PI);
    res.q4 *= (180.0f / M_PI);

    return res;
}

/**
 * @brief Envía los ángulos a los registros del Timer. 
 * Ajusta '500' y '2500' según la calibración de tus servos (ms).
 */
void PWM_Update_Servos(Robot_State_t state) {
    if (state.error) return; // No mover si el cálculo falló

    // Mapeo lineal: 0-180 grados -> 500-2500 pulse (para ARR=20000)
    uint32_t p1 = (uint32_t)(500 + (state.q1 * 2000.0f / 180.0f));
    uint32_t p2 = (uint32_t)(500 + (state.q2 * 2000.0f / 180.0f));
    uint32_t p3 = (uint32_t)(500 + (state.q3 * 2000.0f / 180.0f));
    uint32_t p4 = (uint32_t)(500 + (state.q4 * 2000.0f / 180.0f));

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, p4);
}
/* USER CODE END 4 */


//prueba para ver que funciona
/* USER CODE BEGIN 2 */
// Iniciar canales PWM
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
/* USER CODE END 2 */

/* Infinite loop */
while (1) {
    // 1. Ir a posición de reposo
    miRobot = IK_Compute(150.0f, 0.0f, 100.0f, -90.0f);
    PWM_Update_Servos(miRobot);
    HAL_Delay(2000);

    // 2. Extender brazo al frente
    miRobot = IK_Compute(280.0f, 0.0f, 180.0f, 0.0f);
    PWM_Update_Servos(miRobot);
    HAL_Delay(2000);
    
    // 3. Mover a un lateral (Base gira 45 grados)
    miRobot = IK_Compute(180.0f, 180.0f, 120.0f, -45.0f);
    PWM_Update_Servos(miRobot);
    HAL_Delay(2000);
}

# Robot Vincent
Este código es la implementación de un robot pintor, llamado Vincent en honor al pintor Vincent van Gogh. El robot está compuesto por 5 actuadores, 4 de los cuales están situados en las articulaciones para proporcionar un movimiento de rotación, lo que le confiere 4 grados de libertad. Esto nos dará la flexibilidad necesaria como para poder pintar en todo nuestro espacio de trabajo.
Se ha optado por utilizar la placa STM32F407 debido a su alta potencia computacional en comparación con las placas de Arduino, además de una mejor visibilidad de las variables en memoria y el uso a bajo nivel que proporciona de hardware. Además para poder hacer un robot con movimiento fluido y con el objetivo de descomponer el código de forma que sea escalable, se implementa mediante el uso de RTOS, lo que nos permite un control completo en la ejecución de tareas y con tiempos estrictos.

Estas tareas se ha optado por descomponerlas en las siguientes:
- task_PID: Esta función se encarga de recibir los ángulos (tipo definido Angle_t) que deberían tener los servomotores de realimentación e implementa un
   PID para ajustar la salida PWM. Para ello se hará uso de una lectura ADC (potenciómetro del servomotor), que servirá como referencia
   para caracterizar la realimentación.
- task_IK: Esta función recibe las coordenadas (tipo Point3D_t) y se encarga de calcular la cinemática inversa del robot (los ángulos requeridos por los actuadores). También actualiza el PWM de los servomotores de posición, y envía el ángulo (tipo Angles_t) a task_PID.
- task_Planner: Recibe los comandos (tipo Command_t) y se encarga de descomponer la trayectoria en puntos que task_IK pueda calcular (tipo Point3D_t). Además tiene una máquina de estados (espera, coger_color y pintando)
- task_Comm: Escucha UART (nueva coordenada) y controla el gripper.

Además se hace uso de colas (queues) que permiten a estas tareas comunicarse entre sí. Son las siguientes:
- Queue_commands: comunicación de task_IK a task_PID
- Queue_points: comunicación de task_Planner a task_IK
- Queue_anles: comunicación de task_IK a task_PID

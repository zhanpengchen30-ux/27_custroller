/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_arm_control.h"
#include "task_telemetry.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ControlTaskHandle;
osThreadId CommTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControlTask(void const * argument);
void StartCommTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityHigh, 0, 512);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of CommTask */
  /* 512 words = 2KB：VOFA_SendData 里有 snprintf(%.3f x12) + 39 个实参，
     原来的 256 words (1KB) 会在浮点格式化时踩爆栈，进而破坏相邻的
     ControlTask，表现就是电机突然全部失能（红灯）。
     ⚠ CubeMX 重新生成会改回 256，每次生成后都要检查。 */
  osThreadDef(CommTask, StartCommTask, osPriorityNormal, 0, 512);
  CommTaskHandle = osThreadCreate(osThread(CommTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* 挂载并进入机械臂核心控制任务（高优先级，~300Hz） */
  Task_ArmControl(argument);
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */
  /* 挂载并进入遥测任务（20Hz） */
  Task_Telemetry(argument);
  /* USER CODE END StartCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  栈溢出钩子（configCHECK_FOR_STACK_OVERFLOW == 2）。
  *         走到这里说明某个任务的栈已经踩穿了。这里直接停住等调试器接管，
  *         比让溢出静默改坏 heap_4 里相邻任务的栈 / TCB 安全得多。
  *         调试时在 Watch 里看 prvOverflowTaskName 就知道是哪个任务。
  */
volatile const char *prvOverflowTaskName = NULL;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  prvOverflowTaskName = pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
/* USER CODE END Application */

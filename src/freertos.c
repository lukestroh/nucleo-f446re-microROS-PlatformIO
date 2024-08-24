/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

// microROS includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/int32.h>
#include <branch_detection_interfaces/msg/vl53l8cx8x8.h>

// Custom includes
#include "freertos.h"
#include "microros_extras.h"
#include "system_state.h"
#include "rcl_utils.h"



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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE BEGIN Header_StartBlinkLEDTask */
/**
  * @brief  Function implementing the blink_LED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlinkLEDTask */
void StartBlinkLEDTask(void *argument) {
  /* USER CODE BEGIN 5 */
  uint32_t last_time = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    if (xTaskGetTickCount() - last_time > 1000) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      last_time = xTaskGetTickCount();
    }
  }
  /* USER CODE END 5 */
}


/* USER CODE BEGIN Header_StartmicroROSTask */
/**
* @brief Function implementing the microROS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartmicroROSTask */
void StartmicroROSTask(void *argument) {
  /* USER CODE BEGIN StartmicroROSTask */
  rmw_uros_set_custom_transport(
    true,
    (void *) argument, // Passing the huart2 pointer here
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read
  );

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // system state
  SystemState system_state = AGENT_WAIT;

  // micro-ROS app
  rcl_publisher_t publisher;
  rclc_support_t support;
  rcl_node_t node;
  
  // Message setup
  branch_detection_interfaces__msg__Vl53l8cx8x8 msg;
  if (!branch_detection_interfaces__msg__Vl53l8cx8x8__init(&msg)) {
    Error_Handler();
  };
  msg.data.capacity = 64;
  msg.data.size = 64;
  if (!rosidl_runtime_c__int32__Sequence__init(&msg.data, msg.data.size)) {
    Error_Handler();
  }

  

  // executor
  // rclc_executor_init(&executor, &support.context, 1, &freeRTOS_allocator);

  rcl_ret_t status = RCL_RET_OK;

  // loop params
  uint32_t last_time = xTaskGetTickCount();

  for(;;) {
    // TODO: Add VL53l8cx read sensor here

    // Check system_state for connection status to micro_ros_agent
    // // TODO: check if this can be done with the interrupts...
    //
    switch(system_state) {
      case AGENT_WAIT:
        execute_every_n_ms(500, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : AGENT_WAIT);
        break;
      case AGENT_AVAILABLE:
        system_state = (true == create_rcl_entities(&freeRTOS_allocator, &support, &node, &publisher)) ? AGENT_CONNECTED : AGENT_WAIT;
        if (system_state == AGENT_WAIT) {
          status = destroy_rcl_entities(&support, &node, &publisher);
        }
        break;
      case AGENT_CONNECTED:
        execute_every_n_ms(10, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
        if (system_state == AGENT_CONNECTED) {
          for (uint8_t i=0; i<msg.data.size; ++i){
            msg.data.data[i]++;
          }
          status = rcl_publish(&publisher, &msg, NULL);
        }
        break;
      case AGENT_DISCONNECTED:
        status = destroy_rcl_entities(&support, &node, &publisher);
        system_state = AGENT_WAIT;
        break;
      default:
        break;
    }
  }
  /* USER CODE END StartmicroROSTask */
}



/* USER CODE END Application */

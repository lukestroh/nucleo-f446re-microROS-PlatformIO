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
#include "freertos.h"

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
  // rmw_uros_set_custom_transport(
  //   true,
  //   (void *) &huart2,
  //   cubemx_transport_open,
  //   cubemx_transport_close,
  //   cubemx_transport_write,
  //   cubemx_transport_read);

  // rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  // freeRTOS_allocator.allocate = microros_allocate;
  // freeRTOS_allocator.deallocate = microros_deallocate;
  // freeRTOS_allocator.reallocate = microros_reallocate;
  // freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  // if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
  //     printf("Error on default allocators (line %d)\n", __LINE__); 
  // }

  // // system state
  // SystemState system_state = AGENT_WAIT;

  // // micro-ROS app
  // rcl_publisher_t publisher;
  // // branch_detection_interfaces__msg__Vl53l8cx8x8 msg;
  // std_msgs__msg__Int32 msg;
  // rclc_support_t support;
  // // rcl_allocator_t allocator;
  // // rclc_executor_t executor;
  // rcl_node_t node;
  // // rcl_timer_t timer;


  /* Infinite loop */
  
  // // create timer // TODO: Probably don't use this, just publish a message when it's necessary
  // const unsigned uint32 timer_period {10};
  // rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_period),
  //   timer_callback
  // );
  
  // zero-initializae the message array
  // rosidl_runtime_c__int32__Sequence__init(&msg.data, 64);
  // // branch_detection_interfaces__msg__Vl53l8cx8x8__Sequence__init(&msg.data.data, 64);
  // memset(msg.data.data, 0, sizeof(uint32_t));

  // executor
  // rclc_executor_init(&executor, &support.context, 1, &freeRTOS_allocator);

  // rcl_ret_t status;

   // create init_options
  // status = rclc_support_init(&support, 0, NULL, &freeRTOS_allocator);

  // // create node
  // const char* _namespace = "microROS";
  // const char* _node_name = "nucleof446re";
  // status = rclc_node_init_default(&node, _node_name, _namespace, &support);

  // // create publisher
  // status = rclc_publisher_init_default(
  //   &publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(branch_detection_interfaces, msg, Vl53l8cx8x8),
  //   "vl53l8cx/data" // topic
  // );

  // for(;;) {
  //   // read from sensors
  //   // TODO


  //   // Check system_state for connection status to micro_ros_agent
  //   // // TODO: check if this can be done with the interrupts...
  //   switch(system_state) {
  //     case AGENT_WAIT:
  //       // execute_every_n_ms(500, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : AGENT_WAIT);
  //       break;
  //     case AGENT_AVAILABLE:
  //       // system_state = (true == create_rcl_entities(&freeRTOS_allocator, &support, &node, &publisher)) ? AGENT_CONNECTED : AGENT_WAIT;
  //   //     if (system_state == AGENT_WAIT) {
  //   //       status = destroy_rcl_entities(&support, &node, &publisher);
  //   //     }
  //       break;
  //     case AGENT_CONNECTED:
  //   //     execute_every_n_ms(10, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED);
  //       // if (system_state == AGENT_CONNECTED) {
  //         // status = rcl_publish(&publisher, &msg, NULL);
  //       // }
  //       break;
  //     case AGENT_DISCONNECTED: // something about this guy
  //       // status = destroy_rcl_entities(&support, &node, &publisher); // problem line
  //   //     system_state = AGENT_WAIT;
  //       break;
  //     default:
  //       break;
  //   }
  // }
  /* USER CODE END StartmicroROSTask */
}



/* USER CODE END Application */


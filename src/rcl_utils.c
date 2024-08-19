#include "main.h"
#include "rcl_utils.h"
#include <branch_detection_interfaces/msg/vl53l8cx8x8.h>

bool create_rcl_entities(
    rcl_allocator_t* allocator,
    rclc_support_t* support,
    rcl_node_t* node,
    rcl_publisher_t* publisher
) {
//     // Init allocator
//   *allocator = rcl_get_default_allocator();

  rcl_ret_t status;

//   // create init_options
//   status = rclc_support_init(support, 0, NULL, allocator);

//   // create node
//   const char* _namespace = "microROS";
//   const char* _node_name = "nucleof446re";
//   status = rclc_node_init_default(node, _node_name, _namespace, support);

//   // create publisher
//   status = rclc_publisher_init_default(
//     publisher,
//     node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(branch_detection_interfaces, msg, Vl53l8cx8x8),
//     "vl53l8cx/data" // topic
//   );s
  
  return true;
}

rcl_ret_t destroy_rcl_entities(
    rclc_support_t* p_support,
    rcl_node_t* p_node,
    rcl_publisher_t* p_publisher
    // rclc_executor_t* p_executor
) {

    // rmw_context_t* rmw_context = rcl_context_get_rmw_context(&(p_support->context));
    // (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // rcl_ret_t status = rcl_publisher_fini(p_publisher, p_node);
    // // rclc_executor_fini(p_executor);
    // status = rcl_node_fini(p_node);
    // rclc_support_fini(p_support);

    // return status;
    return (rcl_ret_t)0;
}

void execute_every_n_ms(int64_t ms, SystemState system_state) {
    /* Method for periodically pinging the micro_ros_agent */
    do {
        static volatile int64_t init = -1;
        if (init == -1) {
            init = uxr_millis();
        }
        if (uxr_millis() - init > ms) {
            // system_state; // TODO: find out what this does any why it's here
            init = uxr_millis();
        }
    }
    while (0);
}
#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/detail/multi_array_dimension__functions.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int32.h>
#include <micro_ros_platformio.h>
#include "simple_mnist.h"

#define INPUT_SIZE 784
#define OUTPUT_SIZE 1

rclc_executor_t executor;

rcl_publisher_t result_pub;
rcl_subscription_t input_sub;

std_msgs__msg__UInt8MultiArray input_msg;
std_msgs__msg__MultiArrayDimension dimension_buffer[1];
std_msgs__msg__UInt8 result_msg;

rcl_ret_t ret;
float input_data[INPUT_SIZE];
uint8_t input_data_buffer[INPUT_SIZE];
uint8_t output[OUTPUT_SIZE];

int ros_domain_id = 0;

void mnist_callback(const void * msgin) {
  const std_msgs__msg__UInt8MultiArray * input = (const std_msgs__msg__UInt8MultiArray *)msgin;

  for (size_t i = 0; i < INPUT_SIZE; i++) {
    input_data[i] = (float)input->data.data[i] / 255.0f;
  }

  ebnn_compute(input_data, output);

  result_msg.data = output[0];
  ret = rcl_publish(&result_pub, &result_msg, NULL);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(2000);
  set_microros_serial_transports(Serial);
  delay(2000);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  ret = rcl_init_options_init(&init_options, allocator);
  ret = rcl_init_options_set_domain_id(&init_options, ros_domain_id);

  rclc_support_t support;
  ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  rcl_node_t node;
  rclc_node_init_default(&node, "ebnn_node", "", &support);

  ret = rclc_executor_init(&executor, &support.context, 2, &allocator);

  input_msg.data.data = input_data_buffer;
  input_msg.data.size = 0;
  input_msg.data.capacity = INPUT_SIZE;

  input_msg.layout.dim.data = dimension_buffer;
  input_msg.layout.dim.size = 0;
  input_msg.layout.dim.capacity = 1;
  input_msg.layout.data_offset = 0;

  dimension_buffer[0].label.data = NULL;
  dimension_buffer[0].label.size = 0;
  dimension_buffer[0].label.capacity = 0;
  dimension_buffer[0].size = 0;
  dimension_buffer[0].stride = 0;

  ret = rclc_publisher_init_default(
      &result_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/mnist_result");
  
  ret = rclc_subscription_init_default(
      &input_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
      "/mnist_image_to_ebnn");
  
  ret = rclc_executor_add_subscription(
      &executor,
      &input_sub,
      &input_msg,
      &mnist_callback,
      ON_NEW_DATA);
  
  result_msg.data = 0;
}

void loop() {
  ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (ret != RCL_RET_OK) {
    Serial.println("Executor spin failed");
  }
  delay(10);
}
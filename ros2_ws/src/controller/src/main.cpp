#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>

rcl_publisher_t publisher;
geometry_msgs__msg__Vector3 msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_support_t support;

// Joystick struct
typedef struct {
  float altitude;
  float forwardBackward;
  float leftRight;
} JoystickData;

JoystickData incomingData;

// ESP-NOW receive callback
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataRaw, int len){
  // Copy the received data into our struct
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  
  // Debug output
  Serial.print("Data received: ");
  Serial.print(incomingData.altitude); Serial.print(", "); 
  Serial.print(incomingData.forwardBackward); Serial.print(", ");
  Serial.println(incomingData.leftRight);

  // Populate ROS message
  msg.x = incomingData.altitude;
  msg.y = incomingData.forwardBackward;
  msg.z = incomingData.leftRight;

  rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to publish joystick data");
  }
}

// micro-ROS transport setup (Serial)
void setupMicroROS() {
  set_microros_serial_transports(Serial);
  delay(2000);  // give time for agent connection

  allocator = rcl_get_default_allocator();
  
  // Initialize support, node, and publisher
  rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize support");
    return;
  }
  
  ret = rclc_node_init_default(&node, "esp32_joystick_node", "", &support);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize node");
    return;
  }
  
  ret = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "joystick_data"
  );
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize publisher");
    return;
  }
  
  // Initialize executor
  ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (ret != RCL_RET_OK) {
    Serial.println("Failed to initialize executor");
    return;
  }
  
  Serial.println("micro-ROS initialized successfully");
}

void setup() {
  Serial.begin(115200);
  
  // Setup WiFi in station mode
  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  // Init micro-ROS
  setupMicroROS();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
  
  Serial.println("ESP-NOW initialized successfully");
}

void loop() {
  // Process any pending ROS communications
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  if (ret != RCL_RET_OK) {
    Serial.println("Error in executor spin");
  }
  
  delay(10);  // yield to FreeRTOS
}
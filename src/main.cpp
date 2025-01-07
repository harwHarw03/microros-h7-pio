#include <Arduino.h>
#include <micro_ros_platformio.h>
#include "w5500_ethernet_transport.h"

#include <SPI.h>
#include <Ethernet.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/string_utilities.h>

#include <custom_msgs/msg/imu_info.h>

#include "qmc5883l.h"
#include "imu.h"

byte mac[]       = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,1,177);
IPAddress agent_ip(192,168,1,1);
const size_t agent_port = 8888;

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t euler_pub;
custom_msgs__msg__ImuInfo euler_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)  { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

float linear_acceleration_x = 0.2; // m/s^2
float angular_velocity_z = 0.5;    // rad/s
float current_angle = 0.0;
unsigned long last_time = 0;

void error_loop() {
  while (true) { delay(100); }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (!timer) return;

  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0f;
  last_time = current_time;

  hmc5883l_GetData_calibed(); 
  hmc5883l_computeHeading();  
  imu_update();

  imu_msg.orientation.x = q0;
  imu_msg.orientation.y = q1;
  imu_msg.orientation.z = q2;
  imu_msg.orientation.w = q3;

  imu_msg.angular_velocity.x = gxrs;
  imu_msg.angular_velocity.y = gyrs;
  imu_msg.angular_velocity.z = gzrs;

  imu_msg.linear_acceleration.x = axg;
  imu_msg.linear_acceleration.y = ayg;
  imu_msg.linear_acceleration.z = azg;

   euler_msg.roll  = roll;
  euler_msg.pitch = pitch;
  euler_msg.yaw   = yaw;

  // (timestamp)
   imu_msg.header.stamp.sec = (int)(current_time / 1000);
  imu_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
  imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");

  RCSOFTCHECK(rcl_publish(&imu_pub,   &imu_msg,   NULL));
  RCSOFTCHECK(rcl_publish(&euler_pub, &euler_msg, NULL));
}

void setup() {
  Serial.begin(115200);

  hmc5883l_init();
  imu_init(false);

  //ethernet transport setup 
  Ethernet.init(PB12);//cs pin
  Ethernet.begin(mac, ip);
  delay(1500);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) error_loop();
  set_microros_w5500_ethernet_udp_transports(mac, ip, agent_ip, agent_port);
  delay(500);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "imu_ethernet_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"));

  // RCCHECK(rclc_publisher_init_default(
  //   &euler_pub,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, Euler),
  //   "imu/euler"));
  RCCHECK(rclc_publisher_init_default(
    &euler_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msgs, msg, ImuInfo), // Assumes this is the correct type support
    "imu/euler")); 

  const uint64_t period_ns = RCL_MS_TO_NS(20);
  RCCHECK(rclc_timer_init_default(&timer, &support, period_ns, timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50))); //50Hz
  delay(10);
}











//==========worked serial transport

// #include <Arduino.h>
// #include <micro_ros_platformio.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/int32.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

// rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// void error_loop() {
//   while(1) {
//     delay(100);
//   }
// }

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data++;
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);
//   delay(2000);

//   allocator = rcl_get_default_allocator();

//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

//   RCCHECK(rclc_publisher_init_default(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//     "micro_ros_platformio_node_publisher"));

//   const unsigned int timer_timeout = 1000;
//   RCCHECK(rclc_timer_init_default(
//     &timer,
//     &support,
//     RCL_MS_TO_NS(timer_timeout),
//     timer_callback));

//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));

//   msg.data = 0;
// }

// void loop() {
//   delay(100);
//   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
// }


//======worked ethernet 

// #include <Arduino.h>
// #include <micro_ros_platformio.h>
// #include "w5500_ethernet_transport.h"

// #include <SPI.h>
// #include <Ethernet.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <std_msgs/msg/int32.h>

// // --- ethernet config ---
// byte mac[] = {
//   0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
// };
// IPAddress ip(192, 168, 1, 177);
// IPAddress agent_ip(192, 168, 1, 1);
// size_t agent_port = 8888;

// rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// void error_loop() {
//   while(1) {
//     delay(100);
//   }
// }

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data++;
//   }
// }

// void setup() {
//   Ethernet.init(PB12);

//   Ethernet.begin(mac, ip);
//   delay(1000);

//   if (Ethernet.hardwareStatus() == EthernetNoHardware) {
//     //print an error here
//     error_loop();
//   }
//   if (Ethernet.linkStatus() == LinkOFF) {
//     // Serial.println("Ethernet cable is not connected.");
//   }

//   set_microros_w5500_ethernet_udp_transports(mac, ip, agent_ip, agent_port);
//   delay(2000);

//   allocator = rcl_get_default_allocator();

//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node_ethernet", "", &support));

//   RCCHECK(rclc_publisher_init_default(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//     "micro_ros_platformio_node_publisher"));

//   const unsigned int timer_timeout = 1000;
//   RCCHECK(rclc_timer_init_default(
//     &timer,
//     &support,
//     RCL_MS_TO_NS(timer_timeout),
//     timer_callback));

//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));

//   msg.data = 0;
// }

// void loop() {
//   delay(100);
//   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
// }

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include custom message types
#include <moon_messages/msg/motors.h>
#include <moon_messages/msg/actuators.h>
#include <moon_messages/msg/leds.h>

// Define max number of subscribers
#define MAX_SUBSCRIBERS 5

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ROS entities
rcl_node_t node;
rcl_subscription_t subscribers[MAX_SUBSCRIBERS];
rclc_executor_t executor;
rclc_support_t support;

// Message types using custom messages
moon_messages__msg__Motors motors_msg;
moon_messages__msg__Actuators actuators_msg;
moon_messages__msg__Leds leds_msg;

// Current number of active subscribers
int num_subscribers = 0;

// Callback functions
void motors_callback(const void * msgin)
{
    const moon_messages__msg__Motors * msg = (const moon_messages__msg__Motors *)msgin;
    Serial.println("Motors Control Command received");
    // Process the motors message fields here
}

void actuators_callback(const void * msgin)
{
    const moon_messages__msg__Actuators * msg = (const moon_messages__msg__Actuators *)msgin;
    Serial.println("Actuators Control Command received");
    // Process the actuators message fields here
}

void leds_callback(const void * msgin)
{
    const moon_messages__msg__Leds * msg = (const moon_messages__msg__Leds *)msgin;
    Serial.println("LEDs Control Command received");
    // Process the LEDs message fields here
}

// Error handling function
void error_loop() {
    while(1) {
        Serial.println("Error detected, spinning...");
        delay(1000);
    }
}

// Add subscriber function
template <typename T>
bool add_subscriber(const char* topic_name, const rosidl_message_type_support_t* type_support, 
                                    T* msg, rcl_subscription_callback_t callback) {
    if (num_subscribers >= MAX_SUBSCRIBERS) {
        Serial.println("Max subscribers reached!");
        return false;
    }
    
    RCCHECK(rclc_subscription_init_default(
        &subscribers[num_subscribers],
        &node,
        type_support,
        topic_name
    ));
    
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &subscribers[num_subscribers],
        msg,
        callback,
        ON_NEW_DATA
    ));
    
    num_subscribers++;
    Serial.print("Subscribed to: ");
    Serial.println(topic_name);
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("Starting micro-ROS test...");

    // Set microros transport
    set_microros_transports();
    
    // Initialize rcl
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // Create node
    RCCHECK(rclc_node_init_default(&node, "arduino_subscriber_node", "", &support));
    
    // Create executor with capacity for MAX_SUBSCRIBERS
    RCCHECK(rclc_executor_init(&executor, &rclc_support_get_zero_initialized_context(), MAX_SUBSCRIBERS, &allocator));
    
    // Add subscribers with custom message types
    add_subscriber(
        "/motors_control", 
        ROSIDL_GET_MSG_TYPE_SUPPORT(moon_messages, msg, Motors), 
        &motors_msg, 
        &motors_callback
    );
    
    add_subscriber(
        "/actuators_control", 
        ROSIDL_GET_MSG_TYPE_SUPPORT(moon_messages, msg, Actuators), 
        &actuators_msg, 
        &actuators_callback
    );
    
    add_subscriber(
        "/leds_control", 
        ROSIDL_GET_MSG_TYPE_SUPPORT(moon_messages, msg, Leds), 
        &leds_msg, 
        &leds_callback
    );
    
    Serial.println("Subscribers set up. Waiting for messages...");
}

void loop() {
    // Spin the executor to process messages
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(100);
}
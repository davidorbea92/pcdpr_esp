#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include <mcp_can.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

#define LED_PIN 2

// ROS2 DECLARATIONS
rcl_publisher_t pub_pos;
rcl_publisher_t pub_speed;

rcl_subscription_t m1s;

geometry_msgs__msg__Twist msg;
geometry_msgs__msg__Twist msg_pub_pos;
geometry_msgs__msg__Twist msg_pub_speed;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

//variables de cdpr
//Motor id
float motor1_id=0x141;
float motor2_id=0x142;
float motor3_id=0x143;
float motor4_id=0x144;

//Motor id for receiving
float motor1receive_id=0x241;
float motor2receive_id=0x242;
float motor3receive_id=0x243;
float motor4receive_id=0x244;

//for position control
unsigned long startMillis=0;
unsigned long currentMillis=0;
unsigned long period=50; //period of position publishing in milliseconds

//variables for position response
uint8_t command;
uint8_t data1;
uint8_t data2;
uint8_t data3;
uint8_t data4;
uint8_t data5;
uint8_t data6;
uint8_t data7;

uint32_t posicion;
uint32_t pos1=data1;
uint32_t pos2=data2;
uint32_t pos3=data3;
uint32_t pos4=data4;

//variable for speed response
uint16_t velocidad;
int16_t velocidad1;
int16_t velocidad2;
int16_t velocidad3;
int16_t velocidad4;

uint16_t vel;
uint16_t vel1;

//for speed control
long unsigned int rxId;
unsigned char rxBuf[8];
 
long speedControl=0;
long posControl=0;

float speedControlconstant=5729.577951308;
float posControlconstant=5729.577951308;

long posicion1=0;
long posicion2=0;
long posicion3=0;
long posicion4=0;

unsigned char len=0;
unsigned char buf_speed[8];
unsigned char buf_pos[8];

#define CAN0_INT 21 // GPIO21 in ESP32, GPIO5 IN ESP8266

//byte to change motors to WORKING MODE
byte datac[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


//Pin declaration of MCP module
MCP_CAN CAN0(5);     // Set CS to pin 10 (arduino) and 15 (esp8266) and 5 (esp32)

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void send_speed(float motor_id, float motorreceive_id, float vel){
  speedControl=vel*speedControlconstant;
  buf_speed[0]=0xA2;
  buf_speed[1]=0x00;
  buf_speed[2]=0x00;
  buf_speed[3]=0x00;
  buf_speed[4]=(speedControl & 0xFF);
  buf_speed[5]=(speedControl>>8) & 0xff;
  buf_speed[6]=(speedControl>>16) & 0xff;
  buf_speed[7]=(speedControl>>24) & 0xff;
  
  for (int i=0; i<2; i++){
    CAN0.sendMsgBuf(motor_id, 0, 8, buf_speed);
  }
}

//funciones para publicar y leer los datos de los motores
void rmd_poswrite(){

  buf_pos[0]=0x92;
  buf_pos[1]=0x00;
  buf_pos[2]=0x00;
  buf_pos[3]=0x00;
  buf_pos[4]=0x00;
  buf_pos[5]=0x00;
  buf_pos[6]=0x00;
  buf_pos[7]=0x00;

  for (int i=0; i<2; i++){
    CAN0.sendMsgBuf(motor1_id, 0, 8, buf_pos);
    CAN0.sendMsgBuf(motor2_id, 0, 8, buf_pos);
    CAN0.sendMsgBuf(motor3_id, 0, 8, buf_pos);
    CAN0.sendMsgBuf(motor4_id, 0, 8, buf_pos);
  }
}

void rmd_posread(){
  while(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    if((rxId & 0x40000000) != 0x40000000 && rxId==motor1receive_id){
                   
        command=rxBuf[0];
        data1=rxBuf[1];
        data2=rxBuf[2];
        data3=rxBuf[3];
        data4=rxBuf[4];
        data5=rxBuf[5];
        data6=rxBuf[6];
        data7=rxBuf[7];
        
        if (command==0x92){  
          pos1=data4;
          pos2=data5;
          pos3=data6;
          pos4=data7;
            
          posicion=(pos2<< 8) | pos1;
          posicion=(pos3<<16) | posicion;
          posicion=(pos4<<24) | posicion;
  
          posicion1=posicion;
        }else if (command==0xA2){
          vel=data4;
          vel1=data5;
          
          velocidad=(vel1<< 8) | vel;

          velocidad1=velocidad;
        }
      } else if((rxId & 0x40000000) != 0x40000000 && rxId==motor2receive_id){
        command=rxBuf[0];
        data1=rxBuf[1];
        data2=rxBuf[2];
        data3=rxBuf[3];
        data4=rxBuf[4];
        data5=rxBuf[5];
        data6=rxBuf[6];
        data7=rxBuf[7];
        
        if (command==0x92){
          pos1=data4;
          pos2=data5;
          pos3=data6;
          pos4=data7;
            
          posicion=(pos2<< 8) | pos1;
          posicion=(pos3<<16) | posicion;
          posicion=(pos4<<24) | posicion;
  
          posicion2=posicion;
          
        }else if (command==0xA2){
          vel=data4;
          vel1=data5;
          
          velocidad=(vel1<< 8) | vel;

          velocidad2=velocidad;
        }  
      } else if((rxId & 0x40000000) != 0x40000000 && rxId==motor3receive_id){
        command=rxBuf[0];
        data1=rxBuf[1];
        data2=rxBuf[2];
        data3=rxBuf[3];
        data4=rxBuf[4];
        data5=rxBuf[5];
        data6=rxBuf[6];
        data7=rxBuf[7];
        
        if (command==0x92){  
          pos1=data4;
          pos2=data5;
          pos3=data6;
          pos4=data7;
            
          posicion=(pos2<< 8) | pos1;
          posicion=(pos3<<16) | posicion;
          posicion=(pos4<<24) | posicion;
  
          posicion3=posicion;
          
        }else if (command==0xA2){
          vel=data4;
          vel1=data5;
          
          velocidad=(vel1<< 8) | vel;

          velocidad3=velocidad;
        }  
      } else if ((rxId & 0x40000000) != 0x40000000 && rxId==motor4receive_id){
        command=rxBuf[0];
        data1=rxBuf[1];
        data2=rxBuf[2];
        data3=rxBuf[3];
        data4=rxBuf[4];
        data5=rxBuf[5];
        data6=rxBuf[6];
        data7=rxBuf[7];
        
        if (command==0x92){  
          pos1=data4;
          pos2=data5;
          pos3=data6;
          pos4=data7;
            
          posicion=(pos2<< 8) | pos1;
          posicion=(pos3<<16) | posicion;
          posicion=(pos4<<24) | posicion;

          posicion4=posicion;
          
        }else if (command==0xA2){
          vel=data4;
          vel1=data5;
          
          velocidad=(vel1<< 8) | vel;

          velocidad4=velocidad;
        }  
      }
  }
}

//publicacion de la posicion actual en ros2
void ros_pospub(){
  msg_pub_pos.linear.x=posicion1/posControlconstant;
  msg_pub_pos.linear.y=posicion2/posControlconstant;
  msg_pub_pos.linear.z=posicion3/posControlconstant;
  msg_pub_pos.angular.x=posicion4/posControlconstant;

  msg_pub_speed.linear.x=velocidad1;
  msg_pub_speed.linear.y=velocidad2;
  msg_pub_speed.linear.z=velocidad3;
  msg_pub_speed.angular.x=velocidad4;

  rcl_publish(&pub_pos, &msg_pub_pos, NULL);
  rcl_publish(&pub_speed, &msg_pub_speed, NULL);
}


//twist message cb
void ms_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  send_speed(motor1_id, motor1receive_id, msg->linear.z);
  send_speed(motor2_id, motor2receive_id, msg->angular.x);
  send_speed(motor3_id, motor3receive_id, msg->angular.y);
  send_speed(motor4_id, motor4receive_id, msg->angular.z);
}


void setup() {
  set_microros_transports();

  allocator = rcl_get_default_allocator();

   //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "micro_ros_cdpr_simple", "", &support);

  // create subscriber m1s
  RCCHECK(rclc_subscription_init_best_effort(
    &m1s,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "m_vel"));

  // create publisher motor_pos
  RCCHECK(rclc_publisher_init_best_effort(
    &pub_pos,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "motors_pos"));
  
  // create publisher motor_speed
  RCCHECK(rclc_publisher_init_best_effort(
     &pub_speed,
     &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
     "motors_speed"));

  // create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &m1s, &msg, &ms_callback, ON_NEW_DATA);

  // Initialize MCP2515 running at 8MHz with a baudrate of 1000kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  pinMode(CAN0_INT, INPUT);

  //Send first message to turn motor working mode on
  for (int i=0; i<20; i++){
    CAN0.sendMsgBuf(motor1_id, 0, 8, datac);
    CAN0.sendMsgBuf(motor2_id, 0, 8, datac);
    CAN0.sendMsgBuf(motor3_id, 0, 8, datac);
    CAN0.sendMsgBuf(motor4_id, 0, 8, datac);
  }
}

void loop() {
  delay(0);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  rmd_poswrite();
  rmd_posread();
  currentMillis = millis();
  if (currentMillis - startMillis >= period) {
    ros_pospub();
    startMillis = currentMillis;
  }
}

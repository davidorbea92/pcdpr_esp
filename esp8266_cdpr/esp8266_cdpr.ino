/*Arduino program to control speed of RMD X6-S2 motor through /cmd_vel topic
 Developed by: David Orbea
 Institution: Universidad Politecnica de Madrid
 February 2024
*/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <mcp_can.h>

//Motor id
long motor1_id=0x141;
long motor2_id=0x142;
long motor3_id=0x143;
long motor4_id=0x144;

//Motor id for receiving
long motor1receive_id=0x241;
long motor2receive_id=0x242;
long motor3receive_id=0x243;
long motor4receive_id=0x244;

//for position control
unsigned long startMillis=0;
unsigned long currentMillis=0;
unsigned long period=100; //period of position publishing in milliseconds

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

long speedControlconstant=5729.577951308;
float posControlconstant=5729.577951308;
long speedZero=0;

long posicion1;
long posicion2;
long posicion3;
long posicion4;
long GenPos=0;

unsigned char len=0;
unsigned char buf_speed[8];
unsigned char buf_pos[8];

#define CAN0_INT 5

//byte to change motors to WORKING MODE
byte datac[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


//Pin declaration of MCP module
MCP_CAN CAN0(15);     // Set CS to pin 10 (arduino) and 15 (esp8266)

//ROS DECLARATIONS
ros::NodeHandle  nh;

geometry_msgs::Twist msg_sub_speed;
geometry_msgs::Twist msg_sub_pos;

geometry_msgs::Twist msg_pub_pos;
geometry_msgs::Twist msg_pub_speed;

ros::Publisher pub_speed("motors_speed", &msg_pub_speed);
ros::Publisher pub_pos("motors_pos", &msg_pub_pos);


void send_speed(long motor_id, long motorreceive_id, float vel){
  speedControl=vel*speedControlconstant;
  buf_speed[0]=0xA2;
  buf_speed[1]=0x00;
  buf_speed[2]=0x00;
  buf_speed[3]=0x00;
  buf_speed[4]=(speedControl & 0xFF);
  buf_speed[5]=(speedControl>>8) & 0xff;
  buf_speed[6]=(speedControl>>16) & 0xff;
  buf_speed[7]=(speedControl>>24) & 0xff;
  
  for (int i=0; i<5; i++){
    CAN0.sendMsgBuf(motor_id, 0, 8, buf_speed);
  }
}

void send_position(long motor_id, long motorreceive_id, float pos){
  posControl=pos*posControlconstant;
  buf_speed[0]=0xA4;
  buf_speed[1]=0x00;
  buf_speed[2]=0x64;
  buf_speed[3]=0x00;
  buf_speed[4]=(posControl & 0xFF);
  buf_speed[5]=(posControl>>8) & 0xff;
  buf_speed[6]=(posControl>>16) & 0xff;
  buf_speed[7]=(posControl>>24) & 0xff;
  
  for (int i=0; i<5; i++){
    CAN0.sendMsgBuf(motor_id, 0, 8, buf_speed);
  }
}

void callbackm1_s(const geometry_msgs::Twist& msg_sub){
  float cmd_ros=msg_sub.angular.z;
  send_speed(motor1_id, motor1receive_id, cmd_ros);
}
void callbackm2_s(const geometry_msgs::Twist& msg_sub){
  float cmd_ros=msg_sub.angular.z;
  send_speed(motor2_id, motor2receive_id, cmd_ros);
}

void callbackm3_s(const geometry_msgs::Twist& msg_sub){
  float cmd_ros=msg_sub.angular.z;
  send_speed(motor3_id, motor3receive_id, cmd_ros);
}

void callbackm4_s(const geometry_msgs::Twist& msg_sub){
  float cmd_ros=msg_sub.angular.z;
  send_speed(motor4_id, motor4receive_id, cmd_ros);
}

void callbackm1_pos(const geometry_msgs::Twist& msg_sub){
  float set_pos=msg_sub.angular.z;
  send_position(motor1_id, motor1receive_id, set_pos);
}

void callbackm2_pos(const geometry_msgs::Twist& msg_sub){
  float set_pos=msg_sub.angular.z;
  send_position(motor2_id, motor2receive_id, set_pos);
}

void callbackm3_pos(const geometry_msgs::Twist& msg_sub){
  float set_pos=msg_sub.angular.z;
  send_position(motor3_id, motor3receive_id, set_pos);
}

void callbackm4_pos(const geometry_msgs::Twist& msg_sub){
  float set_pos=msg_sub.angular.z;
  send_position(motor4_id, motor1receive_id, set_pos);
}

ros::Subscriber<geometry_msgs::Twist> m1_s("m1_vel", &callbackm1_s);
ros::Subscriber<geometry_msgs::Twist> m2_s("m2_vel", &callbackm2_s);
ros::Subscriber<geometry_msgs::Twist> m3_s("m3_vel", &callbackm3_s);
ros::Subscriber<geometry_msgs::Twist> m4_s("m4_vel", &callbackm4_s);

ros::Subscriber<geometry_msgs::Twist> m1_pos("m1_pos", &callbackm1_pos);
ros::Subscriber<geometry_msgs::Twist> m2_pos("m2_pos", &callbackm2_pos);
ros::Subscriber<geometry_msgs::Twist> m3_pos("m3_pos", &callbackm3_pos);
ros::Subscriber<geometry_msgs::Twist> m4_pos("m4_pos", &callbackm4_pos);

void rmd_poswrite(){

  buf_pos[0]=0x92;
  buf_pos[1]=0x00;
  buf_pos[2]=0x00;
  buf_pos[3]=0x00;
  buf_pos[4]=0x00;
  buf_pos[5]=0x00;
  buf_pos[6]=0x00;
  buf_pos[7]=0x00;

  for (int i=0; i<5; i++){
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

void ros_pospub(){
 
  msg_pub_pos.linear.x=posicion1/posControlconstant;
  msg_pub_pos.linear.y=posicion2/posControlconstant;
  msg_pub_pos.linear.z=posicion3/posControlconstant;
  msg_pub_pos.angular.x=posicion4/posControlconstant;

  pub_pos.publish(&msg_pub_pos);

  msg_pub_speed.linear.x=velocidad1;
  msg_pub_speed.linear.y=velocidad2;
  msg_pub_speed.linear.z=velocidad3;
  msg_pub_speed.angular.x=velocidad4;
  
  pub_speed.publish(&msg_pub_speed);
}
void setup() {
  //Variables for position publishing
  startMillis = millis();
  
  // Initialize MCP2515 running at 8MHz with a baudrate of 1000kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  pinMode(CAN0_INT, INPUT);

  //Send first message to turn motor working mode on
  for (int i=0; i<10; i++){
    CAN0.sendMsgBuf(motor1_id, 0, 8, datac);
    CAN0.sendMsgBuf(motor2_id, 0, 8, datac);
    CAN0.sendMsgBuf(motor3_id, 0, 8, datac);
    CAN0.sendMsgBuf(motor4_id, 0, 8, datac);
  }

  //Advertising of node, publishers and subscribers in ROS
  nh.initNode();
  nh.subscribe(m1_s);
  nh.subscribe(m2_s);
  nh.subscribe(m3_s);
  nh.subscribe(m4_s);

  nh.subscribe(m1_pos);
  nh.subscribe(m2_pos);
  nh.subscribe(m3_pos);
  nh.subscribe(m4_pos);
  
  nh.advertise(pub_pos);
  nh.advertise(pub_speed);
  
}

void loop() {
  nh.spinOnce();
  rmd_poswrite();
  rmd_posread();
  //For position publish
  currentMillis=millis();
  if (currentMillis-startMillis>=period)
  {
    
    ros_pospub();   
    
    //Reset of the timer
    startMillis=currentMillis;
  }
  
}

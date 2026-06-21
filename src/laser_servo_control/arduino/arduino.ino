#include <ros.h>
#include <std_msgs/Bool.h>
#include <Servo.h>

ros::NodeHandle nh;

const int servoPin = 9;  // Nano D9 引脚
const int laserPin = 10; // 激光控制引脚

Servo myServo;

// 1. 舵机控制回调函数：true 为打开(135°)，false 为关闭(90°)
void servoCallback(const std_msgs::Bool& msg) {
  myServo.attach(servoPin);
  
  if (msg.data) {
    myServo.write(135);
    delay(600); // 确保物理到位
    nh.loginfo("Dropper Action: OPEN");
  } else {
    myServo.write(90);
    delay(600);
    nh.loginfo("Dropper Action: RESET/CLOSE");
  }
  
  myServo.detach(); // 待机不耗电
}

// 2. 激光控制回调函数：true 为亮，false 为灭
void laserCallback(const std_msgs::Bool &msg) {
  // msg.data 为 true 时，我们写入 LOW 来点亮低电平触发的激光
  if (msg.data) {
    digitalWrite(laserPin, LOW); // 反转：true 对应 LOW (开)
    nh.loginfo("Laser Status: ON");
  } else {
    digitalWrite(laserPin, HIGH); // 反转：false 对应 HIGH (关)
    nh.loginfo("Laser Status: OFF");
  }
}

// 声明两个独立的订阅者
ros::Subscriber<std_msgs::Bool> sub_servo("dropper_open", servoCallback);
ros::Subscriber<std_msgs::Bool> sub_laser("laser_on", laserCallback);

void setup() {
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);
  
  // 初始化舵机位置
  myServo.attach(servoPin);
  myServo.write(90);
  delay(500);
  myServo.detach(); 
  
  // 初始化 ROS 节点并注册两个话题
  nh.initNode();
  nh.subscribe(sub_servo);
  nh.subscribe(sub_laser);
}

void loop() {
  nh.spinOnce();
  delay(10); // 适度延时，保证串口通讯稳定
}
#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define OFFSET -9
#define MAX_ANGLE 30
#define MAX_SPEED 255

#define SERVO_PIN 10
#define MOTOR_PWM_PIN 9
#define MOTOR_DIR_PIN 8

// 命令消息处理
void process(const geometry_msgs::Twist &msg);
// 控制转向
void steer_ctl(float angle);
// 控制电机
void steer_motor(float speed);
// 转向角度映射
int map_angle(float angle);
// 电机速度映射
int map_speed(float speed);

// 变量
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", process);
Servo my_servo;

void setup() {
  // put your setup code here, to run once:
  pinMode(SERVO_PIN, OUTPUT);
  my_servo.attach(SERVO_PIN);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}

void process(const geometry_msgs::Twist &msg){
  float angle = msg.angular.z;
  float speed = msg.linear.x;
  ctl_steer(angle);
  ctl_motor(speed);
}

void ctl_steer(float angle){
  int out_angle = map_angle(angle);
  my_servo.write(out_angle);
}

void ctl_motor(float speed){
  int out_speed = map_speed(speed);
  int pwm = 0;
  int dir = 0;
  
  if(out_speed > 0){
    pwm = out_speed;
    dir = 1;
  }else if(out_speed < 0){
    pwm = -out_speed;
    dir = 0;
  }else{
    pwm = 0;
    dir = 0;
  }

  analogWrite(MOTOR_PWM_PIN, pwm);
  digitalWrite(MOTOR_DIR_PIN, dir);
}

int map_angle(float angle){
  return (int)(angle * MAX_ANGLE) + 90 + OFFSET;
}

int map_speed(float speed){
  return (int)(speed * MAX_SPEED);
}

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#define linear_switch 7 // linear actuator on or off
#define linear_dir 8 // linear actuator direction: extend or shrink

Servo servo1;  // Base servo
Servo servo2;  // arm servo
// twelve servo objects can be created on most boards
int servo1_pos = 60;    // variable to store the servo position 0 - 175
int servo2_pos = 105; //0-200
int scale = 5;
ros::NodeHandle  nh;

void arm_cb( const geometry_msgs::Vector3 &msg){
  if (msg.x > 0){
    if (servo1_pos <= 175) {servo1_pos+=scale;}
    servo1.write(servo1_pos);
    delay(50);
  }
  else if (msg.x < 0){
    if (servo1_pos >= 0) {servo1_pos-=scale;}
    servo1.write(servo1_pos);
    delay(50);
  }

  if (msg.y > 0){
    if (servo2_pos <= 200) {servo2_pos+=scale;}
    servo2.write(servo2_pos);
    delay(50);
  }
  else if (msg.y < 0){
    if (servo2_pos >= 0) {servo2_pos-=scale;}
    servo2.write(servo2_pos);
    delay(50);
  }
  
  if (msg.z > 0){
        digitalWrite(linear_switch, HIGH);
        digitalWrite(linear_dir, HIGH);
  }
  else if (msg.z < 0){  
    digitalWrite(linear_switch, HIGH);
    digitalWrite(linear_dir, LOW);
    }
  else
  {
    digitalWrite(linear_switch, LOW);
    }

  
  
}


ros::Subscriber<geometry_msgs::Vector3> arm_sub("arm_control", &arm_cb);



void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(arm_sub);
  //pinMode(8, OUTPUT);
  servo1.attach(10,800,2200);  // First (base) servo
  servo2.attach(9,800,2200);   // Second servo
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
}
void loop() {
  nh.spinOnce();
  delay(1);
}

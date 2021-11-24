/* 
 * rosserial Planar Odometry Example
 */
#include <Encoder.h>
#include <Wire.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

ros::NodeHandle  nh;
geometry_msgs::Vector3Stamped encoder_ticks; 

int left_f = 2;
int left_r = 3;
int right_f = 4;
int right_r = 6;

Encoder right_encoder(16,17);
Encoder left_encoder(18,19);

long new_ticks_right = 0;
long new_ticks_left = 0;

char base_link[] = "base_link";
char odom[] = "odom";

void callback(const geometry_msgs::Twist& vel)
{
  if(vel.linear.x > 0)
  {
    analogWrite(left_f, vel.linear.x*255);
    analogWrite(right_f, vel.linear.x*255);

    analogWrite(left_r, 0);
    analogWrite(right_r, 0);
  }
  if(vel.linear.x < 0)
  { 
    analogWrite(left_f, 0);
    analogWrite(left_f, 0);
    
    analogWrite(left_r, vel.linear.x*255);
    analogWrite(right_r, vel.linear.x*255);      
  }
  if(vel.angular.z > 0)
  {
    analogWrite(left_r, vel.angular.z*255);
    analogWrite(right_f, vel.angular.z*255);

    analogWrite(left_f, 0);
    analogWrite(right_r, 0);
  }
  if(vel.angular.z < 0)
  {
    analogWrite(left_f, vel.angular.z*255);
    analogWrite(right_r, vel.angular.z*255);

    analogWrite(left_r, 0);
    analogWrite(right_f, 0);
  }
  if(vel.linear.x ==0 && vel.angular.z == 0)
  {
    analogWrite(left_f, 0);
    analogWrite(right_f, 0);

    analogWrite(left_r, 0);
    analogWrite(right_r, 0);
  }
}
ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", callback);
ros::Publisher encoder_publisher("encoder_ticks", &encoder_ticks);

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(encoder_publisher);
  nh.subscribe(twist_sub);
  pinMode(left_f, OUTPUT);
  pinMode(left_r, OUTPUT);
  pinMode(right_f, OUTPUT);
  pinMode(right_r, OUTPUT);
}

void loop()
{ 
  nh.spinOnce();
  new_ticks_right = right_encoder.read();
  new_ticks_left = left_encoder.read();
  encoder_ticks.header.stamp = nh.now();      //timestamp for odometry data
  encoder_ticks.vector.x = new_ticks_left;    //left wheel speed (in m/s)
  encoder_ticks.vector.y = new_ticks_right;   //right wheel speed (in m/s)
  encoder_ticks.vector.z = 0;         //looptime, should be the same as specified in LOOPTIME (in s)
  encoder_publisher.publish(&encoder_ticks);
}

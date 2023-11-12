#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 target_distance_msg;

std_msgs::Int32 encoder_msg;
ros::Publisher encoder_pub("encoder_position", &encoder_msg);
void targetDistanceCallback(const std_msgs::Float32& msg) {
  Serial.print("Received target distance: ");
  Serial.println(msg.data);
  // Add your code here to perform actions based on the received distance
}

ros::Subscriber<std_msgs::Float32> sub("target_distance", &targetDistanceCallback);

void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

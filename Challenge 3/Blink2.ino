
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::Float32& msg){
  Serial.println(msg.data);

}

ros::Subscriber<std_msgs::Float32> sub("signal", &messageCb );

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(9600);
}

void loop()
{  
  nh.spinOnce();
  delay(1);

  
}

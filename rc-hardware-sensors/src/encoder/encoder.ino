#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder left_encoder(3, 6);
Encoder right_encoder(2, 5);
//   avoid using pins with LEDs attached

ros::NodeHandle nh;
std_msgs::Int32 left_encoder_msg;
std_msgs::Int32 right_encoder_msg;
std_msgs::Float32 steering_angle_msg;

ros::Publisher encoder_left_pub("encoder/left", &left_encoder_msg);
ros::Publisher encoder_right_pub("encoder/right", &right_encoder_msg);
ros::Publisher steering_angle_pub("steering_angle", &steering_angle_msg);


void setup()
{
    Serial.begin(9600);
    Serial.println("Basic Encoder Test:");
    nh.initNode();
    nh.advertise(encoder_left_pub);
    nh.advertise(encoder_right_pub);
    nh.advertise(steering_angle_pub);
}

void loop()
{
    left_encoder_msg.data = (int) right_encoder.read();
    right_encoder_msg.data = (int) left_encoder.read();
    steering_angle_msg.data = 57.62;

    encoder_left_pub.publish( &left_encoder_msg );
    encoder_right_pub.publish( &right_encoder_msg );
    steering_angle_pub.publish( &steering_angle_msg );
    

    nh.spinOnce();
    delay(100);
}

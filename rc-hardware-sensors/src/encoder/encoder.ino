#include <Encoder.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <ros_lib/rc_hardware_sensors/SensorCollect.h>

Encoder left_encoder(3, 6);
Encoder right_encoder(2, 5);

ros::NodeHandle nh;
rc_hardware_sensors::SensorCollect msg;

ros::Publisher sensor_collect_pub("sensor_collect", &msg);

void setup()
{
    Serial.begin(9600);
    Serial.println("Basic Encoder Test:");
    nh.initNode();
    nh.advertise(sensor_collect_pub);
    nh.negotiateTopics();
}

void loop()
{
    // Populate the message with sensor data
    msg.encoder_left.data = (int) left_encoder.read();
    msg.encoder_right.data = (int) right_encoder.read();
    msg.steering_angle.data = 57.62;

    // Publish the sensor data
    sensor_collect_pub.publish(&msg);

    nh.spinOnce();
    delay(100);
}

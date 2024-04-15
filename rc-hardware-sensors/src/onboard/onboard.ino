#include <ros.h>
#include <Encoder.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <rc_hardware_sensors/SensorCollect.h>
#include <ackermann_msgs/AckermannDrive.h>

void ackermannDriveCallback(const ackermann_msgs::AckermannDrive msg)
{
    // This function will be called whenever a new Ackermann drive message is received
    Serial.print("Received Ackermann drive message:");
}

Encoder left_encoder(3, 6);
Encoder right_encoder(2, 5);

ros::NodeHandle nh;
rc_hardware_sensors::SensorCollect msg;

ros::Publisher sensor_collect_pub("sensor_collect", &msg);
ros::Subscriber<ackermann_msgs::AckermannDrive> sub("ackermann_cmd", ackermannDriveCallback);

long last_time;

void setup()
{
    Serial.begin(9600);
    nh.initNode();
    nh.advertise(sensor_collect_pub);

    // Helps when using custom topics over rosserial
    nh.negotiateTopics();
}

void loop()
{
    // Set last time for rate
    last_time = millis();
    // Populate the message with sensor data
    msg.timestamp.data = millis();
    msg.encoder_left.data = (int) left_encoder.read();
    msg.encoder_right.data = (int) right_encoder.read();
    msg.steering_angle.data = 57.62; // Placeholder value

    // Publish the sensor data
    sensor_collect_pub.publish(&msg);

    // Spin and sleep until time for next loop

    while (10 > millis()-last_time) { // 1 Hz
      delay(0);
    }
    nh.spinOnce();
}

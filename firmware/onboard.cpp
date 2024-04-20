#include <Arduino.h>
#include <ros.h>
#include <Encoder.h>
#include <Servo.h>
#include <rc_localization_odometry/SensorCollect.h>
#include <ackermann_msgs/AckermannDrive.h>

#define enA 5
#define in1 A1
#define in2 A2

#define enB 6
#define in3 A3
#define in4 A4

#define servoPin 9
Servo servo;

const float MAX_SPEED = 1.0;
const float MIN_SPEED = -1.0;

const float STEERING_ZERO_ANGLE = 110.0;
const float STEERING_MAX_ANGLE = 180.0;
const float STEERING_MIN_ANGLE = 40.0;


void writeAngle(float angle) {
  if (angle > 0) {
    servo.write(min(STEERING_ZERO_ANGLE + angle, STEERING_MAX_ANGLE));
  } else {
    servo.write(max(STEERING_ZERO_ANGLE - angle, STEERING_MIN_ANGLE));
  }
}

void writePercent(float value) {
  if (value >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(enA, value * 255);
    analogWrite(enB, value * 255);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, -value * 255);
    analogWrite(enB, -value * 255);
  }
}

void writeAckermann(float angle, float speed) {
  writeAngle(angle);
  writePercent(speed);
}



void ackermannDriveCallback(const ackermann_msgs::AckermannDrive& msg)
{
    // This function will be called whenever a new Ackermann drive message is received
    Serial.print("Received Ackermann drive message:");
}

Encoder left_encoder(2, 5);
Encoder right_encoder(3, 6);

ros::NodeHandle nh;
rc_localization_odometry::SensorCollect msg;

ros::Publisher sensor_collect_pub("sensor_collect", &msg);
ros::Subscriber<ackermann_msgs::AckermannDrive> sub("ackermann_msg", &ackermannDriveCallback);

long last_time;

void setup()
{
    Serial.begin(9600);
    nh.initNode();
    nh.advertise(sensor_collect_pub);

    // Helps when using custom topics over rosserial
    nh.negotiateTopics();

    servo.attach(servoPin);

    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    digitalWrite(enA, LOW);
    digitalWrite(enB, LOW);
}

void loop()
{

    writeAckermann(25, 0.5);

    // Set last time for rate
    last_time = millis();
    // Populate the message with sensor data
    msg.timestamp = millis();
    msg.encoder_left = -(int)left_encoder.read();
    msg.encoder_right = (int)right_encoder.read();
    msg.steering_angle = 57.62; // Placeholder value

    // Publish the sensor data
    sensor_collect_pub.publish(&msg);

    // Spin and sleep until time for next loop

    nh.spinOnce();
}

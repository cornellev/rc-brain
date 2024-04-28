#include <Arduino.h>
#include <ros.h>
#include <Encoder.h>
#include <Servo.h>
#include <rc_localization_odometry/SensorCollect.h>
#include <ackermann_msgs/AckermannDrive.h>

#define EN_A 5
#define IN_1 A1
#define IN_2 A2

#define EN_B 6
#define IN_3 A3
#define IN_4 A4

#define ENCODER_LEFT_C1 2
#define ENCODER_LEFT_C2 11
#define ENCODER_RIGHT_C1 3
#define ENCODER_RIGHT_C2 12

#define SERVO_PIN 9
#define POTENTIOMETER_PIN A6

const float MAX_SPEED = 1.0; // Max percent speed of vehicle
const float MIN_SPEED = -1.0; // Min percent speed

const float STEERING_ZERO_ANGLE = 98.0; // Calibrated servo angle corresponding to a steering angle of 0
const float MAX_INPUT_STEER = 20.0; // Steering range is from -MAX_INPUT_STEER to MAX_INPUT_STEER

const double WHEEL_DIAMETER_METERS = 9.5 / 100;
const double TICKS_PER_REV = 827.2;
const double TICKS_TO_METERS = (1 / TICKS_PER_REV) * (2 * M_PI * (WHEEL_DIAMETER_METERS / 2));

// Initialize hardware/sensors
Encoder left_encoder(ENCODER_LEFT_C1, ENCODER_LEFT_C2);
Encoder right_encoder(ENCODER_RIGHT_C1, ENCODER_RIGHT_C2);
Servo servo;


/**
   Turn steering servo to specified angle. Angle will be clamped to be between -MAX_INPUT_STEER and MAX_INPUT_STEER.

   @param angle: Angle to turn the servo to. Positive values turn the wheels to the right, negative values turn the wheels to the left. Zero is straight ahead.
*/
void writeAngle(float angle) {
  angle = max(min(STEERING_ZERO_ANGLE + angle, STEERING_ZERO_ANGLE + MAX_INPUT_STEER), STEERING_ZERO_ANGLE - MAX_INPUT_STEER);
  servo.write(angle);
}


/**
   Set the speed of the vehicle. Positive values move the vehicle forward, negative values move the vehicle backward.

   @param value: Speed of the vehicle as a percentage of the maximum speed.
   @pre -1 <= value <= 1
*/
void writePercent(float value) {
  if (value >= 0) {
    digitalWrite(IN_1, HIGH);
    digitalWrite(IN_2, LOW);
    digitalWrite(IN_3, HIGH);
    digitalWrite(IN_4, LOW);

    analogWrite(EN_A, value * 255);
    analogWrite(EN_B, value * 255);
  } else {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, HIGH);
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, HIGH);

    analogWrite(EN_A, -value * 255);
    analogWrite(EN_B, -value * 255);
  }
}


/**
   Write the steering angle and speed of the vehicle.

   @param angle: Angle to turn the steering servo to. Positive values turn the wheels to the right, negative values turn the wheels to the left. Zero is straight ahead.
   @param speed: Speed of the vehicle as a percentage of the maximum speed.
   @pre -1 <= speed <= 1
*/
void writeAckermann(float angle, float speed) {
  writeAngle(angle);
  writePercent(speed);
}


/**
   Callback for ackermann drive messages. Converts the steering angle and speed to the appropriate servo angle and motor speed.
*/
void ackermannDriveCallback(const ackermann_msgs::AckermannDrive& msg) {
  writeAckermann(msg.steering_angle * MAX_INPUT_STEER, msg.speed);
  //  last_message_time = millis(); // Update received last message time
}


/**
 * Get the traveled distance of the right encoder in meters (relative to 0).
 */
float leftEncoderPosition() {
  return left_encoder.read() * TICKS_TO_METERS;
}

/**
 * Get the traveled distance of the right encoder in meters (relative to 0).
 */
float rightEncoderPosition() {
  return right_encoder.read() * TICKS_TO_METERS;
}

// Setup ROS interface
ros::NodeHandle nh;
rc_localization_odometry::SensorCollect msg;
ros::Publisher sensor_collect_pub("sensor_collect", &msg); // Publishes all sensor data
ros::Subscriber<ackermann_msgs::AckermannDrive> sub("rc_movement_msg", &ackermannDriveCallback); // Listens to ackermann drive messages and drives the vehicle

float last_encoder_left;
float last_encoder_right;
long last_time;

void setup()
{
  // Start node
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(sensor_collect_pub);

  // Helps when using custom topics over rosserial
  nh.negotiateTopics();

  // Initialize servo
  servo.attach(SERVO_PIN);

  // Set up motor pins
  pinMode(EN_A, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(EN_B, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  // Start vehicle at 0 speed and 0 steering angle
  writeAckermann(0, 0);

  last_encoder_left = leftEncoderPosition();
  last_encoder_right = rightEncoderPosition();
  last_time = millis();
  
  delay(2000);
}

void loop()
{
  if (100 < millis() - last_time) { // Run once every ~100 ms
    int current_time = millis();
    
    // Encoder delta calculations
    float encoder_left =  leftEncoderPosition();
    float encoder_right = rightEncoderPosition();
    float avg_dist = (
      (encoder_left - last_encoder_left) + 
      (encoder_right - last_encoder_right)
    ) / 2.0;
    
    // Velocity calculations
    double encoder_velocity = avg_dist / ((current_time - last_time) / 1000);

    // Store values for next update
    last_encoder_left = encoder_left;
    last_encoder_right = encoder_right;
    last_time = current_time;

    // Populate the message with sensor data
    msg.timestamp = current_time;

    msg.steering_angle = analogRead(POTENTIOMETER_PIN);
    msg.velocity = encoder_velocity;

    // Publish the sensor data
    sensor_collect_pub.publish(&msg);
  }

  nh.spinOnce();
}

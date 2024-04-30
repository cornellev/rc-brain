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

const float MAX_VELOCITY = 2.0; // Max velocity of vehicle in m/s
const float MIN_VELOCITY = -2.0; // Min velocity of vehicle in m/s

const float MAX_POWER_SPEED = 1.0; // Max percent speed of vehicle
const float MIN_POWER_SPEED = -1.0; // Min percent speed

const float STEERING_ZERO_ANGLE = 98.0; // Calibrated servo angle corresponding to a steering angle of 0  TODO: Calibrate
const float MAX_INPUT_STEER = 20.0; // Steering range is from -MAX_INPUT_STEER to MAX_INPUT_STEER (Degrees)  TODO: Calibrate

const float WHEEL_DIAMETER_METERS = 9.5 / 100;
const float TICKS_PER_REV = 827.2;
const float TICKS_TO_METERS = (1 / TICKS_PER_REV) * (2 * M_PI * (WHEEL_DIAMETER_METERS / 2));

const float POTENTIOMETER_ZERO = 370.0; // Calibrated potentiometer value corresponding to a steering angle of 0  TODO: Calibrate
const float POTENTIOMETER_RANGE = 50.0; // Range of potentiometer values corresponding to a steering angle of -MAX_INPUT_STEER to MAX_INPUT_STEER  TODO: Calibrate

float current_angle = 0.0;
float data_points = 0.0;

float current_velocity;
float target_velocity;
float target_angle;
float next_vel = 0;

float last_encoder_left;
float last_encoder_right;
long last_update_time;
long last_push_time;
long last_error_time;

// SID controller values
const float kP = .002; // Proportional gain for SID controller
const float kI = .005; // Integral gain for SID controller
const float kD = .002; // Derivative gain for SID controller // .003
//float integral = 0;
float last_error = 0.0;
float total_error = 0.0;

float given_power = 0.0;
float max_speed = 0.0;

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
  value = max(min(value, 1), -1);
  
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


void updateAckermann() {
  long current_time = millis();

  float error = target_velocity - current_velocity;
  float delta_error = error - last_error;

  given_power = max(-1, min(1, (given_power + kP * error + kD * delta_error)));

  // given_power = kP * error + kI * total_error + kD * ((error - last_error) / ((current_time - last_error_time) / 1000.0));

  last_error = error;
//  total_error += error;
  last_error_time = current_time;

  writeAckermann(target_angle, given_power + (target_velocity / MAX_VELOCITY));
}


/**
   Callback for ackermann drive messages. Converts the steering angle and speed to the appropriate servo angle and motor speed.
*/
void ackermannDriveCallback(const ackermann_msgs::AckermannDrive& msg) {
  target_angle = msg.steering_angle;
  target_velocity = msg.speed;
}

void updateVelocity() {
  long current_time = millis();
    
  // Encoder delta calculations
  // float encoder_left =  left_encoder.read() * TICKS_TO_METERS;
  float encoder_left = right_encoder.read() * TICKS_TO_METERS;
  float encoder_right = right_encoder.read() * TICKS_TO_METERS;
  float avg_dist = (
    (encoder_left - last_encoder_left) + 
    (encoder_right - last_encoder_right)
  ) / 2.0;
  
  // Velocity calculations
  current_velocity = avg_dist / ((current_time - last_update_time) / 1000.0);

  // Store values for next update
  last_encoder_left = encoder_left;
  last_encoder_right = encoder_right;
  last_update_time = current_time;
}

void updateAngle() {
  float poten = (analogRead(POTENTIOMETER_PIN) - POTENTIOMETER_ZERO) / POTENTIOMETER_RANGE * MAX_INPUT_STEER;
  float multiplier = max(poten - current_angle, current_angle - poten) * 4.0;

  current_angle = (current_angle * data_points + multiplier * poten) / (data_points + multiplier);
  data_points += 1.0;
}

void resetAngle() {
  data_points = 100;
}

// Setup ROS interface
ros::NodeHandle nh;
rc_localization_odometry::SensorCollect msg;
ros::Publisher sensor_collect_pub("sensor_collect", &msg); // Publishes all sensor data
ros::Subscriber<ackermann_msgs::AckermannDrive> sub("rc_movement_msg", &ackermannDriveCallback); // Listens to ackermann drive messages and drives the vehicle

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
  target_velocity = 0.0;
  target_angle = 0.0;
  current_velocity = 0.0;

  last_encoder_left = -right_encoder.read() * TICKS_TO_METERS;
  last_encoder_right = -right_encoder.read() * TICKS_TO_METERS;
  delay(2000);
  last_update_time = millis();
  last_push_time = millis();
  last_error_time = millis();
}

void loop()
{
  long current_time = millis();

  if (3 < current_time - last_update_time) { // Run once every ~5 ms
    updateVelocity();
    updateAngle();
    updateAckermann();
  }
  
  if (5 < current_time - last_push_time) { // Run once every ~50 ms
    if (current_velocity > max_speed) {
      max_speed = current_velocity;
    }
    // Populate the message with sensor data
    msg.timestamp = current_time;
     msg.steering_angle = current_velocity;
//    msg.velocity = current_velocity;
    msg.velocity = max_speed;
//    msg.steering_angle = target_angle;
//     msg.velocity = next_vel;
//    msg.steering_angle = (float) right_encoder.read();
//    msg.velocity = (float) left_encoder.read();

    resetAngle();

    // Publish the sensor data
    sensor_collect_pub.publish(&msg);

    last_push_time = current_time;
  }

  nh.spinOnce();
}

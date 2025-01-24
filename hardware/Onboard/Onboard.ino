#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>

#define EN_A 5
#define IN_1 A2
#define IN_2 A3

#define EN_B 3
#define IN_3 A5
#define IN_4 A4

#define ENCODER_LEFT_C1 9
#define ENCODER_LEFT_C2 8
#define ENCODER_RIGHT_C1 6
#define ENCODER_RIGHT_C2 7

#define SERVO_PIN 10
#define POTENTIOMETER_PIN A5

#define TO_RAD(angle) ((angle) * M_PI / 180.0)
#define TO_DEG(angle) ((angle) * 180.0 / M_PI)

const float MAX_VELOCITY = 2.07; // Max velocity of vehicle in m/s
const float MIN_VELOCITY = -2.07; // Min velocity of vehicle in m/s

const float MAX_POWER_SPEED = 1.0; // Max percent speed of vehicle
const float MIN_POWER_SPEED = -1.0; // Min percent speed

const float STEERING_ZERO_ANGLE = 89.27; // Calibrated servo angle corresponding to a steering angle of 0  TODO: Calibrate
const float MAX_INPUT_STEER = 20.0; // Steering range is from -MAX_INPUT_STEER to MAX_INPUT_STEER (Degrees)  TODO: Calibrate

const float WHEEL_DIAMETER_METERS = 9.5 / 100;
const float TICKS_PER_REV = 827.2;
const float TICKS_TO_METERS = (1 / TICKS_PER_REV) * (2 * M_PI * (WHEEL_DIAMETER_METERS / 2));

const float POTENTIOMETER_ZERO = 370.0; // Calibrated potentiometer value corresponding to a steering angle of 0  TODO: Calibrate
const float POTENTIOMETER_RANGE = 50.0; // Range of potentiometer values corresponding to a steering angle of -MAX_INPUT_STEER to MAX_INPUT_STEER  TODO: Calibrate

float current_velocity;
float target_velocity;
float target_angle;
float next_vel = 0;

float debug_right_encoder = 0.0;
float debug_left_encoder = 0.0;

float last_encoder_left;
float last_encoder_right;
long last_update_time;
long last_push_time;
long last_error_time;

float autobrake = MAX_VELOCITY;

// SID controller values (Sproportional Integral Derivative)
float kP = .002; // Proportional gain for SID controller
const float kI = .005; // Integral gain for SID controller
const float kD = .002; // Derivative gain for SID controller // .003
// const float kD = 0.0;
//float integral = 0;
float last_error = 0.0;
float total_error = 0.0;

float given_power = 0.0;
float max_speed = 0.0;

float reported_val = 1.6;

// Initialize hardware/sensors
Encoder left_encoder(ENCODER_LEFT_C1, ENCODER_LEFT_C2);
Encoder right_encoder(ENCODER_RIGHT_C1, ENCODER_RIGHT_C2);
Servo servo;


/**
   Turn steering servo to specified angle. Angle will be clamped to be between -MAX_INPUT_STEER and MAX_INPUT_STEER.

   @param angle: Angle to turn the servo to. Positive values turn the wheels to the right, negative values turn the wheels to the left. Zero is straight ahead.
*/
void writeAngle(float angle) {
  angle = -TO_DEG(angle);
  angle = max(min(STEERING_ZERO_ANGLE - angle, STEERING_ZERO_ANGLE + MAX_INPUT_STEER), STEERING_ZERO_ANGLE - MAX_INPUT_STEER);
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
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, HIGH);
    digitalWrite(IN_3, HIGH);
    digitalWrite(IN_4, LOW);

    analogWrite(EN_A, value * 255);
    analogWrite(EN_B, value * 255);
  } else {
    digitalWrite(IN_1, HIGH);
    digitalWrite(IN_2, LOW);
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

float targeted_power = 0.0;

/**
  Update the target power and steering angle based on set targets and feedback.
 */
void updateAckermann() {
  // float target_vel = target_velocity;

  float target_vel = min(target_velocity, autobrake);

  reported_val = target_vel;

  if ((autobrake < .6 && target_vel > 0.0) || autobrake < current_velocity) {
    given_power = 0.0;
    kP = 4.5;
  } else {
    kP = .002;
  }

  if (target_vel > MAX_VELOCITY) {
    target_vel = MAX_VELOCITY;
  } else if (target_vel < MIN_VELOCITY) {
    target_vel = MIN_VELOCITY;
  }
  
  long current_time = millis();

  float error = target_vel - current_velocity;
  float delta_error = error - last_error;

  given_power = max(-1, min(1, (given_power + kP * error + kD * delta_error)));

  // given_power = kP * error + kI * total_error + kD * ((error - last_error) / ((current_time - last_error_time) / 1000.0));

  last_error = error;
//  total_error += error;
  last_error_time = current_time;

//  if (((given_power + target_velocity / MAX_VELOCITY) < .2 && (given_power + target_velocity / MAX_VELOCITY) > -.2) || target_velocity == 0.0) {
//    writeAckermann(target_angle, 0);
//  } else {
  if (abs(current_velocity < .01) && abs(target_vel) < .01) {
    writeAckermann(target_angle, 0);
    targeted_power = 0.0;
    // reported_val = 0.0;
  } else {
    targeted_power = given_power + (target_vel / MAX_VELOCITY);
    writeAckermann(target_angle, given_power + (target_vel / MAX_VELOCITY));
    // reported_val = given_power + (target_vel / MAX_VELOCITY);
  }
//  }
}

void updateEncoders() {
  debug_right_encoder = left_encoder.read() * TICKS_TO_METERS;
  debug_left_encoder = right_encoder.read() * TICKS_TO_METERS;
}

/**
  Use encoder value changes to update current velocity.
 */
void updateVelocity() {
  long current_time = millis();
    
  // Encoder delta calculations
  // float encoder_left =  left_encoder.read() * TICKS_TO_METERS;
  // float avg_dist = (
  //   (encoder_left - last_encoder_left) + 
  //   (encoder_right - last_encoder_right)

  float encoder_left = left_encoder.read() * TICKS_TO_METERS;
  float encoder_right = right_encoder.read() * TICKS_TO_METERS;

  // ) / 2.0;
  float avg_dist = ((encoder_left - last_encoder_left) + (encoder_right - last_encoder_right)) / 2.0; // TODO: Temporarily remove right encoder bc not working
  // float avg_dist = encoder_left;
  
  // Velocity calculations
  current_velocity = avg_dist / ((current_time - last_update_time) / 1000.0);
  // current_velocity = avg_dist;
  // current_velocity = encoder_right;

  // Store values for next update
  last_encoder_left = encoder_left;
  last_encoder_right = encoder_right;
}

/**
  Callback for drive messages. Sets the target angle and velocity.
  @param angle: Steering angle in radians. Positive turns right, negative turns left (i know this is dumb TODO)
  @param velocity: Target velocity in m/s
*/
void ackermannDriveCallback(float angle, float velocity) {
  target_angle = angle;
  target_velocity = velocity;
}

/**
  Callback for autobrake message. Sets the maximum allowed velocity.
  @param max_vel: Maximum velocity allowed by autobrake
 */
void autobrakeCallback(float max_vel) {
  autobrake = max_vel;
}

void parseMessage(String received_data) {
    // Split the string based on spaces
    int first_space = received_data.indexOf(' ');
    int second_space = received_data.indexOf(' ', first_space + 1);

    if (first_space > 0 && second_space > 0) {
        // Extract substrings for each value
        String velocity_str = received_data.substring(0, first_space);
        String angle_str = received_data.substring(first_space + 1, second_space);
        String autobrake_str = received_data.substring(second_space + 1);

        // Convert to float
        target_velocity = velocity_str.toFloat();
        target_angle = angle_str.toFloat();
        autobrake = autobrake_str.toFloat();

        // Print the parsed values
        // Serial.print("Parsed Velocity: ");
        // Serial.println(target_velocity);
        // Serial.print("Parsed Steering Angle: ");
        // Serial.println(target_angle);
        // Serial.print("Parsed Max Velocity: ");
        // Serial.println(autobrake);
    } else {
      // target_velocity = 0.0;
    }
}

void setup() {
  Serial.begin(115200);

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

  servo.attach(SERVO_PIN);

  last_encoder_left = left_encoder.read() * TICKS_TO_METERS;
  last_encoder_right = right_encoder.read() * TICKS_TO_METERS;
  delay(2000);
  last_update_time = millis();
  last_push_time = millis();
  last_error_time = millis();
}

void loop() {
  long current_time = millis();
  updateEncoders();

  if (5 < (current_time - last_update_time)) { // Run once every ~3 ms
    updateVelocity();
    updateAckermann();
    last_update_time = current_time;
  }

  // writeAckermann(.2, .2);

  if (20 < (current_time - last_push_time)) { // Run once every ~10 ms
    // updateVelocity();
    // updateAckermann();
    // last_update_time = current_time;
    // updateVelocity();
    // updateAckermann();
    // last_update_time = current_time;

    // if (autobrake < reported_val) {
    //   reported_val = autobrake;
    // }

    if (Serial.available() > 0) {
      // Read the incoming data as a string
      String received_data = Serial.readStringUntil('\n');
      
      parseMessage(received_data);
    }

    if (current_velocity > max_speed) {
      max_speed = current_velocity;
    }

    last_push_time = current_time;

    Serial.print(current_time);
    Serial.print(" ");
    Serial.print(current_velocity, 4);
    Serial.print(" ");
    Serial.println(target_angle, 4);
  }
}

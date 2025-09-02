// #include <math.h>

// // Struct to hold robot parameters
// struct RobotParams {
//     float encoder_ticks_per_revolution = 2000.0;
//     float wheel_radius = 0.05;  // Wheel radius in meters
//     float wheel_base = 0.4;     // Distance between wheels in meters
//     float ticks_to_distance;
// } params;

// // Struct to represent a point
// struct Point {
//     float x;
//     float y;
// };

// // Position and orientation of the robot
// struct Pose {
//     float x = 0.0;
//     float y = 0.0;
//     float theta = 0.0;
// } pose;

// // PID parameters
// struct PID {
//     float kp = 1.0, ki = 0.5, kd = 0.1;
//     float integral = 0, previous_error = 0;
// } pid_left, pid_right;

// // Motor control pins
// const int pwmLeft = 9, pwmRight = 10, dirLeft = 2, dirRight = 3;

// // Initialize constants and parameters
// void initializeParams() {
//     params.ticks_to_distance = 2.0 * M_PI * params.wheel_radius / params.encoder_ticks_per_revolution;
// }

// // Step 1: Update position based on encoder ticks
// void updatePosition(long left_ticks, long right_ticks) {
//     float d_left = left_ticks * params.ticks_to_distance;
//     float d_right = right_ticks * params.ticks_to_distance;

//     float d = (d_left + d_right) / 2.0;
//     float d_theta = (d_right - d_left) / params.wheel_base;

//     pose.x += d * cos(pose.theta);
//     pose.y += d * sin(pose.theta);
//     pose.theta += d_theta;
// }

// // Step 2: Calculate global waypoint from local distance and angle
// Point calculateWaypoint(float distance, float angle) {
//     Point waypoint;
//     waypoint.x = pose.x + distance * cos(pose.theta + angle);
//     waypoint.y = pose.y + distance * sin(pose.theta + angle);
//     return waypoint;
// }

// // Step 3: Calculate wheel velocities to reach target waypoint
// void calculateWheelVelocities(float target_x, float target_y, float& v_left, float& v_right) {
//     float dx = target_x - pose.x;
//     float dy = target_y - pose.y;
//     float distance = sqrt(dx * dx + dy * dy);
//     float target_theta = atan2(dy, dx);
//     float error_theta = target_theta - pose.theta;

//     float v = distance * 0.5;          // Proportional control for distance
//     float omega = error_theta * 0.5;   // Proportional control for angle

//     v_left = v - (omega * params.wheel_base / 2);
//     v_right = v + (omega * params.wheel_base / 2);
// }

// // Helper function for PID control on wheel speeds
// float computePID(float target_speed, float current_speed, PID &pid, float loop_interval) {
//     float error = target_speed - current_speed;
//     pid.integral += error * loop_interval;
//     float derivative = (error - pid.previous_error) / loop_interval;
//     pid.previous_error = error;
//     return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
// }

// // Step 4: Control wheels based on target speeds using PID and send to H-bridge
// void controlWheels(float targetSpeedLeft, float targetSpeedRight, int loopInterval) {
//     long leftTicks = leftEncoder.read();
//     long rightTicks = rightEncoder.read();

//     float currentSpeedLeft = (leftTicks - previousLeftTicks) / loopInterval;
//     float currentSpeedRight = (rightTicks - previousRightTicks) / loopInterval;

//     float controlSignalLeft = computePID(targetSpeedLeft, currentSpeedLeft, pid_left, loopInterval);
//     float controlSignalRight = computePID(targetSpeedRight, currentSpeedRight, pid_right, loopInterval);

//     // Set motor direction and PWM values
//     digitalWrite(dirLeft, controlSignalLeft >= 0 ? HIGH : LOW);
//     analogWrite(pwmLeft, constrain(abs(controlSignalLeft), 0, 255));
//     digitalWrite(dirRight, controlSignalRight >= 0 ? HIGH : LOW);
//     analogWrite(pwmRight, constrain(abs(controlSignalRight), 0, 255));

//     previousLeftTicks = leftTicks;
//     previousRightTicks = rightTicks;
//     delay(loopInterval);  // Control interval delay
// }


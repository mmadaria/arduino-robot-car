/*
The MIT License (MIT)

Copyright (c) 2016 Milko Madariaga

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Servo.h>         // Include Servo Motor library
#include <NewPing.h>       // Include HC-SR04 Ultrasonic Distance sensor

// Define Gear Motors PINs
#define LEFT_MOTOR_PIN1                   4       // Left motor's pin 1         
#define LEFT_MOTOR_PIN2                   5       // Left motor's pin 2
#define RIGHT_MOTOR_PIN1                  6       // Right motor's pin 1
#define RIGHT_MOTOR_PIN2                  7       // Right motor's pin 2

// Define Servo Motor PIN
#define SERVO_MOTOR_PIN                   8       // Digital PIN for controlling servo motor

// Define Ultrasonic Distance Sensor PINs
#define DISTANCE_SENSOR_TRIGGER_PIN       2       // Digital PIN tied to trigger pin on ultrasonic distance sensor
#define DISTANCE_SENSOR_ECHO_PIN          3       // Digital PIN tied to echo pin on ultrasonic distance sensor

// Define settings for Ultrasonic Distance Sensor
#define DISTANCE_SENSOR_MAX_DISTANCE_CMS  50      // Maximum distance we want to ping for (in centimeters). Maximum HC-SR04 sensor distance is rated at 400-500cm.
#define MIN_OBSTACLE_DISTANCE_THRESOLD    28      // Minimum distance allowed to move forward (in centimeters)
#define DISTANCE_SENSOR_ITERATIONS_QTY    5       // Number of iterations to calculate the median distance

// Define settings for Servo Motor
#define SERVO_MOTOR_RIGHT_POS_VALUE       0       // Real angle for servo motor "right position"
#define SERVO_MOTOR_CENTER_POS_VALUE      75      // Real angle for servo motor "center position"
#define SERVO_MOTOR_LEFT_POS_VALUE        160     // Real angle for servo motor "left position"
#define SERVO_MOTOR_LOOK_LEFT_DELAY_MS    700     // Delay needed for servo motor to look left (ms)
#define SERVO_MOTOR_LOOK_RIGHT_DELAY_MS   700     // Delay needed for servo motor to look right (ms)

// Define settings for Gear Motors
#define MOTORS_LEFT_ROTATION_DELAY_MS     300     // Delay needed for motors to rotate left (ms)
#define MOTORS_RIGHT_ROTATION_DELAY_MS    300     // Delay needed for motors to rotate right (ms)
#define MOTORS_BACKWARD_DELAY_MS          500     // Delay needed for motors to move backward (ms)
#define MOTORS_STOP_DELAY_MS              500     // Delay needed for motors to stop (ms)

// Define settings for debugging
#define IS_DEBUG_ENABLED false                    // Enable or disable debug mode (it prints messages using serial output)

// Define global variables
int straightObstacleDistance;
int leftObstacleDistance;
int rightObstacleDistance;

// Prepare Servo Motor
Servo servoMotor;

// Prepare ultrasonic distance sensor
NewPing sonar(DISTANCE_SENSOR_TRIGGER_PIN, DISTANCE_SENSOR_ECHO_PIN, DISTANCE_SENSOR_MAX_DISTANCE_CMS);

void setup (){
  #if IS_DEBUG_ENABLED 
    // Initialize serial communication
    Serial.begin(9600);
  #endif
  
  // Initialize INPUT/OUPUT mode for PINs
  pinMode(LEFT_MOTOR_PIN1,OUTPUT);
  pinMode(LEFT_MOTOR_PIN2,OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1,OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2,OUTPUT);

  // Initialize random seed using unused analog pin 0
  randomSeed(analogRead(0));

  // Initialize Servo Motor (look straight)
  servoMotor.attach(SERVO_MOTOR_PIN);
  servoMotor.write(SERVO_MOTOR_CENTER_POS_VALUE);
  delay(700);  
}
void loop(){
  // Get straight obstacle's distance
  straightObstacleDistance = getObstacleDistance();

  // Check if must continue forward or find a route (rotate)
  if (straightObstacleDistance == 0 || straightObstacleDistance >= MIN_OBSTACLE_DISTANCE_THRESOLD){
    moveForward();                          
  } else {                              
    chooseRotation();
  }
  
  delay(50);
}

int getObstacleDistance(){
  // Calculate median of distance values (using N iterations) and convert it to centimeters
  int result = sonar.ping_median(DISTANCE_SENSOR_ITERATIONS_QTY);
  result = sonar.convert_cm(result);                             

  #if IS_DEBUG_ENABLED 
    Serial.print("Get obstacle's distance: ");
    Serial.println(result);
  #endif
  
  return result;
}
 
void moveForward() {                       
  #if IS_DEBUG_ENABLED 
    Serial.println("Move forward.");
  #endif

  // Move both motors forward
  digitalWrite(LEFT_MOTOR_PIN1,HIGH);
  digitalWrite(LEFT_MOTOR_PIN2,LOW);
  digitalWrite(RIGHT_MOTOR_PIN1,HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2,LOW);
 }
 
void chooseRotation() {
  #if IS_DEBUG_ENABLED 
    Serial.println("Choose rotation.");
  #endif

  // Execute the robot's behaviour for finding the best path
  stop();                         
  moveBackward();                 
  lookLeft();                     
  lookRight();                    

  // Evaluate maximum distance found and execute rotation                                 
  if (leftObstacleDistance > rightObstacleDistance){
    rotateLeft();
  } else if (leftObstacleDistance < rightObstacleDistance){
    rotateRight();
  } else {
    // Choose a random rotation
    rotateRandom();
  }
}

void moveBackward() {
  #if IS_DEBUG_ENABLED 
    Serial.println("Move backward.");
  #endif

  // Move both motors backward
  digitalWrite(LEFT_MOTOR_PIN1,LOW);
  digitalWrite(LEFT_MOTOR_PIN2,HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1,LOW);
  digitalWrite(RIGHT_MOTOR_PIN2,HIGH);
  delay(MOTORS_BACKWARD_DELAY_MS);
  stop();
}

void stop(){
  #if IS_DEBUG_ENABLED 
    Serial.println("Stop.");
  #endif

  // Stop both motors
  digitalWrite(LEFT_MOTOR_PIN1,LOW);
  digitalWrite(LEFT_MOTOR_PIN2,LOW);
  digitalWrite(RIGHT_MOTOR_PIN1,LOW);
  digitalWrite(RIGHT_MOTOR_PIN2,LOW);
  delay(MOTORS_STOP_DELAY_MS);
}
 
void lookLeft() {
  #if IS_DEBUG_ENABLED 
    Serial.println("Look left.");
  #endif

  // Rotate servo motor to the left and measure distance 
  servoMotor.write(SERVO_MOTOR_LEFT_POS_VALUE);
  delay(SERVO_MOTOR_LOOK_LEFT_DELAY_MS);
  leftObstacleDistance = getObstacleDistance();
  servoMotor.write(SERVO_MOTOR_CENTER_POS_VALUE);
  delay(SERVO_MOTOR_LOOK_LEFT_DELAY_MS);
}

void lookRight () {
  #if IS_DEBUG_ENABLED 
    Serial.println("Look right.");
  #endif

  // Rotate servo motor to the right and measure distance
  servoMotor.write(SERVO_MOTOR_RIGHT_POS_VALUE);
  delay(SERVO_MOTOR_LOOK_RIGHT_DELAY_MS);  
  rightObstacleDistance = getObstacleDistance();
  servoMotor.write(SERVO_MOTOR_CENTER_POS_VALUE);                                  
  delay(SERVO_MOTOR_LOOK_RIGHT_DELAY_MS);  
}

void rotateLeft () {
  #if IS_DEBUG_ENABLED 
    Serial.println("Rotate left.");
  #endif

  // Use Differential Drive technique: move right motor forward and left motor backward
  digitalWrite(LEFT_MOTOR_PIN1,HIGH);       
  digitalWrite(LEFT_MOTOR_PIN2,LOW);     
  digitalWrite(RIGHT_MOTOR_PIN1,LOW);
  digitalWrite(RIGHT_MOTOR_PIN2,HIGH);
  delay(MOTORS_LEFT_ROTATION_DELAY_MS);                 
  stop();
}

void rotateRight () {
  #if IS_DEBUG_ENABLED 
    Serial.println("Rotate right.");
  #endif

    // Use Differential Drive technique: move left motor forward and right motor backward
  digitalWrite(LEFT_MOTOR_PIN1,LOW); 
  digitalWrite(LEFT_MOTOR_PIN2,HIGH);
  digitalWrite(RIGHT_MOTOR_PIN1,HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2,LOW);
  delay(MOTORS_RIGHT_ROTATION_DELAY_MS);
  stop();
}

void rotateRandom(){
  #if IS_DEBUG_ENABLED 
    Serial.print("Rotate random: ");
  #endif

  // Pick a random number between 0 and 1
  long randomNumber = random(2);
  if (randomNumber == 0){
    rotateLeft();
  } else {
    rotateRight();
  }
}

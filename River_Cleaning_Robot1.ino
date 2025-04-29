#include <Servo.h>
#include <NewPing.h>

#define LEFT_MOTOR_FORWARD 5
#define LEFT_MOTOR_BACKWARD 6
#define RIGHT_MOTOR_FORWARD 10
#define RIGHT_MOTOR_BACKWARD 11

// Ultrasonic Sensor Pins
#define TRIG_PIN 7
#define ECHO_PIN 8
#define MAX_DISTANCE 200 // (in cm)


#define LED_PIN 13
#define BUZZER_PIN 2


#define SHOVEL_SERVO_PIN 9

// Motor Speed
const int MOTOR_SPEED = 200; 

Servo shovelServo; 
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 

void setup() {
  // Motor setup
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Indicator setup
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Shovel setup
  shovelServo.attach(SHOVEL_SERVO_PIN);
  shovelServo.write(0); // Initial shovel position

  // Serial Monitor
  Serial.begin(9600);
}

void loop() {
  moveForward();
  
  int distance = getDistance();
  Serial.print("Measured Distance: ");
  Serial.println(distance);

  if (distance > 0 && distance < 50) { 
    handleObstacle();
  }

  operateStairBrushes();
  operateShovel();
}


void moveForward() {
  analogWrite(LEFT_MOTOR_FORWARD, MOTOR_SPEED);
  analogWrite(RIGHT_MOTOR_FORWARD, MOTOR_SPEED);
  
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void moveBackward() {
  analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_SPEED);
  analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_SPEED);
  
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

// Obstacle Handling
void handleObstacle() {
  stopMotors();
  alertObstacle();
  delay(1500);

  moveBackward();
  delay(1000);

  stopMotors();
}

// Buzzer Alert
void alertObstacle() {
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(400);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}


// ultraSonic logic
int getDistance() {
  return sonar.ping_cm(); // Get distance in cm
}

// Shovel Operation
void operateShovel() {
  shovelServo.write(90); 
  delay(1500);
  shovelServo.write(0);  
  delay(1500);
}

// Stair Brush Operation
void operateStairBrushes() {
  analogWrite(LEFT_MOTOR_BACKWARD, 180); 
  analogWrite(RIGHT_MOTOR_BACKWARD, 180);

  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);

  delay(500); 
}

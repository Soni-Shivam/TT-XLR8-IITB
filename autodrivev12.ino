#include <Servo.h>
#include <pthread.h>
#include <unistd.h>  // For sleep function

#define N 1000000

// Constants
const int TRIG_PIN = 27; // Trigger pin for ultrasonic sensor
const int ECHO_PIN = 26; // Echo pin for ultrasonic sensor
const int SERVO_PIN = 14; // Servo pin for the distance scanner

// Motor control pins for the LM298 H-bridge
const int MOTOR_LEFT_IN1 = 20; // Left motors control IN1
const int MOTOR_LEFT_IN2 = 21; // Left motors control IN2
const int MOTOR_RIGHT_IN1 = 18; // Right motors control IN1
const int MOTOR_RIGHT_IN2 = 19; // Right motors control IN2
const int ENA = 22; // Enable pin for left motors
const int ENB = 17; // Enable pin for right motors


// Constants for PWM
const int PWM_VALUE = 160; // PWM value (0-255)

// Distance parameters
const int SAFE_DISTANCE = 25; // Minimum safe distance from obstacles
const int REVERSE_TIME = 300; // Time to reverse in milliseconds
const int FORWARD_TIME = 500; // Time to move forward in milliseconds
const int TURN_90_DEG = 750*255/PWM_VALUE; // Time to turn 90 degrees in milliseconds
int distSeen = 0;
int maxDist = 0;
int reqAngle = 0;
// Servo Motor Setup
Servo servo;
int servoPos = 90; // Starting position

void setup() {
  Serial.begin(11520);

  // Initialize the servo motor
  servo.attach(SERVO_PIN);
  servo.write(servoPos);

  // Initialize motor pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize onboard LED pin
  pinMode(LED_BUILTIN, OUTPUT);

  // Set PWM for enable pins
  analogWrite(ENA, PWM_VALUE); // Set PWM for left motors
  analogWrite(ENB, PWM_VALUE); // Set PWM for right motors

  // Ensure motors are initially disabled
  stop();
  pthread_t high_priority_thread, low_priority_thread;

    // Create high-priority thread
    pthread_create(&high_priority_thread, NULL, high_priority_loop, NULL);

    // Optionally sleep or do some work here
    // sleep(1);

    // Create low-priority thread
    pthread_create(&low_priority_thread, NULL, low_priority_loop, NULL);

}

void loop() {

   
    // Wait for threads to finish
    pthread_join(high_priority_thread, NULL);
    pthread_join(low_priority_thread, NULL);

    printf("Both loops finished\n");
    


  // Continuous scanning while moving forward
  /* continuousScan();
  moveForward(); */
}

void* high_priority_loop(void* arg)  {
  // Turn on the onboard LED to indicate scanning
  digitalWrite(LED_BUILTIN, HIGH);

  // Continuously check for obstacles while moving forward
  while (true) {
    servo.write(90);
    int distance = ping();
    delay(200);
    Serial.print("Front Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance < SAFE_DISTANCE) {
      stop();
      avoidObstacle();
      // Turn off the LED when obstacle avoidance starts
      digitalWrite(LED_BUILTIN, LOW);
      return; // Return to avoid continuous scanning while handling the obstacle
    }

    delay(100); // Short delay to avoid excessive processing
  }
  return NULL;

}

int ping() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return microsecondsToCentimeters(duration);
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void* low_priority_loop(void* arg)  {
  // Forward movement for left and right motors
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  Serial.println("Moving forward");
  

/*   // Move forward for the specified time
  delay(FORWARD_TIME);

  // Stop after moving forward
  stop(); */
  return NULL;

}

void stop() {
  // Ensure motors are stopped
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  Serial.println("Stopped");
}

void reverse() {
/*   stop();
   // Short stop before reversing */

  // Reverse movement for left and right motors
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  Serial.println("Reversing");

  // Reverse for the specified time
  delay(REVERSE_TIME);
  
  stop();
  delay(200); // Short stop after reversing
}
/* 
void turn90Degrees() {
  stop();
  delay(300); // Short stop before turning

  // Turn 90 degrees
  Serial.println("Turning 90 degrees");
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);

  // Turn for the specified time to achieve approximately 90 degrees
  delay(TURN_90_DEG);

  stop();
  delay(300); // Short stop after turning
} */

void avoidObstacle() {
  stop();
  reverse();
  delay(200); // Stop for a short period
  int distances[5];
  int reqI;
  maxDist = 0;
  Serial.print("Measured readings array ; \n");   // Array to store distance values
  for (int i =0; i <5; ++i) { // look around and check
    servo.write(i*45);
    delay(400);
    distSeen = ping();
    distances[i] = distSeen;
    Serial.println(distSeen);
    if (distSeen >= maxDist) { // store the max dist and req angle to variables
      maxDist = distSeen;
      reqAngle = ((i*45)-90);
      reqI = i;
    } 
  }
  Serial.print("index = ");
  Serial.println(reqI);
  Serial.print("The car needs to rotate by angle = ");
  Serial.println(reqAngle);
  turnCar(reqAngle);

/*   moveForward(); */
  
  // Re-enable scanning
  digitalWrite(LED_BUILTIN, HIGH);
  /* delay(500); // Allow time for scanning */
}

void turnCar(int angle) {
  stop();
  if (angle > 0) { 
    Serial.print("Rotating CW by angle ");
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  }
  else if (angle < 0) { //anti-clockwise
    Serial.print("Rotating ACW by angle ");
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  }
  else if (angle == 0) { //anti-clockwise
    reverse();
  }
  Serial.println(angle);
  Serial.print("Calculated Time for turn = ");
  Serial.println(TURN_90_DEG/* *angle/90 */);
  // Turn for the specified time with respect to that to achieve approximately 90 degrees
  delay(TURN_90_DEG);
  stop();
/*   delay(200); // Short stop after turning */
}

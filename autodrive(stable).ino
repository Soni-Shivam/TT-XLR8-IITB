#include <Servo.h>

// Constants
const int TRIG_PIN = 27;
const int ECHO_PIN = 26;
const int SERVO_PIN = 14;

const int MOTOR_LEFT_IN1 = 20;
const int MOTOR_LEFT_IN2 = 21;
const int MOTOR_RIGHT_IN1 = 18;
const int MOTOR_RIGHT_IN2 = 19;
const int ENA = 22;
const int ENB = 17;

const int PWM_VALUE = 160;
const int SAFE_DISTANCE = 25;
const int REVERSE_TIME = 300;
const int TURN_90_DEG = 750 * 255 / PWM_VALUE;

int distSeen = 0;
int maxDist = 0;
int reqAngle = 0;
bool obstacleDetected = false;

unsigned long previousScanMillis = 0;
const long scanInterval = 100; // Time between distance scans

Servo servo;
int servoPos = 90;

void setup() {
  Serial.begin(115200);

  servo.attach(SERVO_PIN);
  servo.write(servoPos);

  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  analogWrite(ENA, PWM_VALUE);
  analogWrite(ENB, PWM_VALUE);

  stop();
}

void loop() {
  unsigned long currentMillis = millis();

  // Run continuous scan at regular intervals
  if (currentMillis - previousScanMillis >= scanInterval) {
    previousScanMillis = currentMillis;
    distSeen = ping();
    Serial.print("Front Distance: ");
    Serial.print(distSeen);
    Serial.println(" cm");

    if (distSeen < SAFE_DISTANCE) {
      obstacleDetected = true;
      stop();
      avoidObstacle();
    }
  }

  // Move forward if no obstacle is detected
  if (!obstacleDetected) {
    moveForward();
  } else {
    // Reset the obstacle flag after avoiding it
    obstacleDetected = false;
  }
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

void moveForward() {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  /* Serial.println("Moving forward"); */
}

void stop() {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  Serial.println("Stopped");
}

void reverse() {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  Serial.println("Reversing");

  delay(REVERSE_TIME);
  stop();
  delay(200); 
}

void avoidObstacle() {
  reverse();
  delay(300);

  int distances[5];
  int reqI;
  maxDist = 0;
  Serial.print("Measured readings array ; \n");   
  for (int i = 0; i < 5; ++i) {
    servo.write(i * 45);
    delay(400);
    int distance = ping();
    distances[i] = distance;
    Serial.println(distance);
    if (distance >= maxDist) {
      maxDist = distance;
      reqAngle = ((i * 45) - 90);
      reqI = i;
    } 
  }
  Serial.print("index = ");
  Serial.println(reqI);
  Serial.print("The car needs to rotate by angle = ");
  Serial.println(reqAngle);
  turnCar(reqAngle);

  digitalWrite(LED_BUILTIN, HIGH);
}

void turnCar(int angle) {
  stop();
  if (angle > 0) {
    Serial.print("Rotating CW by angle ");
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  } else if (angle < 0) {
    Serial.print("Rotating ACW by angle ");
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  } else {
    reverse();
  }
  Serial.println(angle);
  Serial.print("Calculated Time for turn = ");
  Serial.println(TURN_90_DEG * abs(angle) / 90);
  delay(TURN_90_DEG * abs(angle) / 90);
  Serial.print("Rotation complete");
  stop();
}

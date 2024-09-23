// this code was written after the run
// I was trying to improve the decision making algorithm and also integrate an onboard imu so we being inexperienced took help from claude 3.5 sonnet

#include <Adafruit_MPU6050.h>
#include <WiFi.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
WiFiServer server(80);

const int TRIG_PIN = 27;
const int ECHO_PIN = 26;
const int SERVO_PIN = 14;

const int MOTOR_LEFT_IN1 = 21;
const int MOTOR_LEFT_IN2 = 20;
const int MOTOR_RIGHT_IN1 = 19;
const int MOTOR_RIGHT_IN2 = 18;
const int ENA = 17;
const int ENB = 22;

const int PWM_VALUE = 200;
const int SAFE_DISTANCE = 15;
const int REVERSE_TIME = 300;
const int TURN_90_DEG = 750 * 255 / PWM_VALUE;
const int MAX_VALID_DISTANCE = 800;  // Maximum valid distance in cm

int distSeen = 0;
int maxDist = 0;
int reqAngle = 0;
bool obstacleDetected = false;

unsigned long previousScanMillis = 0;
const long scanInterval = 100;
Servo servo;
int servoPos = 90;

float initial_z_angle = 0.0;
bool initial_set = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  delay(1000);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  // Set up MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize Wi-Fi AP
  WiFi.softAP("TT's Pico", "Bigblackcoffee");
  Serial.println("Wi-Fi AP started");

  IPAddress IP = WiFi.softAPIP();
  Serial.print("IP Address: ");
  Serial.println(IP);

  server.begin();

  // Initialize Servo and Motors
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

  // Calibrate IMU
  calibrateIMU();
}

void loop() {
  delay(100);
  servo.write(servoPos);
  unsigned long currentMillis = millis();
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

  if (!obstacleDetected) {
    moveForward();
    Serial.println("Running forward");
  } else {
    obstacleDetected = false;
  }

  // Handle HTTP requests
  WiFiClient client = server.available();
  if (client) {
    handleHTTPRequest(client);
  }

  // Update IMU data
  updateIMUData();
}

void calibrateIMU() {
  Serial.println("Calibrating IMU...");
  float sum_z = 0;
  int num_samples = 100;

  for (int i = 0; i < num_samples; i++) {
    sensors_event_t accel;
    mpu.getAccelerometerSensor()->getEvent(&accel);
    sum_z += atan2(accel.acceleration.y, accel.acceleration.z) * (180.0 / PI);
    delay(10);
  }

  initial_z_angle = sum_z / num_samples;
  initial_set = true;
  Serial.println("IMU calibration complete.");
}

void updateIMUData() {
  sensors_event_t accel;
  sensors_event_t gyro;
  mpu.getAccelerometerSensor()->getEvent(&accel);
  mpu.getGyroSensor()->getEvent(&gyro);

  // Use complementary filter to combine accelerometer and gyro data
  float accel_angle = atan2(accel.acceleration.y, accel.acceleration.z) * (180.0 / PI);
  static float angle = 0;
  angle = 0.98 * (angle + gyro.gyro.z * 0.01) + 0.02 * accel_angle;

  float angular_deviation = angle - initial_z_angle;

  // Use angular_deviation for navigation adjustments if needed
  adjustMotorsForDeviation(angular_deviation);
}

void adjustMotorsForDeviation(float deviation) {
  // Implement motor speed adjustments based on the angular deviation
  int leftSpeed = PWM_VALUE;
  int rightSpeed = PWM_VALUE;

  if (deviation > 5) {
    // Turn right
    leftSpeed += 20;
    rightSpeed -= 20;
  } else if (deviation < -5) {
    // Turn left
    leftSpeed -= 20;
    rightSpeed += 20;
  }

  // Ensure speeds are within valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Apply new speeds
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

int ping() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = microsecondsToCentimeters(duration);

  // Filter out readings greater than MAX_VALID_DISTANCE
  if (distance > MAX_VALID_DISTANCE) {
    return 0;  // Return the maximum valid distance instead
  }
  return distance;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void moveForward() {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  delay(200);
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
  Serial.println("Measured readings array:");
  for (int i = 0; i < 5; ++i) {
    servo.write(i * 45);
    delay(400);
    int distance = ping();
    distances[i] = distance;
    Serial.println(distance);
    if (distance >= maxDist && distance < MAX_VALID_DISTANCE) {
      maxDist = distance;
      reqAngle = ((i * 45) - 90);
      reqI = i;
    }
  }
  Serial.print("index = ");
  Serial.println(reqI);
  Serial.print("The car needs to rotate by angle = ");
  Serial.println(reqAngle);
  distSeen = ping();
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
    delay(1000);
  }
  Serial.println(angle);
  Serial.print("Calculated Time for turn = ");
  Serial.println(TURN_90_DEG * abs(angle) / 90);
  delay(TURN_90_DEG * abs(angle) / 90);
  Serial.print("Rotation complete");
  stop();
}

void handleHTTPRequest(WiFiClient &client) {
  String request = "";
  while (client.connected() || client.available()) {
    if (client.available()) {
      char c = client.read();
      request += c;
      if (c == '\n') {
        if (request.endsWith("\r\n\r\n")) {
          break;
        }
      }
    }
  }

  // Get MPU6050 data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu.getTemperatureSensor()->getEvent(&temp);
  mpu.getAccelerometerSensor()->getEvent(&accel);
  mpu.getGyroSensor()->getEvent(&gyro);

  // Calculate current Z-axis angle
  float z_angle = atan2(accel.acceleration.y, accel.acceleration.z) * (180.0 / PI);
  float angular_deviation = z_angle - initial_z_angle;

  // Build the HTML content
  String html =
    "<html><head><title>MPU6050 & Car Data</title><meta http-equiv=\"refresh\" content=\"1\"></head><body>"
    "<h1>MPU6050 & Car Sensor Data</h1>"
    "<p>Front Distance: "
    + String(distSeen) + " cm</p>"
                         "<p>Temperature: "
    + String(temp.temperature) + " °C</p>"
                                 "<p>Accel X: "
    + String(accel.acceleration.x) + " m/s²</p>"
                                     "<p>Accel Y: "
    + String(accel.acceleration.y) + " m/s²</p>"
                                     "<p>Accel Z: "
    + String(accel.acceleration.z) + " m/s²</p>"
                                     "<p>Gyro X: "
    + String(gyro.gyro.x) + " rad/s</p>"
                            "<p>Gyro Y: "
    + String(gyro.gyro.y) + " rad/s</p>"
                            "<p>Gyro Z: "
    + String(gyro.gyro.z) + " rad/s</p>"
                            "<p>Angular Deviation from Initial Z-axis: "
    + String(angular_deviation) + " degrees</p>"
                                  "<p>Status: "
    + (obstacleDetected ? "Obstacle Detected" : "No Obstacle") + "</p>"
                                                                 "<p>Requested Turn Angle: "
    + String(reqAngle) + " degrees</p>"
                         "<p>Max Distance Detected: "
    + String(maxDist) + " cm</p>"
                        "</body></html>";

  // Send HTTP response
  client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  client.print(html);
  client.stop();
  Serial.println("Client disconnected");
}

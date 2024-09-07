#include <Servo.h> 

// Pin assignments for motor control
const int servoPin = 14;
const int ENA = 22;
const int ENB = 17;
const int IN1 = 21;
const int IN2 = 20;
const int IN3 = 19;
const int IN4 = 18;
const int pingPin = 27; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 26; // Echo Pin of Ultrasonic Sensor
const int calibratedTime = 1500;
const int calibratedSpeed = 109; // cm / sec
long duration;
int cm = 0;
int cmd = 0;
int spd = 0;
bool bot_cannot_continue = false;
int stoppingDistance = 10;

// Create a servo object
Servo uServo;

void setup() {
  // Start Serial for debugging
  Serial.begin(115200);
  // Configure motor control pins as outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);

  uServo.attach(servoPin, 400, 2600); // Minimum and maximum pulse width (in µs) to go from 0° to 180°.
  Serial.begin(115200); // Starting Serial Terminal
/*   uServo.write(90); // look in front
 */}

void applyMotorControl(int cmd, int spd) {
  // Display motor control information
  Serial.print("cmd: ");
  Serial.print(cmd);   // Display motor command
  Serial.print(", speed: ");
  Serial.println(spd); // Display motor speed
  switch (cmd) {
    case 2:  // Forward
      digitalWrite(IN1, HIGH); 
      digitalWrite(IN2, LOW); 
      digitalWrite(IN3, HIGH); 
      digitalWrite(IN4, LOW); 
      break;
    case 1:  // Backward
      digitalWrite(IN1, LOW); 
      digitalWrite(IN2, HIGH); 
      digitalWrite(IN3, LOW); 
      digitalWrite(IN4, HIGH); 
      break;
    case 3:  // Right
      digitalWrite(IN1, HIGH); 
      digitalWrite(IN2, LOW); 
      digitalWrite(IN3, LOW); 
      digitalWrite(IN4, HIGH); 
      break;
    case 4:  // Left
      digitalWrite(IN1, LOW); 
      digitalWrite(IN2, HIGH); 
      digitalWrite(IN3, HIGH); 
      digitalWrite(IN4, LOW); 
      break;
    default:  // Stop
      digitalWrite(IN1, LOW); 
      digitalWrite(IN2, LOW); 
      digitalWrite(IN3, LOW); 
      digitalWrite(IN4, LOW); 
      spd = 0;
      break;
    
  }
  // Apply the calculated motor speed to both motors
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
}


int ping() {
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);
  return microsecondsToCentimeters(duration);
}


void turnCar(int angle) {
  if (angle > 90) { 
    cmd = 3; //  clockwise
  }
  else if (angle < 90) { //anti-clockwise
    cmd = 4;
  }
  else if (angle == 90) { //anti-clockwise
    cmd = 1;
  }
  applyMotorControl(cmd, 255);
  delay(calibratedTime*angle/360);
  applyMotorControl(0, 0);
}


long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

void loop() {
  uServo.write(90); // look in front
  ping(); // send ultrasonic
  if (cm > stoppingDistance && cm < 30) {
    bot_cannot_continue = false;
    // half speed
    digitalWrite(LED_BUILTIN, LOW);
    applyMotorControl(1, 128);
    delay((cm-9)/calibratedSpeed*1000);
  }
  else if (cm > 30) {
    bot_cannot_continue = false;
    // full speed
    digitalWrite(LED_BUILTIN, LOW);
    applyMotorControl(1, 255);
    delay((cm-10)/calibratedSpeed*1000);
  }
  else { // stop and decide
    bot_cannot_continue = true;
    applyMotorControl(0, 0);
    digitalWrite(LED_BUILTIN, HIGH);
    void nested_loop() {
    //NEW CODE BEGINS HERE FOR ROTATING SERVO IN 45 INCREMENTS AND GETTING MINIMUM DISTANCE ANGLE
    const int numAngles = 5;    // Number of angle increments (0, 45, 90, 135, 180)
    int angles[numAngles] = {0, 45, 90, 135, 180}; // Angle increments
    int distances[numAngles];   // Array to store distance values
    int minIndex = 0;        // Index of the angle with the minimum distance
    int minDistance = MAX_DISTANCE; // Initialize minimum distance to a high value

  // Rotate the servo and measure distances
  for (int i = 0; i < numAngles; i++) {
    uServo.write(angles[i]); // Set servo to the current angle
    delay(1000);              // Wait for the servo to reach the position

    // Send a ping and get the distance
    delay(50);                // Wait for the sensor to settle
    distances[i] = ping(); // Get distance in cm

    // Print angle and distance
    Serial.print("Angle: ");
    Serial.print(angles[i]);
    Serial.print("° Distance: ");
    Serial.print(distances[i]);
    Serial.println(" cm");

    // Check if this is the minimum distance so far
    if (distances[i] < minDistance) {
      minDistance = distances[i];
      minIndex = i;
    }
  }

  // Print the angle with the minimum distance
  Serial.print("Angle with minimum distance: ");
  Serial.print(angles[minIndex]);
  Serial.print("° Distance: ");
  Serial.println(minDistance);
  // Wait before the next measurement cycle
  delay(5000);}

  turnCar(angles[minIndex]);
  // full speed
    digitalWrite(LED_BUILTIN, LOW);
    applyMotorControl(1, 255);
    delay((cm-10)/calibratedSpeed*1000);



  }
  

  /*if (bot_cannot_continue = true) { 
    int measurements[5];
    int d;
    
    for (int Angle = 0; Angle <= 180; Angle += 45) {
      uServo.write(Angle);
      measurements[Angle/45] = ping();
    }

    int size = sizeof(measurements) / sizeof(measurements[0]);
    int reqIndex = findMaxIndex(measurements, size);
    d = measurements[reqIndex];
    
    if (d > stoppingDistance && d < 30) {
        bool bot_cannot_continue = false;
        digitalWrite(LED_BUILTIN, LOW);
        turnCar(reqIndex*45);     
      }   
      else if (d > 30) {
        bool bot_cannot_continue = false;
        applyMotorControl(1,255);
        digitalWrite(LED_BUILTIN, LOW);
        delay((cm-10)/calibratedSpeed*1000);

      }
      else {
        bool bot_cannot_continue = true;
        applyMotorControl(0,0);
        digitalWrite(LED_BUILTIN, HIGH);
      }
    
  }
}

// function to find the index of the max distance
int findMaxIndex(int arr[], int size) {
    if (size <= 0) {
        return -1;
    }
    int maxIndex = 0; 
    for (int i = 1; i < size; i++) {
        if (arr[i] > arr[maxIndex]) {
            maxIndex = i;
        }
    }
    return maxIndex;
}*/

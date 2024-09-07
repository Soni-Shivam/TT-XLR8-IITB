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
    case 2:  // backward
      digitalWrite(IN1, HIGH); 
      digitalWrite(IN2, LOW); 
      digitalWrite(IN3, HIGH); 
      digitalWrite(IN4, LOW); 
      break;  //could the jerking stop if we reomve the breka command??????
    case 1:  // Forward
      digitalWrite(IN1, LOW); 
      digitalWrite(IN2, HIGH); 
      digitalWrite(IN3, LOW); 
      digitalWrite(IN4, HIGH); 
      break;
    case 4:  // left
      digitalWrite(IN1, HIGH); 
      digitalWrite(IN2, LOW); 
      digitalWrite(IN3, LOW); 
      digitalWrite(IN4, HIGH); 
      break;
    case 3:  // right
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
  }
  else if (cm > 30) {
    bot_cannot_continue = false;
    // full speed
    digitalWrite(LED_BUILTIN, LOW);
    applyMotorControl(1, 255);
  }
  else { // stop and decide
    bot_cannot_continue = true;
    applyMotorControl(0, 0);
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(10);
    for (int Angle = 0; Angle <= 180; Angle += 10) {
      uServo.write(Angle);
      ping();
      if (cm > stoppingDistance && cm < 30) {
          bool bot_cannot_continue = false;
          digitalWrite(LED_BUILTIN, LOW);
          turnCar(Angle);
          applyMotorControl(1,128);
        }   
        else if (cm > 30) {
          bool bot_cannot_continue = false;
          turnCar(Angle);
          applyMotorControl(1,255);
          digitalWrite(LED_BUILTIN, LOW);
        }
  }////check validity
  }
 



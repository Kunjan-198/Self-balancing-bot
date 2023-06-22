
int angle =0;
const int angleCompensate =0;

const int motorPin1 = 9; // Motor pin 1
const int motorPin2 = 10; // Motor pin 2

const float targetAngle = 0.0;  // Target angle for balancing
const float Kp = 10.0;          // Proportional constant
const float Ki = 2.0;           // Integral constant
const float Kd = 1.0;           // Derivative constant
const float motorSpeed = 150.0; // Base motor speed

// Variables for PID control
float lastError = 0.0;
float integral = 0.0;

void setup()
{
  Serial.begin(9600);          
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void loop()
{
  angle =map(analogRead(A0), 96, 927, -90, 90) + angleCompensate; 
    float error = targetAngle - angle;

  // Calculate the motor speed adjustment using PID control
  float proportional = Kp * error;
  integral += Ki * error;
  float derivative = Kd * (error - lastError);
  float motorSpeedAdjustment = proportional + integral + derivative;

  // Apply the motor speed adjustment to both motors
  int motorSpeed1 = int(motorSpeed + motorSpeedAdjustment);
  int motorSpeed2 = int(motorSpeed - motorSpeedAdjustment);

  // Control the motors
  if (motorSpeed1 >= 0) {
    analogWrite(motorPin1, motorSpeed1);
    digitalWrite(motorPin2, LOW);
  } else {
    analogWrite(motorPin2, -motorSpeed1);
    digitalWrite(motorPin1, LOW);
  }

  if (motorSpeed2 >= 0) {
    analogWrite(motorPin2, motorSpeed2);
    digitalWrite(motorPin1, LOW);
  } else {
    analogWrite(motorPin1, -motorSpeed2);
    digitalWrite(motorPin2, LOW);
  }

  // Update last error for the next iteration
  lastError = error;
}

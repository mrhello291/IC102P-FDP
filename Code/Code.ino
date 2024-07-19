#define IR_SENSOR_RIGHT_2 3
#define IR_SENSOR_RIGHT_1 4
#define IR_SENSOR_LEFT_1 6
#define IR_SENSOR_LEFT_2 7

#define MOTOR_SPEED_MAX 255

#define KP //kp value
#define KI //ki value
#define KD //kd value
#define DT //dt value

float calculatePID(int error);
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);

int enableLeftMotor = A0;
int leftMotorPin1 = 9;
int leftMotorPin2 = 10;

int rightMotorPin1 = 11;
int rightMotorPin2 = 12;
int enableRightMotor = A1;

float init_speed = 255;
float error = 0;
float lastError = 0;
float integral = 0;
int sensors[4] = {0, 0, 0, 0};

void setup() {
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT_2, INPUT);
  pinMode(IR_SENSOR_RIGHT_1, INPUT);
  pinMode(IR_SENSOR_LEFT_1, INPUT);
  pinMode(IR_SENSOR_LEFT_2, INPUT);

  rotateMotor(0, 0);
  Serial.begin(9600);
}

void loop() {

  sensors[0] = digitalRead(IR_SENSOR_RIGHT_2);
  sensors[1] = digitalRead(IR_SENSOR_RIGHT_1);
  sensors[2] = digitalRead(IR_SENSOR_LEFT_1);
  sensors[3] = digitalRead(IR_SENSOR_LEFT_2);

  if (sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 1) error = -3;
  else if (sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 1 && sensors[3] == 1) error = -2;
  else if (sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 1 && sensors[3] == 0) error = -1;
  else if (sensors[0] == 1 && sensors[1] == 1 && sensors[2] == 1 && sensors[3] == 0) error = 1; // For overshoot
  else if (sensors[0] == 1 && sensors[1] == 1 && sensors[2] == 0 && sensors[3] == 1) error = 1; // For intersection
  else if (sensors[0] == 0 && sensors[1] == 1 && sensors[2] == 1 && sensors[3] == 0) error = 0;
  else if (sensors[0] == 1 && sensors[1] == 0 && sensors[2] == 1 && sensors[3] == 1) error = -1; // For intersection
  else if (sensors[0] == 0 && sensors[1] == 1 && sensors[2] == 1 && sensors[3] == 1) error = -1; // For overshoot
  else if (sensors[0] == 0 && sensors[1] == 1 && sensors[2] == 0 && sensors[3] == 0) error = 1;
  else if (sensors[0] == 1 && sensors[1] == 1 && sensors[2] == 0 && sensors[3] == 0) error = 2;
  else if (sensors[0] == 1 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0) error = 3;
  else if (sensors[0] == 1 && sensors[1] == 1 && sensors[2] == 1 && sensors[3] == 1) error = 0; // Move forward in this case too
  else if (sensors[0] == 0 && sensors[1] == 0 && sensors[2] == 0 && sensors[3] == 0){
    if (error == 0) error = 0;
    else if (error < 0) error -= 1;
    else error += 1;
  }
  Serial.println(error);

  float output = calculatePID(error);

  float rightMotorSpeed = constrain(init_speed - output, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
  float leftMotorSpeed = constrain(init_speed + output, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);

  rotateMotor(rightMotorSpeed, leftMotorSpeed);

  delay(DT);
}

float calculatePID(int error) {
  integral = constrain(integral + (error * DT), -MOTOR_SPEED_MAX/4, MOTOR_SPEED_MAX/4);
  float derivative = (error - lastError) / DT;
  float output = KP * error + KI * integral + KD * derivative;
  lastError = error;
  
  return output;
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);    
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);      
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);      
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);    
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);      
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);      
  }
  
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
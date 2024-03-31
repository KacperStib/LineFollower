const uint8_t speed = 75;
const uint8_t leftMotorR = 7;
const uint8_t leftMotorF = 6;
const uint8_t leftMotorEn = 5;
const uint8_t rightMotorF = 9;
const uint8_t rightotorR = 8;
const uint8_t rightotorEn = 10;
const uint8_t leftSensor;
const uint8_t rightSensor;
bool leftAlarm;
bool rightAlarm;

void setup() {
  pinMode(leftMotorEn, OUTPUT);
  pinMode(leftMotorR, OUTPUT);
  pinMode(leftMotorF, OUTPUT);

  pinMode(rightotorR, OUTPUT);
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightotorEn, OUTPUT);

  analogWrite(leftMotorEn, speed);
  analogWrite(rightotorEn, speed);
}


void loop() {

  leftAlarm = digitalRead(19);
  rightAlarm = digitalRead(17);

  if(leftAlarm == 0 && rightAlarm == 1){
    RIGHT();
  }
  else if(leftAlarm == 1 && rightAlarm == 0){
    LEFT();
  }
  else if(leftAlarm == 0 && rightAlarm == 0){
    FORWARD();
  }
  else if(leftAlarm == 1 && rightAlarm == 1){
    STOP();
  }

delay(1);

}

void FORWARD(){
  digitalWrite(leftMotorF,1);
  digitalWrite(leftMotorR,0);
  digitalWrite(rightMotorF,1);
  digitalWrite(rightotorR,0);
  delay(1);
}

void BACKWARD(){
  digitalWrite(leftMotorF,0);
  digitalWrite(leftMotorR,1);
  digitalWrite(rightMotorF,0);
  digitalWrite(rightotorR,1);
  delay(1);
}

void RIGHT(){
  digitalWrite(leftMotorF,1);
  digitalWrite(leftMotorR,0);
  digitalWrite(rightMotorF,0);
  digitalWrite(rightotorR,1);
  delay(1);
}

void LEFT(){
  digitalWrite(leftMotorF,0);
  digitalWrite(leftMotorR,1);
  digitalWrite(rightMotorF,1);
  digitalWrite(rightotorR,0);
  delay(1);
}

void STOP(){
  digitalWrite(leftMotorR,0);
  digitalWrite(leftMotorR,0);
  digitalWrite(rightMotorF,0);
  digitalWrite(rightotorR,0);
  delay(1);
}

// Pins
// IR Sensors Left=A3 Center=A4 Right=A5
// Ultrasonic Sensor echoPin=A2 trigPinger=A1
// L298n Driver ENA=6 IN1=5 IN2=4 IN3=8 IN4=7 ENB=9
// Switch Mode 12 and/or 11

// Motors
#define motor1Speed 6
#define motor1Input1 5
#define motor1Input2 4

#define motor2Speed 9
#define motor2Input1 8
#define motor2Input2 7

// IR Sensors
#define leftIR A3
#define centerIR A4
#define rightIR A5

// Ultrasonic Sensor
#define echoPin A2
#define trigPin A1

// Switch Mode
#define calibrationPin 11
#define switchMode 12

bool mode; 
bool isNotCalibration;

// For Line Follower
//PID properties
const double Kp = 14; //14 //10
const double Ki = 0.2; //0.2
const double Kd = 14; //14 //10
double lastError = 0;
double sumError = 0;
// unsigned long startTime = 0;
int maxSpeed = 80; //80
const int Rspeed = maxSpeed;
const int Lspeed = maxSpeed;
float straight;
bool onTurbo = true;

short sensorMin1 = 1023;   // minimum sensor value
short sensorMin2 = 1023;
short sensorMin3 = 1023;

short sensorMax1 = 0;    // maximum sensor value
short sensorMax2 = 0;
short sensorMax3 = 0;

short sensorValue1 = 0;  // Placeholder reading value
short sensorValue2 = 0;
short sensorValue3 = 0;

short leftMedian = 0;
short centerMedian = 0;
short rightMedian = 0;

// For Sumobot
unsigned int duration, cm;
bool isDetected = false;
static bool prevDetected = isDetected;

unsigned long prevMillis = 0;
unsigned long prevMillisReadIR = 0;
unsigned long prevMillisMove = 0;
unsigned long moveDuration = 0;

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(115200); //115200
  // Setup Motor
  pinMode(motor1Input1, OUTPUT);
  pinMode(motor1Input2, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  analogWrite(motor1Speed, maxSpeed);

  pinMode(motor2Input1, OUTPUT);
  pinMode(motor2Input2, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  analogWrite(motor2Speed, maxSpeed);

  // Setup IR sensors
  pinMode(leftIR, INPUT);
  pinMode(centerIR, INPUT);
  pinMode(rightIR, INPUT);

  // Setup mode
  pinMode(switchMode, INPUT_PULLUP);
  pinMode(calibrationPin, INPUT_PULLUP);

  mode = digitalRead(switchMode);
  isNotCalibration = digitalRead(calibrationPin);

  // Setup Ultrasonic sensors
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}
void loop() {
  // put your main code here, to run repeatedly:
  while(mode){
    //LineFollower
    // readUltrasonic();
    if(isNotCalibration){
      if(onTurbo){
      onTurbo = false;
      forward(motor1Input1, motor1Input2, 100, motor1Speed);
      forward(motor2Input1, motor2Input2, 100, motor2Speed);
      delay(100);
      }
    lineFollower(); 
    }else{
      isNotCalibration = digitalRead(calibrationPin);
      calibrate(); 
    } 
  }
  while(!mode){
    //Sumobot  
    if(millis() <= 5000){
      delay(5000); 
    }  
    sumoBot();
  }
}
// Sumobot
void sumoBot(){
  // bool val_left = 0;
  // bool val_center = 0;
  // bool val_right = 0;
  // if(millis()-prevMillisReadIR >= 25){
  //   val_left = digitalRead(leftIR);
  //   val_center = digitalRead(centerIR);
  //   val_right = digitalRead(rightIR);
  // }

  bool val_left = digitalRead(leftIR);
  bool val_center = digitalRead(centerIR);
  bool val_right = digitalRead(rightIR);
  
  // Serial.print("Left: ");
  // Serial.print(val_left);
  // Serial.print(" Center: ");
  // Serial.print(val_center);
  // Serial.print(" Right: ");
  // Serial.println(val_right);
  
  if(!val_left && val_center && val_right){
    reverse(motor1Input1, motor1Input2, 100, motor1Speed);
    reverse(motor2Input1, motor2Input2, 60, motor2Speed);
    delay(600);
  }else if (!val_left && !val_center && val_right) {
    reverse(motor1Input1, motor1Input2, 100, motor1Speed);
    reverse(motor2Input1, motor2Input2, 80, motor2Speed);
    delay(600);
  }else if (!val_left && !val_center && !val_right) {
    reverse(motor1Input1, motor1Input2, 100, motor1Speed);
    reverse(motor2Input1, motor2Input2, 100, motor2Speed);
    delay(600);
  }else if (val_left && !val_center && !val_right) {
    reverse(motor1Input1, motor1Input2, 80, motor1Speed);
    reverse(motor2Input1, motor2Input2, 100, motor2Speed);
    delay(600);
  }else if (val_left && val_center && !val_right) {
    reverse(motor1Input1, motor1Input2, 60, motor1Speed);
    reverse(motor2Input1, motor2Input2, 100, motor2Speed);
    delay(600);
  }else if(val_left && val_center && val_right){
    searchEnemy();
    if(millis()-prevMillisMove >= 2000){
      prevMillisMove=millis();
      moveDuration=10;
    }
    if(moveDuration>0){
      moveDuration--;
      forward(motor1Input1, motor1Input2, 70, motor1Speed);
      forward(motor2Input1, motor2Input2, 70, motor2Speed);
      // Serial.print("Forward: ");
      // Serial.println(moveDuration);
    }
    delay(25);
  } 
  else{
    reverse(motor1Input1, motor1Input2, 100, motor1Speed);
    reverse(motor2Input1, motor2Input2, 100, motor2Speed);
    delay(600);
  }
  // searchEnemy();
}
// For Sumobot Ultrasonic sensor
short CmThreshold = 20; short AdditionalSpeed=100; short SumoSpeed; //Change AdditionalSpeed to 155
bool readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm = (duration/2) / 29;
  if(cm>CmThreshold){
    cm = 0;
  }
  // Serial.print("duration: ");
  // Serial.println(duration);
  // Serial.print("cm: ");
  // Serial.println(cm);
  return (cm<=CmThreshold && cm>0);
}
void searchEnemy(){
   if (millis() - prevMillis >= 50) {
    isDetected = readUltrasonic();
    prevMillis = millis();  // Update prevMillis for next check
  }
  
  // //Debugging
  // Serial.print("duration: ");
  // Serial.print(duration);
  // Serial.print(" cm: ");
  // Serial.print(cm);
  // Serial.print(" isDetected: ");
  // Serial.println(isDetected);
  
  if (isDetected) {
    if (!prevDetected && cm > 5) {
      // Enemy detected for the first time
      // Serial.println("Inside the rising edge");
      stop(motor1Input1, motor1Input2, motor1Speed);
      stop(motor2Input1, motor2Input2, motor2Speed);
      delay(100);
      reverse(motor1Input1, motor1Input2, 70, motor1Speed);
      forward(motor2Input1, motor2Input2, 70, motor2Speed);
      delay(170);
      stop(motor1Input1, motor1Input2, motor1Speed);
      stop(motor2Input1, motor2Input2, motor2Speed);
      delay(200);
    }
    // Move forward when enemy is detected
    // SumoSpeed = (cm>0)?AdditionalSpeed/cm:0;
    AdditionalSpeed = (cm<7)?155:0;
    if(isDetected)
    forward(motor1Input1, motor1Input2, 100+AdditionalSpeed, motor1Speed);
    forward(motor2Input1, motor2Input2, 100+AdditionalSpeed, motor2Speed);
    // forward(motor1Input1, motor1Input2, 100, motor1Speed);
    // forward(motor2Input1, motor2Input2, 100, motor2Speed);
  } else {
    // No enemy detected, adjust movement accordingly
    // stop(motor1Input1, motor1Input2, motor1Speed);
    // stop(motor2Input1, motor2Input2, motor2Speed);
    // delay(100);
    forward(motor1Input1, motor1Input2, 70, motor1Speed);
    reverse(motor2Input1, motor2Input2, 70, motor2Speed);
  }

  prevDetected = isDetected;
}
int readErrorBool(bool val_left, bool val_center, bool val_right) {
  if(val_left && !val_center && !val_right) return -2; //Detect left only
  else if(val_left && val_center && !val_right) return -1; //Detect left and center
  else if(!val_left && val_center && !val_right) return 0; //Detect center
  else if(!val_left && val_center && val_right) return 1; //Detect right and center
  else if(!val_left && !val_center && val_right) return 2; //Detect right only
  else if(!val_left && !val_center && !val_right) return 3; //No Detection
  else if(val_left && val_center && val_right) return 4; //Detect all black
}

// Line follower
void lineFollower(){
  short left = analogRead(leftIR);
  short center = analogRead(centerIR);
  short right = analogRead(rightIR);

  if(millis()-prevMillis >= 1000){
    prevMillis = millis();
    Serial.print("Left: ");
    Serial.print(left);
    Serial.print(" Center: ");
    Serial.print(center);
    Serial.print(" Right: ");
    Serial.println(right);
  }

  // bool val_left = digitalRead(leftIR);
  // bool val_center = digitalRead(centerIR);
  // bool val_right = digitalRead(rightIR);
  // Serial.print("Center: ");
  // Serial.println(val_center);
  short error = readError(left, center, right);

  // Serial.println(error);
  if(error < 3){
    sumError = (lastError==error)?sumError+error:0;
    // sumError += error;
    straight = (error==0)?straight+0.03:0;
    // straight = 0;
    int pid = Kp*error + (Ki*sumError) +Kd*(error-lastError)+20;
    // pid = (lastError==error)?pid=0.01:pid;
    int lSpeed = constrain(straight+(maxSpeed/3.5)+pid, 0, maxSpeed); //maxSpeed only /3
    // int lSpeed = map(maxSpeed+pid, 0, 100, 0, maxSpeed); 
    int rSpeed = constrain(straight+(maxSpeed/3.5)-pid, 0, maxSpeed); //maxSpeed only /3
    lastError = error;
    // int rSpeed =  map(maxSpeed-pid, 0, 100, 0, maxSpeed);
    forward(motor1Input1, motor1Input2, lSpeed, motor1Speed);
    forward(motor2Input1, motor2Input2, rSpeed, motor2Speed);
    prevMillis=millis();
  }else if(error == 3){
    if(millis()-prevMillis >= 680){
      reverse(motor1Input1, motor1Input2, maxSpeed, motor1Speed);
      reverse(motor2Input1, motor2Input2, maxSpeed, motor2Speed);
    } 
    // if(lastError==3){
    //   reverse(motor1Input1, motor1Input2, maxSpeed, motor1Speed);
    //   reverse(motor2Input1, motor2Input2, maxSpeed, motor2Speed);
    // } 
    // else if(lastError<0){
    //   forward(motor1Input1, motor1Input2, 0, motor1Speed);
    //   forward(motor2Input1, motor2Input2, maxSpeed, motor2Speed);
    // }else if(lastError>0){
    //   forward(motor1Input1, motor1Input2, maxSpeed, motor1Speed);
    //   forward(motor2Input1, motor2Input2, 0, motor2Speed);
    // }
  }
  else{
    if(millis()-prevMillis >= 200){
      stop(motor1Input1, motor1Input2, motor1Speed);
      stop(motor2Input1, motor2Input2, motor2Speed);
    }  
  }
}
bool analogToDigital(short value, short median){
  bool digital = (value>median)?1:0;
  return digital;
} 
void calibrate(){
  if(millis()<=2500){
    forward(motor1Input1, motor1Input2, 70, motor1Speed);
    reverse(motor2Input1, motor2Input2, 70, motor2Speed);
  }else{
    stop(motor1Input1, motor1Input2, motor1Speed);
    stop(motor2Input1, motor2Input2, motor2Speed);
    leftMedian = (sensorMax1 + sensorMin1) /2;
    centerMedian = (sensorMax2 + sensorMin2) /2;
    rightMedian = (sensorMax3 + sensorMin3) /2;

    // if(millis()-prevMillis >= 1000){
    //   prevMillis = millis();
    //   Serial.print("Left Median: ");
    //   Serial.print(leftMedian);
    //   Serial.print(" Center Median: ");
    //   Serial.print(centerMedian);
    //   Serial.print(" Right Median: ");
    //   Serial.println(rightMedian);
    // }
  }

  short sensorValue1 = analogRead(leftIR);
  short sensorValue2 = analogRead(centerIR);
  short sensorValue3 = analogRead(rightIR);
  
  // Record the maximum sensor value, and sets as new limit
  if (sensorValue1 > sensorMax1) {
    sensorMax1 = sensorValue1;
  }
  if (sensorValue2 > sensorMax2) {
    sensorMax2 = sensorValue2;
  }
  if (sensorValue3 > sensorMax3) {
    sensorMax3 = sensorValue3;
  }

  // Record the minimum sensor value, and sets as new limit
  if (sensorValue1 < sensorMin1) {
    sensorMin1 = sensorValue1;
  }
  if (sensorValue2 < sensorMin2) {
    sensorMin2 = sensorValue2;
  }
  if (sensorValue3 < sensorMin3) {
    sensorMin3 = sensorValue3;
  }
  delay(10);
}
int readError(short val_left, short val_center, short val_right) {
  if(val_left>leftMedian && val_center<=centerMedian && val_right<=rightMedian) return -2; //Detect left only
  else if(val_left>leftMedian && val_center>centerMedian && val_right<=rightMedian) return -1; //Detect left and center
  else if(val_left<=leftMedian && val_center>centerMedian && val_right<=rightMedian) return 0; //Detect center
  else if(val_left<=leftMedian && val_center>centerMedian && val_right>rightMedian) return 1; //Detect right and center
  else if(val_left<=leftMedian && val_center<=centerMedian && val_right>rightMedian) return 2; //Detect right only
  else if(val_left<=leftMedian && val_center<=centerMedian && val_right<=rightMedian) return 3; //No Detection
  else if(val_left>leftMedian && val_center>centerMedian && val_right>rightMedian) return 4; //Detect all black
}

void reverse(int a, int b, int speed, int pwm){
  digitalWrite(a,HIGH);
  digitalWrite(b,LOW);
  analogWrite(pwm, speed);
}
void forward(int a, int b,int speed, int pwm){
  digitalWrite(a,LOW);
  digitalWrite(b,HIGH);
  analogWrite(pwm, speed);
}
void stop(int a, int b,int pwm){
  digitalWrite(a,LOW);
  digitalWrite(b,LOW);
  analogWrite(pwm, 0);
}







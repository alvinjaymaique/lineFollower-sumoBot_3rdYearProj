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
#define echoPin A1
#define trigPin A2

// Switch Mode
#define switchMode 12
bool mode = digitalRead(switchMode);

// For Line Follower
//PID properties
const double Kp = 14; //14 //10
const double Ki = 0.2; //0.2
const double Kd = 14; //14 //10
double lastError = 0;
double sumError = 0;
// unsigned long startTime = 0;
int defaultSpeed = 70; //39
const int Rspeed = defaultSpeed;
const int Lspeed = defaultSpeed;
float straight;


// For Sumobot
int duration, cm;

unsigned long prevMillis = 0;

void setup() {
  // put your setup code here, to run once:
  // put your setup code here, to run once:
  Serial.begin(115200); //115200
  // Setup Motor
  pinMode(motor1Input1, OUTPUT);
  pinMode(motor1Input2, OUTPUT);
  pinMode(motor1Speed, OUTPUT);
  analogWrite(motor1Speed, defaultSpeed);

  pinMode(motor2Input1, OUTPUT);
  pinMode(motor2Input2, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  analogWrite(motor2Speed, defaultSpeed);

  // Setup IR sensors
  pinMode(leftIR, INPUT);
  pinMode(centerIR, INPUT);
  pinMode(rightIR, INPUT);

  // Setup mode
  pinMode(switchMode, INPUT_PULLUP);

  // Setup Ultrasonic sensors
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(mode){
    //LineFollower
    if(millis() <= 100){
      forward(motor1Input1, motor1Input2, 100, motor1Speed);
      forward(motor2Input1, motor2Input2, 100, motor2Speed);
    }else{
      lineFollower();
    }   
    // lineFollower();
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
  readUltrasonic();
  Serial.println(cm);
  if (cm > 1 && cm < 50) { 
    forward(motor1Input1, motor1Input2, defaultSpeed, motor1Speed);
    forward(motor2Input1, motor2Input2, defaultSpeed, motor2Speed);
    delay(300);
  } else {              
    reverse(motor1Input1, motor1Input2, defaultSpeed, motor1Speed);
    forward(motor2Input1, motor2Input2, defaultSpeed, motor2Speed);
    delay(10);
  }
}

// For Sumobot Ultrasonic sensor
void readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm = (duration/2) / 29;
}

void lineFollower(){
  // int left = analogRead(leftIR);
  // int center = analogRead(centerIR);
  // int right = analogRead(rightIR);

  // if(millis()-prevMillis >= 1000){
  //   prevMillis = millis();
  //   Serial.print("Left: ");
  //   Serial.print(left);
  //   Serial.print(" Center: ");
  //   Serial.print(center);
  //   Serial.print(" Right: ");
  //   Serial.println(right);
  // }

  bool val_left = digitalRead(leftIR);
  bool val_center = digitalRead(centerIR);
  bool val_right = digitalRead(rightIR);
  // Serial.print("Center: ");
  // Serial.println(val_center);
  int error = readError(val_left, val_center, val_right);
  // Serial.println(error);
  if(error < 3){
    sumError = (lastError==error)?sumError+error:0;
    // sumError += error;
    straight = (error==0)?straight+0.015:0;
    // straight = 0;
    int pid = Kp*error + (Ki*sumError) +Kd*(error-lastError);
    // pid = (lastError==error)?pid=0.01:pid;
    int lSpeed = constrain(straight+(defaultSpeed/3)+pid, 0, defaultSpeed); //defaultSpeed only
    // int lSpeed = map(defaultSpeed+pid, 0, 100, 0, defaultSpeed); 
    int rSpeed = constrain(straight+(defaultSpeed/3)-pid, 0, defaultSpeed); //defaultSpeed only
    lastError = error;
    // int rSpeed =  map(defaultSpeed-pid, 0, 100, 0, defaultSpeed);
    forward(motor1Input1, motor1Input2, lSpeed, motor1Speed);
    forward(motor2Input1, motor2Input2, rSpeed, motor2Speed);
    prevMillis=millis();
  }else if(error == 3){
    if(millis()-prevMillis >= 680){
      reverse(motor1Input1, motor1Input2, defaultSpeed, motor1Speed);
      reverse(motor2Input1, motor2Input2, defaultSpeed, motor2Speed);
    }  
  }
  else{
    if(millis()-prevMillis >= 45){
      stop(motor1Input1, motor1Input2, motor1Speed);
      stop(motor2Input1, motor2Input2, motor2Speed);
    }  
  }
}

// For Line follower 
int readError(bool val_left, bool val_center, bool val_right) {
  if(val_left && !val_center && !val_right) return -2; //Detect left only
  else if(val_left && val_center && !val_right) return -1; //Detect left and center
  else if(!val_left && val_center && !val_right) return 0; //Detect center
  else if(!val_left && val_center && val_right) return 1; //Detect right and center
  else if(!val_left && !val_center && val_right) return 2; //Detect right only
  else if(!val_left && !val_center && !val_right) return 3; //No Detection
  else if(val_left && val_center && val_right) return 4; //Detect all black
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







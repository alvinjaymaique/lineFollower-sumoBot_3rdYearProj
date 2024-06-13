// Pins
// IR Sensors Left=A3 Center=A4 Right=A5
// Ultrasonic Sensor Echo=A2 Trigger=A1
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
#define echo A2
#define trig A1

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
int defaultSpeed = 75; //39
const int Rspeed = defaultSpeed;
const int Lspeed = defaultSpeed;
float straight;
// For Line Follower

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
}

void loop() {
  // put your main code here, to run repeatedly:
  while(mode){
    //LineFollower
    lineFollower();
  }
  while(!mode){
    //Sumobot    
  }
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
    int error = readError(val_left, val_center, val_right);
    Serial.println(error);
    if(error < 3){
      sumError = (lastError==error)?sumError+error:0;
      straight = (error==0)?straight+0.007:0;
      int pid = Kp*error + (Ki*sumError) +Kd*(error-lastError);
      int lSpeed = constrain(straight+(defaultSpeed/1.7)+pid, 0, defaultSpeed); //defaultSpeed only
      // int lSpeed = map(defaultSpeed+pid, 0, 100, 0, defaultSpeed); 
      int rSpeed = constrain(straight+(defaultSpeed/1.7)-pid, 0, defaultSpeed); //defaultSpeed only
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
      if(millis()-prevMillis >= 400){
        stop(motor1Input1, motor1Input2, motor1Speed);
        stop(motor2Input1, motor2Input2, motor2Speed);
      }  
    }
  }

  int readError(bool val_left, bool val_center, bool val_right) {
    if(val_left && !val_center && !val_right) return -2;
    else if(val_left && val_center && !val_right) return -1;
    else if(!val_left && val_center && !val_right) return 0;
    else if(!val_left && val_center && val_right) return 1;
    else if(!val_left && !val_center && val_right) return 2;
    else if(val_left && val_center && val_right) return 4;
    else if(!val_left && !val_center && !val_right) return 3;
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

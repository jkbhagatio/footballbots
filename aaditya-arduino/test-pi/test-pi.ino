typedef struct {
  int powerPin;
  int logicPins[2];
  int interruptPin;
  long crossTime = -1;
  int dir = 0;
  bool is_on = false;
//  double s = -1.0;
} Motor;

Motor motors[2];
bool backingUp = false;
int motorFlip = 1;
long lastBackUp = 0;

void setupMotor(int i) {
  pinMode(motors[i].powerPin, OUTPUT);
  pinMode(motors[i].interruptPin, INPUT);
  for (int j = 0; j < 2; j++) {
    pinMode(motors[i].logicPins[j], OUTPUT);
  }
}

void set(int i, int dir) {
  motors[i].dir = dir;
  switch (dir) {
    case 1:
      digitalWrite(motors[i].logicPins[0], HIGH);
      digitalWrite(motors[i].logicPins[1], LOW);
      break;
    case -1:
      digitalWrite(motors[i].logicPins[0], LOW);
      digitalWrite(motors[i].logicPins[1], HIGH);
      break;
    case 0:
      digitalWrite(motors[i].logicPins[0], LOW);
      digitalWrite(motors[i].logicPins[1], LOW);
      break;
  }
}

void flip(int i) {
  set(i, -1*motors[i].dir);
}

void forward(int i) {
  set(i, 1);
}

void pause(int i) {
  set(i, 0);
}

void backward(int i) {
  set(i, -1);
}

void turnRight() {
  set(0, 1);
  set(1, -1);
}

void turnLeft() {
  set(1, 1);
  set(0, -1);
}

void setup() {
  Serial.begin(9600);
  // Initialize motors
  // Left motor -- note we switched the power and logic pin since pin 4 is not a PWM output pin
  motors[0].powerPin = 6; motors[0].logicPins[0] = 9; motors[0].logicPins[1] = 10; motors[0].interruptPin = 3; //motors[0].sensorPin = 10; //motors[0].crossA = {-1,-1}; motors[0].crossB = {-1,-1}; motors[0].s = -1;
  // Right motor
  motors[1].powerPin = 5;  motors[1].logicPins[0] = 7; motors[1].logicPins[1] = 8; motors[1].interruptPin = 2; //motors[1].sensorPin = 11; //motors[1].crossA = {-1,-1}; motors[1].crossB = {-1,-1}; motors[1].s = -1;
  // Setup motors and attach interrupts
  for (int i = 0; i < 2; i++) setupMotor(i);
  attachInterrupt(digitalPinToInterrupt(motors[0].interruptPin), interruptL, RISING);
  attachInterrupt(digitalPinToInterrupt(motors[1].interruptPin), interruptR, RISING);
  pause(0);
  pause(1);
  analogWrite(5, 150);
  analogWrite(6, 150);
}

void loop() {
//  Serial.println("--");
  if (millis() > max(motors[0].crossTime, motors[1].crossTime)+1000) {
    flip(0); flip(1);
    backingUp = true;
    if (millis() > lastBackUp + 10000) {
      motorFlip = random(2);
    }
    lastBackUp = millis();
    delay(random(500,1000));
    
    flip(motorFlip);
    delay(random(500,1000));
    flip(1-motorFlip);
  }
  else if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data.equals("L")) {
      turnLeft();
    }
    else if (data.equals("R")) {
      turnRight();
    }
    else if (data.equals("F")) {
      forward(0);
      forward(1);
    }
    else if (data.equals("B")) {
      backward(0);
      backward(1);
    }
    else {
      Serial.print("You sent me: ");
      Serial.println(data);
      Serial.println(data.length());
      Serial.println("I didn't recognize it!");
    }
  }
}

void interruptL() {
//  Serial.println("FLIP");
  int t = millis();
//  motors[0].s = 1.0/(t-motors[0].crossTime);
  motors[0].crossTime = t;
}

void interruptR() {
//  Serial.println("FLIP");
  int t = millis();
//  motors[1].s = 1.0/(t-motors[1].crossTime);
  motors[1].crossTime = t;
}

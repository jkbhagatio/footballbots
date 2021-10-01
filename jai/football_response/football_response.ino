/*
* This will be the arduino code that will listen to serial commands from raspberry pi
* to control robot in response to ball tracking (see `football_commands.py`).
* 
* - unstick
* - move_to_ball
* - move_to_goal
* - power_fan
* - limit_movement
* 
* Possible movements: forward, turn right and forward, turn left and forward, turn left in place.
*/

// Define motor struct.
typedef struct {
    int power_pin;
    int logic_pins[2];
    int interrupt_pin;
    long crosstime = -1;
    int dir = 0;
} Motor;

Motor motors[2];  // 0 is left, 1 is right.

// SET PINS!
motors[0].power_pin = 10; 
motors[0].logic_pins[0] = 12; 
motors[0].logic_pins[1] = 11; 
motors[0].interrupt_pin = 3;
motors[1].power_pin = 6; 
motors[1].logic_pins[0] = 7; 
motors[1].logic_pins[1] = 8; 
motors[1].interrupt_pin = 2;

// Sets up pinmodes for a motor.
void setup_motor_pinmode(int i) {
    pinMode(motors[i].power_pin, OUTPUT);
    pinMode(motors[i].interrupt_pin, INPUT_PULLUP);
    for (int j = 0; j < 2; j++) {
        pinMode(motors[i].logic_pins[j], OUTPUT);
  }
}

// Set some robo movement vars.
int motor_flip = 1;
long last_backup = 0;  // timestamp of last backup

bool stuck;  // true if robot is stuck
int STUCK_T = 1500;  // threshold time (ms) for interrupt not being triggered -> unstick

void setup() {
    Serial.begin(57600);
    // Set up pinmodes.
    for (int i = 0; i < 2; i++) setup_motor_pinmode(i);
    attachInterrupt(digitalPinToInterrupt(motors[0].interruptPin), interruptL, RISING);
    attachInterrupt(digitalPinToInterrupt(motors[1].interruptPin), interruptR, RISING);
}

void loop() {
    unstick();
    move_to_ball();
}

void interruptL() {
  motors[0].crossTime = millis();
}

void interruptR() {
  motors[0].crossTime = millis();
}

void set_motor_dir(int m, int dir) {
  motors[m].dir = dir;
  switch (dir) {
    case 1:  // forward
      digitalWrite(motors[m].logicPins[0], HIGH);
      digitalWrite(motors[m].logicPins[1], LOW);
      break;
    case -1:  // backward
      digitalWrite(motors[m].logicPins[0], LOW);
      digitalWrite(motors[m].logicPins[1], HIGH);
      break;
    case 0:  // brake
      digitalWrite(motors[m].logicPins[0], LOW);
      digitalWrite(motors[m].logicPins[1], LOW);
      break;
  }
}

void set_motor_pow(int m, int pwm) {
    analogWrite(motors[m].power_pin, pwm);
}

void unstick() {
    if (millis() > (max(motor_l_crosstime, motor_r_crosstime) + STUCK_T)) {
        // Flip direction of both motors.
        flip_motor_dir(0); flip_motor_dir(1);
        // If stuck in oppo direction, randomly flip one motor to turn.
        if (millis() > last_backup + (STUCK_T * 2)) {
            motor_flip = random(2);
        }
        last_backup = millis();
        // Unstick for .5-1 s in oppo direction.
        delay(random(500, 1000));
        flip_motor_dir(motor_flip);
        // Turn for .5-1 s.
        delay(random(500,1000));
        // Move in direction of original heading.
        flip_motor_dir(1 - motor_flip);
  }
}

void flip_motor_dir(int i) {
  set_motor_dir(i, -1*motors[i].dir);
}

// Listens to pi to move towards ball
void move_to_ball() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'r') {  // forward right
            forward_right();
        }
        if (c == 'l') {  // forward left
            forward_left();
        }
        if (c == 'f') {  // directly forward
            forward();
        }
        if (c == 't') {  // turn
            turn_left();
        }
        if (c == 's') {  // stop
            stopit();
        }
    }
}






/*
void increase_speed() {
    pwm_l_val = pwm_l_val + PWM_TOGGLE;
    pwm_r_val = pwm_r_val + PWM_TOGGLE;
    // Set limit.
    if (pwm_l_val > PWM_RES) {pwm_l_val = PWM_RES;}
    if (pwm_r_val > PWM_RES) {pwm_r_val = PWM_RES;}
    analogWrite(PWM_LEFT, pwm_l_val);
    analogWrite(PWM_RIGHT, pwm_r_val);
}

void decrease_speed() {
    pwm_l_val = pwm_l_val - PWM_TOGGLE;
    pwm_r_val = pwm_r_val - PWM_TOGGLE;
    // Set limit.
    if (pwm_l_val < PWM_MIN) {pwm_l_val = PWM_MIN;}
    if (pwm_r_val < PWM_MIN) {pwm_r_val = PWM_MIN;}
    analogWrite(PWM_LEFT, pwm_l_val);
    analogWrite(PWM_RIGHT, pwm_r_val);
}

void forward() {
    analogWrite(PWM_RIGHT, PWM_RES);
    analogWrite(PWM_LEFT, PWM_RES);
    digitalWrite(LEFT_LOG1, HIGH);
    digitalWrite(LEFT_LOG2, LOW);
    digitalWrite(RIGHT_LOG1, HIGH);
    digitalWrite(RIGHT_LOG2, LOW);
}

void forward_right() {
    analogWrite(PWM_RIGHT, PWM_MIN);
    analogWrite(PWM_LEFT, PWM_RES);
    digitalWrite(LEFT_LOG1, HIGH);
    digitalWrite(LEFT_LOG2, LOW);
    digitalWrite(RIGHT_LOG1, HIGH);
    digitalWrite(RIGHT_LOG2, LOW);
}

void forward_left() {
    analogWrite(PWM_RIGHT, PWM_RES);
    analogWrite(PWM_LEFT, PWM_MIN);
    digitalWrite(LEFT_LOG1, HIGH);
    digitalWrite(LEFT_LOG2, LOW);
    digitalWrite(RIGHT_LOG1, HIGH);
    digitalWrite(RIGHT_LOG2, LOW);
}

void turn_right() {
    digitalWrite(LEFT_LOG1, HIGH);
    digitalWrite(LEFT_LOG2, LOW);
    digitalWrite(RIGHT_LOG1, LOW);
    digitalWrite(RIGHT_LOG2, HIGH);
}

void turn_left() {
    analogWrite(PWM_RIGHT, PWM_RES);
    analogWrite(PWM_LEFT, PWM_MIN);
    digitalWrite(LEFT_LOG1, LOW);
    digitalWrite(LEFT_LOG2, HIGH);
    digitalWrite(RIGHT_LOG1, HIGH);
    digitalWrite(RIGHT_LOG2, LOW);
}

void stopit() {
    digitalWrite(LEFT_LOG1, LOW);
    digitalWrite(LEFT_LOG2, LOW);
    digitalWrite(RIGHT_LOG1, LOW);
    digitalWrite(RIGHT_LOG2, LOW);
}
*/

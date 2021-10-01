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
    
    // SET PINS!
    motors[0].power_pin = 10; 
    motors[0].logic_pins[0] = 12; 
    motors[0].logic_pins[1] = 11; 
    motors[0].interrupt_pin = 3;
    motors[1].power_pin = 6; 
    motors[1].logic_pins[0] = 7; 
    motors[1].logic_pins[1] = 8; 
    motors[1].interrupt_pin = 2;
    int fan_pin;  //todo: needs to be set.

    for (int i = 0; i < 2; i++) setup_motor_pinmode(i);
    attachInterrupt(digitalPinToInterrupt(motors[0].interrupt_pin), interruptL, RISING);
    attachInterrupt(digitalPinToInterrupt(motors[1].interrupt_pin), interruptR, RISING);
}

void loop() {
    unstick();
    act();
    
}

void interruptL() {
  motors[0].crosstime = millis();
}

void interruptR() {
  motors[0].crosstime = millis();
}

void set_motor_dir(int m, int dir) {
  motors[m].dir = dir;
  switch (dir) {
    case 1:  // forward
      digitalWrite(motors[m].logic_pins[0], HIGH);
      digitalWrite(motors[m].logic_pins[1], LOW);
      break;
    case -1:  // backward
      digitalWrite(motors[m].logic_pins[0], LOW);
      digitalWrite(motors[m].logic_pins[1], HIGH);
      break;
    case 0:  // brake
      digitalWrite(motors[m].logic_pins[0], LOW);
      digitalWrite(motors[m].logic_pins[1], LOW);
      break;
  } 
}

void set_motor_pow(int m, int pwm) {
    analogWrite(motors[m].power_pin, pwm);
}

void unstick() {
    if (millis() > (max(motors[0].crosstime, motors[1].crosstime) + STUCK_T)) {
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

// Listens to pi to act
void act() {
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'r') {  // forward right
            set_motor_pow(0, 255);
            set_motor_pow(1, 127);
            set_motor_dir(0, 1);
            set_motor_dir(1, 1);
        }
        if (c == 'l') {  // forward left
            set_motor_pow(0, 127);
            set_motor_pow(1, 255);
            set_motor_dir(0, 1);
            set_motor_dir(1, 1);
        }
        if (c == 'f') {  // directly forward
            set_motor_pow(0, 255);
            set_motor_pow(1, 255);
            set_motor_dir(0, 1);
            set_motor_dir(1, 1);
        }
        if (c == 'a') {  // turn left in place
            set_motor_pow(0, 127);
            set_motor_pow(1, 255);
            set_motor_dir(0, -1);
            set_motor_dir(1, 1);
        }
        if (c == 'd') {  // turn right in place
            set_motor_pow(0, 255);
            set_motor_pow(1, 127);
            set_motor_dir(0, -1);
            set_motor_dir(1, 1);
        }
        if (c == 's') {  // stop
            set_motor_dir(0, 0);
            set_motor_dir(0, 0);
        }
        if (c == 'f') {  // blow fan
            //todo: do something with fan pin for X amount of time.
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

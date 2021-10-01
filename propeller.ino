// defining propeller pin

const int myPropeller = 11;


void setup() {
// initialize serial communication at 9600 bps
Serial.begin(9600);

// defining propeller as output pin
pinMode(myPropeller, OUTPUT);

}

void loop(){  
	
  if(Serial.available() > 0){
    char cmdLetter = Serial.read();
    
      if(cmdLetter == 'p'){
        digitalWrite(myPropeller, HIGH);
	delay(1500);
	digitalWrite(myPropeller, LOW);
      }
  }
 delay(10);
}

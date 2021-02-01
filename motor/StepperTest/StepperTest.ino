/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/


#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);


void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  myMotor->setSpeed(255);
}

float cm_per_step = 1.68/1000.0f;
float inches_per_step = cm_per_step / 2.54f;
float pos_inches = 0;
float target_inches = 0;
String received = "";

void loop() {
  if (Serial.available() > 0) {
    char b = Serial.read();
    if (b == ',') {
      target_inches = received.toFloat();
      received = "";
//      Serial.print("New Target: ");
//      Serial.println(target_inches);
    } else {
      received += b;
    }
  }
  if (target_inches > (pos_inches+2*inches_per_step)) {
    myMotor->step(1, FORWARD, SINGLE);
    pos_inches += inches_per_step;
  } else if (target_inches < (pos_inches-2*inches_per_step)) {
    myMotor->step(1, BACKWARD, SINGLE);
    pos_inches -= inches_per_step;
  } else {
//    Serial.println("Done");
    myMotor->release();
  }
//  Serial.print("Target: ");
//  Serial.print(target_inches);
//  Serial.print(" | Pos: ");
//  Serial.println(pos_inches);
//  delay(100);

//  Serial.println("Double coil steps");
//  myMotor->step(100, FORWARD, DOUBLE); 
//  myMotor->step(100, BACKWARD, DOUBLE);
//  
//  Serial.println("Interleave coil steps");
//  myMotor->step(100, FORWARD, INTERLEAVE); 
//  myMotor->step(100, BACKWARD, INTERLEAVE); 
//  
//  Serial.println("Microstep steps");
//  myMotor->step(50, FORWARD, MICROSTEP); 
//  myMotor->step(50, BACKWARD, MICROSTEP);
}

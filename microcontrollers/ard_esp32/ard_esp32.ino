#include <ESP32Servo.h>
#include "ESC.h"
#include "Wire.h"

/********** I2C COMMS **********/

// I2C-0 pins
#define SDA_0 21
#define SCL_0 22


#define I2C_FREQ 400000 // specifies clockspeed in bts
#define I2C_DEV_ADDR 0x50 // slave address
#define CONTROLLER_BUFFSIZE 6 // how many bytes sent per packet
/********** END I2C COMMS **********/

/********** MOTOR DEFS **********/
#define FULL_SPEED 500 // set full speed which is 1500 (1000 + 500) ECS sets baseline as 1000 (forward and reverse)
#define MOTOR_DIR_SUPPORT ESC::MODE_FORWARD_BACKWARD // motors support both forward/reverse
#define AVAILABLE_MOTORS 3 // use this to prevent buffer overreading

// motor pins
#define MOTOR_1 25
#define MOTOR_2 26
#define MOTOR_3 27

/********** END MOTOR DEFS **********/

// I2C classes
TwoWire I2C_0 = TwoWire(0);

// Motor classes
ESC motors[] = {
  ESC(MOTOR_1, MOTOR_DIR_SUPPORT),
  ESC(MOTOR_2, MOTOR_DIR_SUPPORT),
  ESC(MOTOR_3, MOTOR_DIR_SUPPORT)
};


unsigned char cBuff[CONTROLLER_BUFFSIZE];

void onRequest(){
//  Wire.print(i++);
  I2C_0.print("WIRE Request");
  Serial.println("WIRE Request");
}

void onReceive(int len){
  Serial.printf("onReceive[%d]: ", len);
  if (len != CONTROLLER_BUFFSIZE) {
    Serial.println("Invalid buffer size!");
    return;
  }

  // read I2C buffer and update motors
  int ind = 0;
  while(I2C_0.available()){
//    cBuff[ind++] = (unsigned char) I2C_0.read();
    if (ind >= AVAILABLE_MOTORS) {
      I2C_0.read(); // clear buffer
      continue;
    }

    // valid so let's read motors
    ESC &motor = motors[ind];
    int speed = (signed char) I2C_0.read();
    speed = BASE_FORWARD_BACKWARD + (int) (((float) FULL_SPEED) * (((float) speed) / 127.0));  // scale from -FULL_SPEED to FULL_SPEED and add BASE_SPEED (usually 1500)

    Serial.print("Speed: ");
    Serial.println(speed);
 
    // make sure speed is within pulse range
    // @NOTE you might get weird behaviour if you send below 1000 with a forward only mode
    if (speed >= MIN_PULSE && speed <= MAX_PULSE) {
      motor.writePulse((unsigned int) speed);  // make sure we're within range
    }
    
//    if (speed < 0) {
//      motor.setDirection(ESC::BACKWARD);
//      motor.setSpeed(-speed);
//    } else {
//      motor.setDirection(ESC::FORWARD);
//      motor.setSpeed(speed);
//    }
  }

  // TESTING purposes
//  int speed = ((signed char) cBuff[0]); // this will be between -127 and 127 (256 including 0 so 1 byte)
//  
//  Serial.println();
}


void setup() 
{
  Serial.begin(115200);

  // arm all of the motors
  for (int i = 0; i < AVAILABLE_MOTORS; i++) {
    motors[i].arm();
  }
  
  delay(500); // short time for arming
  Serial.println("Armed the controller");

  // handle requests
  I2C_0.begin((uint8_t)I2C_DEV_ADDR, SDA_0, SCL_0, I2C_FREQ);
  I2C_0.onReceive(onReceive);
  I2C_0.onRequest(onRequest);

  // set motors to initially be disabled
  for (int i = 0; i < AVAILABLE_MOTORS; i++) {
    motors[i].setDirection(ESC::FORWARD);
    motors[i].setSpeed(0);
  }
}

void loop() 
{ 
//  if (Serial.available() > 0) {
//    char c = Serial.read();
//    if (c == '0') {
//      Serial.println("ZERO");
//      esc.setSpeed(0);
//    } else if (c == '1') {
//      Serial.println("FULL");
//      esc.setSpeed(FULL_SPEED);
//    }
//  }
//    esc.setSpeed(0);
//    esc.setDirection(ESC::FORWARD);
//    esc.setSpeed(400);
//    delay(1000);
//    esc.setSpeed(0);
//    esc.setDirection(ESC::BACKWARD);
//    esc.setSpeed(400);
//    delay(1000);
} 
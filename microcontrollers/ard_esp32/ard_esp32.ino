#include "config.h"

#include <ESP32Servo.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "ESC.h"
#include "SPI.h"
#include "Wire.h"

/********** I2C COMMS **********/

// I2C-0 pins
#define SDA_0 21
#define SCL_0 22


#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 8
#define LSM9DS1_MCS 5

//1E, 6B

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

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

unsigned char cBuff[CONTROLLER_BUFFSIZE];

void setup_gyro(){
  //Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // i2c sensor
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS1 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring!"));
    while(1);
  }

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);

  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
}

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

  
  Serial.begin(9600);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  Serial.println("Serial Init"); 

  #if defined(RUN_GYRO)
     setup_gyro();
  #endif


  // arm all of the motors
  #if defined(RUN_MOTORS)
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
  #endif
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

  #if defined(RUN_GYRO)
  lsm.read();  /* ask it to read in the data */ 

  //Get a new sensor event
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" uT");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" uT");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" uT");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" rad/s");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" rad/s");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" rad/s");

  Serial.println();
  delay(200);
  #endif
} 
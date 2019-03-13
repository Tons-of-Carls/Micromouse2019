#include "Macros.h"

#define BobaTime
//ControllerTest
//AccelerometerTest
//EncoderTest
//MotorTest
//BobaTime

/*--------------------------------------------------------------------------------------------------------------------------------------*/


#ifdef BobaTime

#include "Controller.cpp"

Controller rightWheel(.01, 0, 2, 15, 12, 11, 5);// float kp, float ki, uint8_t encPin1, uint8_t encPin2, int mPin1, int mPin2, int mpwm
Controller leftWheel(.01, 0, 16, 17, 8, 7, 4);

int state = 0;

bool rightDone = false;
bool leftDone = false;

void setup() {
  Serial.begin(9600);
  Serial.print("Starting State 0");
}

// Steps to get boba
// 1. Move forward 18 cm (734 encoder ticks)
// 2. Turn left some amount
// 3. Forward 18 cm (734 encoder ticks)
// 4. Turn right some amount

void loop() {
  if(state == 0){
    if(!rightDone){
      rightDone = rightWheel.update(734);
    }
    if(!leftDone){
      leftDone = leftWheel.update(-734);
    }

    if(leftDone && rightDone){
      rightDone = false;
      leftDone = false;
      state++;
      Serial.println("State 0 Finished");
      Serial.println();
      Serial.println("Starting State 1");
    }
  }

  if(state == 1){
    if(!rightDone){
      rightDone = rightWheel.update(734);
    }
    if(!leftDone){
      leftDone = leftWheel.update(734);
    }

    if(leftDone && rightDone){
      rightDone = false;
      leftDone = false;
      state++;
      Serial.println("State 1 Finished");
      Serial.println();
      Serial.println("Starting State 2");
    }
  }

  if(state == 2){
    if(!rightDone){
      rightDone = rightWheel.update(734);
    }
    if(!leftDone){
      leftDone = leftWheel.update(-734);
    }

    if(leftDone && rightDone){
      rightDone = false;
      leftDone = false;
      state++;
      Serial.println("State 2 Finished");
      Serial.println();
      Serial.println("Starting State 3");
    }
  }

  if(state == 3){
    if(!rightDone){
      rightDone = rightWheel.update(-734);
    }
    if(!leftDone){
      leftDone = leftWheel.update(-734);
    }

    if(leftDone && rightDone){
      rightDone = false;
      leftDone = false;
      state++;
      Serial.println("State 3 Finished");
    }
  }
}

#endif



#ifdef MotorTest

#include "Motor.cpp"
#include <Encoder.h>

//for our future reference
//IR sensors...
//T1~4 is from left to right of mouse (clockwise)
//T1-21 LeftLeft
//T2-20 FrontLeft
//T3-19 FrontRight
//T4-18 RightRight

const int irPinLL = 21;
const int irPinFL = 20;
const int irPinFR = 19;
const int irPinRR = 18;

//LEDs...
//L1 is outer, L2 is inner
//L1-0
//L2-1

const int ledPin1 = 0;
const int ledPin2 = 1;

//Encoders...
//ENC1 is left, ENC2 is right
//ENC1A-16
//ENC1B-17
//ENC2A-2
//ENC2B-15

//format
//ENC1A, ENC1B
Encoder leftEncoder(16,17);
Encoder rightEncoder(2,15);

//Motors...
//1 is left, 2 is right
//M1R-8
//M1F-7
//M1E-4
//M2R-12
//M2F-11
//M2E-5

//format
// M1R M1F M1E
Motor motorL(8,7,4);
Motor motorR(12,11,5);
//builtinLED - 11


//Switches...
//1 is back, 2 is front
//S1-13
//S2-14
//
//Buzzer...
//BZ-3
//
//Unused pins...
//6
//9
//10
//13
//14



const int d = 10; //unknown

int ir1 = 0; //unknown
int ir2 = 0; //unknown

unsigned int prevIR = 0;
unsigned int currIR = 0;

unsigned long time;



bool stop = false;

void setup() {
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
//  pinMode(12, OUTPUT); //because LED PIN
  time = millis();
  Serial.begin(9600);
}

void loop() {
  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);
//  digitalWrite(12, LOW);
  
//  Serial.println("Running Motor...");
  
 
  long oldPosition  = -999;
  long newPosition = rightEncoder.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.print("Encoders: ");
    Serial.print(leftEncoder.read());
    Serial.print(" ");
    Serial.println(newPosition);
  }


/*   currIR = analogRead(irPinFL);
  if (currIR != prevIR) 
  {
    Serial.print("IR Value: ");
    Serial.println(currIR);
    prevIR = currIR;
  } */

  // IR is unreliable right now, so 900 is a good threshhold as it varies 920-980 at start
  if(leftEncoder.read() > 1400){
    stop = true;
    Serial.println("stop condition reached by encoder");
  }

//  if (currIR < 800 && time-millis() > 11000){
//    stop=true;
//    Serial.println("stop condition reached by ir");
//  }

  if(stop){
    motorL.update(0);
    motorR.update(0);
  }
  else{
    motorL.update(-1);
    motorR.update(1);
  }
  
  delay(PROGRAM_DELAY_MS);
  
}

#endif


/*--------------------------------------------------------------------------------------------------------------------------------------*/


#ifdef ControllerTest
#include "Controller.cpp"

Controller motor1(.01,.01,1,2,3,4,5);

void setup() {
  
}

void loop() {

  
  
  motor1.update(100);
  delay(PROGRAM_DELAY_MS);
}
#endif


/*--------------------------------------------------------------------------------------------------------------------------------------*/


#ifdef AccelerometerTest
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

//int16_t ax, ay, az;
//int16_t gx, gy, gz;



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
// #define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


//#define LED_PIN 13
//bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED pin for output
    //pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("gyroX: ");
  Serial.println(accelgyro.getRotationX());

  Serial.print("gyroY: ");
  Serial.println(accelgyro.getRotationY());

  Serial.print("gyroZ: ");
  Serial.println(accelgyro.getRotationZ());



  Serial.print("accelX: ");
  Serial.println(accelgyro.getAccelerationX());

  Serial.print("accelY: ");
  Serial.println(accelgyro.getAccelerationY());

  Serial.print("accelZ: ");
  Serial.println(accelgyro.getAccelerationZ());
}

#endif


/*--------------------------------------------------------------------------------------------------------------------------------------*/


#ifdef EncoderTest

#include <Encoder.h>
Encoder leftEncoder(6,7);
//Encoder rightEncoder(3,4);

void setup() {
  // put your setup code here, to run once: 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Left Encoder: ");
  Serial.println(leftEncoder.read());
  //Serial.print("Right Encoder: ");
  //Serial.println(rightEncoder.read());
}
#endif

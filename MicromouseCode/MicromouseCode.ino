#include "Macros.h"

#define MotorTest
//ControllerTest
//AccelerometerTest
//EncoderTest
//MotorTest

/*--------------------------------------------------------------------------------------------------------------------------------------*/


#ifdef MotorTest

#include "Motor.cpp"
#include <Encoder.h>

const int ledPin = 0;
const int ledPin1 = 1;
const int d = 10;

int ir1 = 0;
int ir2 = 0;

unsigned int prevIR = 0;
unsigned int currIR = 0;

unsigned long time;
//ENC1A, ENC1B
Encoder leftEncoder(6,7);
// M1R M1F M1E
Motor motor1(16,17,15);

bool stop = false;

void setup() {
  pinMode(ledPin, OUTPUT);
  //pinMode(ledPin1, OUTPUT);
  time = millis();
}

void loop() {
  digitalWrite(ledPin, HIGH);
  
  Serial.println("Running Motor...");
  
  Serial.print("Left Encoder: ");
  Serial.println(leftEncoder.read());

  currIR = analogRead(21);
  if (currIR != prevIR) 
  {
    Serial.print("IR Value: ");
    Serial.println(currIR);
    prevIR = currIR;
  }

  // IR is unreliable right now, so 900 is a good threshhold as it varies 920-980 at start
  if(leftEncoder.read() > 14320 || (currIR < 900 && time-millis() > 11000)){
    stop = true;
    Serial.println("stop condition reached");
  }

  if(!stop){
    motor1.update(1);
  }
  else{
    motor1.update(0);
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

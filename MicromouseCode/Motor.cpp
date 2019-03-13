#include "Macros.h"
#include "Arduino.h"

class Motor{
private:
  int _speed;
  const int _polarity1;
  const int _polarity2;
  const int _pwm;

  int sign(int val){
    if(val < 0){
      return -1;
    }
    return 1;
  }
  
public:
  Motor(int p1, int p2, int pwm): _polarity1(p1), _polarity2(p2), _pwm(pwm){
    _speed = 0;

    pinMode(_pwm,OUTPUT) ;   //we have to set PWM pin as output
    pinMode(_polarity1,OUTPUT) ;  //Logic pins are also set as output
    pinMode(_polarity2,OUTPUT) ;
  }

  void update(float speed){
    if(DEBUG){
      Serial.print("Input Speed: ");
      Serial.print(speed);
    }
    
    if(speed > 1){
      speed = 1;
    }
    else if(speed < -1){
      speed = -1;
    }

    /*int diff = _speed - (int)(speed * 255);

    // Handle gradient
    if(diff != 0){
      if(abs(diff) > MOTOR_GRADIENT){
        _speed += sign(diff)*MOTOR_GRADIENT;
      }
      else{
        _speed += diff;
      }
    }

    Serial.print("Speed: ");
    Serial.println(_speed);*/

    if(speed > 0){
      digitalWrite(_polarity1,HIGH);
      digitalWrite(_polarity2,LOW);
//      Serial.println("Forward");
    }
    else if(speed < 0){
      digitalWrite(_polarity1,LOW);
      digitalWrite(_polarity2,HIGH);
//      Serial.println("Backwards");
    }
    else{
      digitalWrite(_polarity1,LOW);
      digitalWrite(_polarity2,LOW);
//      Serial.println("Stop");
    }
    
//    analogWrite(_pwm,abs(_speed));

//    digitalWrite(_polarity1,LOW);
//    digitalWrite(_polarity2,HIGH);

    if(DEBUG){
      Serial.print("Speed: ");
      Serial.print(speed);
    }
    
    analogWrite(_pwm,abs((int)(speed * 255)));
    
  }
};
















/*
const int pwm = 2 ;  //initializing pin 2 as pwm
const int in_1 = 8 ;
const int in_2 = 9 ;

//For providing logic to L298 IC to choose the direction of the DC motor 

void setup()
{
pinMode(pwm,OUTPUT) ;   //we have to set PWM pin as output
pinMode(in_1,OUTPUT) ;  //Logic pins are also set as output
pinMode(in_2,OUTPUT) ;
}

void loop()
{
//For Clock wise motion , in_1 = High , in_2 = Low

digitalWrite(in_1,HIGH) ;
digitalWrite(in_2,LOW) ;
analogWrite(pwm,255) ;

setting pwm of the motor to 255
we can change the speed of rotaion
by chaning pwm input but we are only
using arduino so we are using higest
value to driver the motor  

//Clockwise for 3 secs
delay(3000) ;     

//For brake
digitalWrite(in_1,HIGH) ;
digitalWrite(in_2,HIGH) ;
delay(1000) ;

//For Anti Clock-wise motion - IN_1 = LOW , IN_2 = HIGH
digitalWrite(in_1,LOW) ;
digitalWrite(in_2,HIGH) ;
delay(3000) ;

//For brake
digitalWrite(in_1,HIGH) ;
digitalWrite(in_2,HIGH) ;
delay(1000) ;
 }

*/

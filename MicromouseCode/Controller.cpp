#include "Macros.h"

#include <Encoder.h>
#include "PID.cpp"
#include "Motor.cpp"
#include "Arduino.h"

class Controller{
private:
  Encoder encoder;
  PID pidController;
  Motor dcMotor;
  int32_t _oldSensorValue;
  int _mp;
  int _encVal;
  
public:
  Controller(float kp, float ki, uint8_t encPin1, uint8_t encPin2, int mPin1, int mPin2, int mpwm):encoder(encPin1,encPin2), pidController(kp, ki), dcMotor(mPin1,mPin2,mpwm){
    //_setPoint = 0;
    //_speed = 0;
    _oldSensorValue = 0;
    _mp = mPin1;
    _encVal = 0;
  }

  /*void setSetPoint(int setPoint){
    _setPoint = setPoint;
  }*/

  void init(){
    encoder.read();
    encoder.read();
    encoder.read();
    encoder.read();
    encoder.read();
    reset();
  }
  
  // setpoint: encoder ticks / second
  // encoder: 12 ticks / revolution of motor
  // motor gear ration: 29.86:1
  // motor output: 29.86 * 12 = 358.32 ticks / revolution of wheel
  // 734.59 ticks / 18 cm (1 square)
  bool update(int setPoint = 0){
    //dcMotor.update(pidController.correction(setPoint - (encoder.read() - _oldSensorValue)/PROGRAM_DELAY_SEC));

    _encVal = _encVal + SIGN(abs(_encVal) - abs(encoder.read()));
    int error = setPoint - encVal;
    
    
    dcMotor.update(pidController.correction(error));
    _oldSensorValue = abs(error);
    

    if(DEBUG){
      Serial.println();
      
      Serial.println("--------------------------------------------------------");
      Serial.print("Motor Pin: ");
      Serial.println(_mp);
      
      Serial.print("Encoder Val: ");
      Serial.println(encVal);

      Serial.print("Error: ");
      Serial.println(error);

      Serial.print("abs(error): ");
      Serial.println(_oldSensorValue);
      Serial.println("--------------------------------------------------------");
      
      Serial.println();
      
    }

    if(_oldSensorValue < 20){
      return true;
    }
    
    return false;
  }

  void reset(){
    pidController.reset();
    encoder.write(0);
    dcMotor.update(0);

    //_setPoint = 0;
    //_speed = 0;
  }
  
private:
  
};

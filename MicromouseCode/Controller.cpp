#include "Macros.h"

#include <Encoder.h>
#include "PID.cpp"
#include "Motor.cpp"

class Controller{
private:
  Encoder encoder;
  PID pidController;
  Motor dcMotor;
  int32_t _oldSensorValue;
  //int _setPoint;
  
public:
  Controller(float kp, float ki, uint8_t encPin1, uint8_t encPin2, int mPin1, int mPin2, int mpwm):encoder(encPin1,encPin2), pidController(kp, ki), dcMotor(mPin1,mPin2,mpwm){
    //_setPoint = 0;
    //_speed = 0;
    _oldSensorValue = 0;
  }

  /*void setSetPoint(int setPoint){
    _setPoint = setPoint;
  }*/

  // setpoint: encoder ticks / second
  // encoder: 12 ticks / revolution of motor
  // motor gear ration: 29.86:1
  // motor output: 29.86 * 12 = 358.32 ticks / revolution of wheel
  // 734.59 ticks / 18 cm (1 square)
  void update(int setPoint = 0){
    dcMotor.update(pidController.correction(setPoint - (encoder.read() - _oldSensorValue)/PROGRAM_DELAY_SEC));
    _oldSensorValue = encoder.read();
  }

  void reset(){
    pidController.reset();
    //_setPoint = 0;
    //_speed = 0;
    encoder.write(0);
  }
  
private:
  
};

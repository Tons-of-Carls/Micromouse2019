#include "Macros.h"

class PID
{
private:
  int _prevError;
  float _intgError;
  const float _kp;
  const float _ki;
  //float _kd;

public:
  PID(const float kp, const float ki): _kp(kp), _ki(ki){
    //_kd = kd;
    _prevError = 0;
    _intgError = 0;
  }
  
  float correction(int error){
    float correction = calculateP(error) + caculateI(error);
    _prevError = error;
    return correction;
  }
  
  void reset(){
    _prevError = 0;
    _intgError = 0;
  }
private:

  float calculateP(int error){
    return error * _kp;
  }
  
  float caculateI(int error){
    _intgError += (_prevError - error) * PROGRAM_DELAY_SEC;
    float temp = _intgError;
    _intgError *= PID_INTEGRAL_DECAY_CONST;
    return temp * _ki;
  }
  
  /*float calculateD(int error){
    return ((error - _prevError)/.02) * _kd;
  }*/
};

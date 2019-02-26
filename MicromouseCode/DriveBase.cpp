/*#include "Macros.h"
#include "Controller.cpp"



//board: 6cm x 7.5cm
//wheels: 2.79cm diameter; 1.27cm wide
//wheel location on board:  
//Calculation hell
class DriveBase{
private:
  Controller leftMotor;
  Controller rightMotor;

public:
  DriveBase():leftMotor(0.1,0.1,1,2,3,4,5), leftMotor(0.1,0.1,6,7,8,9,10){
    
  }

  bool forward(float squares){
    return true;
  }

  // lr: which way to turn relative to micromouse
  //    true: right
  //    false: left
  // ud: which way to turn relative to micromouse
  //    true: right
  //    false: left
  bool diagonal(float squares, bool lr, bool ud){
    return true;
  }

  // lr: which way to turn relative to micromouse
  //    true: right
  //    false: left
  bool pointTurn(bool lr){
    return true;
  }

  // lr: which way to turn relative to micromouse
  //    true: right
  //    false: left
  bool arcTurn(bool lr){
    return true;
  }

  bool fullTurn(){
    return true;
  }

private:
  float arcLength(float radius, int deg = 90){
    return 2*M_PI*radius*(deg/360.0)
  }

  

};*/

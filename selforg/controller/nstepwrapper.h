#ifndef __NSTEPWRAPPER_H
#define __NSTEPWRAPPER_H

#include "invertmotorcontroller.h"
#include "invertmotornstep.h"
#include <selforg/matrix/matrix.h>

class NStepWrapper : public InvertMotorNStep{

public:
  virtual matrix::Matrix model(const matrix::Matrix* x_buffer,int delay, const matrix::Matrix& y){
    return InvertMotorNStep::model(x_buffer,delay,y);
  }

};

#endif

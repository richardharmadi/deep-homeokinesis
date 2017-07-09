#include "stackinvertmotornstep.h"
#include "invertmotornstep.h"

using namespace matrix;
using namespace std;

StackInvertMotorNStep::StackInvertMotorNStep(int buffersize, int nlayers)
    : InvertMotorController(buffersize, "StackInvertMotorNStep", "$Id$")
{
  nlayers = nlayers; // only used for memory reservation
  buffersize = buffersize;
  controllers.reserve(nlayers); // memory reserve
};

void StackInvertMotorNStep::addLayer (InvertMotorNStep cont){
  controllers.push_back(cont);
  actual_nlayer+=1;
}

StackInvertMotorNStep::~StackInvertMotorNStep()
{
  InvertMotorNStep* ptr;
  for(vector<InvertMotorNStep>::iterator it = controllers.begin(); it!= controllers.end();++it){
    ptr = &(*it);
    delete ptr;
  }
  controllers.clear();
}

void StackInvertMotorNStep::init(int sensornumber, int motornumber, RandGen* randGen)
{
  
  assert(sensornumber>=motornumber);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  number_motors  = motornumber;
  number_sensors = sensornumber;
  for(vector<InvertMotorNStep>::iterator it = controllers.begin(); it!= controllers.end();++it){
    it->init(number_sensors,number_motors,randGen);
  }
}

/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void StackInvertMotorNStep::step(const sensor* x_, int number_sensors,
                            motor* y_, int number_motors)
{

  // buffer for previous layer output
  double temp_pred_x[number_sensors];
  double temp_inv_y[number_motors];
  double temp_inv_x[number_sensors]; 
  double ynext_buffer[number_motors]; // new output for next layer (averaged output from reconstructed and controller next layer)
  // learning step layer 1
  controllers[0].step(x_,number_sensors,y_,number_motors);

  if (controllers[0].getStepCounter()>buffersize){
    controllers[0].getPredSensorValue(temp_pred_x);
    //cout << "Xprime 0: " << temp_pred_x[0] << ", " << temp_pred_x[1] << endl;
    for(int i=0;i<controllers.size();i++){
      controllers[i].getPredSensorValue(temp_pred_x);
      controllers[i].getInvMotorValue(temp_inv_y);
      controllers[i].getInvSensorValue(temp_inv_x);

      pred_x.push_back(temp_pred_x);
      inv_y.push_back(temp_inv_y);
      inv_x.push_back(temp_inv_x);

      controllers[i+1].stepNextLayer(inv_x[i],number_sensors,ynext_buffer,number_motors,inv_y[i]);
      ynext.push_back(ynext_buffer); // motor output start from second layer
      }
  }else{ 
    for(int i=0;i<controllers.size();i++){
      controllers[i+1].stepNoLearning(x_,number_sensors,y_,number_motors);
  }
}

/// performs one step without learning. Calulates motor commands from sensor inputs.
void StackInvertMotorNStep::stepNoLearning(const sensor* x, int number_sensors,
                                      motor*  y, int number_motors )
{
  /*
  fillBuffersAndControl(x, number_sensors, y, number_motors);
  // update step counter
  t++;
  */
}

sensor* StackInvertMotorNStep::getPredInputFromLayer(int layernumber){
  return pred_x[layernumber]; 
}

motor* StackInvertMotorNStep::getInvOutputFromLayer(int layernumber){
  return inv_y[layernumber];
}

sensor* StackInvertMotorNStep::getInvInputFromLayer(int layernumber){
  return inv_x[layernumber];
}


bool StackInvertMotorNStep::store(FILE* f) const
{
  /*
  matrix::Matrix Xpred(number_sensors,1,pred_x);
  matrix::Matrix Yinv(number_motors,1,inv_y);
  matrix::Matrix Xinv(number_sensors,1,inv_x); 
  // save matrix values
  Xpred.store(f);
  Yinv.store(f);
  Xinv.store(f);
  Configurable::print(f,0);
  */
  return true;
}

bool StackInvertMotorNStep::restore(FILE* f)
{
  /*
  matrix::Matrix Xpred(number_sensors,1,pred_x);
  matrix::Matrix Yinv(number_motors,1,inv_y);
  matrix::Matrix Xinv(number_sensors,1,inv_x); 
  // save matrix values
  Xpred.restore(f);
  Yinv.restore(f);
  Xinv.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  */
  return true;
}

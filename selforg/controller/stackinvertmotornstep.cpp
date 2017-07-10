#include "stackinvertmotornstep.h"
#include "invertmotornstep.h"
#include <vector>

using namespace matrix;
using namespace std;

StackInvertMotorNStep::StackInvertMotorNStep(int buffersize, int nlayers)
    : InvertMotorController(buffersize, "StackInvertMotorNStep", "$Id$")
{
  // layers only used for memory reservation
  buffer = buffersize;
  controllers.reserve(nlayers); // memory reserve
};

void StackInvertMotorNStep::addLayer (InvertMotorNStep cont){
    //cont = new InvertMotorNStep(); 
  controllers.push_back(cont);
  actual_nlayer+=1;
}

StackInvertMotorNStep::~StackInvertMotorNStep()
{
  InvertMotorNStep* ptr;
  for(vector<InvertMotorNStep>::iterator it = controllers.begin(); it!= controllers.end();++it){
    ptr = &(*it);
    //delete ptr;
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
  // learning step layer 1
  controllers[0].step(x_,number_sensors,y_,number_motors);
  //copy(begin(y_), end(y_), begin(ynext_buffer)); // copy motor output to ynext buffer

  if (controllers[0].getStepCounter()>buffer){
    controllers[0].getPredSensorValue(temp_pred_x);
    controllers[0].getInvMotorValue(temp_inv_y);
    controllers[0].getInvSensorValue(temp_inv_x);

    //cout << "Xprime 0: " << temp_pred_x[0] << ", " << temp_pred_x[1] << endl;
    //cout << "Y1 : " << temp_inv_y[0] << ", " << temp_inv_y[1] << endl;
    //cout << "X1 : " << temp_inv_x[0] << ", " << temp_inv_x[1] << endl;

    vector<double> vector_temp_pred_x(temp_pred_x, temp_pred_x + sizeof(temp_pred_x) / sizeof(sensor)); 
    vector<double> vector_temp_inv_y(temp_inv_y, temp_inv_y + sizeof(temp_inv_y) / sizeof(motor)); 
    vector<double> vector_temp_inv_x(temp_inv_x, temp_inv_x + sizeof(temp_inv_x) / sizeof(sensor));
    if((pred_x.size()==0) && (inv_y.size()==0) && (inv_x.size()==0)){ // if the vector of our output has less value than the controller (size starts from index 0 while controllers 1, so we can use equals)
      pred_x.push_back(vector_temp_pred_x); // we add it to the vector
      inv_y.push_back(vector_temp_inv_y);
      inv_x.push_back(vector_temp_inv_x);
    }else{
      pred_x[0] = vector_temp_pred_x; // if its already there, we update the value for each learning step
      inv_y[0] = vector_temp_inv_y;
      inv_x[0] = vector_temp_inv_x;
    }
    cout << "X1 from vector :" << inv_x[0][0] << ", " << inv_x[0][1] << endl;
    vector<double> vector_ynext(ynext_buffer, ynext_buffer + sizeof(ynext_buffer) / sizeof(motor));
    if(ynext.size()==0){
      ynext.push_back(vector_ynext); // motor output start from second layer
    }else{
      ynext[0] = vector_ynext; //y1 is in index 0, that's why it's called ynext, the index is for the output of next layer
    }
    //controllers[1].stepNextLayer(temp_pred_x,number_sensors,ynext_buffer,number_motors,temp_inv_y);
    /*
    for(size_t i=0;i<controllers.size();i++){
      controllers[i].getPredSensorValue(temp_pred_x);
      controllers[i].getInvMotorValue(temp_inv_y);
      controllers[i].getInvSensorValue(temp_inv_x);

      //cout << "Xprime 0: " << temp_pred_x[0] << ", " << temp_pred_x[1] << endl;
      //cout << "Y1 : " << temp_inv_y[0] << ", " << temp_inv_y[1] << endl;
      //cout << "X1 : " << temp_inv_x[0] << ", " << temp_inv_x[1] << endl;
      
      vector<double> vector_temp_pred_x(temp_pred_x, temp_pred_x + sizeof(temp_pred_x) / sizeof(sensor)); 
      vector<double> vector_temp_inv_y(temp_inv_y, temp_inv_y + sizeof(temp_inv_y) / sizeof(motor)); 
      vector<double> vector_temp_inv_x(temp_inv_x, temp_inv_x + sizeof(temp_inv_x) / sizeof(sensor));
      if((pred_x.size()==i) && (inv_y.size()==i) && (inv_x.size()==i)){ // if the vector of our output has less value than the controller (size starts from index 0 while controllers 1, so we can use equals)
        pred_x.push_back(vector_temp_pred_x); // we add it to the vector
        inv_y.push_back(vector_temp_inv_y);
        inv_x.push_back(vector_temp_inv_x);
      }else{
        pred_x[i] = vector_temp_pred_x; // if its already there, we update the value for each learning step
        inv_y[i] = vector_temp_inv_y;
        inv_x[i] = vector_temp_inv_x;
      }
      //cout << "X1 from vector :" << inv_x[0][0] << ", " << inv_x[0][1] << endl;
      
      controllers[i+1].stepNextLayer(temp_pred_x,number_sensors,ynext_buffer,number_motors,temp_inv_y);
      vector<double> vector_ynext(ynext_buffer, ynext_buffer + sizeof(ynext_buffer) / sizeof(motor));
      if(ynext.size()==i){
        ynext.push_back(vector_ynext); // motor output start from second layer
      }else{
        ynext[i] = vector_ynext; //y1 is in index 0, that's why it's called ynext, the index is for the output of next layer
      }
      }*/
  }else{
    if(controllers.size()>1){
      for(size_t i=0;i<controllers.size();i++){
        controllers[i+1].stepNoLearning(x_,number_sensors,y_,number_motors);
      }
    }
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

vector<sensor> StackInvertMotorNStep::getPredInputFromLayer(int layernumber){
  return pred_x[layernumber]; 
}

vector<motor> StackInvertMotorNStep::getInvOutputFromLayer(int layernumber){
  return inv_y[layernumber];
}

vector<sensor> StackInvertMotorNStep::getInvInputFromLayer(int layernumber){
  return inv_x[layernumber];
}

vector<motor> StackInvertMotorNStep::getAvgOutputFromLayer(int layernumber){
  return ynext[layernumber];
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

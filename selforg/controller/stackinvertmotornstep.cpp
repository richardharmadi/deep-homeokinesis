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
  //controllers.reserve(nlayers); // memory reserve
  for (int i = 0; i < nlayers; ++i) {
    controllers.push_back(new InvertMotorNStep());
    actual_nlayer+=1;
  }   
};

void StackInvertMotorNStep::addLayer (InvertMotorNStep* cont){
    //cont = new InvertMotorNStep(); 
  controllers.push_back(cont);
  actual_nlayer+=1;
}

StackInvertMotorNStep::~StackInvertMotorNStep()
{
  /*InvertMotorNStep* ptr;
  for(vector<InvertMotorNStep>::iterator it = controllers.begin(); it!= controllers.end();++it){
    ptr = &(*it);
    delete ptr;
    }*/
  controllers.clear();
}

void StackInvertMotorNStep::init(int sensornumber, int motornumber, RandGen* randGen)
{
  
  assert(sensornumber>=motornumber);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  number_motors  = motornumber;
  number_sensors = sensornumber;
  for(vector<InvertMotorNStep*>::iterator it = controllers.begin(); it!= controllers.end();++it){
    (*it)->init(number_sensors,number_motors,randGen);
  }
  ynext_buffer = new motor[number_motors];
  //controllers[0]->step(x_,number_sensors,y_,number_motors); // run one step of the first controller to get the value for next layer input
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
  //controllers[0]->step(x_,number_sensors,y_,number_motors); // run one step of the first controller to get the value for next layer input
  for(size_t i=0;i<controllers.size();i++){
    //cout << "step " << i << ": "  << controllers[i]->getStepCounter() << endl;
    if (controllers[i]->getStepCounter()<=buffer){ // if the buffer haven't been filled up
      controllers[i]->stepNoLearning(x_,number_sensors,y_,number_motors);
    }else { // if the buffer has filled
      if (i==0){ // if its the first controller
        controllers[i]->step(x_,number_sensors,y_,number_motors); // run the learning step (it will first re-check if the buffer already filled) (step up)

        // output all the prediction and inverted value (step down)
        controllers[i]->getPredSensorValue(temp_pred_x);
        controllers[i]->getInvMotorValue(temp_inv_y);
        controllers[i]->getInvSensorValue(temp_inv_x);

        //cout << "Xprime 0: " << temp_pred_x[0] << ", " << temp_pred_x[1] << endl;
        //cout << "Y1 : " << temp_inv_y[0] << ", " << temp_inv_y[1] << endl;
        //cout << "X1 : " << temp_inv_x[0] << ", " << temp_inv_x[1] << endl;
      }else{ // if it's the next controller
        controllers[i]->step(temp_inv_x,number_sensors,ynext_buffer,number_motors); // we use previous inv_x as input, and put the out to a new ynext_buffer
        updateSensorValue(i,ynext_buffer); // use this previous output to generate new prediction input from previous model
        updateMotorValue(i,ynext_buffer); // we update the previous controller i-1 output buffer value with this controller new generated output
        vector<double> vector_ynext; // initiate temporary vector
        for(int j=0;j<number_motors;j++){ // fill buffer to vector
          vector_ynext.push_back(ynext_buffer[j]);
        }
        if(ynext.size()<i){ // if element in the vector is less than controller number, then push back
          ynext.push_back(vector_ynext); // motor output start from second layer
        }else{
          ynext[i-1] = vector_ynext; //y1 is in index 0, that's why it's called ynext, the index is for the output of next layer
        }
        //cout << "Y1 " << ynext[0][0] << ", " << ynext[0][1] <<endl;
        
        // output all the prediction and inverted value (step down)
        controllers[i]->getPredSensorValue(temp_pred_x);
        controllers[i]->getInvMotorValue(temp_inv_y);
        controllers[i]->getInvSensorValue(temp_inv_x);
      }
      // put the output value into into a temporary vector, for being pushed to our vector of vector output later
      vector<double> vector_temp_pred_x(temp_pred_x, temp_pred_x + sizeof(temp_pred_x) / sizeof(sensor)); 
      vector<double> vector_temp_inv_y(temp_inv_y, temp_inv_y + sizeof(temp_inv_y) / sizeof(motor)); 
      vector<double> vector_temp_inv_x(temp_inv_x, temp_inv_x + sizeof(temp_inv_x) / sizeof(sensor));
      //if(controllers[i]->getStepCounter()==buffer+1){
      if((pred_x.size()<=i) && (inv_y.size()<=i) && (inv_x.size()<=i)){ // if the vector of our output has less value than the controller (size starts from index 0 while controllers 1, so we can use equals)
        pred_x.push_back(vector_temp_pred_x); // we add it to the vector
        inv_y.push_back(vector_temp_inv_y);
        inv_x.push_back(vector_temp_inv_x);
      }else{
        pred_x[i] = vector_temp_pred_x; // if its already there, we update the value for each learning step
        inv_y[i] = vector_temp_inv_y;
        inv_x[i] = vector_temp_inv_x;
        //cout << "X1 from vector :" << inv_x[0][0] << ", " << inv_x[0][1] << endl;
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

void StackInvertMotorNStep::updateMotorValue(int layernumber, motor* y_){ // top down control mode, update previous layer output with next layer output (averaged)
  int n = controllers[layernumber]->getStepCounter(); // get number of step
  matrix::Matrix yupdate (number_motors,1,y_);
  matrix::Matrix ybuffer = controllers[layernumber-1]->getYbuffer(n % buffersize);
  ybuffer += yupdate; // add the previous controller buffer output value of this step with this layer output value
  ybuffer *= 0.5; // avg them
  controllers[layernumber-1]->setYbuffer((n % buffersize),ybuffer);
}

void StackInvertMotorNStep::updateSensorValue(int layernumber, motor* y_){ // top down control mode, update previous layer input with next layer virtual input (== prediction input produced by next layer output, averaged)
  int n = controllers[layernumber]->getStepCounter(); // get number of step
  matrix::Matrix yupdate (number_motors,1,y_);
  controllers[layernumber-1]->setXbufferUpdate((n % buffersize),yupdate);
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

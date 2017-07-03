#include "stackinvertmotornstep.h"
#include "nstepwrapper.h"

using namespace matrix;
using namespace std;

StackInvertMotorNStep::StackInvertMotorNStep(int buffersize, int nlayers)
    : InvertMotorController(buffersize, "StackInvertMotorNStep", "$Id$")
{
  nlayers = nlayers;
  controllers.reserve(nlayers);
};

void StackInvertMotorNStep::addLayer(NStepWrapper cont){
  controllers.push_back(cont);
}

StackInvertMotorNStep::~StackInvertMotorNStep()
{
  NStepWrapper pd;
  for(vector<NStepWrapper>::iterator it = controllers.begin(); it!= controllers.end();++it){
    pd = *it;
    delete pd;
  }
  controllers.clear();
}

void StackInvertMotorNStep::init(int sensornumber, int motornumber, RandGen* randGen)
{
  /*
  assert(sensornumber>=motornumber);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak

  number_motors  = motornumber;
  number_sensors = sensornumber;
  if(conf.numberContext == 0) // we just declare all sensors as context
    conf.numberContext = number_sensors;

  //  z.set(number_motors,1);
  A.set(number_sensors, number_motors);
  // S gets context sensors
  if (conf.useS) S.set(number_sensors, conf.numberContext);
  // S gets first and second derivative of context sensors
  if (conf.useSD) SD.set(number_sensors, number_sensors*2);
  C.set(number_motors,  number_sensors);
  H.set(number_motors,  1);
  B.set(number_sensors, 1);
  R.set(number_motors, number_motors);
  SmallID.set(number_motors, number_motors);
  SmallID.toId();
  SmallID *= 0.001;

  xsi.set(number_sensors,1);
  xsi_norm=0;
  xsi_norm_avg=0.2;
  pain=0;
  sensorweights.set(number_sensors,1);
  sensorweights.toMapP(1, constant); // fill it with ones

  BNoiseGen = new WhiteUniformNoise();
  BNoiseGen->init(number_sensors);
  YNoiseGen = new ColorUniformNoise(0.1);
  //  YNoiseGen = new WhiteUniformNoise();
  YNoiseGen->init(number_motors);

  A.toId(); // set A to identity matrix;
  //  if (conf.useS) S.mapP(randGen, random_minusone_to_one)*0.01; // set S to small random matrix;
  // if (conf.useSD) S.mapP(randGen, random_minusone_to_one)*0.01; // set SD to small random matrix;

  if(conf.initialC.getM() != number_motors || conf.initialC.getN() != number_sensors ){
    if(!conf.initialC.isNulltimesNull()) cerr << "dimension of Init C are not correct, default is used! \n";
    // initialise the C matrix with identity + noise scaled to cInit value
    C = ((C^0) + C.mapP(randGen, random_minusone_to_one) * conf.cNonDiag) * conf.cInit;
  }else{
    C=conf.initialC; // use given matrix
  }
  cnondiagabs = conf.cNonDiagAbs;

  x_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  eta_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++)
  {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
    eta_buffer[k].set(number_motors,1);
  }
  y_teaching.set(number_motors, 1);

  zero_eta.set(number_motors, 1);
  v=zero_eta;

  t_rand = int(randGen->rand()*10);
  initialised = true;
  */
}

/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void StackInvertMotorNStep::step(const sensor* x_, int number_sensors,
                            motor* y_, int number_motors)
{
  /*
  fillBuffersAndControl(x_, number_sensors, y_, number_motors);
  if(t>buffersize)
  {
    int delay = max(int(s4delay)-1,0);
    calcXsi(delay);            // calculate the error (use delayed y values)
    calcEtaAndBufferIt(delay);
    // learn controller with effective input/output
    learnController(delay);

    // learn Model with actual sensors and with effective motors;
    learnModel(delay);
  }
  // update step counter
  t++;

//   // Georg: This a very special case. Should be removed! Make a copy to the precise simulation
  if (cfactor!=1)
  {
    C = (C^0) *cfactor;
  }
  if (cnondiagabs!=0)
  {
    C.val(0,1)=cnondiagabs;
    C.val(1,0)=cnondiagabs;
  }
  if (cdiagabs!=0)
  {
    C.val(0,0)=cdiagabs;
    C.val(1,1)=cdiagabs;
  }
  */
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

//Matrix& StackInvertMotorNStep::getC(int layernumber){
//
//}

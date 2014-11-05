/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Simon Smith <artificialsimon at ed dot ac fot uk>                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 ***************************************************************************/

#include "soxmf.h"
using namespace matrix;
using namespace std;

SoxMF::SoxMF(const SoxMFConf& conf)
  : AbstractController("SoxMF", "1.1"),
    conf(conf)
{
  this->model = NULL;
  this->nameModel = "linear";
  //assert(this->model->getInitialized());
  constructor();
}


SoxMF::SoxMF(const SoxMFConf& conf,const std::string& nameModel, InvertableModel* model)
  : AbstractController("SoxMF", "1.1"),
    conf(conf)
{
  this->model = model;
  this->nameModel = nameModel;
  //assert(this->model->getInitialized());
  constructor();
}


SoxMF::SoxMF(double init_feedback_strength, bool useExtendedModel, bool useTeaching )
  : AbstractController("SoxMF", "1.1"),
    conf(getDefaultConf()){

  conf.initFeedbackStrength = init_feedback_strength;
  //conf.useExtendedModel     = useExtendedModel;
  conf.useTeaching          = useTeaching;
  constructor();
}

void SoxMF::constructor(){
  t=0;

  addParameterDef("Logarithmic", &loga, false, "whether to use logarithmic error");
  addParameterDef("epsC", &epsC, 0.1,     0,5, "learning rate of the controller");
  addParameterDef("sense",  &sense,    1, 0.2,5,      "sensibility");
  addParameterDef("creativity", &creativity, 0, 0, 1, "creativity term (0: disabled) ");
  addParameterDef("damping",   &damping,     0.00001, 0,0.01, "forgetting term for model");
  addParameterDef("harmony",    &harmony,    0, 0,0.1,
                  "dynamical harmony between internal and external world");
  addParameterDef("pseudo",   &pseudo   , 0  ,
    "type of pseudo inverse: 0 moore penrose, 1 sensor space, 2 motor space, 3 special");
  if(!conf.onlyMainParameters){
    addParameter("s4avg", &conf.steps4Averaging, 1, buffersize-1,
        "smoothing (number of steps)");
    addParameter("s4delay", &conf.steps4Delay,   1, buffersize-1,
        "delay  (number of steps)");
    swtich(nameModel) {
      case "linear":
        addParameter("factorS", &conf.factorS,  0, 2,
            "factor for learning rate for S");
        addParameter("factorb", &conf.factorb,  0, 2,
            "factor for learning rate for b");
        break;
    }
    addParameter("factorh", &conf.factorh,  0, 2,
        "factor for learning rate for h");
  }


  gamma=0;
  if(conf.useTeaching){
    addParameterDef("gamma",  &gamma,    0.01, 0, 1, "guidance factor (teaching)");
    addInspectableMatrix("y_G", &y_teaching, "teaching signal at motor neurons");
  }

  addInspectableMatrix("C", &C, conf.someInternalParams, "controller matrix");
  addInspectableMatrix("L", &L, conf.someInternalParams, "Jacobi matrix");
  addInspectableMatrix("h", &h, conf.someInternalParams, "controller bias");

  addInspectableMatrix("v_avg", &v_avg, "input shift (averaged)");

  addInspectableValue("xi", &snXi, "xi square norm");
  addInspectableValue("v_norm", &vNormSqr, "v square norm");
  addInspectableMatrix("x_hat", &x_hat, "x predicted by model");
  
  intern_isTeaching = false;

  switch(nameModel) {
    case "linear":
      break;
    default:
      addInspectable(model);
      break;
   }
};

SoxMF::~SoxMF(){
}


void SoxMF::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak

  number_sensors= sensornumber;
  number_motors = motornumber;
  C.set(number_motors, number_sensors);
  h.set(number_motors, 1);
  L.set(number_sensors, number_sensors);
  v_avg.set(number_sensors, 1);
  C_native.set(number_motors, number_sensors);

  C.toId(); // set a to identity matrix;
  C*=conf.initFeedbackStrength;

  C_native.toId();
  C_native*=1.2;

  y_teaching.set(number_motors, 1);

  x.set(number_sensors,1);
  x_smooth.set(number_sensors,1);
  x_hat.set(number_sensors, 1);
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
  }

  accXi = .0;
  accV = .0;
}


// performs one step (includes learning). Calculates motor commands from sensor inputs.
void SoxMF::step(const sensor* x_, int number_sensors,
                       motor* y_, int number_motors){
  stepNoLearning(x_, number_sensors, y_, number_motors);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // learn controller and model
    learn();

  // update step counter
  t++;
};


// performs one step without learning. Calulates motor commands from sensor inputs.
void SoxMF::stepNoLearning(const sensor* x_, int number_sensors,
                                 motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors
      && (unsigned)number_motors <= this->number_motors);

  x.set(number_sensors,1,x_); // store sensor values

  // averaging over the last s4avg values of x_buffer
  conf.steps4Averaging = ::clip(conf.steps4Averaging,1,buffersize-1);
  if(conf.steps4Averaging > 1)
    x_smooth += (x - x_smooth)*(1.0/conf.steps4Averaging);
  else
    x_smooth = x;

  x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value

  // calculate controller values based on current input values (smoothed)
  Matrix y =   (C*(x_smooth + (v_avg*creativity)) + h).map(g);
  // Put new output vector in ring buffer y_buffer
  y_buffer[t%buffersize] = y;

  // convert y to motor*
  y.convertToBuffer(y_, number_motors);

  // update step counter
  t++;
};


// learn values h,C, and Model
void SoxMF::learn(){
  const Matrix dumb;
  const Matrix R = model->response(dumb);

  // the effective x/y is (actual-steps4delay) element of buffer
  int s4delay = ::clip(conf.steps4Delay,1,buffersize-1);
  const Matrix& x       = x_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& y_creat = y_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& x_fut   = x_buffer[t% buffersize]; // future sensor (with respect to x,y)

  x_hat                 = model->learn(y_creat, x_fut);
  const Matrix& xi      = x_fut - x_hat;
  snXi                  = xi.norm_sqr();
  accXi                 += snXi;

  const Matrix& z       = (C * (x) + h);
  const Matrix& y       = z.map(g);
  const Matrix& g_prime = z.map(g_s);
  //L = A * (C & g_prime) + S; // Simon: why S is outside in the linear case?
  L = R * (C & g_prime);

  const Matrix& Lplus  = pseudoInvL(L,R,C);
  const Matrix& v      = Lplus * xi;
  const Matrix& chi    = (Lplus^T) * v;
  const Matrix& mu     = ((R^T) & g_prime) * chi;
  const Matrix& epsrel = (mu & (C * v)) * (sense * 2);
  const Matrix& v_hat  = v + x * harmony;

  vNormSqr  = v.norm_sqr();
  accV      += vNormSqr;
  v_avg     += (v - v_avg) * .1;

  double EE = 1.0;
  if(loga){
    EE = .1/(v.norm_sqr() + .001); // logarithmic error (E = log(v^T v))
  }

  // Controller learning
  if(epsC > .0){
    C += (( mu * (v_hat^T)
            - (epsrel & y) * (x^T))   * (EE * epsC) ).mapP(.05, clip);
    if(damping)
      C += (((C_native-C).map(power3))*damping      ).mapP(.05, clip);
    h += ((mu*harmony - (epsrel & y)) * (EE * epsC * conf.factorh) ).mapP(.05, clip);

    if(intern_isTeaching && gamma > 0){
      // scale of the additional terms
      const Matrix& metric = (R^T) * Lplus.multTM() * R;

      const Matrix& y      = y_buffer[(t-1)% buffersize];
      const Matrix& xsi    = y_teaching - y;
      const Matrix& delta  = xsi.multrowwise(g_prime);
      C += ((metric * delta*(x^T) ) * (gamma * epsC)).mapP(.05, clip);
      h += ((metric * delta)        * (gamma * epsC * conf.factorh)).mapP(.05, clip);
      // after we applied teaching signal it is switched off until new signal is given
      intern_isTeaching    = false;
    }
  }

};


Matrix SoxMF::pseudoInvL(const Matrix& L, const Matrix& A, const Matrix& C){
  if(pseudo == 0){
    return L.pseudoInverse();
  }else{
    const Matrix& P = pseudo==1 || pseudo==2 ? A^T : C;
    const Matrix& Q = pseudo==1              ? C^T : A;
    return Q *((P * L * Q)^(-1)) * P;
  }
}


double SoxMF::getXiAverage() {
  if (t > 1)
    return accXi / (t-1);
  else if (t == 1)
    return accXi;
  return .0;
};

double SoxMF::getVaverage() {
  if (t > 1)
    return accV / (t-1);
  else if (t==1)
    return accV;
  return .0;
}

matrix::Matrix SoxMF::getC(){
  return C;
}

void SoxMF::setC(const matrix::Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}

matrix::Matrix SoxMF::geth(){
  return h;
}

void SoxMF::seth(const matrix::Matrix& _h){
  assert(h.getM() == _h.getM() && h.getN() == _h.getN());
  h=_h;
}



void SoxMF::setMotorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_motors && teaching.getN() == 1);
  // Note: through the clipping the otherwise effectless
  //  teaching with old motor value has now an effect,
  //  namely to drive out of the saturation region.
  y_teaching= teaching.mapP(0.95,clip);
  intern_isTeaching=true;
}

void SoxMF::setSensorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_sensors && teaching.getN() == 1);
  // calculate the y_teaching,
  // that belongs to the distal teaching value by the inverse model.
  //y_teaching = (A.pseudoInverse() * (teaching-b)).mapP(0.95, clip);
  //intern_isTeaching=true;
}

matrix::Matrix SoxMF::getLastMotorValues(){
  return y_buffer[(t-1+buffersize)%buffersize];
}

matrix::Matrix SoxMF::getLastSensorValues(){
  return x_buffer[(t-1+buffersize)%buffersize];
}

/* stores the controller values to a given file. */
bool SoxMF::store(FILE* f) const{
  // save matrix values
  C.store(f);
  h.store(f);
  //A.store(f);
  //b.store(f);
  //S.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool SoxMF::restore(FILE* f){
  // save matrix values
  C.restore(f);
  h.restore(f);
  //A.restore(f);
  //b.restore(f);
  //S.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

void SoxMF::motorBabblingStep(const sensor* x_, int number_sensors,
                            const motor* y_, int number_motors){
}



/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICESNE:                                                              *
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

#include "soxmm.h"
using namespace matrix;
using namespace std;

SoxMM::SoxMM(const SoxMMConf& conf, Models m)
  : AbstractController("SoxMM", "1.1"),
    conf(conf)
{
  this->model = NULL;
  modelName = m;
  constructor();
}


SoxMM::SoxMM(HomeokineticModel* model, Models m, const SoxMMConf& conf) 
  : AbstractController("SoxMM", "1.1"),
    conf(conf)
{
  this->model = model;
  modelName = m;
  constructor();
}


SoxMM::SoxMM(double init_feedback_strength, bool useExtendedModel, bool useTeaching )
  : AbstractController("SoxMM", "1.1"),
    conf(getDefaultConf()){

  conf.initFeedbackStrength = init_feedback_strength;
  conf.useExtendedModel     = useExtendedModel;
  conf.useTeaching          = useTeaching;
  constructor();
}

void SoxMM::constructor(){
  t=0;

  addParameterDef("Logarithmic", &loga, false, "whether to use logarithmic error");
  addParameterDef("epsC", &epsC, 0.1,     0,5, "learning rate of the controller");
  addParameterDef("sense",  &sense,    1, 0.2,5,      "sensibility");
  addParameterDef("creativity", &creativity, 0, 0, 1, "creativity term (0: disabled) ");
  addParameterDef("damping",   &damping,     0.00001, 0,0.01, "forgetting term for model");
  addParameterDef("causeaware", &causeaware, conf.useExtendedModel ? 0.01 : 0 , 0,0.1,
                  "awarness of controller influences");
  addParameterDef("harmony",    &harmony,    0, 0,0.1,
                  "dynamical harmony between internal and external world");
  addParameterDef("pseudo",   &pseudo   , 0  ,
    "type of pseudo inverse: 0 moore penrose, 1 sensor space, 2 motor space, 3 special");

  if(!conf.onlyMainParameters){
    addParameter("s4avg", &conf.steps4Averaging, 1, buffersize-1,
                    "smoothing (number of steps)");
    addParameter("s4delay", &conf.steps4Delay,   1, buffersize-1,
                    "delay  (number of steps)");
    addParameter("factorS", &conf.factorS,  0, 2,
                    "factor for learning rate for S");
    addParameter("factorb", &conf.factorb,  0, 2,
                    "factor for learning rate for b");
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

  intern_isTeaching = false;

  /********************************/
  switch(modelName) {
    case LINEAR:
      addInspectableMatrix("R", &R, conf.someInternalParams, "linear response matrix");
    case LINEAR_REGRESSION:
      addParameterDef("epsA", &epsA, 0.1,     0,5, "learning rate of the model");
    case LWR:
      addInspectableMatrix("A", &A, conf.someInternalParams, "model matrix");
      addInspectableMatrix("w", &w, conf.someInternalParams, "w matrix");
      addParameterDef("sigma_sqr_LWR", &sigma_sqr_LWR, 0.2,     0.01,1, "sigma square for distance LWR");
      if(conf.useExtendedModel)
        addInspectableMatrix("S", &S, conf.someInternalParams, "model matrix (sensor branch)");
      addInspectableMatrix("b", &b, conf.someInternalParams, "model bias");
      addInspectableMatrix("xi_m", &xi, conf.someInternalParams, "xi error matrix");
      break;
    case ESN_MODEL:
      addInspectable(model);
      addConfigurable(model);
      break;
  }
  addInspectableMatrix("x_hat", &x_hat, conf.someInternalParams, "x hat");
};


SoxMM::~SoxMM(){
  /* ERROR FREQUENCY */
  if (hist) {
    //gsl_histogram_fprintf (stdout, hist, "%g", "%g");
    //for (int i = 0; i < N_BINS; i++) {
      //cout << i << " " << (MAX_XI / N_BINS * i) << " " << gsl_histogram_get(hist, i) << endl;
    //}
  }
  /* MOTOR SPACE FREQUENY */
  /*
  for (int j = 0; j < N_BINS; j++) {
    cout << (2.4 / N_BINS * (double)j - 1.2) 
      << " " << gsl_histogram_get(histogram[0], j) 
      << " " << gsl_histogram_get(histogram[1], j) 
      << " " << gsl_histogram_get(histogram[2], j) 
      << " " << gsl_histogram_get(histogram[3], j) 
      << endl;
  }
  */
}


void SoxMM::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak

  number_sensors= sensornumber;
  number_motors = motornumber;
  if (modelName == LINEAR_REGRESSION || modelName == LWR) {
    number_sensors++;
  }
  C.set(number_motors, number_sensors);
  h.set(number_motors, 1);
  L.set(number_sensors, number_sensors);
  v_avg.set(number_sensors, 1);
  C_native.set(number_motors, number_sensors);

  C.toId(); // set a to identity matrix;
  C*=conf.initFeedbackStrength;

  // if motor babbling is used then this is overwritten
  C_native.toId();
  C_native *= 1.2;

  y_teaching.set(number_motors, 1);

  x.set(number_sensors, 1);
  xi.set(number_sensors, 1);
  x_hat.set(number_sensors, 1);
  x_smooth.set(number_sensors,1);
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
  }

  accXi = .0;
  accV = .0;
  /* ERROR FREQUENCY */
  hist = gsl_histogram_alloc(N_BINS); // n is number of bins
  gsl_histogram_set_ranges_uniform(hist, .0, MAX_XI); // minimum and maximum value

  /* MOTOR SPACE FREQUENCY */
  /*
  for (int i = 0; i < 4; i++) {
    histogram[i] = gsl_histogram_alloc(N_BINS); // n is number of bins
    gsl_histogram_set_ranges_uniform(histogram[i], -1.2, 1.2); // minimum and maximum value
  }
  */

  /*******************************************/
  switch(modelName){
    case LINEAR_REGRESSION:
      P.set(number_sensors, number_sensors);
      P_t_minus_1.set(number_sensors, number_sensors);
      Q.set(number_motors, number_motors);
      Q_t_minus_1.set(number_motors, number_motors);
      P.toId();
      Q.toId();
      P *= 1. / .001;
      Q *= 1. / .001;
      // no break
    case LINEAR:
      R.set(number_sensors, number_sensors);
      A.set(number_sensors, number_motors);
      S.set(number_sensors, number_sensors);
      b.set(number_sensors, 1);
      A_native.set(number_sensors, number_motors);
      A.toId(); // set a to identity matrix;
      if (conf.useExtendedModel) {
        S.toId();
        S *= .05;
      }
      A_native.toId();
      break;
    case ESN_MODEL: 
      model->init(number_sensors, number_motors, number_sensors, 1., randGen); 
      break;
    case LWR:
      X.set(buffersize_LWR, number_sensors);
      Y.set(buffersize_LWR, number_motors);
      w.set(buffersize_LWR, 1);
      A.set(number_sensors, number_motors);

      R.set(number_sensors, number_sensors);
      S.set(number_sensors, number_sensors);
      b.set(number_sensors, 1);
      A_native.set(number_sensors, number_motors);
      A.toId(); // set a to identity matrix;
      if (conf.useExtendedModel) {
        S.toId();
        S *= .05;
      }
      A_native.toId();
      break;
  }
  if (modelName == LINEAR_REGRESSION || modelName == LWR) number_sensors--;
}

matrix::Matrix SoxMM::getA(){
  return A;
}

void SoxMM::setA(const matrix::Matrix& _A){
  assert(A.getM() == _A.getM() && A.getN() == _A.getN());
  A=_A;
}

matrix::Matrix SoxMM::getC(){
  return C;
}

void SoxMM::setC(const matrix::Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}

matrix::Matrix SoxMM::geth(){
  return h;
}

void SoxMM::seth(const matrix::Matrix& _h){
  assert(h.getM() == _h.getM() && h.getN() == _h.getN());
  h=_h;
}

// performs one step (includes learning). Calculates motor commands from sensor inputs.
void SoxMM::step(const sensor* x_, int number_sensors,
                       motor* y_, int number_motors){
  stepNoLearning(x_, number_sensors, y_, number_motors);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // learn controller and model
  if(epsC!=0 || epsA!=0)
    learn();

  // update step counter
  t++;
};


// performs one step without learning. Calulates motor commands from sensor inputs.
void SoxMM::stepNoLearning(const sensor* x_, int number_sensors,
                                 motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors
         && (unsigned)number_motors <= this->number_motors);

  switch (modelName) {
    case LINEAR:
    case ESN_MODEL:
      x.set(number_sensors,1,x_); // store sensor values
      break;
    case LINEAR_REGRESSION:
    case LWR:
      Matrix x_temp, x_temp2;
      x_temp.set(1, 1);
      x_temp.val(0, 0) = 1.;
      x_temp2.set(number_sensors, 1, x_); // store sensor values
      x_temp2.toAbove(x_temp);
      x = x_temp2;
      break;
   }


  // averaging over the last s4avg values of x_buffer
  conf.steps4Averaging = ::clip(conf.steps4Averaging,1,buffersize-1);
  if(conf.steps4Averaging > 1)
    x_smooth += (x - x_smooth)*(1.0/conf.steps4Averaging);
  else
    x_smooth = x;

  x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value

  // calculate controller values based on current input values (smoothed)
  Matrix y = (C * (x_smooth + (v_avg * creativity)) + h).map(g);

  // Put new output vector in ring buffer y_buffer
  y_buffer[t % buffersize] = y;

  // convert y to motor*
  y.convertToBuffer(y_, number_motors);

  if(modelName == LWR) {
    int buffer_pos = (t + buffersize_LWR) % buffersize_LWR;
    for(int i = 0; i <= number_sensors; i++) {
      X.val(buffer_pos, i) = x.val(i, 0);
    }
    for(int j = 0; j < number_motors; j++)
      Y.val(buffer_pos, j) = y.val(j, 0);
  }


  /* MOTOR SPACE FREQUENCY */
  /*
  for (int i = 0; i < 4; i++) 
    gsl_histogram_increment(histogram[i], y_[i]);
  */

  // update step counter
  t++;
};


void SoxMM::motorBabblingStep(const sensor* x_, int number_sensors,
                            const motor* y_, int number_motors){
}


Matrix SoxMM::pseudoInvL(const Matrix& L, const Matrix& A, const Matrix& C){
  if(pseudo == 0){
    return L.pseudoInverse();
  }else{
    const Matrix& P = pseudo==1 || pseudo==2 ? A^T : C;
    const Matrix& Q = pseudo==1              ? C^T : A;
    return Q *((P * L * Q)^(-1)) * P;
  }
}


// learn values h,C and models 
void SoxMM::learn(){


  // the effective x/y is (actual-steps4delay) element of buffer
  int s4delay = ::clip(conf.steps4Delay, 1, buffersize - 1);
  const Matrix& x       = x_buffer[(t - max(s4delay, 1) + buffersize) % buffersize];
  const Matrix& y_creat = y_buffer[(t - max(s4delay, 1) + buffersize) % buffersize];
  const Matrix& x_fut   = x_buffer[t% buffersize]; // future sensor (with respect to x,y)
  Matrix x_prime;

  /*******************************************/
  switch(modelName) {
    case LINEAR:
      x_hat = A * y_creat + b + S * x;
      xi = x_fut  - x_hat; // (A * y_creat + b + S * x); // creativity
      break;
    case ESN_MODEL:
      x_hat = model->process(x, y_creat);
      xi = x_fut - x_hat; // model->process(x, y_creat);
      break;
    case LINEAR_REGRESSION:
      x_hat = A * y_creat + S * x;
      x_hat.val(number_sensors, 0) = 1.;
      xi = x_fut  - x_hat; // (A * y_creat + b + S * x); // creativity
      break;
    case LWR: // learn has to be made before prediction
      for(int i = 0; i < buffersize_LWR; i++) {
        double dd = 0;
        double d;
        for (int j = 0; j < number_sensors; j++) {
          d  = pow(x.val(j, 0) - X.val(i, j), 2);
          dd += d;
        }
        dd = sqrt(dd);
        w.val(i, 0) = exp(-dd/(2 * sigma_sqr_LWR));
      }
      const Matrix WY = Y.multrowwise(w);
      const Matrix WX = X.multrowwise(w);
      const Matrix YTWY = (Y^T) * WY;
      const Matrix invYTWY = YTWY.pseudoInverse();
      A = (invYTWY * (Y^T) * WX)^T;

      x_hat = ((y_creat^T) * (A^T))^T;
      xi = x_fut  - x_hat; 
      break;
  }

  const Matrix z       = (C * x + h); // here no creativity
  const Matrix y       = z.map(g);
  const Matrix g_prime = z.map(g_s);

  Matrix eta;
  Matrix y_hat;
  Matrix Lplus;
  /*******************************************/
  switch(modelName) {
    case LINEAR: 
    case LINEAR_REGRESSION:
    case LWR:
      L      = A * (C & g_prime) + S;
      R      = A * C+S; // this is only used for visualization
      eta    = A.pseudoInverse() * xi;
      y_hat  = y + eta * causeaware;
      Lplus  = pseudoInvL(L,A,C);
      break;
    case ESN_MODEL: 
      L     = model->response(C & g_prime);
      Lplus = L.pseudoInverse();
      break;
  }


  const Matrix v      = Lplus * xi;
  const Matrix chi    = (Lplus^T) * v;

  snXi = v.norm_sqr();
  if (t > 6000) {
    accXi += snXi;
    /* ERROR FREQUENCY */
    gsl_histogram_increment(hist, snXi);
  }
  Matrix mu;
  Matrix upsilon;
  upsilon.set(number_motors, 1); // not sure about this MxN (too tired to think)
  upsilon.toZero();
  Matrix epsrel, epsrel2;
  /*******************************************/
  switch(modelName) {
    case LINEAR: 
    case LINEAR_REGRESSION:
    case LWR:
      mu     = ((A^T) & g_prime) * chi;
      break;
    case ESN_MODEL: 
      mu      = ((model->response_controller()^T)  & g_prime) * chi;
      upsilon = ((model->response_controller2()^T) & g_prime) * chi;
      break;
  }

  epsrel  = (mu      & (C * v)) * (sense * 2);
  epsrel2 = (upsilon & (C * v)) * (sense * 2);
  const Matrix v_hat = v + x * harmony;

  vNormSqr  = v.norm_sqr();
  accV      += vNormSqr;
  v_avg += ( v  - v_avg ) *.1;

  double EE = 1.0;
  if(loga){
    EE = .1/(v.norm_sqr() + .001); // logarithmic error (E = log(v^T v))
  }
  /*******************************************/
  switch(modelName) {
    case LINEAR: 
      if(epsA > 0){
        double epsS = epsA * conf.factorS;
        double epsb = epsA * conf.factorb;
        A   += (xi * (y_hat^T) * epsA).mapP(0.1, clip);
        if(damping)
          A += (((A_native - A).map(power3)) * damping).mapP(0.1, clip);
        if(conf.useExtendedModel)
          S += (xi * (x^T) * (epsS)+ (S *  -damping*10) ).mapP(0.1, clip);
        b   += (xi * (epsb) + (b *  -damping)    ).mapP(0.1, clip);
      }
      break;
    case ESN_MODEL :
      model->learn(x, y_creat, x_fut);
      break;
    case LINEAR_REGRESSION:
      if(epsA > 0){
        double epsS = epsA * conf.factorS;

        if(conf.useExtendedModel) {
          S += (xi * (x^T) * P_t_minus_1 * epsS).mapP(.1, clip);
          double denominator_P = 1 + ((x^T) * P * x).val(0, 0);
          P_t_minus_1 = P;
          P -= (P * x * (x^T) * P) * (1. / denominator_P);
        }

        A += (xi * (y_hat^T) * Q_t_minus_1 * epsA).mapP(.1, clip);
        double denominator_Q = 1 + ((y_hat^T) * Q * y_hat).val(0, 0);
        Q_t_minus_1 = Q;
        Q -= (Q * y_hat * (y_hat^T) * Q) * (1. / denominator_Q);
        if(damping)
          A += (((A_native - A).map(power3)) * damping).mapP(0.1, clip);
      }
      break;
  }
  if(epsC > 0){
    C += ((mu * (v_hat^T)
           - (epsrel  & y) * (x^T)
           - (epsrel2 & y) * (x^T)) * (EE * epsC)).mapP(.05, clip);
    if(damping)
      C += (((C_native - C).map(power3)) * damping).mapP(.05, clip);
    if (modelName != LINEAR_REGRESSION) // Integrated in C for LR
      h += ((mu*harmony - (epsrel & y) - (epsrel2 & y)) * (EE * epsC * conf.factorh) ).mapP(.05, clip);

    if(intern_isTeaching && gamma > 0){
      // scale of the additional terms
      const Matrix& metric = (A^T) * Lplus.multTM() * A;

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


double SoxMM::getXiAverage() {
  if (t > 1)
    return accXi / (t-1);
  else if (t == 1)
    return accXi;
  return .0;
};


double SoxMM::getVaverage() {
  if (t > 1)
    return accV / (t-1);
  else if (t==1)
    return accV;
  return .0;
}

void SoxMM::setMotorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_motors && teaching.getN() == 1);
  // Note: through the clipping the otherwise effectless
  //  teaching with old motor value has now an effect,
  //  namely to drive out of the saturation region.
  y_teaching= teaching.mapP(0.95,clip);
  intern_isTeaching=true;
}

void SoxMM::setSensorTeaching(const matrix::Matrix& teaching){
  assert(teaching.getM() == number_sensors && teaching.getN() == 1);
  // calculate the y_teaching,
  // that belongs to the distal teaching value by the inverse model.
  y_teaching = (A.pseudoInverse() * (teaching-b)).mapP(0.95, clip);
  intern_isTeaching=true;
}

matrix::Matrix SoxMM::getLastMotorValues(){
  return y_buffer[(t-1+buffersize)%buffersize];
}

matrix::Matrix SoxMM::getLastSensorValues(){
  return x_buffer[(t-1+buffersize)%buffersize];
}

/* stores the controller values to a given file. */
bool SoxMM::store(FILE* f) const{
  // save matrix values
  C.store(f);
  h.store(f);
  /*******************************************/
  switch(modelName) {
    case LINEAR :
      A.store(f);
      b.store(f);
      S.store(f);
      break;
  }
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */ bool SoxMM::restore(FILE* f){ // save matrix values
  C.restore(f);
  h.restore(f);
  /*******************************************/
  switch(modelName) {
    case LINEAR:
      A.restore(f);
      b.restore(f);
      S.restore(f);
      break;
  }
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}


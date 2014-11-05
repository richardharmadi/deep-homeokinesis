/*****************************************************************************
 *             FIAS winter school, playful machine group                     *
 *                Supervisors: Ralf Der, Georg Martius                       *

 * Members: Fabien Benureau, Chrisantha Fernando, Quan Wang, Jimmy Baraglia  *
 *                    Echo State Network c++ File                            *
 *                                                                           *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                           *
 *****************************************************************************/

#include "esn.h"
#include <selforg/controller_misc.h>
#include <selforg/matrixutils.h>

using namespace std;
using namespace matrix;

/**
 * ESN class constructor
 */
ESN::ESN(const ESNConf& _conf)
  : HomeokineticModel("ESN","0.1"), conf(_conf)
{
  assert((conf.connectionRatio >= .0) && (conf.connectionRatio <= 1.));
  assert((conf.inputRatio >= .0) && (conf.inputRatio <= 1.));
  assert((conf.fbRatio >= .0) && (conf.fbRatio <= 1.));

  addParameter("learningrate",&conf.learningRate,0,1,"learning rate");
  addInspectableMatrix("Wos", &Wos, false, "reservoir to output weights");
  addInspectableMatrix("Wox", &Wox, false, "direct x_input to output weights");
  addInspectableMatrix("Woy", &Woy, false, "direct y_input to output weights");
  if(conf.inspectInternals){
    addInspectableMatrix("ESNState",      &ESNState,      false, "internal state");
    addInspectableMatrix("ESNWeights",    &ESNWeights,    false, "internal weights");
    addInspectableMatrix("Wsy",  &Wsy,  false, "input weights y");
    addInspectableMatrix("Wsx",  &Wsx,  false, "input weights x");
    //addInspectableMatrix("FeedBackWeights",  &fbWeights,  false, "feed back weights");
  }
  addInspectableValue("error",&error,"Learning error");
  addParameterDef("causeaware", &causeaware, conf.useExtendedModel ? 0.01 : 0 , 0,0.1,
                  "awarness of controller influences");
  addParameterDef("damping",   &damping,     0.00001, 0,0.01, "forgetting term for model");
  error = .0;
  initialized = false;
}


void ESN::init(unsigned int inputDim, unsigned  int outputDim, double unit_map, RandGen* randGen) {}


void ESN::init(unsigned int x_inputDim, unsigned int y_inputDim, unsigned  int outputDim, double unit_map, RandGen* randGen)
{
  int nbInternalConnection;
  int nbInputConnectionPN;  // per neuron

  Wos.set(outputDim, conf.numNeurons);
  Woy.set(outputDim, y_inputDim);
  Woy_native.set(outputDim, y_inputDim);
  Woy_native.toId();
  Wox.set(outputDim, x_inputDim); // x_inputDim == outputDim
  ESNState.set(conf.numNeurons, 1);
  ESNActivations.set(conf.numNeurons, 1);


  // Input x to reservoir connections
  Wsx.set(conf.numNeurons, x_inputDim);
  if (conf.useExtendedModel) {
    nbInputConnectionPN = conf.numNeurons * conf.inputRatio;
    for(int count1 = 0; count1 < (int)x_inputDim; count1++) {
      for(int count2 = 0; count2 < nbInputConnectionPN; count2++){
        int i = rand()%conf.numNeurons;
        if (Wsx.val(i,count1) == .0)
          Wsx.val(i,count1) = random_minusone_to_one(0)*conf.inputStrength;
        else
          count2--;
      }
    }
  }
  // Input y to reservoir connections
  Wsy.set(conf.numNeurons, y_inputDim);
  nbInputConnectionPN = conf.numNeurons * conf.inputRatio;
  for(int count1 = 0; count1 < (int)y_inputDim; count1++) {
    for(int count2 = 0; count2 < nbInputConnectionPN; count2++){
      int i = rand()%conf.numNeurons;
      if (Wsy.val(i,count1) == .0)
        Wsy.val(i,count1) = random_minusone_to_one(0)*conf.inputStrength;
      else
        count2--;
    }
  }
  

  // Reservoir to reservoir connections
  ESNWeights.set(conf.numNeurons, conf.numNeurons);
  nbInternalConnection = conf.numNeurons*conf.numNeurons*conf.connectionRatio;
  for(int count = 0; count < nbInternalConnection; count++) {
    int i = rand()%conf.numNeurons;
    int j = rand()%conf.numNeurons;
    if (ESNWeights.val(i,j) == .0) 
      ESNWeights.val(i,j) = random_minusone_to_one(0);
    else
      count--;
  }

  // x input to output connections
  if (conf.useExtendedModel)
    Wox = (Wox^0) * unit_map;

  // y input to output connections
  Woy = (Woy^0) * unit_map;

  // calculate the eigenvalues
  Matrix real;
  Matrix img;
  assert(eigenValues(ESNWeights, real,img));
  Matrix abs = Matrix::map2(hypot, real, img); // calc absolute of complex number
  ESNWeights *= conf.spectralRadius/max(abs);

  initialized = true;
}


//const Matrix ESN::processFB (const Matrix& input, const Matrix& teachingForce)
//{
  //assert(initialized);
  //ESNActivations = (inputWeights * input) + (ESNWeights * ESNState) + (fbWeights * teachingForce);
  //ESNState = ESNActivations.map(tanh);
  //return Wos* ESNState + outputDirectWeights * input;
  //}

const Matrix ESN::process (const Matrix& input) {
  return input;
}


const Matrix ESN::process (const Matrix& x_input, const Matrix& y_input)
{
  assert(initialized);
  ESNActivations = Wsx * x_input + Wsy * y_input + ESNWeights * ESNState;
  ESNState = ESNActivations.map(tanh);
  return Wos* ESNState + Wox * x_input + Woy * y_input;
}

const Matrix ESN::learn (const Matrix& x_input, const Matrix& nom_output, double learnRateFactor) {
  return x_input;
}


const Matrix ESN::learn (const Matrix& x_input, const Matrix& y_input, const Matrix& nom_output, double learnRateFactor)
{
  const Matrix& output = process(x_input, y_input);
  const Matrix& delta = nom_output - output;
  error = delta.norm_sqr();
  if (conf.learningRate != .0) {
    //if (conf.spectralRadius != .0)  // Mimic Linear Model
      Wos       += ((delta * (ESNState^T)) * (learnRateFactor * conf.learningRate) 
                 + (Wos * -damping)).mapP(.1, clip);
    
    if (conf.useExtendedModel) {
      Wox += ((delta * (x_input^T)) * (learnRateFactor * conf.learningRate) 
             + (Wox * -damping*10)).mapP(.1, clip);;
    }
    // Woy is causeaware
    const Matrix& eta = Woy.pseudoInverse() * delta;
    const Matrix& y_hat = y_input + eta * causeaware;
    Woy += ((delta * (y_hat^T)) * (learnRateFactor * conf.learningRate)).mapP(.1, clip);;
    if(damping)
      Woy += (((Woy_native - Woy).map(power3)) * damping).mapP(.1, clip);
  }
  return output;
}


const Matrix ESN::response_controller2() const {
  const Matrix& k = ESNActivations.map(tanh);
  return Wos * (Wsy & k);
}


const Matrix ESN::response_controller() const {
  return Woy;
}

const Matrix ESN::response(const matrix::Matrix& input) const {
  const Matrix& k_prime = ESNActivations.map(tanh_prime); // ds/dx != 0
  return Wos * ((Wsy * input + Wsx) & k_prime) + Woy * input + Wox; // ds/dx != 0
  //return Woy * input + Wox; // ds/dx == 0
}

const Matrix ESN::inversion(const matrix::Matrix& input, const matrix::Matrix& xsi) const {
  // todo
  return Matrix();
}


void ESN::damp(double damping)//Damp is Dumb
{
}

unsigned int ESN::getInputDim() const
{
  return Wsx.getM() + Wsy.getM();
}

unsigned int ESN::getOutputDim() const
{
  return Wos.getN();
}


bool ESN::store(FILE* f) const{
  // save matrix values

  Wox.store(f);
  Woy.store(f);
  Wos.store(f);
  Wsx.store(f);
  Wsy.store(f);
  ESNWeights.store(f);
  ESNState.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the ESN values from a given file. */
bool ESN::restore(FILE* f){
  // save matrix values
  Wox.restore(f);
  Woy.restore(f);
  Wos.restore(f);
  Wsx.restore(f);
  Wsy.restore(f);
  ESNWeights.restore(f);
  ESNState.restore(f);
  Configurable::parse(f);
  return true;
}

bool ESN::getIsInitialized() {
  return initialized;
}

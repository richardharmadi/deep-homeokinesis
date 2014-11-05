/***************************************************************************
 *                   Linear Model C++ File                                 *
 *                                                                         *    
 *    @author: Simon Smith                                                 *
 *    @e-mail: artificialsimon@ed.ac.uk                                    *
 *                                                                         *
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
 *                                                                         *
 ***************************************************************************/

#include "linear.h"
#include <selforg/controller_misc.h>
#include <selforg/matrixutils.h>

using namespace std;
using namespace matrix;

/**
 * Linear class constructor
 */
Linear::Linear(const LinearConf& _conf)
  : InvertableModel("Linear Model","0.1"), conf(_conf)
{
  //assert((conf.connectionRatio >= .0) && (conf.connectionRatio <= 1.));
  assert((conf.inputRatio >= .0) && (conf.inputRatio <= 1.));

  addParameterDef("epsA", &epsA, 0.1,     0,5, "learning rate of the model");
  addParameterDef("damping",   &damping,     0.00001, 0,0.01, "forgetting term for model");
  addParameterDef("causeaware", &causeaware, conf.useExtendedModel ? 0.01 : 0 , 0,0.1,
                  "awarness of controller influences");
  if(!conf.onlyMainParameters){
    addParameter("factorS", &conf.factorS,  0, 2,
                    "factor for learning rate for S");
    addParameter("factorb", &conf.factorb,  0, 2,
                    "factor for learning rate for b");
  }

  addInspectableMatrix("A", &A, conf.someInternalParams, "model matrix");
  if(conf.useExtendedModel)
    addInspectableMatrix("S", &S, conf.someInternalParams, "model matrix (sensor branch)");
  addInspectableMatrix("b", &b, conf.someInternalParams, "model bias");
  addInspectableValue("xi_norm_sqr", &xiNormSqr,"Square norm of learning error in model");
   
  initialized = false;
}

void Linear::init(unsigned int inputDim, unsigned  int outputDim)
{
  nSensors  = inputDim;
  nMotors   = outputDim;
  A.set(nSensors, nMotors);
  S.set(nSensors, nSensors);
  b.set(nSensors, 1);
  A_native.set(nSensors, nMotors);

  A.toId(); // set a to identity matrix;
  A_native.toId();
  S.toId();
  S*=0.05;
  xiNormSqr = .0;

  initialized = true;
}


const Matrix Linear::process (const Matrix& input)
{
  assert(initialized);
  const Matrix x_hat = A * input.row(nMotors) + b;
  if (conf.useExtendedModel)
    x_hat += S * input.rows(nMotors, nMotors + nSensors);
  return x_hat;
}


const Matrix Linear::learn (const Matrix& y, const Matrix& x, double learnRateFactor)
{
  const Matrix& x_hat = process(y);
  const Matrix& xi = x - x_hat;

  xiNormSqr = delta.norm_sqr();

  if (conf.learningRate != .0 && learnRateFactor != .0) {
    double epsb = epsA * conf.factorb;
    A += (xi * ((y.row(nMotors))^T) * epsA).mapP(0.1, clip);
    if (conf.damping) {
      A += (((A_native - A).map(power3)) * conf.damping).mapP(0.1, clip);
    }
    b += (xi * (epsb) + (b * - conf.damping)).mapP(0.1, clip);

    if (conf.useExtendedModel) {
      double epsS = epsA * conf.factorS;
      S += (xi * (x^T) * (epsS) + (S * -conf.damping * 10)).mapP(0.1, clip);
    }
  }

  return output;
}


const Matrix Linear::response(const matrix::Matrix& _ignored) const {
  return A;
}

const Matrix Linear::inversion(const matrix::Matrix& input, const matrix::Matrix& xsi) const {
  // todo
  return Matrix();
}


void Linear::damp(double damping)//Damp is Dumb
{
}

unsigned int Linear::getInputDim() const
{
  return inputWeights.getM();
}

unsigned int Linear::getOutputDim() const
{
  return outputWeights.getN();
}


bool Linear::store(FILE* f) const{
  // save matrix values

  inputWeights.store(f);
  outputWeights.store(f);
  outputDirectWeights.store(f);
  LinearWeights.store(f);
  LinearState.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the Linear values from a given file. */
bool Linear::restore(FILE* f){
  // save matrix values
  inputWeights.restore(f);
  outputWeights.restore(f);
  outputDirectWeights.restore(f);
  LinearWeights.restore(f);
  LinearState.restore(f);
  Configurable::parse(f);
  return true;
}

bool Linear::getIsInitialized() {
  return initialized;
}

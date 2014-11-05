/***************************************************************************
 *                   Linear Model Header File                              *
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
#ifndef __LINEAR_H
#define __LINEAR_H


#include <stdio.h>
#include <cmath>
#include <selforg/invertablemodel.h>
#include <selforg/matrix.h>


struct LinearConf {
  double  useExtendedModel;
  bool   inspectInternals;
};


/**
 * class for robot control with sine, sawtooth and impuls
 *
 * period is the length of the period in steps and
 * phaseshift is the phase difference between channels given in Pi/2
 */
class Linear : public InvertableModel {
public:

  /**
     @param controlmask bitmask to select channels to control (default all)
     @param function controller function to use
   */
  Linear(const LinearConf& conf = getDefaultConf());

  static LinearConf getDefaultConf() {
    LinearConf c;
    c.someInternalParams = false;
    c.useExtendedModel = true;
    c.onlyMainParameters = true;
    c.factorS = 1.;
    c.factorb = 1.;
    return c;
  }

  /** initialisation of the network with the given number of input and output units
      @param inputDim length of input vector
      @param outputDim length of output vector
      @param unit_map if 0 the parametes are choosen randomly.
             Otherwise the model is initialised to represent a unit_map
             with the given response strength.
      @param randGen pointer to random generator, if 0 an new one is used
   */
  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0);

  /** passive processing of the input
     (this function is not constant since a recurrent network
     for example might change internal states
  */
  virtual const matrix::Matrix process (const matrix::Matrix& input);

  /* performs learning and returns the network output before learning.
     Neural networks process the input before. (no need to call process before)
     \param learnRateFactor can be given to modify eps for this learning step.
  */
  virtual const matrix::Matrix learn (const matrix::Matrix& input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1.);

  /* calculates the partial derivative of the of the output with repect to the input (Jacobi matrix).

      \f[J_{ij} = \frac{\partial output_i}{\partial input_j}\f]

      The input is ignored, the network must  be processed or learned before!
   */
  virtual const matrix::Matrix response(const matrix::Matrix& _input) const;

  /* calculates the input shift v to given output shift xsi via pseudo inversion.

      \f[o+\xi = \pi(i+v)\f]

      The input is ignored, the network must  be processed or learned before!
   */
  virtual const matrix::Matrix inversion(const matrix::Matrix& input, const matrix::Matrix& xsi) const;

  /// damps the weights and the biases by multiplying (1-damping)
  virtual void damp(double damping);

  /// returns the number of input neurons
  virtual unsigned int getInputDim() const;
  /// returns the number of output neurons
  virtual unsigned int getOutputDim() const;

  virtual bool store(FILE*) const;

  virtual bool restore(FILE*);

  virtual bool getInitialized() {
    return initialized;
  }

  static double tanh_prime(double z)
  {
    double k=tanh(z);
    return 1.0 - k*k;
  };

  virtual bool getIsInitialized();
protected:

  LinearConf conf;

  int nbInputs;
  int nbOutputs;
  matrix::Matrix inputWeights; //W_in_res all-to-all random-value no-learn
  matrix::Matrix outputWeights; // W_res_out all-to-all 0-value learn
  matrix::Matrix outputDirectWeights; // W_in_out all-to-all learn
  matrix::Matrix LinearWeights; // W_res_res random_number random-value no-lear normilised-by-spectral_radius

  matrix::Matrix LinearState; // State of reservoir after applying k = tanh
  matrix::Matrix LinearActivations; // State of reservoir before applying k

  double xiNormSqr;
  bool initialized;

  //
};

#endif


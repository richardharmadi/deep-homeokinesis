
/***************************************************************************
 *                                                                         *
 *    Simon Smith <artificialsimon@ed.ac.uk>                               *
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
#ifndef __HOMEOKINETIC_MODEL_H
#define __HOMEOKINETIC_MODEL_H

#include <selforg/matrix.h>
#include <selforg/invertablemodel.h>

/** abstract class (interface) for invertable models designed for Homeokinesis. 
*   HKmodels overloads input functions separating input to (x, y) = (sensors, motors)
*/
class HomeokineticModel : public InvertableModel {
 public: 
  HomeokineticModel(const std::string& name, const std::string& revision)
    : InvertableModel(name, revision) {}
  virtual ~HomeokineticModel(){};

  /* performs learning and returns the network output before learning.
     Neural networks process the input before. (no need to call process before)
     \param learnRateFactor can be given to modify eps for this learning step.
  */
  virtual const matrix::Matrix learn (const matrix::Matrix& x_input,
                                      const matrix::Matrix& y_input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1) = 0;

  /** passive processing of the input
     (this function is not constant since a recurrent network
     for example might change internal states
  */
  virtual const matrix::Matrix process (const matrix::Matrix& x_input, const matrix::Matrix& y_input) = 0;

  /* calculates the partial derivative of the of the output with repect to the control
   * parameter.

      \f[J_{ij} = \frac{\partial output_i}{\partial parameter_j}\f]

      The input is ignored, the network must  be processed or learned before!
   */
  virtual const matrix::Matrix response_controller() const = 0;
  virtual const matrix::Matrix response_controller2() const = 0;
  virtual void init(unsigned int y_inputDim, unsigned int x_inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0) = 0;

};


#endif

/*
    ergodic_iSAC_exploration: Real-time receding horizon ergodic exploration. 
    Copyright (C) 2017 Anastasia Mavrommati

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef U2_COST_HPP
#define U2_COST_HPP

namespace sac {

  /*! 
    Provides a cost function of time that can be optimized to find the best time 
    to apply the optimal control law u2*. e.g. \f$cost(t) = \sqrt{u_2^*(t)^T 
    u_2^*(t)} + \frac{dJ_1}{d \lambda^+}(t) + t^{1.6}\f$
  */
  class u2_cost {
    mode_insert_grad & dJdlam_;
    double dJdlam_curr_;
  
  public:
    u2_cost( mode_insert_grad & dJdlam ) : 
					   dJdlam_( dJdlam ),
					   dJdlam_curr_(0.0) { }
  
    /*! 
      Returns the value of a cost function at the specified time. The cost 
      function can be searched to find the best time to apply 

      \param[in] t The time at which to evaluate the cost
      \return The value of the cost at the specified time
    */
    double operator() ( const double t )
    {
      dJdlam_(t, dJdlam_curr_);
      
      return dJdlam_curr_;
    }
  };

}

#endif  // U2_COST_HPP

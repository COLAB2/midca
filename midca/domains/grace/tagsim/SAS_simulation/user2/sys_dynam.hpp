
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

#ifndef SYS_DYNAM_HPP
#define SYS_DYNAM_HPP

namespace sac {

  //[ The rhs of x' = f(x) defined as a class
  // USER SPECIFIED:
  class sys_dynam {
    b_control & u_;
    state_type u_curr_;
  
  public:
    sys_dynam( b_control & u ) : u_(u) , u_curr_(ulen) { }
  
    void operator() (const state_type &x, state_type &dxdt, const double t)
    {
      u_(t, x, u_curr_);


	  
	/*vx_dot = u_curr_[0];
	vy_dot = u_curr_[1];
	x = x[0];
	y = x[1]; 
	vx = x[2];
  vy= x[3]*/

	// System dynamics
  float a = .25;
	dxdt[0] = x[2];
	dxdt[1] = x[3];
	dxdt[2] = u_curr_[0]-a*x[2];
    dxdt[3] = u_curr_[1]-a*x[3];
   }
  };
  //]

}

#endif  // SYS_DYNAM_HPP

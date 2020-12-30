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

#ifndef SYS_LIN_HPP
#define SYS_LIN_HPP

namespace sac {
	/* Remove A_at_t and A_pd functions for typical linearizations. Update and use A*/
  //[ Linearizations of the system defined as a class
  // USER SPECIFIED:
  class sys_lin {
  size_t i_;
  iter_1d low, up;
  state_type Kp_curr, Kd_curr;//for pd linearization;
  public:
    sys_lin( ) :  Kp_curr(ulen), Kd_curr(ulen) {  }
	
	
	    /* Returns appropriate linearization at time t depending on whether there is default action or not. */
   void A_at_t( double t, const state_type & x, state_type & u, 
		std::vector<state_type> &default_actions,
		Eigen::Matrix< double, xlen, xlen > & Amat )
  {  	

	if(default_actions[0].size() > 0) {//if we have default actions
		//Calculate bounds on the ti and tf subvector ot default actions such that ti<=t<=tf
		low = std::upper_bound (default_actions[0].begin(), default_actions[0].end(), t); //ti row
		up = std::upper_bound (default_actions[1].begin(), default_actions[1].end(), t); // tf row	 		
				
		if((low - default_actions[0].begin())>(up - default_actions[1].begin())){//if indexes of bounds of ti and tf vectors are different
			A( x, u, Amat );//we have action so original linearization
		} 
		else {
				for(i_ = 0; i_ < ulen; ++i_) {
					if((u[i_] < usat[i_][0]) && (u[i_] > usat[i_][1])) {
						Kp_curr[i_] = Kp; Kd_curr[i_] = Kd; u[i_] = 0;
					} 
					else { Kp_curr[i_] = 0; Kd_curr[i_] = 0;}					
				}
				A_pd( x, u, Amat, Kp_curr, Kd_curr );//pd linearization	
			}
	} else {
			for(i_ = 0; i_ < ulen; ++i_) {
				if((u[i_] < usat[i_][0]) && (u[i_] > usat[i_][1])) {
					Kp_curr[i_] = Kp; Kd_curr[i_] = Kd; u[i_] = 0;
				} 
				else { Kp_curr[i_] = 0; Kd_curr[i_] = 0;}					
			}
			A_pd( x, u, Amat, Kp_curr, Kd_curr );//pd linearization	
			}
  }


  
    void A( const state_type & x, const state_type & u, 
	    Eigen::Matrix< double, xlen, xlen > & Amat ) {
		
		Amat = Eigen::Matrix< double, xlen, xlen >::Zero();
		
		Amat(0,2) = 1;
		Amat(1,3) =1;
		}

		
	void A_pd( const state_type & x, const state_type & u_curr_, 
	    Eigen::Matrix< double, xlen, xlen > & Amat, state_type & Kp,
		state_type & Kd ) {
		
		Amat = Eigen::Matrix< double, xlen, xlen >::Zero();
		float a = .25;
		Amat(0,2) = 1;
		Amat(1,3) =1;
		Amat(2,2) = -a;
		Amat(3,3) =-a;
  }
	
	
	
	
    void B( const state_type & x, const state_type & /*u*/, 
	    Eigen::Matrix< double, xlen, ulen > & Bmat ) {
		
		Bmat = Eigen::Matrix< double, xlen, ulen >::Zero();
				
		Bmat(2,0) = 1;
		Bmat(3,1) = 1;
    }
  };
  //]

}

#endif  // SYS_LIN_HPP








 



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

#ifndef U2_OPTIMAL_HPP
#define U2_OPTIMAL_HPP

namespace sac {

  /*! 
    Stores the optimal switching control,\f$u_2^*(t)\f$.  Keeps references to 
    user maintained state and co-state trajectory interpolation objects so that 
    \f$u_2^*(t)\f$ automatically updates along with changes in trajectory.
    \f[u_2^*(t) = \; (\Lambda + R)^{-1} \, [\Lambda \, u_1(t) + h(x(t))^T \rho(t)
    \, \alpha_d]\f]
    where \f$\Lambda = h(x(t))^T \rho(t) \rho(t)^T h(x(t))\f$ and \f$h(x(t)) = 
    \frac{\partial f_1}{\partial u_1}\f$.  The dynamics, \f$f_1\f$, should be in
    control affine form.
  */
  class u2_optimal {
    state_intp & rx_intp_;
    state_intp & rrho_intp_;
    state_type x_curr_;
    state_type rho_curr_;
    state_type u_curr_;
    double & alpha_;
    Eigen::Matrix< double, ulen, 1 >  mrslt_, mu1_;
    Eigen::Matrix< double, ulen, ulen >  Lambda;
    Eigen::Matrix< double, xlen, 1 > mx_curr_, mrho_curr_;
    Eigen::Matrix< double, xlen, ulen > B_;
    sys_lin lin_;
    size_t i_;
	def_actions & d_actions_;
  
  public:
    /*
      Initializes references to user maintained trajectory objects and the 
      desired rate of change of a trajectory tracking cost functional.
      \param[in] x_intp User maintained state interpolation object
      \param[in] rho_intp User maintained co-state interpolation object
      \param[in] alpha User specified desired change in cost functional
      relative to the duration of activiation of \f$u_2^*(t)\f$. 
      i.e. \f$\frac{\Delta J_1}{\Delta t}\f$.
    */
    u2_optimal( state_intp & x_intp, def_actions & d_actions,
		state_intp & rho_intp, 
		double & alpha ) : rx_intp_( x_intp ) , d_actions_(d_actions),
				   rrho_intp_( rho_intp ) , u_curr_(ulen),
				   x_curr_(xlen) , rho_curr_(xlen) , 
				   alpha_( alpha ) { 
      for ( i_=0; i_<ulen; ++i_ ) {
		mrslt_(i_,0) = 0.0;//initialization
      }
    }
  
    /*!
      Computes the saturated value of the optimal switching control,\f$u_2^*(t)\f$.
      \f[u_2^*(t) = \; (\Lambda + R)^{-1} \, [\Lambda \, u_1(t) + h(x(t))^T 
      \rho(t) \, \alpha_d]\f]
      \param[in] t The time at which to compute the mode insertion gradient
      \param[out] u2Opt_curr The saturated value of the optimal switching control
    */
    void operator() ( const double t, state_type & u2Opt_curr ) {
	double val;
      rx_intp_(t, x_curr_);      State2Mat( x_curr_, mx_curr_ );
      rrho_intp_(t, rho_curr_);  State2Mat( rho_curr_, mrho_curr_ );
	//
	d_actions_.u_default( t, x_curr_, u_curr_ );//Calculate udefault at t
	State2Mat( u_curr_, mu1_ ); // convert state to matrix form
	//
      lin_.B( x_curr_, u_curr_, B_ );// we don't care about second argument for calculation of B (assuming system is in control affine form)
      mrslt_ = B_.transpose()*mrho_curr_;
	Lambda = mrslt_*mrslt_.transpose();
	//std::cout << "Lambda: " << Lambda << "\n";
      mrslt_ = (Lambda + R).inverse() * (Lambda * mu1_ + mrslt_ * alpha_);    

	// saturate controls
      for ( i_=0; i_<ulen; ++i_ ) { 
		val = mrslt_(i_,0);
		if ( val > usat[i_][0] ) { val = usat[i_][0]; }
		else if ( val < usat[i_][1] ) { val = usat[i_][1]; }
		u2Opt_curr[i_] = val;
      }
    }
  };

}

#endif  // U2_OPTIMAL_HPP

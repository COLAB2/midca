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

#ifndef ADJOINT_HPP
#define ADJOINT_HPP

namespace sac {
	/* Remove A_at_t (line 59?) and uncomment line 57 for typical linearizations*/
  /*
    Evaluates the rhs of \f$\dot \rho(t) = -\frac{\partial l}{\partial x}^T 
    - \frac{\partial f}{\partial x}^T \rho(t)\f$ for integration of co-state
    variable, \f$\rho(t)\f$.  Keeps references to state interpolator so that 
    changes state trajectory are automatically accounted for.
  */
  class adjoint {
    state_type x_, u1_, rho_;
    Eigen::Matrix< double, xlen, 1 > mx_, mrho_, mrhodot_;
    Eigen::Matrix< double, 1, xlen > mdldx_;
	Eigen::Matrix< double, xlen, 1 > mdldx_erg;
    Eigen::Matrix< double, xlen, xlen > mdfdx_;
    size_t indx_;
    const int wrap_size_;//number of states to be wrapped
	def_actions & d_actions_;

  public:
    state_intp & m_x_intp;
    sys_lin m_lin;
    inc_cost & m_lofx;
	erg_cost & erg_l_;
  

    /*
      Initializes references to user maintained state interpolation object and a
      cost object.
      \param[in] x_intp User maintained state interpolation object
      \param[in] J The cost object required to provide incremental cost partial, 
      \f$\frac{\partial l}{\partial x}\f$
    */
    adjoint( state_intp & x_intp, def_actions & d_actions, erg_cost & erg_l, 
	     cost & J ) :  x_(xlen), u1_(ulen), d_actions_(d_actions), erg_l_(erg_l),
			   rho_(xlen), m_x_intp( x_intp ), m_lofx(J.m_lofx),
			    wrap_size_(sizeof(x_wrap)/sizeof(x_wrap[0])) { }

    /*!
      Returns the rhs of \f$\dot \rho(t) = -\frac{\partial l}{\partial x}^T 
      - \frac{\partial f}{\partial x}^T \rho(t)\f$.  The dynamics of co-state
      variable, \f$\rho(t)\f$.
      \param[in] rho The co-state variable at time t
      \param[out] rhodot The dynamics of the co-state at time t
      \param[in] t The time variable
    */
    void operator() (const state_type &rho, state_type &rhodot, const double t)
    {
	rho_ = rho;
	m_x_intp(t, x_);        // store the current state in x
	for (indx_ = 0; indx_ < wrap_size_; ++indx_ ) {AngleWrap( x_[x_wrap[indx_]] );} // Angle wrapping (if any)
	State2Mat( x_, mx_ );   // convert state to matrix form
	State2Mat( rho_, mrho_ );
	//
	d_actions_.u_default( t, x_, u1_ );//Calculate udefault at t
	//
	//m_lin.A( x_, u1_, mdfdx_ );
	
	m_lin.A_at_t( t, x_, u1_, d_actions_.default_actions, mdfdx_ );
	//
	m_lofx.dx( t, mx_, mdldx_ );
	//
	erg_l_.dx( mx_, mdldx_erg );
	//
	mrhodot_ = -mdldx_.transpose() - mdldx_erg - mdfdx_.transpose()*mrho_;
	//
	for (indx_ = 0; indx_ < xlen; ++indx_ ) { rhodot[indx_] = mrhodot_[indx_]; }
    }
  };

}

#endif  // ADJOINT_HPP

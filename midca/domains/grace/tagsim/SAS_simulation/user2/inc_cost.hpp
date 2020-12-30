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

#ifndef INC_COST_HPP
#define INC_COST_HPP

namespace sac {
  //! \warning Class MUST BE MODIFIED BY USER for different incremental cost 
  /*
    General incremental trajectory tracking cost, \f$l(x)\f$, for integration
    \f$J_1 = \int_{t_0}^{t_f} l(x) \, dt + m(x(t_f))\f$.  Keeps references to 
    state interpolator so that changes state trajectory are automatically 
    accounted for.
  */
  class inc_cost {
    Eigen::Matrix< double, xlen, 1 > mx_;
    size_t indx_, j_;
    const int wrap_size_;//number of states to be wrapped
	double temp;

  
  public:
    state_intp & m_x_intp; // store current state
    state_type m_x;
    Eigen::Matrix< double, xlen, 1 > m_mxdes;
  
    //! \todo: make inputs const ref type
    /*!
      Initializes references to user maintained trajectory object and a pointer
      to the desired trajectory.
      \param[in] x_intp User maintained state interpolation object
    */
    inc_cost( state_intp & x_intp): 
		 mx_(Eigen::Matrix< double,xlen,1 >::Zero(xlen,1)), 
 		 m_x_intp( x_intp ), m_x( xlen ),
		 m_mxdes(Eigen::Matrix< double,xlen,1 >::Zero(xlen,1)),
		 wrap_size_(sizeof(x_wrap)/sizeof(x_wrap[0])) { }
  
    /*!
      Computes the value of incremental trajectory tracking cost, \f$l(x)\f$.
      \param[in] J The current cost
      \param[out] dJdt The previous incremental cost
      \param[in] t The current time
    */
    void operator() (const state_type &J, state_type &dJdt, const double t)
    {
      m_x_intp(t, m_x); // store the current state in x
	  for (indx_ = 0; indx_ < wrap_size_; ++indx_ ) {AngleWrap( m_x[x_wrap[indx_]] );} // Angle wrapping (if any)
      State2Mat( m_x, mx_ ); // convert state to matrix form
      //
      //get_DesTraj( t, m_mxdes ); // Store desired trajectory point in m_mxdes
      //
      dJdt[0] = l_of_x(mx_, m_mxdes);
    }


    /*!
      Incremental cost formula
      \param[in] J The current state and current desired state
      \param[out] The incremental cost
    */
    inline double l_of_x(const Eigen::Matrix< double, xlen, 1 > & x_curr, const Eigen::Matrix< double, xlen, 1 > & xd_curr) { 
		temp = 0;
		for (indx_ = 0; indx_ < erg_dim; ++indx_ ) {
			for (j_ = 0; j_ < erg_dim; ++j_ ) {
				if(j_==1) {
					temp = temp + 1.0/(1+exp(-slope*(x_curr(indices[indx_],0)-ranges[indx_][j_])));
				} 
				else {
					temp = temp + 1.0/(1+exp(-slope*(-x_curr(indices[indx_],0)+ranges[indx_][j_])));
				}
			}
		}
	return	Qbar*temp;
	//return( ( (x_curr-xd_curr).transpose() * Q * (x_curr-xd_curr) ) / 2.0 )[0];
    }


    /*!
      Computes the value of the derivative of the incremental cost, 
      \f$D_x l(x)\f$.
      \param[in] t The current time
      \param[in] mx The current state (already wrapped)
      \param[out] dldx The derivative \f$D_x l(x)\f$.
    */
    inline void dx( const double t, const Eigen::Matrix< double, xlen, 1 > &mx,
		    Eigen::Matrix< double, 1, xlen > &dldx ) { 
		/*get_DesTraj( t, m_mxdes ); // Store desired trajectory point in m_mxdes
		//
		dldx = (mx-m_mxdes).transpose()*Q;*/

		dldx = Eigen::Matrix< double, 1, xlen >::Zero();//initialize to zero

		for (indx_ = 0; indx_ < erg_dim; ++indx_ ) {
			for (j_ = 0; j_ < erg_dim; ++j_ ) {
				if(j_==1) {
					dldx(0, indices[indx_]) = dldx(0, indices[indx_]) + slope*exp(slope*(ranges[indx_][j_] + 
						mx(indices[indx_],0)))/pow(exp(slope*ranges[indx_][j_]) + exp(slope*mx(indices[indx_],0)), 2.0);
				} 
				else {
					dldx(0, indices[indx_]) = dldx(0, indices[indx_]) - slope*exp(slope*(ranges[indx_][j_] + 
						mx(indices[indx_],0)))/pow(exp(slope*ranges[indx_][j_]) + exp(slope*mx(indices[indx_],0)), 2.0);
				}
			}
		}
		dldx = dldx*Qbar;
    }

    /*!
      Returns the initial time for integration, \f$t_0\f$
      \return Initial integration time \f$t_0\f$
    */
    double begin( ) { return m_x_intp.begin( ); }

    /*!
      Returns the final time for integration, \f$t_f\f$
      \return Final integration time \f$t_f\f$
    */
    double end( ) { return m_x_intp.end( ); }
  };

}

#endif  // INC_COST_HPP

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

#ifndef B_CONTROL_HPP
#define B_CONTROL_HPP

namespace sac {

  /*! 
    This class stores switching control vectors.  The control switches from
    a nominal control to the switching control signal when \f$\tau_1 
    \leq t \leq \tau_2\f$.
  */
  class b_control {
  def_actions & d_actions_;

  public:
    double m_tau1, m_tau2;//stores switching times of SAC action
    state_type m_u_switch;//stores value of SAC action
  
    /*!
      Constructor sets the SAC action control to the zero vector.
    */
    b_control( 	def_actions & d_actions ) : 
			m_tau1(0.0), m_tau2(0.0), m_u_switch(ulen),
 			d_actions_(d_actions) { 
      for ( size_t i=0; i<ulen; ++i ) {
		m_u_switch[i] = 0.0;//initialize
      }
    }
  
    /*!
      Returns the control at time t.
      \param[in] t The time at which to get the current control.
      \param[out] u_curr The control vector at time t.
    */ 
    void operator() ( const double t, const state_type & x, state_type& u_curr )
    {
      if ( (t >= m_tau1) && (t <= m_tau2) && (m_tau1 < m_tau2) ) { u_curr = m_u_switch; } 
      else { d_actions_.u_default( t, x, u_curr );} //Set u_curr equal to udefault at t
    }

    /*
      Sets the (already saturated) value of the switching control when \f$\tau_1 \leq t \leq 
      \tau_2\f$.  
      \param[in] u_switch The desired value of the switching control when 
      \f$\tau_1  \leq t \leq \tau_2\f$.
    */
    void operator= ( const state_type & u_switch ) { 
      for ( size_t i=0; i<ulen; ++i ) {
		m_u_switch[i] = u_switch[i];
      }
    }


    /*!
      Sets switching times \f$\tau_1\f$ and \f$\tau_2\f$ where the switching
      control is applied when \f$\tau_1 \leq t \leq \tau_2\f$.
      \param[in] t1, t2 Desired values of switching times \f$\tau_1\f$ and 
      \f$\tau_2\f$.
    */
    void stimes( const double t1, const double t2 ) { m_tau1 = t1; m_tau2 = t2; }

    /*!
      Re-set the switching control to the zero vector.  Sets switching times 
      \f$\tau_1\f$ and \f$\tau_2\f$ to 0.
    */
    void clear( ) {
      for ( size_t i=0; i<ulen; ++i ) {
	m_u_switch[i] = 0.0;
      }
      m_tau1 = 0.0;    m_tau2 = m_tau1;
    }
  };

}

#endif // B_CONTROL_HPP

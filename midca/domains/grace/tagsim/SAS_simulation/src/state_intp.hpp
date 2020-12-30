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

#ifndef STATE_INTP_HPP
#define STATE_INTP_HPP

namespace sac {

  /*!
    Class holds the address of state_types and times vectors.  It uses 
    interpolation to provide the state at a specified time.
  */
  class state_intp {
    int indx_;
    iter_1d geq_;
    size_t i_;
  
  public:
    std::vector< state_type > * m_states;
    std::vector< double > * m_times;
    size_t m_xlen;

    //! \todo: make inputs const ref type
    /*!
      Initializes pointers to user maintained state_type and times vectors.
      \param[in] states Vector of states sampled at different points in time
      \param[in] times Vector of times corresponding to the sampled states
    */
    state_intp( std::vector< state_type > &states , 
		std::vector< double > &times ) : indx_( 0 ) , 
						 geq_( times.begin() ) ,
						 m_states( &states ) , 
						 m_times( &times ) { 
    
      try { m_xlen = states.at(0).size(); }
      catch( const std::exception& e ) { 
	std::cout << "Exception initializing m_xlen from states.at(0).size()"
		  << " in state_intp constructor.  " << e.what() <<"\n";
      }

    }

    //! \todo: make inputs const ref type
    /*!
      Initializes pointers to user maintained state_type and times vectors.
      \param[in] states Vector of states sampled at different points in time
      \param[in] times Vector of times corresponding to the sampled states
      \param[in] xlength The length of state x(t)
    */
    state_intp( std::vector< state_type > &states , 
		std::vector< double > &times, 
		size_t xlength ) : indx_( 0 ) , 
				   geq_( times.begin() ) ,
				   m_states( &states ) , 
				   m_times( &times ) , 
				   m_xlen( xlength ) { }

    /*!
      Applies linear interpolation to provide the state at the specified time.
      \param[in] t_intp The time to interpolate the state
      \param[out] x_intp The state at the specified time
    */
    void operator() (const double t_intp, state_type& x_intp)
    {
      // find first value in range >= that specified.
      geq_=std::lower_bound( m_times->begin(), m_times->end(), t_intp );
      indx_ = geq_-m_times->begin();

      if ( indx_ == 0 ) { x_intp = (*m_states)[indx_]; }
      else {  
	for (i_=0; i_ < m_xlen; ++i_ ) {
	  x_intp[i_] = (*m_states)[indx_-1][i_] 
	    + ((*m_states)[indx_][i_]-(*m_states)[indx_-1][i_])
	    *(t_intp-(*m_times)[indx_-1])/((*m_times)[indx_]
					   -(*m_times)[indx_-1]);
	}
      }
    }

    //! \todo: make inputs const ref type
    /*!
      Updates the state_type and times vectors pointed to.
      \param[in] states Vector of states sampled at different points in time
      \param[out] times Vector of times corresponding to the sampled states 
    */
    void update( std::vector< state_type > &states , 
		 std::vector< double > &times ) {
      m_states = &states;
      m_times = &times;
    }

    /*!
      \return The first element of the vector of times.
    */
    double begin( ) { return *( m_times->begin( ) ); }

    /*!
      \return The last element of the vector of times.
    */
    double end( ) { return *( m_times->end( )-1 ); }


    /*!
      \return pointer to the vector of times.
    */
    std::vector< double > * time_vec( ) { 
	return m_times;	
	 }

    /*!
      \return pointer to the vector of states.
    */
    std::vector< state_type > * state_vec( ) { 
	return m_states;	
	 }

  };

}

#endif // STATE_INTP_HPP

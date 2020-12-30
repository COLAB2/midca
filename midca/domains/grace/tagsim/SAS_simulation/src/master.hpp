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

#ifndef MASTER_HPP
#define MASTER_HPP

/*! 
  \file master.hpp
  
  The master include file for the library.
*/

/*********************************************/
/* Includes */
#include <iostream> //save to file
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <vector>
#include <boost/numeric/odeint.hpp>
#include <fstream>      // std::ofstream
#include <Eigen/Core>
#include <Eigen/Dense>
#include <isac_settings.hpp>              // SAC settings
#include <erg_settings.hpp>              // ergodic settings


/*********************************************/

namespace sac {

  /*********************************************/
  /* The type of container used to hold the state vector */
  typedef std::vector< double >::iterator iter_1d;
  typedef std::vector< state_type >::iterator iter_2d;


  /* Function Prototypes */
  template< class T >
  inline void State2Mat( state_type & s, T & matOut );
  template < class Scalar >
  inline void AngleWrap( Scalar & theta );
  template < class T , class Scalar >
  void MinSearch( T & fcost, Scalar t0, Scalar tf,
		  std::vector<Scalar>& lclMin, Scalar dt, Scalar eps );
  template<class T, class InputIterator, class Function>
  InputIterator get_min( InputIterator first, InputIterator last, 
			 Function & fn, T & min );
  class sys_dynam; class adjoint;
  size_t simX( sys_dynam& xdot, state_type& x0, double t0, double tf,
	       std::vector<state_type>& x_vec, std::vector<double>& times );
  size_t simRho( adjoint& rho_dot, state_type& rho_Tf, double& t0, double& tf, 
		 std::vector<state_type>& rho_vec,std::vector<double>& rho_times);
  template <typename T> int sgn(T val);
  inline double trapezoid( double & psum, state_type & fvec, state_type & xvec );
  void cart_product(Vvi& rvvi, Vi&  rvi, Vvi::const_iterator me, Vvi::const_iterator end);
  inline bool isequal (double i, double j);
  template < class Scalar >
  inline void calc_norm( double & result, std::vector<Scalar> & vec );


  //[
  /*! 
    Observer class that stores states and times during integration in user 
    specified containers of type std::vector< state_type >.
  */
  struct push_back_state_and_time
  {
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , 
			      std::vector< double > &times ) : 
      m_states( states ) , m_times( times ) { }
  
    void operator()( const state_type &x , double t )
    {
      m_states.push_back( x );
      m_times.push_back( t );
    }
  };
  /*! 
    \fn push_back_state_and_time::push_back_state_and_time( 
    std::vector< state_type > &states , 
    std::vector< double > &times ) : 
    m_states( states ) , m_times( times )

    Initializes member variables to reference user specified containers to
    hold the states and times to be updated during integration.
    \param[in,out] states The container to hold the vector of states.
    \param[in,out] times The container to hold the vector of times.
  */
  /*! 
    \fn void push_back_state_and_time::operator()( const state_type &x , 
    double t )

    Pushes states and times during integration.
    \param[in] x The state to push to the user specified container holding 
    the vector of states.
    \param[in] t The time to push to the user specified container holding 
    the vector of times.
  */
  //]
}

/*********************************************/
/* Class declarations */
/* Includes */
#include <def_actions.hpp>
#include <b_control.hpp>
#include <state_intp.hpp>
#include <sys_dynam.hpp>       
#include <sys_lin.hpp>         
//
//[  Choose between user & generalized versions
#include <u2_optimal.hpp>              
#include <mode_insert_grad.hpp>        
#include <u2_cost.hpp>
#include <inc_cost.hpp>    
#include <phik_hk.hpp>              // ergodic stuff
#include <ck.hpp>              // ergodic stuff
#include <erg_cost.hpp>              // ergodic stuff            
#include <cost.hpp>
#include <adjoint.hpp>                 
#include <isac_step.hpp>
#ifdef IMPACTS
#include <sac_impact_step.hpp>
#endif
//]

namespace sac {

  /*********************************************/
  /* Function definitions */

  //[ Additional prototypes 
  //
  // template < class T, class Scalar >
  // Scalar GoldenSection( T & fcost, Scalar a, Scalar b, Scalar c, Scalar& dt, 
  // 		      Scalar& eps );
  // template < class T >
  // void SaveVec( T src, const char * outputFilename )
  //]


  //! \todo: change input references to const references.
  /*!
    Converts a state_type vector to a matrix type.
    \param[in] s A state_type vector.
    \param[out] matOut A matrix with the same # of rows as the input vector.
  */
  template< class T >
  inline void State2Mat( state_type & s, T & matOut ) {
    for ( size_t i=0; i<s.size(); ++i ) {
      matOut(i,0) = s[i];
    }
  }


  //! \todo: See about making input const type.
  /*!
    Computes the sign of a scalar.
    \param[in] val A state_type vector.
    \return An integer -1, 0, or 1 depending on the sign of the input.
  */
  template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }


  //! \todo: See about making inputs const type.  Remove dt input parameter.
  /*!
    Uses the golden section method to search for the minimum of a callable over a
    specified domain.  It is assumed a single minimum exists over this domain.
    \param[in] fcost The callable to be searched to find a minimum.
    \param[in] a,b,c Values of the callable at three domain points where 
    \f$a<b<c\f$.
    \param[in] dt Change in time.
    \param[in] eps The desired tolerance in finding the minimum (eps<<1).
    \return The scalar value of the minimum point between a and c.
  */
  template < class T, class Scalar >
  Scalar GoldenSection( T & fcost, Scalar a, Scalar b, Scalar c, 
			Scalar& dt, Scalar& eps ) {
    static const Scalar resphi = 2.0 - (1.0 + sqrt(5.0))/2.0;
    static Scalar d;
    d = ( (c-b)>(b-a) ) ? ( b+resphi*(c-b) ) : ( b-resphi*(b-a) );
    if ( std::abs(c-a) < eps ) { return (c+a)/2.0; }
    if ( fcost(d) < fcost(b) ) { 
      if ( (c-b)>(b-a) ) { 
	return GoldenSection( fcost, b, d, c, dt, eps );
      } else {
	return GoldenSection( fcost, a, d, b, dt, eps );
      }
    } else {
      if ( (c-b)>(b-a) ) { 
	return GoldenSection( fcost, a, b, d, dt, eps );
      } else {
	return GoldenSection( fcost, d, b, c, dt, eps );
      }
    }
  }


  //! \todo: See about making inputs const type.
  /*!
    Samples a callable at specified intervals over a domain.  Searches for 
    zero-crossings.  Uses golden section to search for minimizers on sub-
    intervals where zero crossings are detected.  Returns vector of local minima.
    \param[in] fcost The callable to be searched to find local minima.
    \param[in] t0,tf The domain on which to search for minima 
    \f$t0 \leq t \leq tf\f$.
    \param[out] lclMin An empty vector to store the local minima.
    \param[in] dt Change in time.
    \param[in] eps The desired tolerance in finding the minimum (eps<<1).
  */
  template < class T , class Scalar >
  void MinSearch( T & fcost, Scalar t0, Scalar tf, std::vector<Scalar>& lclMin, 
		  Scalar dt, Scalar eps ) {
    int sign1, sign2;
    Scalar dtt=2.0*dt, maxt=tf-2.0*dt;
    Scalar c1 = fcost(t0), c2 = fcost(t0+dt), c3 = fcost(t0+dtt);  
	
	if(std::abs(c2-c1)<eps) {c1=c2;}
	if(std::abs(c3-c2)<eps) {c2=c3;}

    lclMin.push_back( t0 ); /* include endpoint costs */
    for(Scalar t=t0+dt; t<=maxt; t+=dt) {    /* loop over data  */    
      sign1 = sgn(c2-c1);  // forward diff's to approx deriv
      sign2 = sgn(c3-c2);
      if(sign1!=sign2) { // find zero crossings
	if ( (c1 > c2) || (c3 > c2) ) { // store local min + saddle pts	
	  lclMin.push_back( GoldenSection( fcost, t-dt, t, t+dt, dt, eps ) );
	}
      } 
      c1 = c2;
      c2 = c3;
      c3 = fcost(t+dtt);
	  
	if(std::abs(c2-c1)<eps) {c1=c2;}
	if(std::abs(c3-c2)<eps) {c2=c3;}
    }
    lclMin.push_back( tf ); /* include endpoint costs */
  }


  //! \todo: See about making inputs const type.  Possibly also find max
  //!  element since the entire list is searched.
  /*!
    Evaluates a callable exhaustively over each element of an iterable list 
    domain to find the one that minimizes the callable.
    \param[in] first Iterator pointing to first element.
    \param[in] last Iterator pointing to the end element.
    \param[in] fn The callable to evaluate in search of a minimum.
    \param[out] min The minimum value of the callable.
    \return An iterator pointing to the iterable list object element that 
    minimizes the callable.
  */
  template<class T, class InputIterator, class Function>
  InputIterator get_min( InputIterator first, InputIterator last, 
			 Function & fn, T & min ) {
    T tmp;  InputIterator p_indx;
    if (first!=last) { min=fn (*first); p_indx = first; ++first; }
    while (first!=last) {
      tmp=fn (*first);
      if (tmp < min) { min = tmp; p_indx = first; }    
      ++first;
    }
    return p_indx;      // or, since C++11: return move(fn);
  }


  //! \todo: See about making inputs const type.
  /*!
    Simulates the state forward in time from an initial state at t0 to the final 
    state at tf.
    \param[in] xdot The dynamics of the system.
    \param[in,out] x0 The initial state which gets integrated to become the 
    final state.
    \param[in] t0 The initial time.
    \param[in] tf The final time.
    \param[out] x_vec The vector of states resulting from integration.
    \param[out] times The vector of times resulting from integration.
    \return The number of integration steps.
  */
  size_t simX( sys_dynam& xdot, state_type& x0, double t0, double tf,
	       std::vector<state_type>& x_vec, std::vector<double>& times ) {  
    using namespace std;
    using namespace boost::numeric::odeint;
    typedef runge_kutta_dopri5< state_type > stepper_type;

    size_t steps = integrate_adaptive( 
				      make_controlled( 1E-5 , 1E-5 , 
						       stepper_type( ) ) , 
				      xdot , x0 , t0 , tf , 0.0005 , 
				      push_back_state_and_time(x_vec , times) );
					  
	//size_t steps = integrate_const( runge_kutta4< state_type >() , xdot , x0 , t0 , tf , 0.005 , push_back_state_and_time(x_vec , times) );
    return steps;
  }


  //! \todo: See about making inputs const type.
  /*!
    Simulates the adjoint / co-state, \f$\rho\f$, backwards in time from 
    \f$\rho(t_f)\f$ to \f$\rho(t_0)\f$.
    \param[in] rho_dot The dynamics of the system co-state variable.
    \param[in,out] rho_Tf The co-state variable at tf which gets integrated 
    backwards to become the initial co-state variable.
    \param[in] t0 The initial time.
    \param[in] tf The final time.
    \param[out] rho_vec The vector of co-states resulting from integration.
    \param[out] rho_times The vector of times resulting from integration.
    \return The number of integration steps.
  */
  size_t simRho( adjoint& rho_dot, state_type& rho_Tf, double& t0, double& tf, 
		 std::vector<state_type>& rho_vec,std::vector<double>& rho_times)
  {
    using namespace std;
    using namespace boost::numeric::odeint;
    typedef runge_kutta_dopri5< state_type > stepper_type;

    size_t rho_steps = integrate_adaptive( // change to deal with stiff sys
    					  make_controlled( 1E-5 , 1E-5 , 
    							   stepper_type( ) ) , 
    					  rho_dot , rho_Tf , tf , t0 , -0.01 , 
    					  push_back_state_and_time(rho_vec , 
    								   rho_times) );
									   
   //size_t rho_steps = integrate_const( runge_kutta4< state_type >() , rho_dot , rho_Tf , tf , t0 , -0.005 , push_back_state_and_time(rho_vec , rho_times) );

    return rho_steps;
  }



  /*!
    Replaces a scalar angle in radians with a wrapped version 
    \f$ \in [-\pi, \pi)\f$.
    \param[in,out] theta The angle in radians to be wrapped and returned.
  */
  template < class Scalar >
  inline void AngleWrap( Scalar & theta ) {
    theta = fmod( (theta + PI) , (2.0*PI) );
    if ( theta < 0 ) { theta = theta + 2.0*PI; }
    theta = theta - PI;
  }


  /* Function to implement integration of f(x) over the interval
   defined by "xvec" using the trapezoidal rule.
  \param[in] fvec, xvec - vectors with values of dependent and independent vars at desired points
  \param[out] psum: Integration result
  */
  inline double trapezoid( double & psum, state_type & fvec, state_type & xvec )
  {
	psum = 0; size_t index, length = xvec.size();
	// increment the partial sum
	for(index=0; index<length-1; ++index)
	{
		psum = psum + (xvec[index+1] - xvec[index])*(fvec[index]+fvec[index+1])/2.0;
	}
	return 0.0;
  }

	
	/*Finds all combinations of vectors in Vvi*/
	void cart_product(
		Vvi& rvvi,  // final result
		Vi&  rvi,   // current result 
		Vvi::const_iterator me, // current input
		Vvi::const_iterator end) // final input
	{
		if(me == end) {
			// terminal condition of the recursion. We no longer have
			// any input vectors to manipulate. Add the current result (rvi)
			// to the total set of results (rvvvi).
			rvvi.push_back(rvi);
			return;
		}

		// need an easy name for my vector-of-ints
		const Vi& mevi = *me;
		for(Vi::const_iterator it = mevi.begin();
			it != mevi.end();
			it++) {
			// final rvi will look like "a, b, c, ME, d, e, f"
			// At the moment, rvi already has "a, b, c"
			rvi.push_back(*it);  // add ME
			cart_product(rvvi, rvi, me+1, end); //add "d, e, f"
			rvi.pop_back(); // clean ME off for next round
		}
	}


  inline bool isequal (double i, double j) {
  return (std::abs(i-j)<epsilon);
  }
  
  
  template < class Scalar >
  inline void calc_norm( double & result, std::vector<Scalar> & vec ) {
	int length = vec.size();
	result = 0;
	for(int i=0; i < length; ++i) {
		result = result + vec[i]*vec[i];
	}
	result = sqrt(result);
  }



}

#endif // MASTER_HPP

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
#ifndef ISAC_STEP_HPP
#define ISAC_STEP_HPP

namespace sac {

  //[ The class to carry out an isac control step
  class isac_step {
  protected:
    /* protected initialization */
    double alpha, min_val, dtt_win, dt_win;
    state_type x, rho;   
	Eigen::Matrix< double, xlen, 1 > m_mrho_tf;
	std::vector<state_type>  rho_vec;
	state_type times, rho_times;
	state_intp x_intp, rho_intp;  // linear interpolator
	def_actions d_actions; b_control u;
	phik_hk erg; ck c_k_; erg_cost J_erg;
    cost J1;  sys_dynam xdot;  adjoint rho_dot;
    u2_optimal u2Opt;  mode_insert_grad dJdlam;
    double dJdlam_curr;   u2_cost cntrlCost;
    std::vector<double> lclMin;
    iter_1d it1_1d, low, up;   size_t j, steps, length;
    state_type switch_times_within_range;
	state_type c_k_default, ck_prev_default;
  
  public:
    /* public initialization */
    double J0, Jn, Jprev, t_i, t_f, t_app, tf;    
    state_type x0noU, xnext;  
	state_type u_switch;
	std::vector<state_type> x_vec;
    int its, i_;

    
    isac_step( ) : 
		 x(xlen), rho(xlen), u_switch(ulen), 
		 xdot(u), rho_dot(x_intp, d_actions, J_erg, J1),
		 u2Opt(x_intp, d_actions, rho_intp, alpha),
		 dJdlam( x_intp, rho_intp, d_actions, u2Opt ),
		 cntrlCost( dJdlam ), ck_prev_default(erg.no_k_comb, 0.0),
		 J1(x_intp, J_erg), u(d_actions), 
		 t_i(T), t_f(T), tf(T), 
		 x_intp(x_vec , times, xlen),
		 rho_intp(rho_vec , rho_times, xlen),
		 x0noU(xlen), xnext(xlen), c_k_(erg), J_erg( x_intp, c_k_, erg) 
		 { 	//cost weights
			initialize_cost_weights(); 
		 }


    inline void SimInitXRho( double &t0, const state_type &x0);
 
    inline void SimNewX( double &t0, const state_type &x0);

    inline void simx_u_default( state_type& x, const double &tinit, const double &tfinal);

    inline void simrho_u_default();

    inline void operator() ( double &t0, const state_type &x0) {
      /* Initialize final time */
      tf = t0+T;
	  
	  //Update Fourier coefficientd
	  erg.update_phik( t0 );
	  
	  d_actions.get_default_actions( t0 );//remove actions before current t0 and save them to file
	  d_actions.update_default_switch_times( );//prepares default_switch_times vector 
	  
      /* Simulate initial trajectory */      
      SimInitXRho( t0, x0 );
	  
	  /*Store ck_default and ckprev_default in case no action is found if we don't have rolling memory*/
	  c_k_default = c_k_.c_k; 
	  c_k_.update_ck_prev_plus( x_vec, times, ck_prev_default );
	  
      /* Update J0 based on new value calculated in siminitxrho */      
      J0 = J1;                 Jn = J0;

	if (std::abs(t_init-t0) < epsilon) { Jprev = J0; }//initialize Jprev

	dt_win = maxdt;//initial control duration
      dtt_win = dt_win/2.0;    its = 0;

      /* Set alpha based on the initial cost */
      alpha = lam;//*J0;

      /* u2 automatically computed from x_intp - find opt time to apply */
      if ( u2Search ) { 
		lclMin.clear();

		MinSearch( cntrlCost, t0, tf, lclMin, 0.006, 1E-3 ); //runs golden section

		//Used when t_app has to be in sync with physical hardware (i.e. a multiple of 1/ctr_com_freq)
		/*double time;
		for (time = t0; time <= tf; time = time + 1.0/ctr_com_freq) {lclMin.push_back(time);}*/

		it1_1d = get_min(lclMin.begin(), lclMin.end(), cntrlCost, min_val);
		t_app = *it1_1d; //std::cout << t_app <<"\n";
		//	This can be used to plot control cost in matlab
		/*std::ofstream myfile;
		myfile.open ("ctrcost.csv", std::ios::app);//open file to save stuff
		for (double time = t0; time < t0+T; time = time + 0.01)
		{	
			myfile << cntrlCost(time) << ", ";//write to file
		}
		myfile << "\n";//write to file
		myfile.close();//close file*/
		 } 
	else { t_app = t0; }
    
	//Calculate SAC action value from closed-form expression
     u2Opt( t_app, u_switch );    

	//Calculate mode insertion gradient
	dJdlam( t_app, dJdlam_curr );
    
	//Switching times for initial duration
      t_i = (t_app-dtt_win); t_f = (t_app+dtt_win);

	//Check that switching times make sense
      if ( t_i - t0 < epsilon ) { t_i = t0; }       
      if ( t_f - tf > -epsilon ) { t_f = tf; }   
	  if ((std::abs( t_f - t_i ) < epsilon) || (t_i > t_f)) { t_i = t_f; }				
      //
      else if ( dJdlam_curr < 0 ) {  // use control only if dJdlam < 0
		/* Simulate X based on applying u2* at optimal time */
		SimNewX( t0, x0 );
		
		/* Update Cost after applying u2* */
		J1.update();	Jn = J1;

		/* Backtrack until cost is improved by desired amount or its */
		while ( (Jn-Jprev>0) && (its<max_iters) ) {
			//dt_win = dt_win - min_duration;//reduce control duration by specified amount to be consistent with hardware
			dt_win = dt_win*factor;//reduce control duration by specified amount
			dtt_win = dt_win/2.0;
			t_i = (t_app-dtt_win); t_f = (t_app+dtt_win);
			if ( t_i - t0 < epsilon ) { t_i = t0; }  
			if ( t_f - tf > -epsilon ) { t_f=tf; } 
			if ((std::abs( t_f - t_i ) < epsilon) || (t_i > t_f)) { t_i=t_f; break; }
			else {  
				/* Simulate X based on applying new duration */
				SimNewX( t0, x0 );

				/* Update Cost after applying u2* */
				J1.update();      Jn = J1;		
			} /* end else */
			its++;
		} /* end backtracking while */
      
      } /* end else if */  
	
	else { Jn = J0+1; }//don't apply control

	//Check that switching times make sense
      //if ( t_f > t0+ts ) { t_f = t0+ts; }  //allow duration greater than ts
	if ((std::abs( t_f - t_i ) < epsilon) || (t_i > t_f)) { t_i = t_f; }


	/* Finalize action value and switching times */
		if ( Jn > Jprev ) { // cost increased so don't apply control
			//std::cout <<"NO ACTION FOUND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << "\n";
			xnext = x0noU;    // return x
			for (i_ = 0; i_ < ulen; ++i_) { u_switch[i_] = 0; } // return zero action
			t_i = t_f;//zero duration
		    Jn=J0;
			c_k_.c_k = c_k_default; 
		    if( ergodic_memory_steps == 0 ) { c_k_.ck_prev = ck_prev_default; }
			else { c_k_.update_ck_prev_minus( xnext, t0+ts, ck_prev_default ) ; }		
		}
		else { 
				x_intp( t0+ts, xnext ); 
				if( ergodic_memory_steps == 0 ) { c_k_.update_ck_prev_plus( x_vec, times, c_k_.ck_prev ); }
				else { 
						c_k_.update_ck_prev_plus( x_vec, times, c_k_.ck_prev );
						c_k_.update_ck_prev_minus( xnext, t0+ts, c_k_.ck_prev ) ; 
					}	
			 } 

	//Update default actions  
	d_actions.update_default_actions( t_i, t_f, u_switch  );//adds new action if any

	Jprev = Jn;//update Jprev
	
	//PRINTS
	/*std::cout << "Vector of default switch times for this iteration: \n";
	for (std::vector<double>::iterator it=d_actions.default_switch_times.begin(); it!=d_actions.default_switch_times.end(); ++it)
		{std::cout << ' ' << *it;}		
	std::cout << "\n";	
	
	std::cout << "New action: "<< t_i << "," << t_f << ", " << u_switch[0] <<"\n";
	
	std::cout << "Updated default actions matrix: \n";
	for (i_=0; i_ < ulen+2;++i_) {		
		for (std::vector<double>::iterator it=d_actions.default_actions[i_].begin(); it!=d_actions.default_actions[i_].end(); ++it){
			std::cout << ' ' << *it;}
	std::cout << "\n";
	}

	std::cout << Jn << "\n";*/
	
	
  }

};


  inline void isac_step::SimInitXRho( double &t0, const state_type &x0) {
    x_vec.clear(); times.clear(); x = x0;  // initialize state 
    simx_u_default( x, t0, tf);//simulate state with default control
    x_intp( t0+ts, x0noU );//interpolate in case no action is found
	
	J1.update( );//calculate default cost and ck that will be used in costate below

    simrho_u_default();//simulate costate with default control 
	
  }


  inline void isac_step::SimNewX( double &t0, const state_type &x0 ) {
    x_vec.clear(); times.clear(); x = x0;  // re-initialize state 
    //
    u=u_switch; u.stimes( t_i, t_f );              // update u
    //
    if (t_i-t0>epsilon) {// pre-u_new 
    	simx_u_default( x, t0, t_i);//simulate state with default control
      x_vec.pop_back(); times.pop_back();
    }
    // Simulation for new action
    simX( xdot, x, t_i, t_f, x_vec, times ); // u_new 
    //
    //After new action
    if (t_f-(tf)< -epsilon) {
      x_vec.pop_back(); times.pop_back();
      simx_u_default( x, t_f, tf);// post-u_new 
    }
  }
 

  /*!
    Piecewise simulates the state forward in time from an initial state to the final
	using default control (sequence of actions).
    \param[in,out] x0 The initial state which gets integrated to become the 
    final state.
    \param[in] tinit The initial time.
    \param[in] tfinal The final time.
    \param[out] x_vec The vector of states resulting from integration.
    \param[out] times The vector of times resulting from integration.
  */
  inline void isac_step::simx_u_default( state_type& x, const double &tinit, const double &tfinal) {
	switch_times_within_range.clear();
	//Find switching times between the integration limits tinit and tfinal
	low = std::upper_bound (d_actions.default_switch_times.begin(), d_actions.default_switch_times.end(), tinit+epsilon);//d_actions.default_switch_times index that is > tinit   
	up = std::upper_bound (d_actions.default_switch_times.begin(), d_actions.default_switch_times.end(), tfinal-epsilon);//d_actions.default_switch_times index that is > tfinal
	//The insertion below is not inclusive so we get elements <tfinal
	//state_type switch_times_within_range(low, up);

	//add initial and final times
	switch_times_within_range.push_back(tinit);
	switch_times_within_range.insert(switch_times_within_range.end(), low, up);//appends a subvector with as many elements as the range [first,last)!!(so we get elements <tfinal)
	switch_times_within_range.push_back(tfinal);
	//keep unique elements
	switch_times_within_range.erase( unique( switch_times_within_range.begin(), switch_times_within_range.end(), isequal ), switch_times_within_range.end() );
	length = switch_times_within_range.size();


	//Piecewise Integrate
	for (j = 0; j < length-1; ++j) { // simulate with default control (initial condition x is automatically changed from the integrator)
		simX( xdot, x, switch_times_within_range[j], switch_times_within_range[j+1], x_vec, times );
		if(j < length-2) { x_vec.pop_back(); times.pop_back(); }
	}
  }


  /*Piecewise simulate rho based on u_default and "switch_times_within_range" from simx_u_default*/
  inline void isac_step::simrho_u_default() {
	rho_vec.clear(); rho_times.clear(); 
	m_mrho_tf = J1.get_dmdx( );
	for ( j = 0; j < xlen; ++j ) { rho[j] = m_mrho_tf[j]; }// initialize final state 
	//Piecewise Integrate
	for (i_ = length-2; i_ > -1; i_--) { // simulate with default control (initial condition rho is automatically changed from the integrator)
		simRho( rho_dot, rho, switch_times_within_range[i_], switch_times_within_range[i_+1], rho_vec, rho_times ); 
		if(i_ > 0) { rho_vec.pop_back(); rho_times.pop_back(); }
	}

	reverse( rho_vec.begin(), rho_vec.end() );
	reverse( rho_times.begin(), rho_times.end() ); 
  }

 }

#endif  // ISAC_STEP_HPP

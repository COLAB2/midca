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
#ifndef CK_HPP
#define CK_HPP

namespace sac {

  //Class that calculates ck
  class ck {
	phik_hk & erg_;
	size_t i_, j_, o_, indx_;
	double temp, duration1, duration2;	
	state_type integrand_ck, xtime_reduced;
	size_t samples;
	state_type xtemp;
	state_type x_memory_temp;
	std::vector<state_type> x_memory;//stored states representing rolling memory of places visited in memory window
	state_type xtime_memory;//corresponding times
	const int wrap_size_;//number of states to be wrapped
	
  //protected:
  public:
	/*This Value represents the lower integral limit in calculation of ck. It is equal to initial time t_init if
		ergodic_memory_steps = 0. If we have memory, its value is ti-ergodic_memory_steps*ts. */
	double t0_ergodic;  
	state_type c_k, ck_prev;
    
    ck( phik_hk & erg ) :  erg_(erg), c_k(erg_.no_k_comb), ck_prev(erg_.no_k_comb, 0.0),
							t0_ergodic(t_init), x_memory_temp(erg_dim),
							wrap_size_(sizeof(x_wrap)/sizeof(x_wrap[0]))
							{ 
							state_type xnew(xlen);
							xnew.assign(x_init, x_init + xlen);
							update_ck_prev_minus( xnew, t_init, ck_prev );//initialize memory with initial condition
							}
	
	
	inline void ck_integrand( double & integrand, const size_t o, state_type & x ); 
	
	inline void calc_ck( std::vector<state_type> & x_vec, state_type & xtime_vec );
		
	inline void update_ck_prev_plus( std::vector<state_type> & x_vec, state_type & xtime_vec, state_type & ck_prev_out );
	
	inline void update_ck_prev_minus( state_type & x_new, const double & tnew, state_type & ck_prev_in );

  };

	
	/*Calculates integrand value for ck specified*/
	inline void ck::ck_integrand( double & integrand, const size_t o, state_type & x ) { 
		xtemp = x;
		for (indx_ = 0; indx_ < wrap_size_; ++indx_ ) {AngleWrap( xtemp[x_wrap[indx_]] );} // Angle wrapping (if any)
		integrand = 1.0/erg_.hk[o];
		for (j_ = 0; j_ < erg_dim; ++j_) {
			integrand = integrand * cos(erg_.k[o][j_]*PI*xtemp[indices[j_]]/ranges[j_][1]);
		}			
	}
		
	/*Calculate Fourier coefficients of trajectory*/
	inline void ck::calc_ck( std::vector<state_type> & x_vec, state_type & xtime_vec ) {
		samples = xtime_vec.size();
		integrand_ck.resize(samples);
		duration1 = xtime_vec[0] + T - t0_ergodic;
		for (o_ = 0; o_ < erg_.no_k_comb; ++o_) {
			for(i_=0; i_ < samples; ++i_) {
				ck_integrand( integrand_ck[i_], o_, x_vec[i_] );
			}
			trapezoid( c_k[o_], integrand_ck, xtime_vec );
			c_k[o_] = c_k[o_]/duration1 + ck_prev[o_];
		}			
	}
	
	/*Recursively update ck_prev based on its value from previous iterations 
		(integral for ck is split in two parts so that we don't have to 
		repeat overlapping calculations). In particular, instead of calculating integral from
		t_init to ti+T, ckprev represents the integral from t_init to ti.*/
	inline void ck::update_ck_prev_plus( std::vector<state_type> & x_vec, state_type & xtime_vec, state_type & ck_prev_out ) {
		
		integrand_ck.clear();
		xtime_reduced.clear();
		
		duration1 = xtime_vec[0] + T - t0_ergodic;
		duration2 = xtime_vec[0] + ts + T - t0_ergodic;
		for (o_ = 0; o_ < erg_.no_k_comb; ++o_) {
			i_ = 0;
			while(xtime_vec[i_] < xtime_vec[0] + ts) {
				ck_integrand( temp, o_, x_vec[i_] );
				integrand_ck.push_back(temp);
				xtime_reduced.push_back(xtime_vec[i_]);
				i_++;
			}
			trapezoid( temp, integrand_ck, xtime_reduced );
			ck_prev_out[o_] = ck_prev[o_]*duration1/duration2 + 
					temp/duration2;
		}
	}
	
	
	/*This is used if we have system with rolling ergodic memory! After the above function recursively updates ckprev, 
		this function subtracts the portion that corresponds to t < t0_ergodic as memory window moves.
		The function also updates memory of visited places.*/
	inline void ck::update_ck_prev_minus( state_type & x_new, const double & tnew, state_type & ck_prev_in ) {
		for (j_ = 0; j_ < erg_dim; ++j_) {
			x_memory_temp[j_] = x_new[indices[j_]];			
		}
		//Update memory
		xtime_memory.push_back(tnew);
		x_memory.push_back(x_memory_temp);
		
		/*Remove "visited places" if outside rolling memory window
			and subtract portin of ck for t < t0_ergodic*/
		if(xtime_memory.size() > ergodic_memory_steps + 1) {			
			duration2 = tnew-ts + ts + T - t0_ergodic;
			for (o_ = 0; o_ < erg_.no_k_comb; ++o_) {				
				/*ckprev to subtract ***************/
				integrand_ck.clear();
				xtime_reduced.clear();
				for(i_ = 0; i_ < 2; ++i_) {
					ck_integrand( temp, o_, x_memory[i_] );
					integrand_ck.push_back(temp);
					xtime_reduced.push_back(xtime_memory[i_]);					
				}
				trapezoid( temp, integrand_ck, xtime_reduced );				
				ck_prev[o_] = ck_prev_in[o_] - temp/duration2;;
			}		
			xtime_memory.erase(xtime_memory.begin());
			x_memory.erase(x_memory.begin());
		}
		else {
			ck_prev = ck_prev_in;
		}
		
		t0_ergodic = xtime_memory[0];//update t0_ergodic
	}
}


#endif  // CK_HPP

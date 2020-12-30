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

#ifndef PHIK_HK_HPP
#define PHIK_HK_HPP

namespace sac {

  //[ The class that calculates phik and ck
  class phik_hk {
	size_t i_, j_, o_;
  	const size_t grid;	
	Vi outputTemp;
	std::vector<state_type> integrands;//used in trapezoid		
	Vi counters; //set counters to zero
	Eigen::Matrix< double, erg_dim, 1 > x_vec_erg;
	double EID_t_max, EID_sum;//max value of current EID (for normalization)
	
  //protected:
  public:	
	const int no_k_comb, no_terr_comb;
	state_type EID_values_t;//current EID values
	std::vector<state_type> space;//meshgrid of terrain for calculation of hk and phik
	Vvi terr_indices;//indices of terrain (0-30)
	Vvi K_vector;//indices of terrain (0-30)
	Vvi k, all_terr_indices;//vectors that store all combinations of k and terrain indices
	state_type hk, phik;//vector of normalizing values
	
    
    phik_hk( ) :  grid(31), no_terr_comb(pow(grid, erg_dim)), space(erg_dim, state_type(grid)), 
					 terr_indices(erg_dim, Vi(grid)), hk(no_k_comb), phik(no_k_comb), EID_values_t(no_terr_comb),
					 K_vector(erg_dim, Vi(K+1)), no_k_comb(pow(K+1, erg_dim)),
					 integrands(erg_dim+1, state_type(grid)), counters(erg_dim+1) { 
						initialize_cov_means(); //for EID stuff
						initialize();			
						//Find all combinations
						cart_product(k, outputTemp, K_vector.begin(), K_vector.end());
						outputTemp.clear();
						cart_product(all_terr_indices, outputTemp, terr_indices.begin(), terr_indices.end());	
						//calculate normalizing factor for fourier coefficients
						calc_hk( );		
						//Calculate Fourier coefficients
						update_phik( t_init );
						}
			

	inline void initialize( );		
		
	inline void calc_hk( );
	
	inline void calc_norm_EID( const double t, state_type & EID_values_curr );
	
	inline void update_phik( const double t );
	
	inline void hk_integrand( double & integrand, Vi & space_indices, Vi & k_curr );
	
	inline void phik_integrand( double & integrand, const size_t i, const size_t o );

  };
  
  
  		/*Function that finds all combinations of harmonics, and indices of terrain.
			Also calculates the meshgrid values for the terrain (equivalent to linspace)*/
	inline void phik_hk::initialize( ) { 			
		for(int j=0; j < erg_dim; ++j) {				
			for (int i = 0; i < grid; ++i) {			
				//Meshgrid values
				space[j][i] = ranges[j][0] + i*(ranges[j][1]-ranges[j][0])/(grid-1.0);
				//Terrain indices
				terr_indices[j][i] = i;					
			}
			for (int i = 0; i <= K; ++i) { 
				K_vector[j][i] = i;
			}
		}
	}
		
	/*Calculate normalizing factors for Fourier coefficients*/
	inline void phik_hk::calc_hk( ) { 
		for (o_ = 0; o_ < no_k_comb; ++o_) {
			for (i_ = 0; i_ <= erg_dim; ++i_) {counters[i_] = 0;}//reinitialize counters
			for (i_ = 0; i_ < no_terr_comb; ++i_) {
				hk_integrand( integrands[0][counters[0]], all_terr_indices[i_], k[o_] );
				//std::cout << integrands[0][counters[0]] <<"\n";
				counters[0]++;
				for (j_ = 0; j_ < erg_dim; ++j_) {				    
					if (fmod(i_+1.0,pow(grid, j_+1)) < epsilon) {
						 trapezoid( integrands[j_+1][counters[j_+1]], integrands[j_], space[j_] );
						counters[j_] = 0; counters[j_+1]++;
						}
				}
			}			
			hk[o_] = pow(integrands[erg_dim][0], 1.0/erg_dim);//final result for each k vector
			//std::cout << hk[o_]  <<"\n";
		}
			
	}

	/*Calculates EID for all x in terrain and normalizes its values*/
	inline void phik_hk::calc_norm_EID( const double t, state_type & EID_values_curr ) { 
		EID_t_max = 0;//initialize
		for (i_ = 0; i_ < no_terr_comb; ++i_) {
			for (j_ = 0; j_ < erg_dim; ++j_) { x_vec_erg(j_, 0) = space[j_][all_terr_indices[i_][j_]]; }
			EID( t, x_vec_erg, EID_values_curr[i_] );			
			//Find max of current EID
			if(EID_values_curr[i_] > EID_t_max) { EID_t_max = EID_values_curr[i_]; }
		}

		for (j_ = 0; j_ <= erg_dim; ++j_) {counters[j_] = 0;}
		for (o_ = 0; o_ < no_terr_comb; ++o_) {
			integrands[0][counters[0]] = EID_values_curr[o_];
			//std::cout << integrands[0][counters[0]] <<"\n";
			counters[0]++;
			for (j_ = 0; j_ < erg_dim; ++j_) {				    
				if (fmod(o_+1.0,pow(grid, j_+1)) < epsilon) {
					 trapezoid( integrands[j_+1][counters[j_+1]], integrands[j_], space[j_] );
					counters[j_] = 0; counters[j_+1]++;
					}
			}
		}			
		EID_sum = integrands[erg_dim][0];
		
		/*Normalize EID*/
		//EID_t_max = *max_element(EID_values_curr.begin(),EID_values_curr.end());//max value of current EID 
		for (i_ = 0; i_ < no_terr_comb; ++i_) { EID_values_curr[i_] = EID_values_curr[i_]/EID_sum; }
		//std::cout << "The largest element is "  << EID_t_max << '\n';
	}
	
	/*Calculate Fourier coefficients*/
	inline void phik_hk::update_phik( const double t ) {
		calc_norm_EID( t, EID_values_t );//calculate EID	
		for (o_ = 0; o_ < no_k_comb; ++o_) {
			for (i_ = 0; i_ <= erg_dim; ++i_) {counters[i_] = 0;}//reinitialize counters
			for (i_ = 0; i_ < no_terr_comb; ++i_) {
				phik_integrand( integrands[0][counters[0]], i_, o_ );
				//std::cout << integrands[0][counters[0]] <<"\n";
				counters[0]++;
				for (j_ = 0; j_ < erg_dim; ++j_) {				    
					if (fmod(i_+1.0,pow(grid, j_+1)) < epsilon) {
						 trapezoid( integrands[j_+1][counters[j_+1]], integrands[j_], space[j_] );
						counters[j_] = 0; counters[j_+1]++;
						}
				}
			}			
			phik[o_] = integrands[erg_dim][0];//final result for each k vector
			//std::cout << phik[o_]  <<"\n";
		}
	}
			
	/*Calculates integrand value for hk specified by space_indices and k and stores it in integrand.*/
	inline void phik_hk::hk_integrand( double & integrand, Vi & space_indices, Vi & k_curr ) { 
		integrand = 1.0;
		for (j_ = 0; j_ < erg_dim; ++j_) {
			integrand = integrand * pow(cos(k_curr[j_]*PI*space[j_][space_indices[j_]]/ranges[j_][1]), erg_dim);}			
	}
	
	/*Calculates integrand value for phik specified by space_indices and k and stores it in integrand.*/
	inline void phik_hk::phik_integrand( double & integrand, const size_t i, const size_t o ) { 
		integrand = 1.0/hk[o];
		for (j_ = 0; j_ < erg_dim; ++j_) {
			integrand = integrand * cos(k[o][j_]*PI*space[j_][all_terr_indices[i][j_]]/ranges[j_][1]);}			
		integrand = integrand*EID_values_t[i];
	}
	
}


#endif  // PHIK_HK_HPP

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

#ifndef ERG_COST_HPP
#define ERG_COST_HPP

namespace sac {

  /*
    Class that calculates ergodic cost
  */
  class erg_cost {

    size_t j_, i_, o_;
	double temp;
    state_intp & m_x_intp; // store current state
	ck & ck_; // reference to ck instance
	phik_hk & phik_;//reference to phik_hk instance
	std::vector< state_type > * st_vec;//pointer to integrated state vector
	state_type * t_vec;//pointer to time vector from integration
	state_type Lambda_k;//weight the contribution of higher harmonics to cost 
	Eigen::Matrix< double, xlen, 1 > dFkdx_;
  
  public:

    erg_cost( state_intp & x_intp, ck & c_k, phik_hk & phik): 
 		 m_x_intp( x_intp ), st_vec(m_x_intp.m_states),
		 t_vec(m_x_intp.m_times), ck_(c_k), phik_(phik),
		 Lambda_k(phik_.no_k_comb) 
		 { initialize_Lambda_k( phik_.k ); }
  
  
	/*Initialize weights set on harmonics contribution*/
  	void initialize_Lambda_k( Vvi & k ) {
		for(int i = 0; i < phik_.no_k_comb; ++i) {
			calc_norm( temp, k[i] ); 
			Lambda_k[i] = 1.0/pow(1+pow(temp, coef_weight_decay), (erg_dim + 1.0)/2.0);
			//std::cout << Lambda_k[i] << "\n";
		}
	}
  
  
    /*Computes ergodic cost */
    double get_ergodic_cost( )
    {
		ck_.calc_ck( *st_vec, *t_vec );//calculate ck for this trajectory
		temp = 0;
		for (j_ = 0; j_ < phik_.no_k_comb; ++j_) {
			temp = temp + Lambda_k[j_]*(ck_.c_k[j_] - phik_.phik[j_])*(ck_.c_k[j_] - phik_.phik[j_]);
		}		
		return temp*Q_erg;
    }
	


	/* Calculate dFkdx */
    Eigen::Matrix< double, xlen, 1 > calc_dFkdx( const int index, const Eigen::Matrix< double, xlen, 1 > &x_curr ) { 
		dFkdx_ = Eigen::Matrix< double, xlen, 1 >::Zero();//initialize to zero
		for(i_ = 0; i_ < erg_dim; ++i_) {
			dFkdx_(indices[i_],0) = -(phik_.k[index][i_]*PI/ranges[i_][1])*sin(phik_.k[index][i_]*PI/ranges[i_][1]*x_curr(indices[i_],0))/phik_.hk[index];
			for(o_ = 0; o_ < erg_dim; ++o_) {
				if(i_!=o_) { dFkdx_(indices[i_],0) = dFkdx_(indices[i_],0)*cos(phik_.k[index][o_]*PI/ranges[o_][1]*x_curr(indices[o_],0)); }
			}
		}
		return dFkdx_;
    }
	

    /* Compute ergodic dl/dx used in adjoint calculation */
    void dx( const Eigen::Matrix< double, xlen, 1 > &x_curr,
		    Eigen::Matrix< double, xlen, 1 > &dldx ) { 

		dldx = Eigen::Matrix< double, xlen, 1 >::Zero();//initialize to zero				
		temp = (*t_vec)[0] + T - ck_.t0_ergodic;//first term is t0
		for (j_ = 0; j_ < phik_.no_k_comb; ++j_) {
			dldx = dldx + 2.0*Lambda_k[j_]*(ck_.c_k[j_] - phik_.phik[j_])*calc_dFkdx( j_, x_curr )/temp;
		}
		dldx = dldx * Q_erg;
    }

  };

}

#endif  // ERG_COST_HPP

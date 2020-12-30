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

#ifndef ERG_SETTINGS_HPP
#define ERG_SETTINGS_HPP

namespace sac {

	typedef std::vector<int> Vi;
	typedef std::vector<Vi> Vvi;
    
  
	
	/* Terrain dimensions set as [0, Lmax]. 
		Dynamics and measurements will have to be scaled down to this terrain for real systems.
		 Maximum range should be 0-1 for best results (even for rectangular real terrains) !!!*/
	const double ranges[][2] = {{0, 1}, {0, 1}};

	/*Indices of system states that correspond to the range above. Number of indices also defines dimensionality of problem.*/
	const int indices[] = {0, 1};
	const int erg_dim(sizeof(indices)/sizeof(*indices));//don't change
   
	/*max number of harmonics for Fourier coefficients*/
	const int K = 5;
	
	/*Ergodic cost weight*/
	const double Q_erg = 55;
	
	/*Used to place larger weight on lower frequency information - Lambda_k = 1/(1+|k|^coef_weight_decay)^((erg_dim+1)/2.0)
		The smaller this value, the more the higher frequencies contribute to the cost*/
	const double coef_weight_decay = 2.0;
	
	
	/********************************************
	Ergodic memory: Used to create memory of visited places (e.g. for time-varying EIDs)*/
	
	/*Set following to *ZERO* if you want the system to remember *EVERYTHING* from initial time.
		A value of e.g. 10 means that the system has a (rolling) memory window of 10*ts which will be equal
		to the difference between upper and lower integral limit in ck*/
	const int ergodic_memory_steps = 0;

	/*Expected Information Density 
		Bimodal gaussian for this example*/	
	const double EID_max = 40;
		
		
	Eigen::Matrix< double, erg_dim, erg_dim > Sigma;//covariance matrix
	Eigen::Matrix< double, erg_dim, erg_dim > Sigma_inv;//inverse covariance matrix
	Eigen::Matrix< double, erg_dim, 1 > mu;
	
	Eigen::Matrix< double, erg_dim, erg_dim > Sigma2;//covariance matrix
	Eigen::Matrix< double, erg_dim, erg_dim > Sigma_inv2;//inverse covariance matrix
	Eigen::Matrix< double, erg_dim, 1 > mu2;
	
	float EIDVal( Eigen::Matrix< double, erg_dim, 1 > & x);
	void initialize_cov_means() {
		Sigma << pow(0.07,2), 0,        
				 0, pow(0.07,2); 
		Sigma_inv = Sigma.inverse();
	}

	inline void EID( const double t, Eigen::Matrix< double, erg_dim, 1 > & x, double & eid_t ) {
		eid_t = sac::EIDVal(x);//custom EID function
		//mu(0,0) = 0.25*cos(t/50) + 0.5;
		//mu(1,0) = 0.25*sin(t/50) + 0.5;
		//eid_t = (1.0/sqrt(pow(2.0*PI, erg_dim)*Sigma.determinant()))*exp(-0.5*(x-mu).transpose()*Sigma_inv*(x-mu));
	}
	 
	

}

#endif  // ERG_SETTINGS_HPP

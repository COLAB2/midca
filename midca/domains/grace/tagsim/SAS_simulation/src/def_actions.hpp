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
#ifndef DEF_ACTIONS_HPP
#define DEF_ACTIONS_HPP

namespace sac {

  //[ The class to manipulate default actions, i.e. actions within the current time window
  class def_actions {
  size_t j;
  std::ofstream myfile;
  
  public:
    /* public initialization */
	std::vector<state_type> default_actions;//contains default actions (t_i, t_f, magnitude) columnwise in current time window [t0, tf] 
    state_type default_switch_times; //vector containing unique, ordered switching times of default actions

    
	def_actions( ) : default_actions(ulen+2) { 
		if (save_actions_to_file) {myfile.open ("./data/actions.csv");} 
		}

    inline void update_default_actions( double & t_i, double & t_f, state_type & u_switch );
	
	inline void get_default_actions( double &t0 );

    inline void update_default_switch_times( );

	inline void u_default( double t, const state_type & x, state_type & u_def_t );
};


  /*Prepares default actions "matrix" for next iteration, i.e. 
		adds new actions if any*/

  inline void def_actions::update_default_actions( double & t_i, double & t_f, state_type & u_switch ) {
  
	if (!((std::abs( t_f - t_i ) < epsilon) || (t_i > t_f))) { 	//if we have a new action  
		/*************************************************************************************
			Update default actions by adding the newest one*/	
		if (default_actions[0].size() > 0) {//If default actions is not empty 
			iter_1d low, up;
			
			/*First check if new action completely "covers" a default ones; if so erase the corresponding default actions*/
			//Find index of default action such that default ti > t_i
			low=std::upper_bound (default_actions[0].begin(), default_actions[0].end(), t_i); 
			//Find index of default action such that default tf > t_f (erase method is not inclusive so in the end we get default td <=t_f)
			up= std::upper_bound (default_actions[1].begin(), default_actions[1].end(), t_f); 
  
			if ((up - default_actions[1].begin()) > (low- default_actions[0].begin())){//if there is coverage erase corresponding default actions
				default_actions[0].erase(low, default_actions[0].begin() + (up - default_actions[1].begin()));
				default_actions[1].erase(default_actions[1].begin() + (low - default_actions[0].begin()), up);
				for ( j = 0; j < ulen; ++j ) { 
					default_actions[j+2].erase(default_actions[j+2].begin() + (low - default_actions[0].begin()), default_actions[j+2].begin() + (up - default_actions[1].begin())); }
			}					
			
			//Now, find index of first of remaining default actions that satisfies t_i < default_t_i
			//this index (low) is equivalent to index+1 in matlab
			low = std::upper_bound (default_actions[0].begin(), default_actions[0].end(), t_i); //ti row
			//
			if( low!=default_actions[0].begin() ){ //we are not inserting at the first position
				//First check if new action splits any of the old actions in half and if so split it
				size_t idx = low-default_actions[0].begin()-1;//index of previous action
				if(default_actions[1][idx] > t_f + epsilon) { //need to split action
					double temp = default_actions[1][idx];//store t_f of default action to be split
					default_actions[1][idx] = t_i;//replace it with t_i of the new action
					//Create second half of split action and insert it next
					default_actions[0].insert(default_actions[0].begin() + idx + 1, t_f);
					default_actions[1].insert(default_actions[1].begin() + idx + 1, temp);
					for ( j = 0; j < ulen; ++j ) { 
						default_actions[j+2].insert(default_actions[j+2].begin() + idx + 1, default_actions[j+2][idx]);	}

					//Now insert new action
					default_actions[0].insert(default_actions[0].begin() + idx + 1, t_i);
					default_actions[1].insert(default_actions[1].begin() + idx + 1, t_f);
					for ( j = 0; j < ulen; ++j ) { 
						default_actions[j+2].insert(default_actions[j+2].begin() + idx + 1, u_switch[j]); }

				} else {//No need to split action follow matlab's procedure
						if ((default_actions[1][idx] > t_i) || (std::abs(default_actions[1][idx] - t_i) < epsilon)) {
							default_actions[1][idx] = t_i; }

						if (idx != default_actions[0].size()-1) {
							if ( (default_actions[0][idx+1] < t_f) || (std::abs(default_actions[0][idx+1] - t_f) < epsilon) ) {
								default_actions[0][idx+1] = t_f; }
						}
						//Now insert new action
						default_actions[0].insert(default_actions[0].begin() + idx + 1, t_i);
						default_actions[1].insert(default_actions[1].begin() + idx + 1, t_f);
						for ( j = 0; j < ulen; ++j ) { 
							default_actions[j+2].insert(default_actions[j+2].begin() + idx + 1, u_switch[j]); }					
				}
			}
			else{//we are inserting at the first position
				if((default_actions[0][0] < t_f) || (std::abs(t_f - default_actions[0][0])<epsilon)) 
					{default_actions[0][0] = t_f;}
				//Insert new action at the begining
				default_actions[0].insert(default_actions[0].begin(), t_i);	
				default_actions[1].insert(default_actions[1].begin(), t_f);
				for ( j = 0; j < ulen; ++j ) { 
					default_actions[j+2].insert(default_actions[j+2].begin(), u_switch[j]);	}
			}
		}
		else //if empty
		{ 	//Add new action
			default_actions[0].push_back(t_i);
			default_actions[1].push_back(t_f);
			for ( j = 0; j < ulen; ++j ) { default_actions[j+2].push_back(u_switch[j]); }
		}	
	}	
  }
  
  /*Get default actions that occur after current t0 (write to file and erase the rest)*/
  inline void def_actions::get_default_actions( double &t0 ) {
	/*************************************************************************************
	Remove actions that occur before t0 + ts (writes these to file?) */	
	if (default_actions[0].size() > 0) {  //if we have default actions
		int counter = 0;		
		//First find how many entries we are removing
		for ( j = 0; j < default_actions[0].size(); ++j ) { 
			if ((default_actions[0][j] < t0) && (default_actions[1][j] < t0)) {counter++;}
			else {break;}
		}
		//Write to file and erase actions that occured before current t0
		for ( j = 0; j < counter; ++j ) {
			for ( int i = 0; i < 2+ulen; ++i ) { 
				if (save_actions_to_file) {myfile << default_actions[i][0] << ", ";}				
				default_actions[i].erase(default_actions[i].begin());				
				} 
			if (save_actions_to_file) {myfile << "\n";}
		}
		//Fix t_i for first entry
		//if((default_actions[0][0] < t0) || (std::abs(t0 - default_actions[0][0])<epsilon)) 
			//{default_actions[0][0] = t0;}		
	}
  }


  /*Prepares default_switch_times *vector*
	which contains unique, ordered switching times of default actions */
  inline void def_actions::update_default_switch_times( ) {
	default_switch_times.clear();
  	if (default_actions[0].size() > 0) {  //if we have default actions
		default_switch_times = default_actions[0];
		default_switch_times.insert(default_switch_times.end(), default_actions[1].begin(), default_actions[1].end());//append tf values
		//sort values
		std::sort (default_switch_times.begin(), default_switch_times.end());
		//keep unique elements
		default_switch_times.erase( unique( default_switch_times.begin(), default_switch_times.end(), isequal ), default_switch_times.end() );

	}
  }
  
  
    /* Returns default control at time t for x. */
  inline void def_actions::u_default( double t, const state_type & x, state_type & u_def_t )
  {  
	iter_1d low, up;

	if(default_actions[0].size() > 0) {//if we have default actions
		//Calculate bounds on the ti and tf subvector ot default actions such that ti<=t<=tf
		low = std::upper_bound (default_actions[0].begin(), default_actions[0].end(), t); //ti row
		up = std::upper_bound (default_actions[1].begin(), default_actions[1].end(), t); // tf row	 		
				
		if((low - default_actions[0].begin())>(up - default_actions[1].begin())){//if indexes of bounds of ti and tf vectors are different
			for ( size_t i=0; i<ulen; ++i ) { u_def_t[i] = default_actions[i+2][(up - default_actions[1].begin())]; }//Assign default action value	[)		
		} 
		else {u_nominal( t, x, u_def_t );}//calculate nominal control at time t	
	} else {u_nominal( t, x, u_def_t );}//calculate nominal control at time t
  }


}

#endif  // DEF_ACTIONS_HPP

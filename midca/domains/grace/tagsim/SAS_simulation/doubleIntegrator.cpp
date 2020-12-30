
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

#include <master.hpp>               // Master include file
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <sstream>
#include <algorithm>
#include <iterator>
#include <ctime> //for timing
#include <gp.h>
#include <rprop.h>

using namespace libgp;
namespace sac{
	float averageVariance=1;
	float maxVariance=1;
	float maxMean=1;
	float scale;
	//used for EID
	 // initialize Gaussian process for 2-D input using the squared exponential 
     // covariance function with additive white noise.
     GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
	 // initialize hyper parameter vector
	 
	float EIDVal(Eigen::Matrix< double, erg_dim, 1 > & x){
		float pad =-0.01;
		if((x[0] > ranges[0][1]+pad)||(x[0] < ranges[0][0]-pad)){
			return 0.0;
	    }
		if((x[1] > ranges[1][1]+pad)||(x[1] < ranges[1][0]-pad)){
			return 0.0;
	    }
		float minVariance = 0.2;
		float alpha = .4;
		double pos[] = {x[0]*scale, x[1]*scale}; 
		float val;
		float mu=gp.f(pos);
		float var = gp.var(pos);
		val = 1e-6+EID_max*(alpha*mu/maxMean+(1-alpha)*(var>=minVariance)*var/maxVariance);
		//val = 1e-6+mu*var+var+(var>=averageVariance||minVariance>averageVariance)*mu;
		//val = 1e-6+10*var+mu+EID_max/2*(mu<0)+(mu>0)*mu;
		//val=1e-6+(1+(var/averageVariance)*(var>0.002))*mu;
		//val=1e-6+(1-(var/averageVariance))*mu;
        //val =EID_max/2*(mu<=1e-5)+(mu>1e-5)*mu;//second best
		//val =EID_max/2*(mu==0)+(!mu==0)/(mu+.000001);
		//val =10*(gp.var(pos)==0)+gp.var(pos);
		//val = 1.0/(EID_max/10.0-gp.f(pos)*(1+gp.var(x)));
		//val = 1.0/(EID_max/10.0-gp.f(pos));
		//val = 1+gp.f(pos); 
		//val = .1+3*gp.f(pos)*10*(.0001+gp.var(pos)); //interesting
		//val = 0.01+100*gp.var(pos);//best
	    //val = std::min(gp.var(pos)+.00001,EID_max);
		//val= std::min(1/var,EID_max);
		//val = 1/var;
		//if(val>=10){
			//std::cout << "EID function val=" << val<< ", gp mean=" << mu  << ", x="<< x[0]*scale << ", y="<< x[1]*scale << "\n";
		//}
		return val; 
	}//std::cout << x[0] << ","<<x[1] <<"\n";
}
using namespace sac;


template <class Container>
void split1(const std::string& str, Container& cont)
{
    std::istringstream iss(str);
    std::copy(std::istream_iterator<std::string>(iss),
         std::istream_iterator<std::string>(),
         std::back_inserter(cont));
}
/* iSACstep() function operator
  input:  initial state and time
  return: Does not explicitly return anything but the following fields of class "sac_step" can be accessed

  iSACstep.xnext - integrated state at time t0+ts

  iSACstep.u_switch - vector of SAC action values applied from [t_i, t_f] which is a subset of [t0, t0+ts].
          If [t_i, t_f] is not equal to [t0, t0+ts] then the default control is applied over the remaining interval. 
  t_i - initial time for application of the control.  t0 <= t_i <= t0+ts
  t_f - final time for control application.  t0 <= t_f <= t0+ts

  WARNING: iSACstep.u_switch is only applied when t_f-t_i > 0, otherwise u_default is applied.
  WARNING: If [t_i, t_f] is not equal to [t0, t0+ts] then u_default is applied 
           over the remaining interval.
  NOTE: for speed return and input types should be changed and passed as
        references / pointers
*/

#define PORT 8080
int main(int /* argc */ , char** /* argv */ )
{
	using namespace std;
	/*********************************************/
	/* Vars etc*/
	isac_step iSACstep;//instance 
	state_type x0(xlen);
	Eigen::Matrix< double, xlen, 1 > xnext;//for prints
	int i;
	float x,y,rate;
	std::vector<std::string>  holder;
	ofstream myfile,gpdata,plannedTraj;
  	myfile.open ("./data/states.csv");//open file to save stuff
	//gpdata.open ("./gp.csv");
	//plannedTraj.open ("./data/planedstates.csv");
	//gpdata << "EID value, gp mean, gp var,x,y\n";
    
     Eigen::VectorXd params(gp.covf().get_param_dim());
     params << 0.1, 1, -2.0;
	 //params <<4.80081,-0.542749,-0.641606;
	 //params << 0.0, 0.0, -2.0;
     // set parameters of covariance function
     gp.covf().set_loghyper(params);
	 RProp prop;
     prop.init(0.5,0.1,1e-6,50,0.5,1.2);
     int updateCount = 0;
	/*********************************************/



    char buff[1024];
	int master_sock;
    int addrlen , new_socket;
	int opt = 1;
	
	//create unix domain socket
	master_sock = socket(AF_INET,SOCK_STREAM,0);
	struct sockaddr_in addr;
	memset(&addr,0,sizeof(addr));
    addr.sin_addr.s_addr = INADDR_ANY; 
	addrlen = sizeof(addr);
	if(setsockopt(master_sock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt)))
	{ 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    }
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY; 
    addr.sin_port = htons( PORT );
	
	if ((bind(master_sock, (struct sockaddr *)&addr, sizeof(addr))) == -1){
                //fprintf(stderr, "Error on bind --> %s", strerror(errno));
				perror("binding");
                exit(EXIT_FAILURE);//exit(-1);return;
    }
	if (listen(master_sock, 3) < 0){   
        perror("listen");   
        exit(EXIT_FAILURE);   
    }
    if ((new_socket = accept(master_sock,(struct sockaddr *)&addr, (socklen_t*)&addrlen))<0) {   
		perror("accept error"); 
	} 
	/*********************************************/
	
	/* Initializations*/
	//get state from simulation
	read(new_socket,buff, sizeof(buff));
    scale = atof(buff);	
	std::cout << scale <<"\n";
	size_t sz;
	read(new_socket,buff, sizeof(buff));
	split1(buff, holder);
	cout << buff <<"," <<holder.back()<<"\n";
	for (i=0; i < xlen; ++i) { x0[i] = std::stof (holder[i],&sz);}//cout << i <<","<< holder[i] << "\n";}
	//send(new_socket,"get_state",strlen("get_state"),MSG_NOSIGNAL);
	/* Receding horizon loop*/
	double t0 = t_init;
	bool run = true;
	while (run){
	//while (t0 < 60 ){
		t0 = std::stof (holder[xlen],&sz);
		/* Perform SAC iteration - updates: J0, Jn, u, x_intp */
		iSACstep( t0, x0 );
		
		//Prints
		State2Mat( iSACstep.xnext, xnext ); // convert state to matrix form to be able to print state directly
		myfile << t0 << " " << xnext.transpose() << "\n";//write to file
		//cout << t0 << ", " << iSACstep.xnext[0] << ", " << iSACstep.xnext[1] << ", " << iSACstep.u_switch[0] << ", " << iSACstep.u_switch[1] << "\n";
		// send control to simulation
		std::string msg = std::to_string(iSACstep.u_switch[0])+","+std::to_string(iSACstep.u_switch[1]);
		send(new_socket,msg.c_str(),msg.size(),MSG_NOSIGNAL);
		//for (i=0; i < iSACstep.x_vec.size(); ++i) {cout  << iSACstep.x_vec[i][0] <<","<< iSACstep.x_vec[i][1] <<"\n";}	
		//cout  << "\n";
		bool outOfBounds=false;
		for(i=0; i<erg_dim; ++i) {
			if((iSACstep.xnext[indices[i]] > ranges[i][1])||(iSACstep.xnext[indices[i]] < ranges[i][0])) {
				std::cout << "Out of bounds" <<"\n";
				outOfBounds=true;
				//system("pause");
				}
		}
		//get new state from simulation	
		memset(buff,0,sizeof(buff));
	    read(new_socket,buff, sizeof(buff));
		holder.erase(holder.begin(),holder.end());
		split1(buff, holder);
		
		if (holder.front().compare("end")==0){
			cout << buff << "\n";
			run=false;
			continue;
		}
		//cout << buff  <<"-buff2\n"<< run <<"\n";
	    for (i=0; i < xlen; ++i) { x0[i] = std::stof (holder[i],&sz);}//cout << i <<","<< holder[i] << "\n";}
		double pos[] = {x0[0]*scale, x0[1]*scale};
		Eigen::Matrix<double, 2, 1> pos2; pos2(0,0) = x0[0]; pos2(1,0) = x0[1];
		bool updateMeas = holder[xlen+1].compare("None")==0;
		if (!updateMeas) {
			
			cout << holder[xlen+1]  <<"<-measured value, time=" << t0 << ", px=" << iSACstep.xnext[0]*scale << ", py=" << iSACstep.xnext[1]*scale <<"\n";
			double temp = EIDVal(pos2);
			std::cout <<  " EID value =" << temp << ", gp mean=" << gp.f(pos) << ", gp var="<<gp.var(pos)<< ",x=" << pos[0] << ",y=" << pos[1] << ", outOfBounds = "<<outOfBounds<<"\n";
			
			if (outOfBounds){
				//rate=0;
				rate = std::stof (holder.back(),&sz);
			}
			else{
				++updateCount;
				rate = std::stof (holder.back(),&sz);
				if( (updateCount<500 && updateCount>50) || updateCount%50==0){
				     prop.maximize(&gp,std::max(200-updateCount,3),.5);
				}
				if (updateCount%25==0){
					gpdata.open ("./"+std::to_string(updateCount/25)+"gp.csv");
					float res = 20.0;
					//float lastMaxVariance=averageVariance;
					averageVariance=0;
					int avecount=0;
					float avar=0;
					float amu=0;
					maxVariance=0;
	                for(i=0;i<res;i++){for(int j=0;j<res;j++){double p[] = {i*scale/res+.5*scale/res, j*scale/res+.5*scale/res};avar= gp.var(p);if(maxVariance<avar){maxVariance=avar;}}}
					if(maxVariance==0){maxVariance=1;}
					for(i=0;i<res;i++){
						for(int j=0;j<res;j++){
							double p[] = {i*scale/res+.5*scale/res, j*scale/res+.5*scale/res};
							Eigen::Matrix<double, 2, 1> p2; p2(0,0) = i/res; p2(1,0) = j/res;
							avecount+=1;
							avar= gp.var(p);
							amu=gp.f(p);
							averageVariance+=avar;
							//if(lastMaxVariance<avar){lastMaxVariance=avar;}
							if(maxMean<amu){maxMean=amu;}
							double temp2 = EIDVal(p2);
							gpdata <<  temp2 << "," << amu << ","<< avar << "," << p[0] << "," << p[1] << "\n";
						}
					}
					gpdata.close();
					averageVariance=averageVariance/avecount;
					if(averageVariance==0){averageVariance=1;}
					if(maxMean==0){maxMean=1;}
				}
			}
			if (!outOfBounds && !isnan(x0[0]) && !isnan(x0[1]) && !isnan(rate)){
                gp.add_pattern(pos, rate);
			}
			std::cout << "measurement num = " <<updateCount << ", maxMean = " << maxMean<< ", maxVariance = " << maxVariance<< ", average variance = " << averageVariance << "\n"; 

		}
	}
    
    float res = 40.0;
	double maxposval[] ={0,0,0};
	gpdata.open ("./"+std::to_string(updateCount/25+1)+"gp.csv");
	averageVariance=0;
	int avecount=0;
	float avar=0;
	float amu=0;
	maxVariance=0;
	for(i=0;i<=res;i++){for(int j=0;j<=res;j++){double p[] = {i*scale/res+0*scale/res, j*scale/res+0*scale/res};avar= gp.var(p);if(maxVariance<avar){maxVariance=avar;}}}
    for(i=0;i<=res;i++){
		for(int j=0;j<=res;j++){
			//Eigen::Matrix<double, 2, 1> pos = {i/res, j/res};
			double pos[] = {i*scale/res+0*scale/res, j*scale/res+0*scale/res};
		    Eigen::Matrix<double, 2, 1> pos2; pos2(0,0) = i/res; pos2(1,0) = j/res;
			
			avecount+=1;
			avar= gp.var(pos);
			amu=gp.f(pos);
			averageVariance+=avar;
			//if(maxVariance<avar){maxVariance=avar;}
			if(maxMean<amu){maxMean=amu;}
			double temp = EIDVal(pos2);
			std::cout << "EID value =" << temp << ", gp mean=" << amu << ", gp var="<< avar << ",x=" << pos[0] << ",y=" << pos[1] << "\n";
			gpdata <<  temp << "," << amu << ","<< avar << "," << pos[0] << "," << pos[1] << "\n";
			if (temp > maxposval[0]){
				maxposval[0]=temp; maxposval[1]=pos[0]; maxposval[2]=pos[1];
			}
		}
	}
	averageVariance=averageVariance/avecount;
	gpdata.close();
    std::cout << "max EID value =" << maxposval[0] << " at x=" << maxposval[1] << ",y=" << maxposval[2] << "\n";
	std::cout << "maxMean = " << maxMean<< ", maxVariance = " << maxVariance << ", averageVariance = " << averageVariance << ", measurement samples = " <<updateCount<< "\n\n";
    params << gp.covf().get_loghyper();
	cout << "gp params optimized " << params[0] << "," << params[1] << "," << params[2];"\n\n";
	prop.maximize(&gp,200,.1);
	params << gp.covf().get_loghyper();
	cout << "\ngp params second optimization " << params[0] << "," << params[1] << "," << params[2]<<"\n\n";
	myfile.close();//close file
    gpdata.close();
    plannedTraj.close();

	return 0;
}







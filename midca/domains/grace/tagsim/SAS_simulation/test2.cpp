#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <gp.h>
#include <rprop.h>
#include <gp_utils.h>
#include <string>
#include <Eigen/Dense>
#include <iostream>
using namespace libgp;

std::string srv_name ="test";
int main (int argc, char const *argv[])
{
  // char buff[1024];
	 int master_sock;
	// int curr_cli = 0;
    int addrlen , new_socket;
          // max_clients = 40 , 
		  // activity, 
		  // i , 
		  // valread ,
		  // sd;   
    // int max_sd,
	    // client_socket[max_clients];  
	// memset(client_socket,0,sizeof(int)*max_clients);
	//set of socket descriptors  
    // fd_set readfds;
	//create unix domain socket
	master_sock = socket(AF_UNIX,SOCK_STREAM,0);
	struct sockaddr_un addr;
	memset(&addr,0,sizeof(addr));
	addr.sun_path[0]=0;
	for (std::string::size_type i = 0; i < srv_name.size(); i++) {
		addr.sun_path[i+1]=srv_name[i];
	}
	addrlen = strlen(addr.sun_path);
	addr.sun_family = AF_UNIX;
	int len = sizeof(sa_family_t)+srv_name.size()+1;
	//printf("path: %s, should be: %s \n",addr.sun_path+1,srv_name);
	if ((bind(master_sock, (struct sockaddr *)&addr, len)) == -1){
                //fprintf(stderr, "Error on bind --> %s", strerror(errno));
				perror("binding");
                exit(EXIT_FAILURE);//exit(-1);return;
    }
	if (listen(master_sock, 3) < 0){   
        perror("listen");   
        exit(EXIT_FAILURE);   
    } 
	//if ((new_socket = accept(master_sock,(struct sockaddr *)&addr, (socklen_t*)&addrlen))<0) {   
	//	perror("accept error"); 
	//} 
  int n=4000, m=1000;
  double tss = 0, error, f, y;
  // initialize Gaussian process for 2-D input using the squared exponential 
  // covariance function with additive white noise.
  GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");
  
  // initialize hyper parameter vector
  Eigen::VectorXd params(gp.covf().get_param_dim());
  params << 0.0, 0.0, -2.0;
  // set parameters of covariance function
  gp.covf().set_loghyper(params);
  // add training patterns
  for(int i = 0; i < n; ++i) {
    double x[] = {drand48()*4-2, drand48()*4-2};
    y = Utils::hill(x[0], x[1]) + Utils::randn() * 0.1;
    gp.add_pattern(x, y);
  }
  RProp prop;
  prop.init(1,0.1,1e-6,50,0.5,1.2);
  prop.maximize(&gp,100,1);
  // total squared error
  for(int i = 0; i < m; ++i) {
    double x[] = {drand48()*4-2, drand48()*4-2};
    f = gp.f(x);
	std::cout << "mean=" << f << " var="<<gp.var(x)<< " x="<<x[0] << " y=" <<x[1] << std::endl;
    y = Utils::hill(x[0], x[1]);
    error = f - y;
    tss += error*error;
  }
  std::cout << "mse = " << tss/m << std::endl;
	return EXIT_SUCCESS;
}

modified from https://github.com/MurpheyLab/ergodic_iSAC_exploration and https://github.com/mblum/libgp
# ergodic_iSAC_exploration
C++ code implementing ergodic iSAC: an ergodic control algorithm that uses complex agent dynamics to explore a varying probability-of-detection distribution in real time. The current implementation uses a first order linear system and a gaussian process for exploration of unknown fields.

# Dependencies
The code requires the Boost and Eigen libraries.

# To compile and run
	--- Update Makefile.txt with local Boost and Eigen paths
	--- build the GP library from https://github.com/mblum/libgp and copy to include folder
	--- "make"
	--- Run ./singleIntegrator or ./doubleIntegrator in command line
	--- Plot resulting trajectories in Matlab, using ./data/plots_matlab.m or excel

# Customization
All possible changes, e.g. agent dynamics, explored distribution, additional performance cost etc., can be made by updating the files included in the "user" folder.








CPP = g++
#PROJ = singleIntegrator
PROJ = singleIntegrator
#PROJ = singleIntegratorCellByCell
#PROJ = doubleIntegratorCellByCell
DEPS = $(PROJ).cpp
LIBDIR = ./src
INCDIR = ./include
USERDIR = ./user
LFLAGS = -L./include
BOOST = /usr/include/boost
EIGEN = /usr/include/eigen3


# Prefer static libs to dynamic
LIBS = -Wl,-Bstatic \
	  	 -Wl,-Bdynamic
 
#CFLAGS = -I/home/emmanouil/eigen-eigen-07105f7124f9/ -I/home/emmanouil/boost_1_60_0 -I$(USERDIR) -I$(LIBDIR)
#CFLAGS = -I"C:\c_libraries\eigen-eigen-dc6cfdf9bcec" -I"C:\c_libraries\boost_1_61_0" -I$(LIBDIR) -I$(INCDIR) -I$(USERDIR)

CFLAGS = -I$(USERDIR) -I$(LIBDIR) -I$(INCDIR) -I$(BOOST) -I$(EIGEN)

# Default code optimization level
OPT = -O2

all: $(PROJ)

# Debug build with extra warning turned on and optimization off
debug: OPT = -O0 -g -m64
debug: CFLAGS += -Wall -Wextra 
debug: $(PROJ)

$(PROJ): $(DEPS)
	$(CPP) $(CFLAGS) $^ -o $@ $(OPT) $(LFLAGS) $(LIBS) -lgp

clean:
	rm -f *.o $(PROJ)

#!/usr/bin/env bash
# Variables to change
# iterations
# TIME_LIMIT : the maximum time the moos should run in seconds
# Note : please introduce a delay of 5 sec in the python script and terminate the program
#         only then it goes for another run


Experiments=99
midcapath=/home/sampath/Documents/git/midca/midca/examples/midca_with_regular_ergodic.py #path of the python program
fieldpath=/home/sampath/Documents/git/midca/midca/domains/grace/tagsim/anomalies/MidcaErgodicExperimentIncremental.py

counter=0

# For safety
sudo kill -9 $(sudo lsof -t -i:5700) >/dev/null 2>&1
sudo kill -9 $(sudo lsof -t -i:5701) >/dev/null 2>&1
killall -9 Python

sleep 2

while [ $counter -le $Experiments ]
do
  echo "Welcome $counter times"
  sleep 2
  printf "Launching The field demo and midca\n"

  ./singleIntegrator >& SAS.txt & sleep 2
  python $fieldpath $counter >& field_output.txt &
  python $midcapath >& midca_output.txt & wait
  #printf "Killing the process ... \n"

  sudo kill -9 $(sudo lsof -t -i:5700) >/dev/null 2>&1
  sudo kill -9 $(sudo lsof -t -i:5701) >/dev/null 2>&1
  killall -9 Python
  printf "Done killing process.   \n"
  sleep 2
  wait
  ((counter++))
done
  echo "Done, Completed all the iterations"
  exit 0


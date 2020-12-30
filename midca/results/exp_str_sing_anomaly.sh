#!/usr/bin/env bash
# Variables to change
# iterations
# TIME_LIMIT : the maximum time the moos should run in seconds
# Note : please introduce a delay of 5 sec in the python script and terminate the program
#         only then it goes for another run


Experiments=99
midcapath=/Users/sravyakondrakunta/Documents/git/GracegridMIDCA/midca/examples/midca_nsf.py #path of the python program
fieldpath=/Users/sravyakondrakunta/Documents/git/GracegridMIDCA/midca/domains/nbeacons/tagsim/anomalies/MidcaErgodicExperimentIncremental.py

counter=0

# For safety
lsof -ti tcp:5700 | xargs kill
lsof -ti tcp:5701 | xargs kill
killall -9 Python

sleep 2

while [ $counter -le $Experiments ]
do
  echo "Welcome $counter times"
  sleep 2
  printf "Launching The field demo and midca\n"

  ./singleIntegratorCellByCell >& SAS.txt & sleep 2
  python $fieldpath $counter >& field_output.txt &
  python $midcapath >& midca_output.txt & wait
  #printf "Killing the process ... \n"

  lsof -ti tcp:5700 | xargs kill
  lsof -ti tcp:5701 | xargs kill
  killall -9 Python
  printf "Done killing process.   \n"
  sleep 2
  wait
  ((counter++))
done
  echo "Done, Completed all the iterations"
  exit 0


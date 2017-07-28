#!/bin/bash

#./example 1 zero topdown
#./example 1 noise topdown
#./example 1 sinewave topdown
#./example 1 normal topdown

#./example 2 zero topdown
#./example 2 noise topdown
#./example 2 sinewave topdown
#./example 2 normal topdown

./example 3 noise topdownrate #learning rate controller : 0.01 model : 0.001
mv topdownrate_noise_3layer_input.csv newtopdown/
mv topdownrate_noise_3layer_output.csv newtopdown/

#./example 3 zero topdown 
./example 3 noise topdown 
#./example 3 sinewave topdown
./example 3 normal topdown

#./example 5 zero topdown
#./example 5 noise topdown
#./example 5 sinewave topdown
#./example 5 normal topdown

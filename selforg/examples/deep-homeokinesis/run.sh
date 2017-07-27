#!/bin/bash

g++ -Wall -o example deep-homeokinesis.cpp -pthread -lselforg -lgsl -lgslcblas -lconfigurator -L /usr/bin

./example 1 zero topdown
./example 1 noise topdown
./example 1 sinewave topdown
./example 1 normal topdown

./example 2 zero topdown
./example 2 noise topdown
./example 2 sinewave topdown
./example 2 normal topdown

./example 3 zero topdown #learning rate controller : 0.01 model : 0.001
./example 3 noise topdown
./example 3 sinewave topdown
./example 3 normal topdown

./example 5 zero topdown
./example 5 noise topdown
./example 5 sinewave topdown
./example 5 normal topdown

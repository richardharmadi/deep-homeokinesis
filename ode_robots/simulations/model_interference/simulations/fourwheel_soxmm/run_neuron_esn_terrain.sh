#!/bin/bash
## on revision 
## looking for best learning rates for different number neurons


## ESN with extended model
for model in 2
do
  for epsa in 0.0005 0.001 0.0025 0.005 0.0075 0.01
  do
    for epsc in 0.0005 0.001 0.0025 0.005 0.0075 0.01 0.05
    do
      #for playground in 0 0.25 0.5 0.75 1
      #do
        #for lambda in 0 0.2 0.4 0.6 0.8 0.99
        #do
          for reservoir in 12 30 60 90 120 200
          do
            for trial in {1..10}
            do
              echo TRIAL $trial/10
              echo ./start -terrain 1 -playground 0.5 -simtime 20 -model $model -epsa $epsa -epsc $epsc -nographics -threads 1  -spectralradius 0.9  -esn_n $reservoir -log results_terrain/esn_neurons_terrain.txt
              ./start -terrain 1 -playground 0.5 -simtime 20 -model $model -epsa $epsa -epsc $epsc -nographics -threads 1  -spectralradius 0.9  -esn_n $reservoir -log results_terrain/esn_neurons_terrain.txt
            done
          done
        #done
      #done
    done
  done
done


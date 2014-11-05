#!/bin/bash
## on revision 
## looking for best learning rates for LWR
## there is no epsA but sigma is used as learning rate

## LWR NO extended model
for model in 4 
do
  for sigma in 0.01 0.05 0.1 0.2 0.30 0.5 0.75 1.0
  do
    for epsc in 0.001 0.005 0.01 0.025  0.05 0.075 0.1 0.25 0.5
    do
      for trial in {1..10}
      do
        echo TRIAL $trial/10
        echo ./start -terrain 1 -playground 0.5 -simtime 20 -model $model -epsc $epsc -sigma_sqr_LWR $sigma -nographics -threads 1 -noextendedmodel -log results_terrain/lwr_eps.txt
        ./start -terrain 1 -playground 0.5 -simtime 20 -model $model -epsc $epsc -sigma_sqr_LWR $sigma -nographics -threads 1 -noextendedmodel -log results_terrain/lwr_eps.txt
      done
    done
  done
done


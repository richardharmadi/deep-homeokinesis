#!/bin/bash
## on revision 
## looking for best learning rates for linea regression
## didn't try with epsA, so now to see if it improves results


## linear regression  with extended model
for model in 3 
do
  #for epsc in 0.0001 0.0005 0.00075 0.001 0.0025 0.005 0.0075 0.01 0.025 0.05 0.1 0.15 0.2 0.3
  for epsa in 0.1 0.25 0.50 0.75 1.0
  do
    for epsc in 0.001 0.005 0.01 0.025  0.05 0.075 0.1 0.25 0.5
    do
      for trial in {1..10}
      do
        echo TRIAL $trial/10
        echo ./start -terrain 1 -playground 0.5 -simtime 20 -model $model -epsa $epsa -epsc $epsc -nographics -threads 1 -log results_terrain/regression_eps_epsa.txt
        ./start -terrain 1 -playground 0.5 -simtime 20 -model $model -epsa $epsa -epsc $epsc -nographics -threads 1 -log results_terrain/regression_eps_epsa.txt
      done
    done
  done
done

## linear regression NO extended model
for model in 3 
do
  for epsa in 0.1 0.25 0.50 0.75 1.0
  do
    #for epsc in 0.0001 0.0005 0.00075 0.001 0.0025 0.005 0.0075 0.01 0.025 0.05 0.1 0.15 0.2 0.3
    for epsc in 0.001 0.005 0.01 0.025  0.05 0.075 0.1 0.25 0.5
    do
      for trial in {1..10}
      do
        echo TRIAL $trial/10
        echo ./start -terrain 1 -playground 0.5 -simtime 20 -model $model -epsa $epsa -epsc $epsc -nographics -threads 1 -noextendedmodel -log results_terrain/regression_eps_epsa.txt
        ./start -terrain 1 -playground 0.5 -simtime 20 -model $model -epsa $epsa -epsc $epsc -nographics -threads 1 -noextendedmodel -log results_terrain/regression_eps_epsa.txt
      done
    done
  done
done


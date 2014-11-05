#!/bin/bash

for model in 3
do
  for epsa in 1.5 1.25 1.0 0.75 0.5
  do
    for trial in {1..10}
    do
      echo TRIAL $trial/5
      echo ./start -playground 0.2 -log -simtime 20 -model $model -epsa $epsa -epsc 0.001 -nographics -threads 1
      ./start -playground 0.2 -log -simtime 20 -model $model -epsa $epsa -epsc 0.001 -nographics -threads 1
    done
  done
done

for model in 3
do
  for epsc in 0.0001 0.0005 0.001 0.005 0.01 0.05 0.1 0.3
  do
    for trial in {1..10}
    do
      echo ./start -playground 0.2 -log -simtime 20 -model $model -epsa 1. -epsc $epsc -nographics -threads 1
      ./start -playground 0.2 -log -simtime 20 -model $model -epsa 1. -epsc $epsc -nographics -threads 1
    done
  done
done


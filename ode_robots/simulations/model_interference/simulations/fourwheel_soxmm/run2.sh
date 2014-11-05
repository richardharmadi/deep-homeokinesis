#!/bin/bash
## on revision 492
## run for average speed x

#esn for different reservoir size
for esn_n in 12 32  52 72 102 132 152
do
  for playground in 0.2
  do
    for trial in {1..30}
    do
      echo Trial $trial/30
      echo ./start -esn_n $esn_n -playground $playground -log -simtime 30 -nographics -threads 1 -epsa 0.005 -epsc 0.005 -model 2
      ./start -esn_n $esn_n -playground $playground -log -simtime 30 -nographics -threads 1 -epsa 0.005 -epsc 0.005 -model 2
    done
  done
done

#esn model
for esn_n in 100
do
  for playground in 0.05 0.1 0.15 0.2 0.25 0.3
  do
    for trial in {1..30}
    do
      echo Trial $trial/30
      echo ./start -esn_n $esn_n -playground $playground -log -simtime 30 -nographics -threads 1 -epsa 0.005 -epsc 0.005 -model 2
      ./start -esn_n $esn_n -playground $playground -log -simtime 30 -nographics -threads 1 -epsa 0.005 -epsc 0.005 -model 2
    done
  done
done


#linear model
for playground in 0.05 0.1 0.15 0.2 0.25 0.3
do
  for trial in {1..30}
  do
    echo Trial $trial/30
    echo ./start -playground $playground -log -simtime 30 -nographics -threads 1 -epsa 0.05 -epsc 0.001 -model 1
    ./start -playground $playground -log -simtime 30 -nographics -threads 1 -epsa 0.05 -epsc 0.001 -model 1
  done
done


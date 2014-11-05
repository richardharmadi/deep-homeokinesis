#!/bin/bash
## on revision 492
## run for average speed x

for esn_n in 12 22 32 42 52 62 72 82 92 102 202 502
do
  for playground in -1 0 0.1 0.15 0.2 0.25 0.3
  do
    for trial in {1..30}
    do
      echo ./start -esn_n $esn_n -playground $playground -log -simtime 60 -nographics -threads 1
      ./start -esn_n $esn_n -playground $playground -log -simtime 60 -nographics -threads 1
    done
  done
done


#!/bin/bash

  for playground in -1 0 0.1 0.15 0.2 0.25 0.3
  do
    for trial in {1..30}
    do
      echo ./start -model 1 -playground $playground -log -simtime 60 -nographics -threads 1
      ./start -model 1 -playground $playground -log -simtime 60 -nographics -threads 1
    done
  done


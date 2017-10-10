#!/bin/bash

for simtime in 10 20 30 40 50 60
do
    for trial in {1..20}
    do
        ./start -playground 0.5 -terrain 1 -threads 1 -log log-medium-3layer-$simtime-$trial.txt -simtime $simtime -nographics -layer 3 -robot 2
    done
done

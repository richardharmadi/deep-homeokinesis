#!/bin/bash

for layer in 1 2 3 5
do
    for playground in 0.2 0.75
    do
        for trial in {1..20}
        do
            ./start -playground $playground -terrain 1 -threads 1 -log log-$playground-$layer-$trial.txt -simtime 30 -nographics -layer $layer
        done
    done
done

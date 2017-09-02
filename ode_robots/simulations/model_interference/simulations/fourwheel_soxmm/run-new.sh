#!/bin/bash

for layer in 1 3 5
do
    for trial in {1..20}
    do
        ./start -playground 0 -terrain 0 -threads 1 -log log-0-$layer-$trial.txt -simtime 60 -nographics
    done
done

#!/bin/bash

for trial in {1..20}
do
    ./start -playground 0 -terrain 0 -threads 1 -log log-0-$trial.txt -simtime 60 -nographics
done

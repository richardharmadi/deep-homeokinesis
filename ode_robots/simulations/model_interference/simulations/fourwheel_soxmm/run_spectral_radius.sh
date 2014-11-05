  for sr in 0.0 0.10 0.20 0.30 0.40 0.50 0.60 0.70 0.80 0.90 0.99 
  do
    for trial in {1..25}
    do
      echo Trial $trial/5 Spectral Radius $sr
      echo ./start -esn_n 100 -playground 0.2 -log -simtime 30 -nographics -threads 1 -epsa 0.05 -epsc 0.001 -model 2 -spectralradius $sr
      ./start -esn_n 100 -playground 0.2 -log -simtime 30 -nographics -threads 1 -epsa 0.05 -epsc 0.001 -model 2 -spectralradius $sr
    done
  done

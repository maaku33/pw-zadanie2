#!/bin/bash

for i in 1 2 4 8
do
    ./speed_test.sh -b 100 -t $i ./ms386007-new/build/adorate ./data/road_PA.txt >> "./times/$1"
done

for i in 1 2 4 8
do
    ./speed_test.sh -b 10 -t $i ./ms386007-new/build/adorate ./data/kron_g500-logn21.mtx >> "./times/$1"
done

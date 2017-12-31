#!/bin/bash

for i in {1..8}
do
    ./speed_test.sh -b 20 -t $i -s ./data-res/ ./ms386007-new/build/adorate ./data/road_PA.txt >> ./times/basic.txt
    ./speed_test.sh -b 3 -t $i -s ./data-res/ ./ms386007-new/build/adorate ./data/kron_g500-logn21.mtx >> ./times/basic.txt
done

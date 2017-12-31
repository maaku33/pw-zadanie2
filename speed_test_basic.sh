#!/bin/bash

for i in {1..8}
do
    ./speed_test.sh -b 20 -t $i -s ./data-res/ ./ms386007-new/build/adorate ./data/fb_w.txt >> "./times/fb_w.txt"
done

for i in {1..8}
do
    ./speed_test.sh -b 20 -t $i -s ./data-res/ ./ms386007-new/build/adorate ./data/skitter.txt >> "./times/skitter.txt"
done
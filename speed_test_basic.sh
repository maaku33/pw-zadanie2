#!/bin/bash

for i in {1..8}
do
    ./speed_test.sh -b 5000 -t $i ./ms386007-new/build/adorate ~/Downloads/weighted_facebook.txt >> "./times/$1"
done

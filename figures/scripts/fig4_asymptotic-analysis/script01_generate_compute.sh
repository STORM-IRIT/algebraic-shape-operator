#!/bin/bash

mkdir -p groundtruth
mkdir -p estimations

bindir=../../build

surface=goursat
values_n_str=(0010000 0025000 0050000 0075000 0100000 0250000 0500000 0750000 1000000)
values_n_int=(  10000   25000   50000   75000  100000  250000  500000  750000 1000000)

r=0.05

for ((i = 0; i < ${#values_n_str[@]}; ++i)); do
    n_int="${values_n_int[$i]}"
    n_str="${values_n_str[$i]}"
    ${bindir}/dgpgeneratorDGtal -n ${n_int} -o groundtruth/${surface}_${n_str}.txt -s ${surface}
    ${bindir}/dgpComputeCurvatures -i groundtruth/${surface}_${n_str}.txt -o estimations/${surface}_${n_str} -r ${r}
done


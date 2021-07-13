#!/bin/bash

set -ex

mkdir -p groundtruth
mkdir -p estimations
mkdir -p noisy

bindir=../../../c++/build

surface=goursat
values_n_str=( 0250000 ) 
values_n_int=(  250000 )

values_noise=(0.000000001 10 20 30)
values_r=(0.1)

for ((i = 0; i < ${#values_n_str[@]}; ++i)); do
    n_int="${values_n_int[$i]}"
    n_str="${values_n_str[$i]}"
    ${bindir}/dgpgeneratorDGtal -n ${n_int} -o groundtruth/${surface}_${n_str}.txt -s ${surface}

    for ((j = 0; j < ${#values_noise[@]}; ++j)); do
        alpha="${values_noise[$j]}"
        echo $alpha
        ${bindir}/dgpAddNoise  -nor ${alpha} -i groundtruth/${surface}_${n_str}.txt -o noisy/${surface}_norm${alpha}.txt

        for ((k = 0; k < ${#values_r[@]}; ++k)); do
            r="${values_r[$k]}"
            ${bindir}/dgpComputeCurvatures -i noisy/${surface}_norm${alpha}.txt -o estimations/${surface}_norm${alpha}_${r} -r ${r}
        done
    done
done

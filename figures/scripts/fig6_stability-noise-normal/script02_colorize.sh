#!/bin/bash

bindir=../../../c++/build

mkdir -p results

surface=goursat
values_n_str=( 0250000 ) 
values_n_int=(  250000 )
values_method=(APSS ASO)

values_noise=(0.000000001 10 20 30)
values_r=(0.1)

for ((i = 0; i < ${#values_n_str[@]}; ++i)); do
    n_int="${values_n_int[$i]}"
    n_str="${values_n_str[$i]}"

    for ((j = 0; j < ${#values_noise[@]}; ++j)); do
        alpha="${values_noise[$j]}"
        echo $alpha

        ${bindir}/dgpColorizeGroundtruth -i noisy/${surface}_norm${alpha}.txt -o results/${surface}_${alpha}_groundtruth -abs -l 0.5

        for ((k = 0; k < ${#values_r[@]}; ++k)); do
            r="${values_r[$k]}"

            for ((l = 0; l < ${#values_method[@]}; ++l)); do
                method="${values_method[$l]}"
                ${bindir}/dgpColorizeEstimations -txt noisy/${surface}_norm${alpha}.txt -i estimations/${surface}_norm${alpha}_${r}${method}.txt -o results/${surface}_norm${alpha}_${r}_${method} -abs -l 0.5
            done
        done
    done
done


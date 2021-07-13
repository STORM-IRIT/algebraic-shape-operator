#!/bin/bash

mkdir -p errors
mkdir -p stats

bindir=../../build

surface=goursat
values_n_str=(0010000 0025000 0050000 0075000 0100000 0250000 0500000 0750000 1000000) 
values_method=(APSS ASO Barycenter OJets PCAPlane PSS VCM WJets)


for method in "${values_method[@]}"; do
    for n_str in "${values_n_str[@]}"; do
        ${bindir}/dgpComputeErrors -gt groundtruth/${surface}_${n_str}.txt -e estimations/${surface}_${n_str}${method}.txt -o errors/${surface}_${n_str}_${method}.txt
        ${bindir}/dgpComputeStatistics -i errors/${surface}_${n_str}_${method}.txt -o stats/${surface}_${n_str}_${method}.txt
    done
done

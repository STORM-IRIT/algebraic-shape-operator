#/bin/bash

bindir=../../build

mkdir -p results 

${bindir}/dgpComputeGeometricFeatures -i ../../../data/david.ply -o results

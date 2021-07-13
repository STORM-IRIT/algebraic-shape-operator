#/bin/bash

bindir=../../build

mkdir -p results

${bindir}/dgpComputeFlowPlane  -i ../../../data/dragon.ply -o results/plane
${bindir}/dgpComputeFlowSphere -i ../../../data/dragon.ply -o results/sphere

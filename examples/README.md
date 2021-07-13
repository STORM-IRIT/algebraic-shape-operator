Examples
========

## Main example

The [main example](main/main.cpp) computes the differential properties of a 3D point cloud using the single C++ header file [AlgebraicShapeOperator.h](../include/AlgebraicShapeOperator.h).

It only requires that Eigen is installed.

To build the example, run
```bash
cd main
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

To compute the differential properties using the *Algebraic Shape Operator* (ASO), run in the `build` directory
```bash
./computeASO -i ../../../data/bunny.ply -r 0.0025
```
where `-i` specifies the input point cloud as a PLY file containing normal vectors, and `-r` specifies the radius of the neighborhood used to compute the ASO. Run `./computeASO -h` for more information.

It produces a text file `output.txt` with one line per point containing the following 11 comma-seperated values:
```
k1,k2,d1x,d1y,d1z,d2x,d2y,d2z,nx,ny,nz
```

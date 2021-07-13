Examples
========

## Main example

The main example [main/main.cpp](main/main.cpp) computes the differential properties of a 3D point cloud using the single C++ header file [AlgebraicShapeOperator.h](../include/AlgebraicShapeOperator.h).

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


## Example using Ponca

The Ponca example [Ponca/main.cpp](Ponca/main.cpp) is similar to the previous example, except that only [Ponca](https://github.com/poncateam/ponca) is used to compute the ASO.
This library gives more flexibility on the weighting kernel and allows to add user-defined code to the compute method.

:warning: work-in-progress!


## Example using libigl

The libigl example [libigl/main.cpp](libigl/main.cpp) computes the differential properties using the [AlgebraicShapeOperator.h](../include/AlgebraicShapeOperator.h), and display them using the [libigl](https://github.com/libigl/libigl).

To build the example, simply run
```bash
cd libigl
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

To compute and display the differential properties, run in the `build` directory
```bash
./displayASO -i ../../../data/bunny.ply
```
Press the up arrow to display different properties.

:warning: this example uses a k-nearest neighors search instead of a radius-based range search. The weighting kernel width depends on the density. Numerical results are different than those obtained from the other examples.

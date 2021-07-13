Replicate results
=================

To replicate some of the results presented in the article *Stable and efficient differential estimators on oriented point clouds* ([PDF](https://hal.archives-ouvertes.fr/hal-03272493/document)), please follow these 3 steps.

## 1. Set-up

This project requires CMake, a C++ compiler, and Python (with matplotlib).

The only other required dependency is [CGAL](https://www.cgal.org).  
On Linux (Ubuntu or Debian), run this command to install it on the system
```bash
sudo apt-get install libcgal-dev
```

Other dependencies like [Eigen](https://github.com/eigenteam/eigen-git-mirror), [DGtal](https://github.com/DGtal-team/DGtal), [Ponca](https://github.com/poncateam/ponca) and [CLI11](https://github.com/CLIUtils/CLI11) are directly included as git submodules.  

Clone this project, download submodules, and go to the `figures` directory
```bash
git clone --recurse-submodules https://github.com/STORM-IRIT/algebraic-shape-operator.git
cd algebraic-shape-operator/figures
```

Note that OpenMP is optional but highly adviced for better performances.

## 2. Build the project

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
```

:warning: make sure to call the build directory '`build`', which is required for the next step.

## 3. Generate results

The following scripts are provided to generate data used in several figures:

| Fig | Script
| ----| ------------------------------------------------------------------------------------------------------
| 2   | [scripts/fig2_geometric-features/generate.sh](scripts/fig2_geometric-features/generate.sh)
| 3   | [scripts/fig3_geometric-flows/generate.sh](scripts/fig3_geometric-flows/generate.sh)
| 4   | [scripts/fig4_asymptotic-analysis/generate.sh](scripts/fig4_asymptotic-analysis/generate.sh)
| 5   | [scripts/fig5_stability-noise-position/generate.sh](scripts/fig5_stability-noise-position/generate.sh)
| 6   | [scripts/fig6_stability-noise-normal/generate.sh](scripts/fig6_stability-noise-normal/generate.sh)

Results are generated as colored PLY files in a subfolder called `results`.

Example:
```bash
cd scripts/fig2_geometric-features
./generate.sh
ls results/
```

___

Tested with 
- Fedora 31
  - cmake 3.17.4
  - g++ 9.3.1
- Ubuntu 20.04
  - cmake 3.16.3
  - g++ 9.3.0
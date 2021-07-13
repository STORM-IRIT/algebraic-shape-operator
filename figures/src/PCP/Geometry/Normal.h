#pragma once

namespace pcp {

class Geometry;

void compute_normals(Geometry& g, bool verbose = false);
void compute_normals_robust(Geometry& g, float sigma = 1, int iter = 5, bool verbose = false);

namespace internal {

void compute_unoriented_normals(Geometry& g, bool verbose = false);
void compute_unoriented_normals_robust(Geometry& g, float sigma = 1, int iter = 5, bool verbose = false);

} // namespace internal

} // namespace pcp

#include <AlgebraicShapeOperator.h>
#include "example.h"

int main(int argc, char *argv[])
{
    auto [in_input, in_output, in_radius, in_ratio, ok] = parse(argc,argv);
    if(not ok) return 1;

    // load input PLY file
    auto [points, normals] = load_ply(in_input);

    // set the radius of the weighting kernel
    float r;
    if(in_ratio) {
        const auto aabb_diag = compute_aabb_diag(points);
        r = in_radius * aabb_diag;
        Log::info() << "radius = " << r << " (" << in_radius << "*" << aabb_diag << ")";
    } else {
        r = in_radius;
        Log::info() << "radius = " << r;
    }

    // build kdtree
    const auto kdtree = KdTree(points);

    // results
    std::vector<aso::DifferentialProperties<float>> diff_prop(points->size());

    // loop over all points
    auto prog = prog::Progress(points->size());
    #pragma omp parallel for
    for(auto i = 0u; i < points->size(); ++i)
    {
        // range neighbors query
        auto query = kdtree.range_neighbors(i,r);
        auto begin = OrientedPointIterator(query.begin(), points, normals);
        auto end = OrientedPointIterator(query.end(), points, normals);

        // compute differential properties from the Algebraic Shape Operator
        diff_prop[i] = aso::compute((*points)[i], r, begin, end);

        ++prog;
    }

    save_results(in_output, diff_prop);

    return 0;
}

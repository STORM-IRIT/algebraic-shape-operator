#include <AlgebraicShapeOperator.h>
#include "example.h"

int main(int argc, char *argv[])
{
    const auto args = parse(argc,argv);
    if(not args.ok) return 1;

    // load input PLY file
    auto point_cloud = load_ply(args.input);

    // set the radius of the weighting kernel
    float r;
    if(args.ratio) {
        const auto aabb_diag = compute_aabb_diag(point_cloud.points);
        r = args.radius * aabb_diag;
        Log::info() << "radius = " << r << " (" << args.radius << "*" << aabb_diag << ")";
    } else {
        r = args.radius;
        Log::info() << "radius = " << r;
    }

    // build kdtree
    const auto kdtree = KdTree(point_cloud.points);

    // results
    std::vector<aso::DifferentialProperties<float>> diff_prop(point_cloud.points->size());

    // loop over all points
    auto prog = prog::Progress(point_cloud.points->size());
    #pragma omp parallel for
    for(auto i = 0u; i < point_cloud.points->size(); ++i)
    {
        // range neighbors query
        auto query = kdtree.range_neighbors(i,r);
        auto begin = OrientedPointIterator(query.begin(), point_cloud.points, point_cloud.normals);
        auto end = OrientedPointIterator(query.end(), point_cloud.points, point_cloud.normals);

        // compute differential properties from the Algebraic Shape Operator
        diff_prop[i] = aso::compute((*point_cloud.points)[i], r, begin, end);

        ++prog;
    }

    save_results(args.output, diff_prop);

    return 0;
}

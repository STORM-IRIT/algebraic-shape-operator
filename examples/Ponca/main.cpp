#include "example.h"

#define STD_SAFE_AT(C,i) C[i]
#include <Ponca/Ponca>

// point structure processed by Ponca
struct OrientedPoint {
    using Scalar = float;
    using VectorType = Eigen::Vector3f;
    using MatrixType = Eigen::Matrix3f;
    enum {Dim = 3};
    const auto& pos() const {return m_position;}
    const auto& normal() const {return m_normal;}
    Eigen::Vector3f m_position;
    Eigen::Vector3f m_normal;
};

using KdTree = Ponca::KdTree<OrientedPoint>;

using WeightKernel = Ponca::SmoothWeightKernel<float>; // smooth step function
using DistWeightFunc = Ponca::DistWeightFunc<OrientedPoint, WeightKernel>;
using ASO = Ponca::Basket<OrientedPoint, DistWeightFunc,
                          Ponca::OrientedSphereFit,      // APSS algebraic sphere fit
                          Ponca::OrientedSphereSpaceDer, // 1st order derivatives
                          Ponca::MlsSphereFitDer,        // 2nd order derivatives
                          Ponca::CurvatureEstimator>;    // shape operator

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

    // copy to another point cloud for ponca
    std::vector<OrientedPoint> point_cloud_ponca(point_cloud.points->size());
    std::transform(point_cloud.points->begin(), point_cloud.points->end(),
                   point_cloud.normals->begin(), point_cloud_ponca.begin(),
                   [](const auto& p, const auto& n) {
        return OrientedPoint{p,n};
    });

    // build kdtree
    const KdTree kdtree(point_cloud_ponca);

    // results
    std::vector<aso::DifferentialProperties<float>> diff_prop(point_cloud.points->size());

    // loop over all points
    auto prog = prog::Progress(point_cloud.points->size());
    #pragma omp parallel for
    for(auto i = 0u; i < point_cloud.points->size(); ++i)
    {
        // compute the Algebraic Sphere Operator (ASO)
        // in 3 steps (init/addNeighbor/finalize)
        ASO aso;
        aso.setWeightFunc(DistWeightFunc(r));
        aso.init(point_cloud.points->at(i));

        for(int j : kdtree.range_neighbors(point_cloud.points->at(i), r)) {
            aso.addNeighbor(point_cloud_ponca.at(j));
        }
        aso.finalize();

        diff_prop[i] = aso::DifferentialProperties(aso.k1(),
                                                   aso.k2(),
                                                   aso.k1Direction(),
                                                   aso.k2Direction(),
                                                   aso.normal());
        ++prog;
    }

    save_results(args.output, diff_prop);

    return 0;
}

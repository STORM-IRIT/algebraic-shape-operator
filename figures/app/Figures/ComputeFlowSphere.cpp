#include <PCP/Common/Option.h>
#include <PCP/Common/Log.h>
#include <PCP/Common/Progress.h>
#include <PCP/Common/String.h>

#include <PCP/Geometry/Geometry.h>
#include <PCP/Geometry/PLY.h>

#include <PCP/SpacePartitioning/KdTree.h>

#include <Ponca/Fitting>

#include <Eigen/Eigenvalues>

using namespace pcp;

using WeightKernel = Ponca::SmoothWeightKernel<Scalar>;
using WeightFunc   = Ponca::DistWeightFunc<ConstPoint, WeightKernel>;
using SphereFit    = Ponca::Basket<ConstPoint, WeightFunc, Ponca::OrientedSphereFit>;

int main(int argc, char *argv[])
{
    Option opt(argc, argv);
    const String in_input  = opt.get_string("input",  "i").set_required();
    const String in_output = opt.get_string("output", "o").set_default("output");
    const Scalar in_scale  = opt.get_float( "scale"     ).set_default(0.01);
    const int    in_iter   = opt.get_int(   "iter"      ).set_default(30);
    const int    in_every  = opt.get_int(   "every"     ).set_default(30);

    bool ok = opt.ok();
    if(!ok) return 1;
    info() << opt;

    Geometry g;
    ok = PLY::load(in_input, g);
    if(!ok) return 1;
    PCP_ASSERT(g.has_normals());
    Geometry g2 = g;

    const auto aabb = g.aabb();
    const auto aabb_diag = aabb.diagonal().norm();
    const auto radius = in_scale * aabb_diag;
    info() << "radius = " << radius;

    auto prog = Progress(in_iter);
    const int digits = std::to_string(in_iter).size();

    for(int iter=1; iter<=in_iter; ++iter)
    {
        g.build_kdtree();

        #pragma omp parallel for
        for(int i=0; i<g.size(); ++i)
        {
            SphereFit fit;
            fit.setWeightFunc(WeightFunc(radius));
            fit.init(g[i]);

            for(int j : g.kdtree().range_neighbors(g[i], radius))
            {
                fit.addNeighbor(g.at(j));
            }

            const auto status = fit.finalize();

            if(status == Ponca::STABLE)
            {
                // projection on the sphere
                g2.point(i) = fit.project(g[i]);
                g2.normal(i) = fit.primitiveGradient(g2.point(i)).normalized();

                // re-orient if necessary
                if(g2.normal(i).dot(g.normal(i)) < 0) g2.normal(i) *= -1;
            }
            else
            {
                warning() << "Unstable fit at point " << i;
            }
        }
        std::swap(g, g2);

        // save
        if(iter % in_every == 0 || iter == in_iter)
        {
            PLY::save(in_output + "_" + str::to_string(iter,digits) + ".ply", g, false);
        }

        ++prog;
    }

    return 0;
}



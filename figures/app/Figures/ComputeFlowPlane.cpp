#include <PCP/Common/Option.h>
#include <PCP/Common/Log.h>
#include <PCP/Common/Progress.h>
#include <PCP/Common/String.h>

#include <PCP/Geometry/Geometry.h>
#include <PCP/Geometry/PLY.h>

#include <PCP/SpacePartitioning/KdTree.h>

#include <Eigen/Eigenvalues>

using namespace pcp;

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
            Matrix3 C    = Matrix3::Zero();
            Vector3 m    = Vector3::Zero();
            Scalar sum_w = 0;

            for(int j : g.kdtree().range_neighbors(g[i], radius))
            {
                const Vector3 p = g[j] - g[i];

                Scalar w = p.norm() / radius;
                w = (w*w - 1);
                w = w*w;

                C += w * p * p.transpose();
                m += w * p;
                sum_w += w;
            }
            m /= sum_w;
            C = C/sum_w - m * m.transpose();

            Eigen::SelfAdjointEigenSolver<Matrix3> solver(C);
            Vector3 N = solver.eigenvectors().col(0); // vector of least eigenvalue

            // re-orient if necessary (used only for rendering)
            if(N.dot(g.normal(i)) < 0) N *= -1;

            const Vector3 P = g[i] + m;

            // projection on plane {P,N}
            g2.point(i) = g[i] - N.dot(g[i] - P) * N;
            g2.normal(i) = N;
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



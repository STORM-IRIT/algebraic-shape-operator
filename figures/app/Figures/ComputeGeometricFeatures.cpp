#include <PCP/Common/Option.h>
#include <PCP/Common/Log.h>
#include <PCP/Common/Progress.h>
#include <PCP/Common/Colors.h>

#include <PCP/Geometry/Geometry.h>
#include <PCP/Geometry/Loader.h>

#include <PCP/SpacePartitioning/KdTree.h>

#include <Eigen/Eigenvalues>

#include <Ponca/Fitting>

using namespace pcp;

using WeightKernel = Ponca::SmoothWeightKernel<Scalar>;
using WeightFunc   = Ponca::DistWeightFunc<ConstPoint, WeightKernel>;
using SphereFit    = Ponca::Basket<ConstPoint, WeightFunc, Ponca::OrientedSphereFit,
                                                           Ponca::OrientedSphereScaleSpaceDer,
                                                           Ponca::GLSParam,
                                                           Ponca::GLSDer,
                                                           Ponca::GLSGeomVar>;

void colorize_and_save(Geometry& g, const std::vector<Scalar>& feature, const std::string& name, const std::string& path);

int main(int argc, char *argv[])
{
    Option opt(argc, argv);
    const String in_input    = opt.get_string("input",  "i").set_required();
    const String in_output   = opt.get_string("output", "o").set_default(".");
    const Scalar in_scale    = opt.get_float( "scale"      ).set_default(0.01);

    bool ok = opt.ok();
    if(!ok) return 1;
    info() << opt;

    Geometry g;
    ok = Loader::Load(in_input, g);
    if(!ok) return 1;
    PCP_ASSERT(g.has_normals());
    const int point_count = g.size();

    g.build_kdtree();

    const auto aabb = g.aabb();
    const auto aabb_diag = aabb.diagonal().norm();
    const auto radius = in_scale * aabb_diag;

    auto prog = Progress(point_count);

    std::vector<Scalar> feature_uc(point_count);
    std::vector<Scalar> feature_ul(point_count);
    std::vector<Scalar> feature_uq(point_count);
    std::vector<Scalar> feature_gv(point_count);

    #pragma omp parallel for
    for(int i=0; i<point_count; ++i)
    {
        SphereFit fit;
        fit.setWeightFunc(WeightFunc(radius));
        fit.init(g[i]);
        for(int j : g.kdtree().range_neighbors(i, radius))
        {
            fit.addNeighbor(g.at(j));
        }

        const auto status = fit.finalize();

        if(status == Ponca::FIT_RESULT::STABLE)
        {
            feature_uc[i] = fit.m_uc / radius;
            feature_ul[i] = 1 - fit.m_ul.norm();
            feature_uq[i] = fit.m_uq * radius;
            feature_gv[i] = std::sqrt(fit.geomVar());
        }
        else
        {
            warning() << "Unstable fit at point " << i;
        }

        ++prog;
    }

    g.request_colors();

    colorize_and_save(g, feature_uc, "uc", in_output);
    colorize_and_save(g, feature_ul, "ul", in_output);
    colorize_and_save(g, feature_uq, "uq", in_output);
    colorize_and_save(g, feature_gv, "gv", in_output);

    return 0;
}

////////////////////////////////////////////////////////////////////////////////

void colorize_and_save(Geometry& g, const std::vector<Scalar>& feature, const std::string& name, const std::string& path)
{
    const int point_count = g.size();

    const auto colormap = BiColormap::Jet();

    limited_priority_queue<Scalar, std::greater<Scalar>> q(0.10 * point_count);

    Scalar min = +std::numeric_limits<Scalar>::max();
    Scalar max = -std::numeric_limits<Scalar>::max();
    Scalar mean = 0;
    for(int i=0; i<point_count; ++i)
    {
        q.push(std::abs(feature[i]));
        min = std::min(min, feature[i]);
        max = std::max(max, feature[i]);
        mean += feature[i];
    }
    mean /= point_count;

    const Scalar limit = q.bottom();

    info() << name << " in (" << min << "," << max << ") mean=" << mean << " limit=" << limit;

    #pragma omp parallel for
    for(int i=0; i<point_count; ++i)
    {
        g.color(i) = colormap(feature[i], limit);
    }

    Loader::Save(path+"/"+name+".ply", g, false);
}


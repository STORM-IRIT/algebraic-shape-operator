#include <PCP/Curvature/Curvature.h>
#include <PCP/Geometry/Geometry.h>

#include <PCP/SpacePartitioning/KdTree.h>

#include <PCP/Common/Log.h>

#include <Ponca/Fitting>

using namespace pcp;

template<class DataPoint, class _WFunctor, typename T >
struct MongePatchExtension : public T
{
    using Base = T;

    Vector3 normal() const
    {
        Vector3 monge_N = Vector3( - Base::h_u(), - Base::h_v(), 1).normalized();
        monge_N = Vector3(monge_N.z(), monge_N.x(), monge_N.y()); // tangentPlaneToWorld require [h,u,v] = [z,x,y]
        constexpr bool ignoreTranslation = true;
        Vector3 world_N = Base::template tangentPlaneToWorld<ignoreTranslation>(monge_N);

        return world_N.normalized();
    }

    void compute_k1_and_k2(Scalar& k1, Scalar& k2) const
    {
        k1 = 0;
        k2 = 0;

        const Scalar a20 = Base::h_uu () * 2; //!
        const Scalar a02 = Base::h_vv () * 2; //!
        const Scalar a11 = Base::h_uv ();
        const Scalar a10 = Base::h_u  ();
        const Scalar a01 = Base::h_v  ();
//            const Scalar a00 = Base::h_c  ();

        // Shape operator
        Matrix2 W;
        W << a20*(1+a01*a01) - a01*a10*a11, a11*(1+a01*a01) - a01*a10*a02,
             a11*(1+a10*a10) - a01*a10*a20, a02*(1+a10*a10) - a01*a10*a11;
        W *= std::pow(1 + a10*a10 + a01*a01, -3./2.);

        const Scalar delta = (W(0,0) + W(1,1))*(W(0,0) + W(1,1)) - 4 * (W(0,0)*W(1,1) - W(0,1)*W(1,0));

        constexpr Scalar epsilon = 0.001;

        if(delta < - epsilon)
        {
            error() << "delta = " << delta << "!!!!";
            PCP_ERROR;
        }
        else if(delta < +epsilon)
        {
            k1 = (W(0,0) + W(1,1)) / 2;
            k2 = (W(0,0) + W(1,1)) / 2;
        }
        else
        {
            k1 = (W(0,0) + W(1,1) + std::sqrt(delta)) / 2;
            k2 = (W(0,0) + W(1,1) - std::sqrt(delta)) / 2;
        }
    }

    PONCA_MULTIARCH inline Scalar sumW() const {return Base::m_sumW;}
};

using WeightKernel  = Ponca::SmoothWeightKernel<Scalar>;
using WeightFunc    = Ponca::DistWeightFunc<ConstPoint, WeightKernel>;
using Fit           = Ponca::Basket<ConstPoint, WeightFunc, Ponca::CovariancePlaneFit, Ponca::MongePatch, MongePatchExtension>;

namespace pcp {

////////////////////////////////////////////////////////////////////////////////

PointWiseEstimationData CurvatureComputer_PSS::compute(const Geometry& points, int i, Scalar r) const
{
    bool reach_step_max    = false;
    bool reach_convergence = false;
    int step = 0;

    Vector3 p = points[i];

    Scalar  k1 = 666;
    Scalar  k2 = 666;
    Vector3 N  = Vector3::Constant(666);

    Fit fit;
    fit.setWeightFunc(WeightFunc(r));

    int nei_count = 0;

    while(!reach_step_max && !reach_convergence)
    {
        fit.init(p);

        nei_count = 0;

        for(int j : points.kdtree().range_neighbors(p, r))
        {
            fit.addNeighbor(points.at(j));
            ++nei_count;
        }
        auto status = fit.finalize();

        if(nei_count < NEI_COUNT_MIN || fit.sumW() < SUM_WEIGHT_MIN || status != Ponca::NEED_OTHER_PASS)
            return PointWiseEstimationData::Invalid();

        // second pass
        for(int j : points.kdtree().range_neighbors(p, r))
        {
            fit.addNeighbor(points.at(j));
        }
        status = fit.finalize();

        if(nei_count < NEI_COUNT_MIN || fit.sumW() < SUM_WEIGHT_MIN || status != Ponca::STABLE)
            return PointWiseEstimationData::Invalid();

        N  = fit.normal();
        fit.compute_k1_and_k2(k1, k2);

        const Vector3 new_p = fit.project(points[i]);
        ++step;

        reach_step_max    = step == MLS_STEP_MAX;
        reach_convergence = (new_p-p).norm() < MLS_EPSILON;

        p = new_p;
    }

    if(std::abs(k2) > std::abs(k1)) std::swap(k1,k2);

    // PONCA convention: +1 = concave and -1 = convex
    // We want the inverse
    k1 *= -1;
    k2 *= -1;

    const Scalar  H  = 0.5 * (k1 + k2);

    return PointWiseEstimationData(k1, k2, H, N, Vector3::Constant(666), Vector3::Constant(666), nei_count);
}

} // namespace pcp

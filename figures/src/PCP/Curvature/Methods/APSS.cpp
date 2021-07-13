#include <PCP/Curvature/Curvature.h>
#include <PCP/Geometry/Geometry.h>

#include <PCP/SpacePartitioning/KdTree.h>

#include <Ponca/Fitting>

using namespace pcp;

template < class DataPoint, class _WFunctor, typename T = void >
class SphereExtension : public T
{
public:
    using Scalar = typename T::Scalar;
    PONCA_MULTIARCH inline Scalar sumW() const {return T::m_sumW;}
};

using WeightKernel  = Ponca::SmoothWeightKernel<Scalar>;
using WeightFunc    = Ponca::DistWeightFunc<ConstPoint, WeightKernel>;
using Fit           = Ponca::Basket<ConstPoint, WeightFunc, Ponca::OrientedSphereFit, SphereExtension, Ponca::OrientedSphereSpaceDer, Ponca::MlsSphereFitDer, Ponca::CurvatureEstimator>;

namespace pcp {

PointWiseEstimationData CurvatureComputer_APSS::compute(const Geometry& points, int i, Scalar r) const
{
    Fit fit;
    fit.setWeightFunc(WeightFunc(r));
    fit.init(points[i]);

    int nei_count = 0;

    for(int j : points.kdtree().range_neighbors(i, r))
    {
        fit.addNeighbor(points.at(j));
        ++nei_count;
    }
    const auto status = fit.finalize();

    if(nei_count < NEI_COUNT_MIN || fit.sumW() < SUM_WEIGHT_MIN || status != Ponca::STABLE)
        return PointWiseEstimationData::Invalid();

    const Vector3 N = fit.m_ul.normalized();
    const Scalar  H = 2 * fit.m_uq / std::sqrt(fit.m_ul.norm() - 4 * fit.m_uq * fit.m_uc);

    return PointWiseEstimationData(666, 666, H, N, Vector3::Constant(666), Vector3::Constant(666), nei_count);
}

} // namespace pcp

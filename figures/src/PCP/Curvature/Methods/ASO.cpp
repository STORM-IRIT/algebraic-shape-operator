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

PointWiseEstimationData CurvatureComputer_ASO::compute(const Geometry& points, int i, Scalar r) const
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

    fit.computeCurvature(true); // useNormal = true

    const Vector3 N  = fit.normal();
    const Scalar  k1 = fit.k1();
    const Scalar  k2 = fit.k2();
    const Scalar  H  = 0.5 * (k1 + k2);
    const Vector3 dir1 = fit.k1Direction();
    const Vector3 dir2 = fit.k2Direction();

    return PointWiseEstimationData(k1, k2, H, N, dir1, dir2, nei_count);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace pcp

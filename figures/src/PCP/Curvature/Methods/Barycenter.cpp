#include <PCP/Curvature/Curvature.h>
#include <PCP/Curvature/Weight.h>

#include <PCP/SpacePartitioning/KdTree.h>

#include <PCP/Geometry/Geometry.h>

namespace pcp {

PointWiseEstimationData CurvatureComputer_Barycenter::compute(const Geometry& points, int i, Scalar r) const
{
    Vector3 sum_pos = Vector3::Zero();
    Scalar  sum_w   = 0;

    int nei_count = 0;

    for(int j : points.kdtree().range_neighbors(i, r))
    {   
        const Scalar w = weight(points[i], points[j], r);
        sum_pos += w * (points.point(j)-points.point(i));
        sum_w   += w;
        ++nei_count;
    }

    if(nei_count < NEI_COUNT_MIN || sum_w <= SUM_WEIGHT_MIN)
        return PointWiseEstimationData::Invalid();

    const Vector3 mean_pos = sum_pos / sum_w;
    const Scalar  dist = mean_pos.norm();

//    const Scalar H = 4 * dist / (r*r);    if w = 1
    const Scalar H = 8 * dist / (r*r);   // otherwise

    return PointWiseEstimationData(666, 666, H, Vector3::Constant(666), Vector3::Constant(666), Vector3::Constant(666), nei_count);
}

} // namespace pcp

#include <PCP/Curvature/Curvature.h>
#include <PCP/Curvature/Weight.h>

#include <PCP/Geometry/Geometry.h>

#include <PCP/SpacePartitioning/KdTree.h>

#include <Eigen/Eigenvalues>

namespace pcp {

PointWiseEstimationData CurvatureComputer_PCAPlane::compute(const Geometry& points, int i, Scalar r) const
{
    Matrix3 C      = Matrix3::Zero();
    Vector3 m      = Vector3::Zero();
    Scalar  sum_w  = 0;

    int nei_count = 0;

    for(int j : points.kdtree().range_neighbors(i, r))
    {
        const Scalar w = weight(points[i], points[j], r);
        m      += w * (points[j] - points[i]);
        C      += w * (points[j] - points[i]) * (points[j] - points[i]).transpose();
        sum_w  += w;
        ++nei_count;
    }

    if(nei_count < NEI_COUNT_MIN || sum_w < SUM_WEIGHT_MIN)
        return PointWiseEstimationData::Invalid();

    m /= sum_w;
    C = C/sum_w - m * m.transpose();

    // eigenvalues are positive and ordered
    // eigenvectors are normalized
    Eigen::SelfAdjointEigenSolver<Matrix3> solver(C);
    Vector3 N = solver.eigenvectors().col(0);

    const Scalar H = 8 * m.dot(N); // *  1/r^2

    return PointWiseEstimationData(666, 666, -H, N, Vector3::Constant(666), Vector3::Constant(666), nei_count); // -1 due to convention
}

} // namespace pcp

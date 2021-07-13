#include <PCP/Curvature/compute_error.h>
#include <PCP/Curvature/PointWiseCurvatureData.h>
#include <PCP/Curvature/PointWiseEstimationData.h>

namespace pcp {

PointWiseErrorData compute_error(const PointWiseCurvatureData& groundtruth,
                                 const PointWiseEstimationData& estimation)
{
    PointWiseErrorData err;
    // mean curvature
    err.m_err_H = std::abs(std::abs(groundtruth.H()) -
                           std::abs(estimation.H()));

    // principal curvatures
    err.m_err_k1 = std::abs(std::abs(groundtruth.k1()) -
                            std::abs(estimation.k1()));
    err.m_err_k2 = std::abs(std::abs(groundtruth.k2()) -
                            std::abs(estimation.k2()));

    // normal
    err.m_err_N = 1 - std::abs(groundtruth.normal().dot(estimation.normal()));

    // principal directions
    err.m_err_dir1 = 1 - std::abs(groundtruth.dir1().dot(estimation.dir1()));
    err.m_err_dir2 = 1 - std::abs(groundtruth.dir2().dot(estimation.dir2()));

    // other data
    err.m_nei_count = estimation.nei_count();
    err.m_time_ms = estimation.time_ms();

    return err;
}

} // namespace pcp

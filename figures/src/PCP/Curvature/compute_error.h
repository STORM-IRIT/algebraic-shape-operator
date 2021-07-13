#pragma once

#include <PCP/Curvature/PointWiseErrorData.h>

namespace pcp {

class PointWiseCurvatureData;
class PointWiseEstimationData;

PointWiseErrorData compute_error(const PointWiseCurvatureData& groundtruth,
                                 const PointWiseEstimationData& estimation);

} // namespace pcp

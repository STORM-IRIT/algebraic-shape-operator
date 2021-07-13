#pragma once

#include <PCP/Common/Scalar.h>

namespace pcp {

class Geometry;
class GlobalEstimationData;

void compute_VCM(const Geometry& points, Scalar r, GlobalEstimationData& e);

} // namespace pcp

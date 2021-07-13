#pragma once

#include <PCP/Common/Scalar.h>

namespace pcp {

struct IndexSquaredDistance
{
    int index;
    Scalar squared_distance;

    bool operator < (const IndexSquaredDistance& other) const;
};

} // namespace pcp

#pragma once

#include <PCP/Common/Scalar.h>

#include <cmath>

namespace pcp {

class NearestQuery
{
public:
    NearestQuery(){}

    int get() const{return m_nearest;}
    Scalar distance() const {return std::sqrt(m_squared_distance);}

protected:
    int m_nearest;
    Scalar m_squared_distance;
};

} // namespace pcp

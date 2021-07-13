#pragma once

#include <PCP/Curvature/PointWiseCurvatureData.h>

namespace pcp {

class PointWiseEstimationData : public PointWiseCurvatureData
{
public:
    PointWiseEstimationData() :
        PointWiseCurvatureData(),
        m_nei_count(666),
        m_time_ms(666)
    {
    }

    PointWiseEstimationData(Scalar k1,
                            Scalar k2,
                            Scalar H,
                            const Vector3& normal,
                            const Vector3& dir1,
                            const Vector3& dir2,
                            int nei_count,
                            Scalar time_ms = 666) :
        PointWiseCurvatureData(k1, k2, H, normal, dir1, dir2),
        m_nei_count(nei_count),
        m_time_ms(time_ms)
    {
    }

    inline static PointWiseEstimationData Invalid()
    {
        return PointWiseEstimationData(666, 666, 666, Vector3::Constant(666), Vector3::Constant(666), Vector3::Constant(666), 0, 666);
    }

public:
    inline int nei_count() const {return m_nei_count;}
    inline Scalar time_ms() const {return m_time_ms;}

    inline int& nei_count() {return m_nei_count;}
    inline Scalar& time_ms() {return m_time_ms;}

    inline bool valid() const {return m_nei_count > 0;}

public:
    int m_nei_count;
    Scalar m_time_ms;

};

} // namespace pcp

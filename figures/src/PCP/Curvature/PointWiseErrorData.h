#pragma once

#include <PCP/Geometry/Defines.h>

namespace pcp {

class PointWiseErrorData
{
public:
    PointWiseErrorData() :
        m_err_k1(666),
        m_err_k2(666),
        m_err_H(666),
        m_err_N(666),
        m_err_dir1(666),
        m_err_dir2(666),
        m_nei_count(666),
        m_time_ms(666)
    {}

    PointWiseErrorData(Scalar err_k1, Scalar err_k2, Scalar err_H, Scalar err_N, Scalar err_dir1, Scalar err_dir2, Scalar nei_count, Scalar time_ms) :
        m_err_k1(err_k1),
        m_err_k2(err_k2),
        m_err_H(err_H),
        m_err_N(err_N),
        m_err_dir1(err_dir1),
        m_err_dir2(err_dir2),
        m_nei_count(nei_count),
        m_time_ms(time_ms)
    {}

    inline bool is_valid() const {return m_nei_count > 0;}

public:
    Scalar m_err_k1;
    Scalar m_err_k2;
    Scalar m_err_H;
    Scalar m_err_N;
    Scalar m_err_dir1;
    Scalar m_err_dir2;
    Scalar m_nei_count;
    Scalar m_time_ms;
};



} // namespace pcp

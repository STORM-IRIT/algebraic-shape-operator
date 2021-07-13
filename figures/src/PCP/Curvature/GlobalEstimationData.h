#pragma once

#include <PCP/Curvature/PointWiseEstimationData.h>

namespace pcp {

class GlobalEstimationData
{
public:
    GlobalEstimationData(int size = 0);

    bool load(const std::string& filename);
    bool save(const std::string& filename) const;

public:
    void clear();
    void resize(int size);
    void push_back(Scalar k1, Scalar k2, Scalar H, const Vector3& dir1, const Vector3& dir2, const Vector3& normal, int nei_count, Scalar time_ms);

public:
    int size() const;

public:
    const PointWiseEstimationData& operator[](int i) const;
          PointWiseEstimationData& operator[](int i);

public:
    Scalar k1(int i) const;
    Scalar k2(int i) const;
    const Vector3& dir1(int i) const;
    const Vector3& dir2(int i) const;
    const Vector3& normal(int i) const;
    int nei_count(int i) const;
    Scalar time_ms(int i) const;

public:
    Scalar& k1(int i);
    Scalar& k2(int i);
    Vector3& dir1(int i);
    Vector3& dir2(int i);
    Vector3& normal(int i);
    int& nei_count(int i);
    Scalar& time_ms(int i);

public:
    int average_nei_count() const;
    int valid_count() const;

public:
    std::vector<PointWiseEstimationData> m_data;
};

} // namespace pcp

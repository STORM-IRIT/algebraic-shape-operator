#pragma once

#include <PCP/Curvature/PointWiseErrorData.h>

#include <vector>

namespace pcp {

class GlobalErrorData
{
public:
    GlobalErrorData(int size = 0);

public:
    bool load(const std::string& filename);
    bool save(const std::string& filename) const;

public:
    void clear();
    void resize(int size);
    int size() const;
    void push_back(Scalar err_k1, Scalar err_k2, Scalar err_H, Scalar err_N, Scalar err_dir1, Scalar err_dir2, Scalar nei_count, Scalar time_ms);

public:
    const PointWiseErrorData& operator[](int i) const;
          PointWiseErrorData& operator[](int i);

public:
    std::vector<PointWiseErrorData> m_data;

};

} // namespace pcp

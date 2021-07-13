#pragma once

#include <PCP/Curvature/Statistics.h>

namespace pcp {

class GlobalErrorData;

class GlobalStatisticsData
{
public:
    GlobalStatisticsData(const GlobalErrorData& errors);

    bool load(const std::string& filename);
    bool save(const std::string& filename) const;

public:
    Statistics m_k1;
    Statistics m_k2;
    Statistics m_H;
    Statistics m_N;
    Statistics m_dir1;
    Statistics m_dir2;
    Statistics m_nei_count;
    Statistics m_time_ms;
};

} // namespace pcp

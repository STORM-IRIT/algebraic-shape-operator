#include <PCP/Curvature/GlobalStatisticsData.h>
#include <PCP/Curvature/GlobalErrorData.h>
#include <PCP/Common/Log.h>

#include <fstream>

namespace pcp {

template<typename CheckFunc, typename GetterFunc>
Statistics get_stats(const GlobalErrorData& errors, CheckFunc&& is_valid, GetterFunc&& get)
{
    std::vector<Scalar> vec;
    vec.reserve(errors.size());

    for(int i=0; i<errors.size(); ++i)
    {
        if(is_valid(i)) vec.push_back(get(i));
    }
    return Statistics(vec);
}

GlobalStatisticsData::GlobalStatisticsData(const GlobalErrorData& errors)
{
    const auto is_valid = [&errors](int i){return errors[i].is_valid();};
    m_k1        = get_stats(errors, is_valid, [&errors](int i){return errors[i].m_err_k1   ;});
    m_k2        = get_stats(errors, is_valid, [&errors](int i){return errors[i].m_err_k2   ;});
    m_H         = get_stats(errors, is_valid, [&errors](int i){return errors[i].m_err_H    ;});
    m_N         = get_stats(errors, is_valid, [&errors](int i){return errors[i].m_err_N    ;});
    m_dir1      = get_stats(errors, is_valid, [&errors](int i){return errors[i].m_err_dir1 ;});
    m_dir2      = get_stats(errors, is_valid, [&errors](int i){return errors[i].m_err_dir2 ;});
    m_nei_count = get_stats(errors, is_valid, [&errors](int i){return errors[i].m_nei_count;});
    m_time_ms   = get_stats(errors, is_valid, [&errors](int i){return errors[i].m_time_ms  ;});
}

bool GlobalStatisticsData::load(const std::string& /*filename*/)
{
    PCP_TODO;
    return false;
}

bool GlobalStatisticsData::save(const std::string& filename) const
{
    std::ofstream ofs(filename);
    if(!ofs.is_open())
    {
        warning() << "Failed to open output file " << filename;
        return false;
    }

    ofs << m_k1        << "\n";
    ofs << m_k2        << "\n";
    ofs << m_H         << "\n";
    ofs << m_N         << "\n";
    ofs << m_dir1      << "\n";
    ofs << m_dir2      << "\n";
    ofs << m_nei_count << "\n";
    ofs << m_time_ms   << "\n";

    return true;
}

} // namespace pcp

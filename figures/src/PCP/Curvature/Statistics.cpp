#include <PCP/Curvature/Statistics.h>

#include <algorithm>
#include <numeric>
#include <cmath>
#include <sstream>

namespace pcp {

Statistics::Statistics(const std::vector<Scalar>& vec) :
    m_min(0),
    m_centile25(0),
    m_centile50(0),
    m_centile75(0),
    m_max(0),
    m_mean(0),
    m_std_var(0)
{
    std::vector<Scalar> cpy(vec);

    // remove NaN/Inf
    cpy.erase(std::remove_if(cpy.begin(), cpy.end(), [](const Scalar x)
        {
            return std::isnan(x) || std::isinf(x);
        }),
        cpy.end());

    if(cpy.empty()) return;

    // centiles
    std::sort(cpy.begin(), cpy.end());
    m_min = cpy.front();
    m_centile25 = cpy[0.25 * (cpy.size()-1)];
    m_centile50 = cpy[0.50 * (cpy.size()-1)];
    m_centile75 = cpy[0.75 * (cpy.size()-1)];
    m_max = cpy.back();

    // see https://stackoverflow.com/a/12405793/5317819
    const Scalar sum = std::accumulate(cpy.begin(), cpy.end(), 0.0);
    m_mean =  sum / cpy.size();

    Scalar accum = 0;
    std::for_each(cpy.begin(), cpy.end(), [&](const Scalar x)
    {
        accum += (x - m_mean) * (x - m_mean);
    });

    m_std_var = std::sqrt(accum / (cpy.size()-1));
}

std::string Statistics::to_string(const std::string& sep) const
{
    std::stringstream str;
    str << m_min       << sep
        << m_centile25 << sep
        << m_centile50 << sep
        << m_centile75 << sep
        << m_max       << sep
        << m_mean      << sep
        << m_std_var;
    return str.str();
}

std::ostream& operator <<(std::ostream& os, const Statistics s)
{
    const char sep = ' ';
    os << s.m_min       << sep
       << s.m_centile25 << sep
       << s.m_centile50 << sep
       << s.m_centile75 << sep
       << s.m_max       << sep
       << s.m_mean      << sep
       << s.m_std_var;
    return os;
}

} // namespace pcp

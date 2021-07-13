#pragma once

#include <PCP/Common/Scalar.h>

#include <vector>
#include <string>

namespace pcp {

class Statistics
{
public:
    Statistics() = default;
    Statistics(const std::vector<Scalar>& vec);

public:
    std::string to_string(const std::string& sep = " ") const;

    friend std::ostream& operator <<(std::ostream& os, const Statistics s);

public:
    Scalar m_min; // centile 00
    Scalar m_centile25;
    Scalar m_centile50;
    Scalar m_centile75;
    Scalar m_max; // centile 100

    Scalar m_mean;
    Scalar m_std_var;
};

} // namespace pcp

#pragma once

#include <string>

namespace pcp {

struct MethodProperties
{
    std::string name;
    bool oriented;
    bool H;
    bool N;
    bool k;
    bool k_signed;
    bool dir;
};

} // namespace pcp

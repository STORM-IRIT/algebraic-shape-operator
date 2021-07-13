#pragma once

#include <string>

namespace pcp {

class Geometry;

class PLY
{
public:
    static bool load(const std::string& filename,       Geometry& g, bool verbose = true);
    static bool save(const std::string& filename, const Geometry& g, bool verbose = true, bool binary = true);
};

} // namespace pcp

#pragma once

#include <string>
#include <vector>

namespace pcp {

class Geometry;

class TXT
{
public:
    static bool load(const std::string& filename,       Geometry& g, bool verbose = true);
    static bool save(const std::string& filename, const Geometry& g, bool verbose = true);

    static bool load(const std::string& filename,       Geometry& g,       std::vector<int>& labels, bool verbose = true);
    static bool save(const std::string& filename, const Geometry& g, const std::vector<int>& labels, bool verbose = true);
};

} // namespace pcp

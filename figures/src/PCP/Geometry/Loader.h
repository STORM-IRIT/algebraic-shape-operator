#pragma once

#include <string>
#include <vector>

namespace pcp {

class Geometry;

struct Loader
{
    static bool Load(const std::string& filename, Geometry& g,                           bool verbose = true);
    static bool Load(const std::string& filename, Geometry& g, std::vector<int>& labels, bool verbose = true);

    static bool Save(const std::string& filename, const Geometry& g,                                 bool verbose = true);
    static bool Save(const std::string& filename, const Geometry& g, const std::vector<int>& labels, bool verbose = true);
};

} // namespace pcp

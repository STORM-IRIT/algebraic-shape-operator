#pragma once

#include <string>

namespace pcp {

class Geometry;
class GlobalCurvatureData;

class CustomLoader
{
public:
    static bool Load(const std::string& filename,       Geometry& points,       GlobalCurvatureData& groundtruth);
    static bool Save(const std::string& filename, const Geometry& points, const GlobalCurvatureData& groundtruth);
};

} // namespace pcp

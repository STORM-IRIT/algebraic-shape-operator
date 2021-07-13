#include <PCP/Geometry/TXT.h>
#include <PCP/Geometry/Geometry.h>

#include <PCP/Common/Log.h>

#include <fstream>

namespace pcp {

bool TXT::load(const std::string& filename, Geometry& g, bool verbose)
{
    std::ifstream ifs(filename);
    if(!ifs.is_open())
    {
        error().iff(verbose) << "Failed to open file " << filename;
        return false;
    }

    g = Geometry();

    std::string line;
    Scalar x, y, z;
    while(std::getline(ifs, line))
    {
        if(line.empty()) continue;
        std::stringstream str(line);
        ifs >> x;
        ifs >> y;
        ifs >> z;
        g.emplace_back(Vector3(x,y,z));
    }

    return true;
}

bool TXT::save(const std::string& filename, const Geometry& g, bool verbose)
{
    std::ifstream ifs(filename);
    if(!ifs.is_open())
    {
        error().iff(verbose) << "Failed to open file " << filename;
        return false;
    }

    PCP_TODO;
    PCP_UNUSED(g);

    return true;
}

bool TXT::load(const std::string& filename, Geometry& g, std::vector<int>& labels, bool verbose)
{
    std::ifstream ifs(filename);
    if(!ifs.is_open())
    {
        error().iff(verbose) << "Failed to open file " << filename;
        return false;
    }

    g = Geometry();
    labels.clear();

    std::string line;
    Scalar x, y, z;
    int l;
    while(std::getline(ifs, line))
    {
        if(line.empty()) continue;
        std::stringstream str(line);
        ifs >> x;
        ifs >> y;
        ifs >> z;
        ifs >> l;
        g.emplace_back(Vector3(x,y,z));
        labels.push_back(l);
    }

    return true;
}

bool TXT::save(const std::string& filename, const Geometry& g, const std::vector<int>& labels, bool verbose)
{
    std::ifstream ifs(filename);
    if(!ifs.is_open())
    {
        error().iff(verbose) << "Failed to open file " << filename;
        return false;
    }

    PCP_TODO;
    PCP_UNUSED(g);
    PCP_UNUSED(labels);

    return true;
}

} // namespace pcp

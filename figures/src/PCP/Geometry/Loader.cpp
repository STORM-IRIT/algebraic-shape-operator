#include <PCP/Geometry/Loader.h>
#include <PCP/Geometry/PLY.h>
#include <PCP/Geometry/TXT.h>

#include <PCP/Common/Log.h>

namespace pcp {

bool Loader::Load(const std::string& filename, Geometry& g, bool verbose)
{
    const std::string ext = filename.substr(filename.find_last_of(".") + 1);
         if(ext == "ply") return PLY::load(filename, g, verbose);
    else if(ext == "txt") return TXT::load(filename, g, verbose);
    else
    {
        error() << "Missing or unsupported extension for file " << filename;
        return false;
    }
}

bool Loader::Load(const std::string& filename, Geometry& g, std::vector<int>& labels, bool verbose)
{
    const std::string ext = filename.substr(filename.find_last_of(".") + 1);
         if(ext == "txt") return TXT::load(filename, g, labels, verbose);
    else if(ext == "ply")
    {
         error() << "Cannot load labels from ply file " << filename;
         return false;
    }
    else
    {
        error() << "Missing or unsupported extension for file " << filename;
        return false;
    }
}

bool Loader::Save(const std::string& filename, const Geometry& g, bool verbose)
{
    const std::string ext = filename.substr(filename.find_last_of(".") + 1);
         if(ext == "ply") return PLY::save(filename, g, verbose);
    else if(ext == "txt") return TXT::save(filename, g, verbose);
    else                  return PLY::save(filename + ".ply", g, verbose);
}

bool Loader::Save(const std::string& filename, const Geometry& g, const std::vector<int>& labels, bool verbose)
{
    const std::string ext = filename.substr(filename.find_last_of(".") + 1);
         if(ext == "txt") return TXT::save(filename, g, labels, verbose);
    else if(ext == "ply")
    {
        error() << "Cannot save labels to ply file " << filename;
        return false;
    }
    else return TXT::save(filename, g, labels, verbose);
}

} // namespace pcp

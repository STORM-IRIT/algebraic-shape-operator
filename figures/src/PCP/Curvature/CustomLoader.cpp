#include <PCP/Curvature/CustomLoader.h>
#include <PCP/Curvature/GlobalCurvatureData.h>

#include <PCP/Geometry/Geometry.h>

#include <PCP/Common/Log.h>

#include <fstream>

namespace pcp {

bool CustomLoader::Load(const std::string& filename, Geometry& points, GlobalCurvatureData& groundtruth)
{
    points = Geometry();
    groundtruth = GlobalCurvatureData();
    points.request_normals();

    info() << "Reading data from file " << filename;

    std::ifstream ifs(filename);
    if(!ifs.is_open())
    {
        warning() << "Failed to open input file " << filename;
        return false;
    }

    Scalar x;
    Scalar y;
    Scalar z;
//    Scalar K;     (not used)
    Scalar H;
    Scalar nx;
    Scalar ny;
    Scalar nz;
    Scalar k1;
    Scalar k2;
    Scalar dir1x;
    Scalar dir1y;
    Scalar dir1z;
    Scalar dir2x;
    Scalar dir2y;
    Scalar dir2z;

    std::string str_x;
    std::string str_y;
    std::string str_z;
    std::string str_K;
    std::string str_H;
    std::string str_nx;
    std::string str_ny;
    std::string str_nz;
    std::string str_k1;
    std::string str_k2;
    std::string str_dir1x;
    std::string str_dir1y;
    std::string str_dir1z;
    std::string str_dir2x;
    std::string str_dir2y;
    std::string str_dir2z;

    // read file line by line
    // convert each line to a stringstream
    // read line string by string
    // convert each string to a float
    // we use stof because nan values are not read otherwise

    std::string line;
    int line_count = 0;
    int cmt_count = 0;
    while(std::getline(ifs, line))
    {
        std::stringstream str(line);
        bool ok = str >> str_x &&
                  str >> str_y &&
                  str >> str_z &&
                  str >> str_K &&
                  str >> str_H &&
                  str >> str_nx &&
                  str >> str_ny &&
                  str >> str_nz &&
                  str >> str_k1 &&
                  str >> str_k2 &&
                  str >> str_dir1x &&
                  str >> str_dir1y &&
                  str >> str_dir1z &&
                  str >> str_dir2x &&
                  str >> str_dir2y &&
                  str >> str_dir2z;

        if(!ok)
        {
            error() << "Failed to read line " << line_count+1 << " from file " << filename;
        }
        else {
            if (str_x != "#") {
                x = std::stof(str_x);
                y = std::stof(str_y);
                z = std::stof(str_z);
//            K     = std::stof(str_K    );
                H = std::stof(str_H);
                nx = std::stof(str_nx);
                ny = std::stof(str_ny);
                nz = std::stof(str_nz);
                k1 = std::stof(str_k1);
                k2 = std::stof(str_k2);
                dir1x = std::stof(str_dir1x);
                dir1y = std::stof(str_dir1y);
                dir1z = std::stof(str_dir1z);
                dir2x = std::stof(str_dir2x);
                dir2y = std::stof(str_dir2y);
                dir2z = std::stof(str_dir2z);

                if (std::isnan(x) || std::isinf(x) ||
                    std::isnan(y) || std::isinf(y) ||
                    std::isnan(z) || std::isinf(z) ||
                    std::isnan(H) || std::isinf(H) ||
                    std::isnan(nx) || std::isinf(nx) ||
                    std::isnan(ny) || std::isinf(ny) ||
                    std::isnan(nz) || std::isinf(nz) ||
                    std::isnan(k1) || std::isinf(k1) ||
                    std::isnan(k2) || std::isinf(k2) ||
                    std::isnan(dir1x) || std::isinf(dir1x) ||
                    std::isnan(dir1y) || std::isinf(dir1y) ||
                    std::isnan(dir1z) || std::isinf(dir1z) ||
                    std::isnan(dir2x) || std::isinf(dir2x) ||
                    std::isnan(dir2y) || std::isinf(dir2y) ||
                    std::isnan(dir2z) || std::isinf(dir2z)) {
                    warning() << "Nan/Inf values read at " << filename << ":" << line_count + 1;
                }


                points.emplace_back(Vector3(x, y, z), Vector3(nx, ny, nz));
                groundtruth.push_back(k1, k2, H, Vector3(dir1x, dir1y, dir1z), Vector3(dir2x, dir2y, dir2z),
                                      Vector3(nx, ny, nz));
            }
            else
                ++cmt_count;
        }
        ++line_count;
    }

    PCP_ASSERT(points.size() == line_count-cmt_count);
    PCP_ASSERT(groundtruth.size() == line_count-cmt_count);

    info() << points.size() << " curvatures data loaded from " << filename;

    return true;
}

bool CustomLoader::Save(const std::string& filename, const Geometry& points, const GlobalCurvatureData& groundtruth)
{
    info() << "Saving data to file " << filename;

    if(points.size() != groundtruth.size())
    {
        error() << "Wring sizes: " << points.size() << " != " << groundtruth.size();
        return false;
    }

    std::ofstream ofs(filename);
    if(!ofs.is_open())
    {
        warning() << "Failed to open output file " << filename;
        return false;
    }

    ofs << "# x y x Gauss_Curvature Mean_Curvature nx ny nz k1 k2 d1x d1y d1z d2x d2y d2z\n";
    for(int i=0; i<points.size(); ++i)
    {
        ofs << points.point(i).x()  << " "
            << points.point(i).y()  << " "
            << points.point(i).z()  << " "
            << groundtruth[i].K()   << " "
            << groundtruth[i].H()   << " "
            << points.normal(i).x() << " "
            << points.normal(i).y() << " "
            << points.normal(i).z() << " "
            << groundtruth[i].k1()  << " "
            << groundtruth[i].k2()  << " "
            << groundtruth[i].dir1().x() << " "
            << groundtruth[i].dir1().y() << " "
            << groundtruth[i].dir1().z() << " "
            << groundtruth[i].dir2().x() << " "
            << groundtruth[i].dir2().y() << " "
            << groundtruth[i].dir2().z() << "\n";
    }

    info() << points.size() << " curvatures data saved to " << filename;

    return true;
}

} // namespace pcp

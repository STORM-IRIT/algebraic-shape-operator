#include <PCP/Curvature/GlobalEstimationData.h>
#include <PCP/Common/Log.h>

#include <fstream>

namespace pcp {

GlobalEstimationData::GlobalEstimationData(int size) : m_data(size)
{
}

bool GlobalEstimationData::load(const std::string& filename)
{
    this->clear();

    std::ifstream ifs(filename);
    if(!ifs.is_open())
    {
        warning() << "Failed to open input file " << filename;
        return false;
    }

    Scalar k1;
    Scalar k2;
    Scalar H;
    Scalar nx;
    Scalar ny;
    Scalar nz;
    Scalar dir1x;
    Scalar dir1y;
    Scalar dir1z;
    Scalar dir2x;
    Scalar dir2y;
    Scalar dir2z;
    int    nei_count;
    Scalar time_ms;

    std::string str_k1;
    std::string str_k2;
    std::string str_H;
    std::string str_nx;
    std::string str_ny;
    std::string str_nz;
    std::string str_dir1x;
    std::string str_dir1y;
    std::string str_dir1z;
    std::string str_dir2x;
    std::string str_dir2y;
    std::string str_dir2z;
    std::string str_nei_count;
    std::string str_time_ms;

    std::string line;
    int line_count = 0;
    int cmt_count = 0;
    while(std::getline(ifs, line))
    {
        std::stringstream str(line);
        bool ok = str >> str_k1 &&
                  str >> str_k2 &&
                  str >> str_H  &&
                  str >> str_nx &&
                  str >> str_ny &&
                  str >> str_nz &&
                  str >> str_dir1x &&
                  str >> str_dir1y &&
                  str >> str_dir1z &&
                  str >> str_dir2x &&
                  str >> str_dir2y &&
                  str >> str_dir2z &&
                  str >> str_nei_count &&
                  str >> str_time_ms;

        if(!ok)
        {
            error() << "Failed to read line " << line_count+1 << " from file " << filename;
        }
        else {
            if (str_k1 != "#")
            {
                k1    = std::stof(str_k1   );
                k2    = std::stof(str_k2   );
                H     = std::stof(str_H    );
                nx    = std::stof(str_nx   );
                ny    = std::stof(str_ny   );
                nz    = std::stof(str_nz   );
                dir1x = std::stof(str_dir1x);
                dir1y = std::stof(str_dir1y);
                dir1z = std::stof(str_dir1z);
                dir2x = std::stof(str_dir2x);
                dir2y = std::stof(str_dir2y);
                dir2z = std::stof(str_dir2z);
                nei_count = std::stoi(str_nei_count);
                time_ms   = std::stof(str_time_ms);

                if(std::isnan(k1   ) || std::isinf(k1   ) ||
                   std::isnan(k2   ) || std::isinf(k2   ) ||
                   std::isnan(H    ) || std::isinf(H    ) ||
                   std::isnan(nx   ) || std::isinf(nx   ) ||
                   std::isnan(ny   ) || std::isinf(ny   ) ||
                   std::isnan(nz   ) || std::isinf(nz   ) ||
                   std::isnan(dir1x) || std::isinf(dir1x) ||
                   std::isnan(dir1y) || std::isinf(dir1y) ||
                   std::isnan(dir1z) || std::isinf(dir1z) ||
                   std::isnan(dir2x) || std::isinf(dir2x) ||
                   std::isnan(dir2y) || std::isinf(dir2y) ||
                   std::isnan(dir2z) || std::isinf(dir2z) ||
                   std::isnan(nei_count) || std::isinf(nei_count) ||
                   std::isnan(time_ms)   || std::isinf(time_ms) )
                {
                    warning() << "Nan/Inf values read at " << filename << ":" << line_count+1;
                }

                this->push_back(k1, k2, H, Vector3(dir1x, dir1y, dir1z), Vector3(dir2x, dir2y, dir2z), Vector3(nx, ny, nz), nei_count, time_ms);
            }
            else
                ++cmt_count;
        }
        ++line_count;
    }
    info() << this->size() << " estimations data loaded from " << filename;

    return true;
}

bool GlobalEstimationData::save(const std::string& filename) const
{
    std::ofstream ofs(filename);
    if(!ofs.is_open())
    {
        warning() << "Failed to open output file " << filename;
        return false;
    }

    ofs << "# k1 k2 Mean_Curvature nx ny nz d1x d1y d1z d2x d2y d2z nei time\n";

    for(int i=0; i<this->size(); ++i)
    {
        ofs << m_data[i].m_k1        << " "
            << m_data[i].m_k2        << " "
            << m_data[i].m_H         << " "
            << m_data[i].m_N.x()     << " "
            << m_data[i].m_N.y()     << " "
            << m_data[i].m_N.z()     << " "
            << m_data[i].m_dir1.x()  << " "
            << m_data[i].m_dir1.y()  << " "
            << m_data[i].m_dir1.z()  << " "
            << m_data[i].m_dir2.x()  << " "
            << m_data[i].m_dir2.y()  << " "
            << m_data[i].m_dir2.z()  << " "
            << m_data[i].m_nei_count << " "
            << m_data[i].m_time_ms   << "\n";
    }

    info() << this->size() << " estimations data saved to " << filename;

    return true;
}

void GlobalEstimationData::clear()
{
    m_data.clear();
}

void GlobalEstimationData::resize(int size)
{
    m_data.resize(size);
}

void GlobalEstimationData::push_back(Scalar k1, Scalar k2, Scalar H, const Vector3& dir1, const Vector3& dir2, const Vector3& normal, int nei_count, Scalar time_ms)
{
    m_data.push_back(PointWiseEstimationData(k1, k2, H, normal, dir1, dir2, nei_count, time_ms));
}

int GlobalEstimationData::size() const
{
    return m_data.size();
}

const PointWiseEstimationData& GlobalEstimationData::operator[](int i) const
{
    return m_data[i];
}

PointWiseEstimationData& GlobalEstimationData::operator[](int i)
{
    return m_data[i];
}

Scalar GlobalEstimationData::k1(int i) const
{
    return m_data[i].m_k1;
}

Scalar GlobalEstimationData::k2(int i) const
{
    return m_data[i].m_k2;
}

const Vector3& GlobalEstimationData::dir1(int i) const
{
    return m_data[i].m_dir1;
}

const Vector3& GlobalEstimationData::dir2(int i) const
{
    return m_data[i].m_dir2;
}

const Vector3& GlobalEstimationData::normal(int i) const
{
    return m_data[i].m_N;
}

int GlobalEstimationData::nei_count(int i) const
{
    return m_data[i].m_nei_count;
}

Scalar GlobalEstimationData::time_ms(int i) const
{
    return m_data[i].m_time_ms;
}

Scalar& GlobalEstimationData::k1(int i)
{
    return m_data[i].m_k1;
}

Scalar& GlobalEstimationData::k2(int i)
{
    return m_data[i].m_k2;
}

Vector3& GlobalEstimationData::dir1(int i)
{
    return m_data[i].m_dir1;
}

Vector3& GlobalEstimationData::dir2(int i)
{
    return m_data[i].m_dir2;
}

Vector3& GlobalEstimationData::normal(int i)
{
    return m_data[i].m_N;
}

int& GlobalEstimationData::nei_count(int i)
{
    return m_data[i].m_nei_count;
}

Scalar& GlobalEstimationData::time_ms(int i)
{
    return m_data[i].m_time_ms;
}

int GlobalEstimationData::average_nei_count() const
{
    Scalar sum_nei_count = 0;
    Scalar sum = 0;
    for(const auto& it : m_data)
    {
        if(it.valid())
        {
            sum_nei_count += it.nei_count();
            ++sum;
        }
    }
    return sum > 0 ? int(sum_nei_count/sum) : 0;
}

int GlobalEstimationData::valid_count() const
{
    int sum = 0;
    for(const auto& it : m_data)
    {
        if(it.valid())
        {
            ++sum;
        }
    }
    return sum;
}

} // namespace pcp

























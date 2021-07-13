#include <PCP/Curvature/GlobalErrorData.h>
#include <PCP/Common/Log.h>

#include <fstream>

namespace pcp {

GlobalErrorData::GlobalErrorData(int size) :
    m_data(size)
{
}

bool GlobalErrorData::load(const std::string& filename)
{
    this->clear();

    std::ifstream ifs(filename);
    if(!ifs.is_open())
    {
        warning() << "Failed to open output file " << filename;
        return false;
    }

    Scalar err_k1   ;
    Scalar err_k2   ;
    Scalar err_H    ;
    Scalar err_N    ;
    Scalar err_dir1 ;
    Scalar err_dir2 ;
    Scalar nei_count;
    Scalar time_ms  ;

    std::string str_err_k1   ;
    std::string str_err_k2   ;
    std::string str_err_H    ;
    std::string str_err_N    ;
    std::string str_err_dir1 ;
    std::string str_err_dir2 ;
    std::string str_nei_count;
    std::string str_time_ms  ;

    std::string line;
    int line_count = 0;
    while(std::getline(ifs, line))
    {
        std::stringstream str(line);
        bool ok = str >> str_err_k1    &&
                  str >> str_err_k2    &&
                  str >> str_err_H     &&
                  str >> str_err_N     &&
                  str >> str_err_dir1  &&
                  str >> str_err_dir2  &&
                  str >> str_nei_count &&
                  str >> str_time_ms;

        if(!ok)
        {
            error() << "Failed to read line " << line_count+1 << " from file " << filename;
        }
        else
        {
            err_k1    = std::stof(str_err_k1   );
            err_k2    = std::stof(str_err_k2   );
            err_H     = std::stof(str_err_H    );
            err_N     = std::stof(str_err_N    );
            err_dir1  = std::stof(str_err_dir1 );
            err_dir2  = std::stof(str_err_dir2 );
            nei_count = std::stof(str_nei_count);
            time_ms   = std::stof(str_time_ms  );

            if(std::isnan(err_k1   ) || std::isinf(err_k1   ) ||
               std::isnan(err_k2   ) || std::isinf(err_k2   ) ||
               std::isnan(err_H    ) || std::isinf(err_H    ) ||
               std::isnan(err_N    ) || std::isinf(err_N    ) ||
               std::isnan(err_dir1 ) || std::isinf(err_dir1 ) ||
               std::isnan(err_dir2 ) || std::isinf(err_dir2 ) ||
               std::isnan(nei_count) || std::isinf(nei_count) ||
               std::isnan(time_ms  ) || std::isinf(time_ms  ))
            {
                warning() << "Nan/Inf values read at " << filename << ":" << line_count+1;
            }

            this->push_back(err_k1, err_k2, err_H, err_N, err_dir1, err_dir2, nei_count, time_ms);
        }
        ++line_count;
    }
    info() << this->size() << " errors data loaded from " << filename;

    return true;
}

bool GlobalErrorData::save(const std::string& filename) const
{
    std::ofstream ofs(filename);
    if(!ofs.is_open())
    {
        warning() << "Failed to open output file " << filename;
        return false;
    }

    for(int i=0; i<this->size(); ++i)
    {
        ofs << m_data[i].m_err_k1    << " "
            << m_data[i].m_err_k2    << " "
            << m_data[i].m_err_H     << " "
            << m_data[i].m_err_N     << " "
            << m_data[i].m_err_dir1  << " "
            << m_data[i].m_err_dir2  << " "
            << m_data[i].m_nei_count << " "
            << m_data[i].m_time_ms   << "\n";
    }

    info() << this->size() << " errors data saved to " << filename;

    return true;
}

void GlobalErrorData::clear()
{
    m_data.clear();
}

void GlobalErrorData::resize(int size)
{
    m_data.resize(size);
}

int GlobalErrorData::size() const
{
    return m_data.size();
}

void GlobalErrorData::push_back(Scalar err_k1, Scalar err_k2, Scalar err_H, Scalar err_N, Scalar err_dir1, Scalar err_dir2, Scalar nei_count, Scalar time_ms)
{
    m_data.push_back(PointWiseErrorData(
        err_k1,
        err_k2,
        err_H,
        err_N,
        err_dir1,
        err_dir2,
        nei_count,
        time_ms
    ));
}

const PointWiseErrorData& GlobalErrorData::operator[](int i) const
{
    return m_data[i];
}

PointWiseErrorData& GlobalErrorData::operator[](int i)
{
    return m_data[i];
}

} // namespace pcp

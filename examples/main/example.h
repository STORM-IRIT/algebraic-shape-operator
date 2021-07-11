#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <AlgebraicShapeOperator.h>

#include "include/opt.h"
#include "include/prog.h"
#include "include/log.h"
#include "include/plyio.h"
#include "include/KdTree.h"

#include <tuple>
#include <memory>

using KdTree = kdtree::KdTree<Eigen::Vector3f,3>;

std::tuple<std::string,std::string,float,bool,bool>
parse(int argc, char** argv) {
    opt::Option opt(argc, argv, "Compute differential properties on oriented point cloud using the Algebraic Shape Operator (ASO)");
    const std::string in_input  = opt("input",  "i").set_required().set_brief(
                                  "Input PLY file (3D points and their normal vectors)");
    const std::string in_output = opt("output", "o").set_default("output.txt").set_brief(
                                  "Output TXT file with one line per point containing "
                                  "(k1,k2, d1x,d1y,d1z, d2x,d2y,d2z, nx,ny,nz)");
    const float in_radius = opt("radius", "r").set_required().set_brief(
                                "Absolute value of the radius (or ratio if -ratio)");
    const bool in_ratio  = opt("ratio").set_brief(
                               "Set the radius as a ratio of the axis-aligned "
                               "bounding box diagonal length");
    const auto ok = opt.ok();
    return std::make_tuple(in_input, in_output, in_radius, in_ratio, ok);
}

std::tuple<std::shared_ptr<std::vector<Eigen::Vector3f>>, std::shared_ptr<std::vector<Eigen::Vector3f>>>
load_ply(const std::string& filename)
{
    std::ifstream ifs(filename);
    if(not ifs.is_open()) {
        Log::error() << "Failed to open input PLY file '" << filename << "'";
        std::abort();
    }
    auto ply = plyio::PLYReader();
    auto ok = ply.read_header(ifs);
    if(not ok) {
        Log::error() << "Failed to read header of file '" << filename << "':";
        ply.print_errors();
        std::abort();
    }
    if(not ply.has_element("vertex")) {
        Log::error() << "Missing element 'vertex' in file '" << filename << "'";
        std::abort();
    }
    if(not ply.has_property("vertex", "x")) {
        Log::error() << "Missing vertex property 'x' in file '" << filename << "'";
        std::abort();
    }
    if(not ply.has_property("vertex", "y")) {
        Log::error() << "Missing vertex property 'y' in file '" << filename << "'";
        std::abort();
    }
    if(not ply.has_property("vertex", "z")) {
        Log::error() << "Missing vertex property 'z' in file '" << filename << "'";
        std::abort();
    }
    if(not ply.has_property("vertex", "nx")) {
        Log::error() << "Missing vertex property 'nx' in file '" << filename << "'";
        Log::error() << "Normal vectors are required";
        std::abort();
    }
    if(not ply.has_property("vertex", "ny")) {
        Log::error() << "Missing vertex property 'ny' in file '" << filename << "'";
        Log::error() << "Normal vectors are required";
        std::abort();
    }
    if(not ply.has_property("vertex", "nz")) {
        Log::error() << "Missing vertex property 'nz' in file '" << filename << "'";
        Log::error() << "Normal vectors are required";
        std::abort();
    }

    const auto count = ply.element_count("vertex");

    if(count == 0) {
        Log::error() << "Zero points in file '" << filename << "'";
        std::abort();
    }

    Log::info() << "Loading " << count << " points and normal vectors from '" << filename << "'...";

    auto points = std::make_shared<std::vector<Eigen::Vector3f>>(count);
    auto normals = std::make_shared<std::vector<Eigen::Vector3f>>(count);

    ply.property("vertex", "x").reader.read([&](int i, float f){(*points)[i].x() = f;});
    ply.property("vertex", "y").reader.read([&](int i, float f){(*points)[i].y() = f;});
    ply.property("vertex", "z").reader.read([&](int i, float f){(*points)[i].z() = f;});
    ply.property("vertex", "nx").reader.read([&](int i, float f){(*normals)[i].x() = f;});
    ply.property("vertex", "ny").reader.read([&](int i, float f){(*normals)[i].y() = f;});
    ply.property("vertex", "nz").reader.read([&](int i, float f){(*normals)[i].z() = f;});

    ok = ply.read_body(ifs);

    if(not ok) {
        Log::error() << "Failed to read body of file '" << filename << "':";
        ply.print_errors();
        std::abort();
    }

    Log::info() << count << " points and normal vectors loaded from '" << filename << "'";

    return std::make_tuple(points, normals);
}

float compute_aabb_diag(std::shared_ptr<std::vector<Eigen::Vector3f>> points)
{
    Eigen::AlignedBox3f aabb;
    for(const auto& p : *points) {
        aabb.extend(p);
    }
    return aabb.diagonal().norm();
}

struct OrientedPoint
{
    OrientedPoint(int i,
                  std::shared_ptr<std::vector<Eigen::Vector3f>> points,
                  std::shared_ptr<std::vector<Eigen::Vector3f>> normals) :
        m_i(i), m_points(points), m_normals(normals) {}
    const Eigen::Vector3f& position() const {return (*m_points)[m_i];}
    const Eigen::Vector3f& normal() const {return (*m_normals)[m_i];}
    int m_i;
    std::shared_ptr<std::vector<Eigen::Vector3f>> m_points;
    std::shared_ptr<std::vector<Eigen::Vector3f>> m_normals;
};

struct OrientedPointIterator
{
    OrientedPointIterator(KdTree::RangeIndexQuery::IteratorType it,
                          std::shared_ptr<std::vector<Eigen::Vector3f>> points,
                          std::shared_ptr<std::vector<Eigen::Vector3f>> normals) :
        m_it(it), m_points(points), m_normals(normals) {}
    bool operator !=(const OrientedPointIterator it) {return m_it != it.m_it;}
    void operator ++() {++m_it;}
    OrientedPoint operator *() {return OrientedPoint(*m_it, m_points, m_normals);}
    KdTree::RangeIndexQuery::IteratorType m_it;
    std::shared_ptr<std::vector<Eigen::Vector3f>> m_points;
    std::shared_ptr<std::vector<Eigen::Vector3f>> m_normals;
};

void save_results(const std::string& filename,
                  const std::vector<aso::DifferentialProperties<float>>& diff_prop)
{
    std::ofstream ofs(filename);
    if(not ofs.is_open()) {
        Log::error() << "Failed to open output file '" << filename << "'";
        return;
    }
    for(auto i = 0u; i < diff_prop.size(); ++i) {
        ofs << diff_prop[i].k1() << ','
            << diff_prop[i].k2() << ','
            << diff_prop[i].d1().x() << ','
            << diff_prop[i].d1().y() << ','
            << diff_prop[i].d1().z() << ','
            << diff_prop[i].d2().x() << ','
            << diff_prop[i].d2().y() << ','
            << diff_prop[i].d2().z() << ','
            << diff_prop[i].n().x() << ','
            << diff_prop[i].n().y() << ','
            << diff_prop[i].n().z() << '\n';
    }
    Log::info() << diff_prop.size() << " differential properties saved to '" << filename << "'";
}

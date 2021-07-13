#include <AlgebraicShapeOperator.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/readPLY.h>
#include <igl/octree.h>
#include <igl/knn.h>
#include <igl/parallel_for.h>
#include <igl/colormap.h>

#include "../common/log.h"

struct KnnIterator
{
    KnnIterator(const Eigen::MatrixXi* I, const Eigen::MatrixXd* V,
                const Eigen::MatrixXd* N, int row, int col) :
        m_I(I), m_V(V), m_N(N), m_row(row), m_col(col) {}

    Eigen::Vector3d position() const {
        return m_V->row((*m_I)(m_row,m_col)).transpose();
    }
    Eigen::Vector3d normal() const {
        return m_N->row((*m_I)(m_row,m_col)).transpose();
    }

    bool operator != (const KnnIterator& it) const {return m_col != it.m_col;}
    void operator ++ () {++m_col;}
    const KnnIterator& operator * () const {return *this;}

    const Eigen::MatrixXi* m_I;
    const Eigen::MatrixXd* m_V;
    const Eigen::MatrixXd* m_N;
    int m_row;
    int m_col;
};

int main(int argc, char *argv[])
{
    constexpr auto k = 20;

    Eigen::MatrixXd V, N;

    Eigen::MatrixXd UV,VD,FD,ED;
    Eigen::MatrixXi F,E;
    std::vector<std::string> Vheader,Fheader,Eheader;
    std::vector<std::string> comments;

    const auto ok = igl::readPLY(argv[1],V,F,E,N,UV,VD,
                                 Vheader,FD,Fheader,ED,Eheader,comments);
    if(not ok) {
        return 1;
    }

    std::vector<std::vector<int>> point_indices;
    Eigen::MatrixXi CH;
    Eigen::MatrixXd CN;
    Eigen::VectorXd W;
    Eigen::MatrixXi I;
    igl::octree(V, point_indices, CH, CN, W);
    igl::knn(V, k, point_indices, CH, CN, W, I);

    Eigen::VectorXd K1(V.rows());
    Eigen::VectorXd K2(V.rows());
    Eigen::VectorXd H(V.rows());
    Eigen::VectorXd K(V.rows());
    Eigen::MatrixXd D1(V.rows(),3);
    Eigen::MatrixXd D2(V.rows(),3);
    Eigen::MatrixXd NC(V.rows(),3);

    igl::parallel_for(V.rows(), [&](int i)
    {
        const Eigen::Vector3d p = V.row(i).transpose();
        double r = 0;
        for(auto j = 0; j < k; ++j)
            r = std::max(r, (p - V.row(I(i,j)).transpose()).norm());
        r *= 1.5;
        const auto begin = KnnIterator(&I, &V, &N, i, 0);
        const auto end = KnnIterator(&I, &V, &N, i, k);

        // compute the Algebraic Shape Operator (ASO)
        const auto diff_prop = aso::compute(p, r, begin, end);

        K1(i) = diff_prop.k1();
        K2(i) = diff_prop.k2();
        H[i]  = diff_prop.H();
        K[i]  = diff_prop.K();
        D1.row(i) = diff_prop.d1().transpose();
        D2.row(i) = diff_prop.d2().transpose();
        NC.row(i) = diff_prop.n().transpose();
    });

    Eigen::MatrixXd COLH;
    Eigen::MatrixXd COLK1;
    Eigen::MatrixXd COLK2;
    Eigen::MatrixXd COLK;

    constexpr auto cm = igl::ColorMapType::COLOR_MAP_TYPE_JET;
    constexpr auto normalize = true;
    igl::colormap(cm,H,normalize,COLH);
    igl::colormap(cm,K1,normalize,COLK1);
    igl::colormap(cm,K2,normalize,COLK2);
    igl::colormap(cm,K,normalize,COLK);

    int idx = 0;

    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_points(V,COLH);
    viewer.data().point_size = 5.f;
    viewer.callback_key_up = [&](igl::opengl::glfw::Viewer& viewer,
                                 unsigned int, int) -> bool
    {
        idx = (idx+1) % 4;
        if(idx == 0) {
            Log::info() << "Mean curvature H";
            viewer.data().set_points(V,COLH);
        } else if(idx == 1) {
            Log::info() << "Maximal principal curvature k1";
            viewer.data().set_points(V,COLK1);
        } else if(idx == 2) {
            Log::info() << "Minimal principal curvature k2";
            viewer.data().set_points(V,COLK2);
        } else if(idx == 3) {
            Log::info() << "Gaussian curvature K";
            viewer.data().set_points(V,COLK);
        }
        return true;
    };
    viewer.launch();
}

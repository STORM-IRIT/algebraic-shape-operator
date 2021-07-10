#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

#ifdef __CUDACC__
    # include <cuda.h>
    # define ASO_MULTIARCH __host__ __device__
#else
    # define ASO_MULTIARCH
#endif
#ifdef __CUDA_ARCH__
    #define ASO_MULTIARCH_STD_MATH(FUNC)
#else
    #define ASO_MULTIARCH_STD_MATH(FUNC) using std::FUNC;
#endif

namespace aso {

//!
//! \brief Result of the Algebraic Shape Operator computation
//!
//! Notes:
//! - k1 and k2 are signed
//! - |k1| >= |k2| is always true
//! - {k1,k2,n} does NOT necessarily respect the 'right-hand rule'
//! - d1, d2 and n are normalized (except when the computation fails)
//!
template<typename ScalarT>
class DifferentialProperties
{
public:
    using Scalar = ScalarT;
    using Vector3 = Eigen::Matrix<Scalar,3,1>;

public:
   ASO_MULTIARCH inline DifferentialProperties() = default;
   ASO_MULTIARCH inline DifferentialProperties(Scalar k1,
                                               Scalar k2,
                                               const Vector3& d1,
                                               const Vector3& d2,
                                               const Vector3& n) :
        m_k1(k1), m_k2(k2),
        m_d1(d1), m_d2(d2),
        m_n(n) {}

public:
   //! \brief maximal principal curvature (in absolute value)
    ASO_MULTIARCH inline auto k1() const {return m_k1;}
    //! \brief minimal principal curvature (in absolute value)
    ASO_MULTIARCH inline auto k2() const {return m_k2;}
    //! \brief principal direction of k1
    ASO_MULTIARCH inline const auto& d1() const {return m_d1;}
    //! \brief principal direction of k2
    ASO_MULTIARCH inline const auto& d2() const {return m_d2;}
    //! \brief normal vector
    ASO_MULTIARCH inline const auto& n() const {return m_n;}
    //! \brief Mean curvature
    ASO_MULTIARCH inline auto H() const {return Scalar(0.5) * (m_k1 + m_k2);}
    //! \brief Gaussian curvature
    ASO_MULTIARCH inline auto K() const {return m_k1 * m_k2;}

protected:
    Scalar m_k1;
    Scalar m_k2;
    Vector3 m_d1;
    Vector3 m_d2;
    Vector3 m_n;
}; // DifferentialProperties

//!
//! \brief compute the Algebraic Shape Operator (ASO) differential properties
//!
//! \param p  the point where to compute the ASO
//! \param r  the radius of the weighting kernel
//! \param first  begin iterator to oriented points
//! \param last  end iterator to oriented points
//!
//! \return the computed DifferentialProperties
//!
//! \tparam Point  (like Eigen::Vector3f)
//! \tparam Scalar  (like float)
//! \tparam OrientedPointIterator  iterator over oriented points
//!
//! The value type of OrientedPointIterator must implement two methods:
//! - position() that returns the neighboring point position
//! - normal() that returns the neighboring point normal vector
//! They typically return an Eigen::Vector3f or an Eigen::Map<Eigen::Vector3f>.
//! The range between first and last is used in one single pass.
//!
//! The computation fails if:
//! - less than 3 neighbors are processed
//! - the algebraic sphere is degenerated (can happens if all the points are
//!   sticked together for instance)
//!
//! \warning Scalar must match Point::Scalar and OrientedPointIterator's Scalar
//! \note points with a distance to p greater than r are skipped
//! \note first and last are used in a single pass
//!
template<class Point,
         class Scalar,
         class OrientedPointIterator>
ASO_MULTIARCH inline auto compute(const Point& p,
                                  const Scalar r,
                                  const OrientedPointIterator first,
                                  const OrientedPointIterator last);

//!
//! \brief same as above but using seperate iterators over positions and normals
//!
//! \tparam PositionIterator  iterator over positions (like Eigen::Vector3f)
//! \tparam NormalIterator  iterator over normal vectors (like Eigen::Vector3f)
//!
template<class Point,
         class Scalar,
         class PositionIterator,
         class NormalIterator>
ASO_MULTIARCH inline auto compute(const Point& p,
                                  const Scalar r,
                                  const PositionIterator position_first,
                                  const PositionIterator position_last,
                                  const NormalIterator normal_first,
                                  const NormalIterator normal_last);

} // namespace aso

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// IMPLEMENTATION ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

namespace aso {
namespace weighting {

template<typename T> using Vector3 = Eigen::Matrix<T,3,1>;
template<typename T> using Matrix33 = Eigen::Matrix<T,3,3>;

//! \brief smooth weighting kernel
template<typename Scalar>
ASO_MULTIARCH inline Scalar K(const Scalar& x) {
    const auto y = x*x - Scalar(1.); return y*y;
}
template<typename Scalar>
ASO_MULTIARCH inline Scalar dK(const Scalar& x) {
    return Scalar(4.) * x * (x*x - Scalar(1.));
}
template<typename Scalar>
ASO_MULTIARCH inline Scalar d2K(const Scalar& x) {
    return Scalar(12.) * x*x - Scalar(4.);
}

//! \brief weight function (assuming centered basis)
template<typename Scalar>
ASO_MULTIARCH inline Scalar weight(const Vector3<Scalar>& p, Scalar r) {
  const Scalar d  = p.norm();
  return (d <= r) ? K(d/r) : Scalar(0.);
}

template<typename Scalar>
ASO_MULTIARCH inline Vector3<Scalar> d_weight(const Vector3<Scalar>& p,
                                              Scalar r) {
  const Scalar d = p.norm();
  if (d <= r && d != Scalar(0.)) {
    return (p / (d * r)) * dK(d/r);
  } else {
    return Vector3<Scalar>::Zero();
  }
}

template<typename Scalar>
ASO_MULTIARCH inline Matrix33<Scalar> d2_weight(const Vector3<Scalar>& p,
                                                Scalar r)
{
  Matrix33<Scalar> result = Matrix33<Scalar>::Zero();
  const Scalar d = p.norm();
  if(d <= r && d != Scalar(0.))
  {
    const Scalar der = dK(d/r);
    result = p * p.transpose() / d * (d2K(d/r) / r - der / d);
    result.diagonal().array() += der;
    result *= Scalar(1.) / (r * d);
  }
  return result;
}

} // namespace weighting

template<typename Point, class Scalar, class OrientedPointIterator>
ASO_MULTIARCH auto compute(const Point& p,
                           const Scalar r,
                           const OrientedPointIterator first,
                           const OrientedPointIterator last)
{
    ASO_MULTIARCH_STD_MATH(max);
    ASO_MULTIARCH_STD_MATH(abs);

    using Vector3 = Eigen::Matrix<Scalar,3,1>;
    using Row3 = Eigen::Matrix<Scalar,1,3>;
    using Matrix22 = Eigen::Matrix<Scalar,2,2>;
    using Matrix32 = Eigen::Matrix<Scalar,3,2>;
    using Matrix33 = Eigen::Matrix<Scalar,3,3>;
    using Index = typename Vector3::Index;

    Vector3 sumP = Vector3::Zero();
    Vector3 sumN = Vector3::Zero();
    Scalar sumDotPN = Scalar(0.0);
    Scalar sumDotPP = Scalar(0.0);
    Scalar sumW     = Scalar(0.0);

    Matrix33 dSumN = Matrix33::Zero();
    Matrix33 dSumP = Matrix33::Zero();

    Row3 dSumDotPN = Row3::Zero();
    Row3 dSumDotPP = Row3::Zero();
    Row3 dSumW = Row3::Zero();

    Matrix33 d2SumDotPN = Matrix33::Zero();
    Matrix33 d2SumDotPP = Matrix33::Zero();
    Matrix33 d2SumW     = Matrix33::Zero();

    Matrix33 d2SumP[3] = {Matrix33::Zero(),Matrix33::Zero(),Matrix33::Zero()};
    Matrix33 d2SumN[3] = {Matrix33::Zero(),Matrix33::Zero(),Matrix33::Zero()};

    Index nei_count = 0;

    // add neighbors
    for(auto it = first; it != last; ++it)
    {
        // centered basis
        const Vector3 q = (*it).position() - p;
        const Vector3 n = (*it).normal();
        const Scalar w = weighting::weight(q, r);

        if(w == Scalar(0.)) {
            // skip points with distance to p greater than r
            continue;
        }

        ++nei_count;

        sumP     += w * q;
        sumN     += w * n;
        sumDotPN += w * q.dot(n);
        sumDotPP += w * q.squaredNorm();
        sumW     += w;

        const Row3 dw = - weighting::d_weight(q, r).transpose();

        dSumW     += dw;
        dSumP     += q * dw;
        dSumN     += n * dw;
        dSumDotPN += dw * q.dot(n);
        dSumDotPP += dw * q.squaredNorm();

        const Matrix33 d2w = weighting::d2_weight(q, r);

        d2SumDotPN += d2w * q.dot(n);
        d2SumDotPP += d2w * q.squaredNorm();
        d2SumW     += d2w;

        for(auto i=0; i<3; ++i)
        {
            d2SumP[i] += d2w * q[i];
            d2SumN[i] += d2w * n[i];
        }
    }

    if(sumW == Scalar(0.) || nei_count < 3)
    {
        // too few points
        return DifferentialProperties<Scalar>();
    }

    const Scalar invSumW = Scalar(1.) / sumW;

    const Scalar nume = (sumDotPN - invSumW * sumP.dot(sumN));
    const Scalar den1 = invSumW * sumP.dot(sumP);
    const Scalar deno = sumDotPP - den1;

    // algebraic sphere parameters
    Scalar uc = Scalar(0);
    Vector3 ul = Vector3::Zero();
    Scalar uq = Scalar(0);

    const auto epsilon = Eigen::NumTraits<Scalar>::dummy_precision();

    if(abs(deno) < epsilon * max(sumDotPP, den1))
    {
        // degenerated case
        return DifferentialProperties<Scalar>();
    }
    else
    {
        // closed-form expression of the regression
        uq = Scalar(.5) * nume / deno;
        ul = (sumN - sumP * (Scalar(2.) * uq)) * invSumW;
        uc = - invSumW * (ul.dot(sumP) + sumDotPP * uq);
    }

    // first-order derivatives
    const Row3 dNume = dSumDotPN - invSumW * invSumW * (
        sumW * ( sumN.transpose() * dSumP + sumP.transpose() * dSumN)
        - dSumW * sumP.dot(sumN) );

    const Row3 dDeno = dSumDotPP - invSumW * invSumW * (
        Scalar(2.) * sumW * sumP.transpose() * dSumP
        - dSumW*sumP.dot(sumP) );

    const Row3 dUq = Scalar(.5) * (deno * dNume - dDeno * nume) / (deno * deno);

    const Matrix33 dUl = invSumW * (
        dSumN - ul * dSumW - Scalar(2.) * (dSumP * uq + sumP * dUq) );
    const Row3 dUc = - invSumW * (
        sumP.transpose() * dUl
        + sumDotPP * dUq
        + ul.transpose() * dSumP
        + uq * dSumDotPP
        + dSumW * uc );

    // second-order derivatives
    Matrix33 sumdSumPdSumN  = Matrix33::Zero();
    Matrix33 sumd2SumPdSumN = Matrix33::Zero();
    Matrix33 sumd2SumNdSumP = Matrix33::Zero();
    Matrix33 sumdSumPdSumP  = Matrix33::Zero();
    Matrix33 sumd2SumPdSumP = Matrix33::Zero();

    for(auto i=0; i<3; ++i)
    {
        sumdSumPdSumN  += dSumN.row(i).transpose() * dSumP.row(i);
        sumd2SumPdSumN += d2SumP[i] * sumN(i);
        sumd2SumNdSumP += d2SumN[i] * sumP(i);
        sumdSumPdSumP  += dSumP.row(i).transpose() * dSumP.row(i);
        sumd2SumPdSumP += d2SumP[i] * sumP(i);
    }

    const Matrix33 d2Nume = d2SumDotPN - invSumW*invSumW*invSumW*invSumW * (
        sumW * sumW* (
            sumW * (
                sumdSumPdSumN+sumdSumPdSumN.transpose()
                + sumd2SumPdSumN+sumd2SumNdSumP )
            + dSumW.transpose() * (
                sumN.transpose() * dSumP + sumP.transpose() * dSumN )
            - (sumP.transpose() * sumN) * d2SumW.transpose()
            - (dSumN.transpose() * sumP + dSumP.transpose() * sumN ) * dSumW )
        - Scalar(2.) * sumW * dSumW.transpose() * (
            sumW*(sumN.transpose() * dSumP + sumP.transpose() * dSumN)
            - (sumP.transpose() * sumN) * dSumW) );

    const Matrix33 d2Deno = d2SumDotPP - invSumW*invSumW*invSumW*invSumW * (
        sumW*sumW*(
            Scalar(2.) * sumW * (sumdSumPdSumP + sumd2SumPdSumP)
            + Scalar(2.) * dSumW.transpose() * (sumP.transpose() * dSumP)
            - (sumP.transpose() * sumP) * d2SumW.transpose()
            - Scalar(2.) * (dSumP.transpose() * sumP) * dSumW )
        - Scalar(2.) * sumW * dSumW.transpose() * (
            Scalar(2.) * sumW * sumP.transpose() * dSumP
            - (sumP.transpose() * sumP) * dSumW) );

    const Scalar deno2 = deno * deno;

    const Matrix33 d2Uq = Scalar(.5) / (deno2*deno2) * (
        deno2 * (
            dDeno.transpose()*dNume
            + deno*d2Nume
            - dNume.transpose()*dDeno
            - nume*d2Deno )
        - Scalar(2.) * deno * dDeno.transpose() * (
            deno * dNume - nume * dDeno ) );

    Matrix33 d2Ul[3];
    for(auto i=0; i<3; ++i)
    {
        d2Ul[i] = invSumW * (
            d2SumN[i]
            - Scalar(2.)* (
                d2Uq * sumP[i]
                + dSumP.row(i).transpose() * dUq
                + uq * d2SumP[i]
                + dUq.transpose() * dSumP.row(i) )
            - ul[i] * d2SumW
            - dUl.row(i).transpose() * dSumW
            - dSumW.transpose() * dUl.row(i) );
    }

    Matrix33 sumdUldSumP = Matrix33::Zero();
    Matrix33 sumUld2SumP = Matrix33::Zero();
    Matrix33 sumd2UlsumP = Matrix33::Zero();
    Matrix33 sumdSumPdUl = Matrix33::Zero();

    for(auto i=0; i<3; ++i)
    {
        sumdUldSumP += dUl.row(i).transpose() * dSumP.row(i);
        sumUld2SumP += ul[i] * d2SumP[i];
        sumd2UlsumP += d2Ul[i] * sumP[i];
        sumdSumPdUl += dSumP.row(i).transpose() * dUl.row(i);
    }

    const Matrix33 d2Uc = - invSumW * (
        sumdUldSumP
        + sumUld2SumP
        + sumd2UlsumP
        + sumdSumPdUl
        + dUq.transpose() * dSumDotPP
        + uq * d2SumDotPP
        + dSumDotPP.transpose() * dUq
        + d2Uq * sumDotPP
        + uc * d2SumW
        + dUc.transpose() * dSumW
        + dSumW.transpose() * dUc );

    // shape operator
    Matrix33 dN = d2Uc.transpose() + dUl.transpose() + dUl;
    dN.diagonal().array() += Scalar(2.) * uq;
    const Vector3 grad = dUc.transpose() + ul;
    const Scalar grad_norm = grad.norm();
    dN = dN / grad_norm - grad * grad.transpose() / (grad_norm * grad_norm) * dN;

    // tangent plane transform matrix
    Matrix32 P;
    auto i0 = Index(-1);
    auto i1 = Index(-1);
    auto i2 = Index(-1);
    const Vector3 normal = ul.normalized();
    // i0 is dimension where normal extends the least
    normal.array().abs().minCoeff(&i0);
    i1 = (i0+1) % 3;
    i2 = (i0+2) % 3;
    P.col(0)[i0] = 0;
    P.col(0)[i1] = normal[i2];
    P.col(0)[i2] = -normal[i1];
    P.col(0).normalize();
    P.col(1) = P.col(0).cross(normal);

    // Algebraic Shape Operator (ASO)
    Matrix22 W = P.transpose() * dN * P;

    W(0,1) = W(1,0) = (W(0,1) + W(1,0)) / Scalar(2);
    Eigen::SelfAdjointEigenSolver<Matrix22> eig;
    eig.computeDirect(W);

    Scalar k1 = eig.eigenvalues()(0);
    Scalar k2 = eig.eigenvalues()(1);

    Vector3 d1 = P * eig.eigenvectors().col(0);
    Vector3 d2 = P * eig.eigenvectors().col(1);

    // ensure that |k1| >= |k2|
    if(abs(k1) < abs(k2))
    {
#ifdef __CUDACC__
      Scalar tmpk = k1;
      k1 = k2;
      k2 = tmpk;
      Vector3 tmpd = d1;
      d1 = d2;
      d2 = tmpd;
#else
      std::swap(k1, k2);
      std::swap(d1, d2);
#endif
    }

    const Vector3 n = grad / grad_norm;

    return DifferentialProperties<Scalar>{k1, k2, d1, d2, n};
}

namespace iterator {
//!
//! \brief Zip two iterators into one.
//!
//! It encapsulates two iterators over positions and normal vectors into one
//! unique iterator.
//! The value returned by this iterator provides the two methods required by the
//! compute() function:
//! - position()
//! - normal()
//!
template<class PositionIterator, class NormalIterator>
class OrientedPointIterator
{
public:
    struct Value {
        ASO_MULTIARCH inline Value(PositionIterator p, NormalIterator n) :
            m_p(p), m_n(n) {}
        ASO_MULTIARCH inline const auto& position() const {return *m_p;}
        ASO_MULTIARCH inline const auto& normal() const {return *m_n;}
        PositionIterator m_p;
        NormalIterator m_n;
    }; // Value

    ASO_MULTIARCH inline OrientedPointIterator(PositionIterator p,
                                               NormalIterator n) :
        m_p(p), m_n(n) {}

    ASO_MULTIARCH inline OrientedPointIterator& operator ++() {
        ++m_p;
        ++m_n;
        return *this;
    }
    ASO_MULTIARCH inline bool operator != (const OrientedPointIterator& other) const {
        return m_p != other.m_p || m_n != other.m_n;
    }
    ASO_MULTIARCH inline Value operator *() const {
        return Value(m_p, m_n);
    }

protected:
    PositionIterator m_p;
    NormalIterator m_n;
}; // OrientedPointIterator
} // namespace iterator

template<typename Point,
         class Scalar,
         class PositionIterator,
         class NormalIterator>
ASO_MULTIARCH inline auto compute(const Point& p,
                                  const Scalar r,
                                  const PositionIterator position_first,
                                  const PositionIterator position_last,
                                  const NormalIterator normal_first,
                                  const NormalIterator normal_last)
{
    const auto first = iterator::OrientedPointIterator(position_first,
                                                       normal_first);
    const auto last = iterator::OrientedPointIterator(position_last,
                                                      normal_last);
    return compute(p, r, first, last);
}

} // namespace aso

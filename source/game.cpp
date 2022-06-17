#include "game.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <concepts>
#include <numeric>
#include <ranges>
#include <type_traits>
#include <vector>

namespace lin_alg
{

template<typename T, unsigned D>
class CoordND
{
public:
  constexpr CoordND() = default;
  constexpr CoordND(const CoordND&) = default;
  constexpr CoordND(CoordND&&) = default;

  constexpr CoordND operator=(const CoordND& rhs)
  {
    coords_ = rhs.coords_;
    return *this;
  }
  constexpr CoordND operator=(CoordND&& rhs)
  {
    coords_ = std::move(rhs.coords_);
    return *this;
  }

  constexpr CoordND(std::initializer_list<float> lst)
      : CoordND {}
  {
    assert(lst.size() == D);
    std::copy(lst.begin(), lst.end(), coords_.begin());
  }

  template<typename U, unsigned N>  // clang-format off
  requires (D == N && std::convertible_to<U, T>)
  constexpr CoordND(const CoordND<U, N>& rhs)
      : coords_ {D}
  // clang-format on
  {
    std::copy(rhs.begin(), rhs.end(), coords_.begin());
  }

  constexpr T operator[](unsigned idx) const { return coords_[idx]; }
  constexpr T& operator[](unsigned idx) { return coords_[idx]; }

  constexpr float at(unsigned idx) const
  {
    assert(idx < D);
    return coords_[idx];
  }

  constexpr float& at(unsigned idx)
  {
    assert(idx < D);
    return coords_[idx];
  }

  constexpr float x() const requires(D > 0) { return (*this)[0]; }
  constexpr float& x() requires(D > 0) { return (*this)[0]; }

  constexpr float y() const requires(D > 1) { return (*this)[1]; }
  constexpr float& y() requires(D > 1) { return (*this)[1]; }

  constexpr float z() const requires(D > 2) { return (*this)[2]; }
  constexpr float& z() requires(D > 2) { return (*this)[2]; }

  constexpr float t() const requires(D > 3) { return (*this)[3]; }
  constexpr float& t() requires(D > 4) { return (*this)[3]; }

  constexpr T norm() const
  {
    T ret = 0;
    for (int i = 0; i < D; ++i)
      ret += (*this)[i] * (*this)[i];
    return std::sqrt(ret);
  }

  constexpr CoordND normalize() const
  {
    auto n = norm();
    auto ret = CoordND {0};
    for (int i = 0; i < D; ++i)
      ret[i] = (*this)[i] / n;

    assert(ret.norm() == T {1});

    return ret;
  }

  auto begin() { return coords_.begin(); }
  auto end() { return coords_.end(); }

  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator+(const CoordND<U, N>& lhs, const CoordND<U, N>& rhs);
  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator+(const CoordND<U, N>& rhs);
  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator+(const CoordND<U, N>& lhs, float rhs);
  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator+(float lhs, const CoordND<U, N>& rhs);

  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator-(const CoordND<U, N>& lhs, const CoordND<U, N>& rhs);
  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator-(const CoordND<U, N>& rhs);
  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator-(const CoordND<U, N>& lhs, float rhs);
  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator-(float lhs, const CoordND<U, N>& rhs);

  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator*(const CoordND<U, N>& lhs, float rhs);
  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator*(float lhs, const CoordND<U, N>& rhs);

  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator/(const CoordND<U, N>& lhs, float rhs);
  template<typename U, unsigned N>
  friend constexpr CoordND<U, N> operator/(float lhs, const CoordND<U, N>& rhs);

private:
  std::array<T, D> coords_;
};

template<typename T, unsigned D>
constexpr CoordND<T, D> operator+(const CoordND<T, D>& lhs, const CoordND<T, D>& rhs)
{
  CoordND<T, D> ret = {0};
  for (int i = 0; i < D; ++i)
    ret[i] = lhs[i] + rhs[i];
  return ret;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator+(const CoordND<T, D>& rhs)
{
  CoordND<T, D> ret = {0};
  for (int i = 0; i < D; ++i)
    ret[i] += rhs[i];
  return ret;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator+(const CoordND<T, D>& lhs, const T& rhs)
{
  CoordND<T, D> ret = {0};
  for (int i = 0; i < D; ++i)
    ret[i] = lhs[i] + rhs;
  return ret;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator+(const T& lhs, const CoordND<T, D>& rhs)
{
  return rhs + lhs;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator-(const CoordND<T, D>& lhs, const CoordND<T, D>& rhs)
{
  CoordND<T, D> ret = {0};
  for (int i = 0; i < D; ++i)
    ret[i] = lhs[i] - rhs[i];
  return ret;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator-(const CoordND<T, D>& rhs)
{
  CoordND<T, D> ret = {0};
  for (int i = 0; i < D; ++i)
    ret[i] -= rhs[i];
  return ret;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator-(const CoordND<T, D>& lhs, const T& rhs)
{
  CoordND<T, D> ret = {0};
  for (int i = 0; i < D; ++i)
    ret[i] = lhs[i] - rhs;
  return ret;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator-(const T& lhs, const CoordND<T, D>& rhs)
{
  return rhs - lhs;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator*(const CoordND<T, D>& lhs, const T& rhs)
{
  CoordND<T, D> ret = {0};
  for (int i = 0; i < D; ++i)
    ret[i] = lhs[i] * rhs;
  return ret;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator*(const T& lhs, const CoordND<T, D>& rhs)
{
  return rhs - lhs;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator/(const CoordND<T, D>& lhs, const T& rhs)
{
  CoordND<T, D> ret = {0};
  for (int i = 0; i < D; ++i)
    ret[i] = lhs[i] / rhs;
  return ret;
}

template<typename T, unsigned D>
constexpr CoordND<T, D> operator/(const T& lhs, const CoordND<T, D>& rhs)
{
  return rhs - lhs;
}

template<typename T>
using Coord1D = CoordND<T, 1>;
template<typename T>
using Coord2D = CoordND<T, 2>;
template<typename T>
using Coord3D = CoordND<T, 3>;
template<typename T>
using Coord4D = CoordND<T, 4>;

template<typename T, unsigned D>
class PointND : public CoordND<T, D>
{
};
template<typename T>
using Point1D = PointND<T, 1>;
template<typename T>
using Point2D = PointND<T, 2>;
template<typename T>
using Point3D = PointND<T, 3>;
template<typename T>
using Point4D = PointND<T, 4>;

template<typename T, unsigned D>
class VecND : public CoordND<T, D>
{
public:
  using CoordND<T, D>::CoordND;

  template<typename U, unsigned N>  // clang-format off
  requires (D == N && std::convertible_to<U, T>)
  constexpr VecND(const PointND<U, N>& pnt_beg, const PointND<U, N>& pnt_end)
      : CoordND<T, D>::CoordND(pnt_end - pnt_beg)
  // clang-format on
  {
  }

  constexpr T dot(const VecND<T, D>& rhs) const
  {
    return std::inner_product(this->begin(), this->end(), rhs.begin(), 0);
  }

  constexpr VecND<T, D> cross(const VecND<T, D>& rhs) const requires(D == 3)
  {
    auto x = *this;
    auto y = rhs;
    return {
        // x^y dim 3
        x[1] * y[2] - x[2] * y[1],  // x1.y2 - x2.y1
        x[2] * y[0] - x[0] * y[2],  // x2.y0 - x0.y2
        x[0] * y[1] - x[1] * y[0]  // x0.y1 - x1.y0
    };
  }

  constexpr VecND<T, D> cross(const VecND<T, D>& rhs) const requires(D == 7)
  {
    auto x = *this;
    auto y = rhs;
    return {// x^y dim 7
            // x1.y3 - x3.y1 + x2.y6 - x6.y2 + x4.y5 - x5.y4
            x[1] * y[3] - x[3] * y[1] + x[2] * y[6] - x[6] * y[2] + x[4] * y[5] - x[5] * y[4],
            // x2.y4 - x4.y2 + x3.y0 - x0.y3 + x5.y6 - x6.y5
            x[2] * y[4] - x[4] * y[2] + x[3] * y[0] - x[0] * y[3] + x[5] * y[6] - x[6] * y[5],
            // x3.y5 - x5.y3 + x4.y1 - x1.y4 + x6.y0 - x0.y6
            x[3] * y[5] - x[5] * y[3] + x[4] * y[1] - x[1] * y[4] + x[6] * y[0] - x[0] * y[6],
            // x4.y6 - x6.y4 + x5.y2 - x2.y5 + x0.y1 - x1.y0
            x[4] * y[6] - x[6] * y[4] + x[5] * y[2] - x[2] * y[5] + x[0] * y[1] - x[1] * y[0],
            // x5.y0 - x0.y5 + x6.y3 - x3.y6 + x2.y3 - x3.y2
            x[5] * y[0] - x[0] * y[5] + x[6] * y[3] - x[3] * y[6] + x[2] * y[3] - x[3] * y[2],
            // x6.y1 - x1.y6 + x0.y4 - x4.y0 + x2.y3 - x3.y2
            x[6] * y[1] - x[1] * y[6] + x[0] * y[4] - x[4] * y[0] + x[2] * y[3] - x[3] * y[2],
            // x0.y2 - x2.y0 + x1.y5 - x5.y1 + x3.y4 - x4.y3
            x[0] * y[2] - x[2] * y[0] + x[1] * y[5] - x[5] * y[1] + x[3] * y[4] - x[4] * y[3]};
  }
};
template<typename T>
using Vec1D = VecND<T, 1>;
template<typename T>
using Vec2D = VecND<T, 2>;
template<typename T>
using Vec3D = VecND<T, 3>;
template<typename T>
using Vec4D = VecND<T, 4>;

template<typename T, unsigned D>
constexpr auto angle(const VecND<T, D>& lhs, const VecND<T, D>& rhs)
{
  return std::acos(dot(lhs, rhs) / (lhs.norm() * rhs.norm()));
}

template<typename T, unsigned D>
constexpr auto euclidian_norm(const VecND<T, D>& vec)
{
  return std::sqrt(dot(vec, vec));
}

template<typename T, unsigned D>
class TriangleABC
{
public:
  TriangleABC(const PointND<T, D>& aa, const PointND<T, D>& bb, const PointND<T, D>& cc)
      : a_(aa)
      , b_(bb)
      , c_(cc)
      , ab_(b_ - a_)
      , bc_(c_ - b_)
      , ac_(c_ - a_)
      , na_(bc_.norm())
      , nb_(ac_.norm())
      , nc_(ab_.norm())
      // Al-Kashi
      // c² = a² + b² -2ab.cos(AC, CB)
      // (c² - a² - a²) / (-2ab) = cos(AC,CB)
      // angle(AC,CB) = acos((c² - a² - b²) / (-2ab))
      , acb_(std::acos((nc_ * nc_ - na_ * na_ - nb_ * nb_) / (-2. * na_ * nb_)))
      , cba_(std::acos((nb_ * nb_ - na_ * na_ - nc_ * nc_) / (-2. * na_ * nc_)))
      , bac_(std::acos((na_ * na_ - nb_ * nb_ - nc_ * nc_) / (-2. * nb_ * nc_)))
      , circumscribed_circle_center_(na_ * std::cos(bac_), nb_ * std::cos(cba_), nc_ * std::cos(acb_))
      , circumscribed_circle_radius_((na_ / std::sin(bac_)) / 2)
  {
  }

  const PointND<T, D>& a() const { return a_; }
  const PointND<T, D>& b() const { return b_; }
  const PointND<T, D>& c() const { return c_; }

  const VecND<T, D>& ab() const { return ab_; }
  const VecND<T, D>& bc() const { return bc_; }
  const VecND<T, D>& ac() const { return ac_; }

  T na() const { return na_; }
  T nb() const { return nb_; }
  T nc() const { return nc_; }

  T acb() const { return acb_; }
  T cba() const { return cba_; }
  T bac() const { return bac_; }

  const PointND<T, D>& circumscribed_circle_center() const { return circumscribed_circle_center_; }
  T circumscribed_circle_radius() const { return circumscribed_circle_radius_; }

private:
  PointND<T, D> a_, b_, c_;
  VecND<T, D> ab_, bc_, ac_;
  T na_, nb_, nc_;
  T acb_, cba_, bac_;
  PointND<T, D> circumscribed_circle_center_;
  T circumscribed_circle_radius_;
};

/*
struct Axes3D
{
  Point3D orig = {0, 0, 0};
  std::array<Vec3D, 3> axes = {Vec3D {1, 0, 0}, Vec3D {0, 1, 0}, Vec3D {0, 0, 1}};
};

class Box3D
{
public:
  constexpr Box3D(float width, float height, float depth, const Point3D& bl_fg)
      : width_ {width}
      , height_ {height}
      , depth_ {depth}
      , bl_fg_ {bl_fg}
  {
  }

  constexpr Box3D(const Point3D& tl_bg, const Point3D br_fg)
      : width_ {(br_fg - tl_bg).x()}
      , height_ {(tl_bg - br_fg).y()}
      , depth_ {(br_fg - tl_bg).z()}
      , bl_fg_ {tl_bg.x(), br_fg.y(), br_fg.z()}
  {
  }

  constexpr Point3D bl_fg() const { return bl_fg_; }
  constexpr Point3D br_fg() const { return bl_fg_ + Vec3D {width_, 0, 0}; }
  constexpr Point3D tl_fg() const { return bl_fg_ + Vec3D {0, height_, 0}; }
  constexpr Point3D tr_fg() const { return bl_fg_ + Vec3D {width_, height_, 0}; }

  constexpr Point3D bl_bg() const { return bl_fg_ + Vec3D {0, 0, depth_}; }
  constexpr Point3D br_bg() const { return bl_fg_ + Vec3D {width_, 0, depth_}; }
  constexpr Point3D tl_bg() const { return bl_fg_ + Vec3D {0, height_, depth_}; }
  constexpr Point3D tr_bg() const { return bl_fg_ + Vec3D {width_, height_, depth_}; }

  constexpr std::array<Point3D, 8> vertices() const
  {
    return {bl_fg(), br_fg(), tl_fg(), tr_fg(), bl_bg(), br_bg(), tl_bg(), tr_bg()};
  }

private:
  float width_, height_, depth_;
  Point3D bl_fg_;
};
*/

}  // namespace lin_alg

Game::Game()
    : m_is_running(false)
{
}

void Game::run()
{
  m_is_running = true;
}

void Game::quit()
{
  m_is_running = false;
}

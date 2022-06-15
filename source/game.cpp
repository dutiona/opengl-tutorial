#include "game.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <ranges>
#include <type_traits>
#include <vector>

using namespace std::literals;

class Point3D
{
public:
  constexpr Point3D() = default;
  constexpr Point3D(const Point3D&) = default;
  constexpr Point3D(Point3D&&) = default;

  constexpr Point3D operator=(const Point3D& pnt)
  {
    coords_ = pnt.coords_;
    return *this;
  }
  constexpr Point3D operator=(Point3D&& pnt)
  {
    coords_ = std::move(pnt.coords_);
    return *this;
  }

  constexpr Point3D(std::initializer_list<float> lst)
      : Point3D {}
  {
    assert(lst.size() < 3);
    {
      auto b_l = lst.begin();
      auto e_l = lst.end();

      for (int i = 0; i < 3 && b_l != e_l; ++i, ++b_l)
        coords_[i] = *b_l;
    }
  }

  constexpr float x() const { return coords_[0]; }
  constexpr float& x() { return coords_[0]; }

  constexpr float y() const { return coords_[1]; }
  constexpr float& y() { return coords_[1]; }

  constexpr float z() const { return coords_[2]; }
  constexpr float& z() { return coords_[2]; }

  constexpr float operator[](unsigned idx) const { return coords_[idx]; }
  constexpr float& operator[](unsigned idx) { return coords_[idx]; }

  constexpr float at(unsigned idx) const
  {
    assert(idx < 3);
    return coords_[idx];
  }

  constexpr float& at(unsigned idx)
  {
    assert(idx < 3);
    return coords_[idx];
  }

  friend constexpr Point3D operator+(const Point3D& lhs, const Point3D& rhs);
  friend constexpr Point3D operator+(const Point3D& rhs);
  friend constexpr Point3D operator+(const Point3D& lhs, float rhs);
  friend constexpr Point3D operator+(float lhs, const Point3D& rhs);

  friend constexpr Point3D operator-(const Point3D& lhs, const Point3D& rhs);
  friend constexpr Point3D operator-(const Point3D& rhs);
  friend constexpr Point3D operator-(const Point3D& lhs, float rhs);
  friend constexpr Point3D operator-(float lhs, const Point3D& rhs);

  friend constexpr Point3D operator*(const Point3D& lhs, float rhs);
  friend constexpr Point3D operator*(float lhs, const Point3D& rhs);

  friend constexpr Point3D operator/(const Point3D& lhs, float rhs);
  friend constexpr Point3D operator/(float lhs, const Point3D& rhs);

private:
  std::array<float, 3> coords_ = {0, 0, 0};
};

constexpr Point3D operator+(const Point3D& lhs, const Point3D& rhs)
{
  return {lhs.x() + rhs.x(), lhs.y() + rhs.y(), lhs.z() + rhs.z()};
}

constexpr Point3D operator+(const Point3D& rhs)
{
  return {+rhs.x(), +rhs.y(), +rhs.z()};
}

constexpr Point3D operator+(const Point3D& lhs, float rhs)
{
  return {lhs.x() + rhs, lhs.y() + rhs, lhs.z() + rhs};
}

constexpr Point3D operator+(float lhs, const Point3D& rhs)
{
  return rhs + lhs;
}

constexpr Point3D operator-(const Point3D& lhs, const Point3D& rhs)
{
  return {lhs.x() - rhs.x(), lhs.y() - rhs.y(), lhs.z() - rhs.z()};
}

constexpr Point3D operator-(const Point3D& rhs)
{
  return {-rhs.x(), -rhs.y(), -rhs.z()};
}

constexpr Point3D operator-(const Point3D& lhs, float rhs)
{
  return {lhs.x() - rhs, lhs.y() - rhs, lhs.z() - rhs};
}

constexpr Point3D operator-(float lhs, const Point3D& rhs)
{
  return rhs - lhs;
}

constexpr Point3D operator*(const Point3D& lhs, float rhs)
{
  return {lhs.x() * rhs, lhs.y() * rhs, lhs.z() * rhs};
}
constexpr Point3D operator*(float lhs, const Point3D& rhs)
{
  return rhs * lhs;
}

constexpr Point3D operator/(const Point3D& lhs, float rhs)
{
  return {lhs.x() / rhs, lhs.y() / rhs, lhs.z() / rhs};
}
constexpr Point3D operator/(float lhs, const Point3D& rhs)
{
  return rhs / lhs;
}

struct Vec3D : public Point3D
{
  using Point3D::Point3D;
  constexpr Vec3D(const Point3D& pnt_s, const Point3D& pnt_e)
      : Point3D(pnt_e - pnt_s)
  {
  }
};

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

#include "common/math/math_utils.hpp"

#include <cmath>
#include <utility>
#include "common/math/vec2d.hpp"

namespace msquare {
namespace planning_math {

double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double InnerProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
}

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * x1 + y0 * y1;
}

double WrapAngle(const double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;  //-pi~pi
}

double AngleDiff(const double from, const double to) {
  return NormalizeAngle(to - from);
}

int RandomInt(const int s, const int t, unsigned int rand_seed) {
  if (s >= t) {
    return s;
  }
  return s + rand_r(&rand_seed) % (t - s + 1);
}

double RandomDouble(const double s, const double t, unsigned int rand_seed) {
  return s + (t - s) / 16383.0 * (rand_r(&rand_seed) & 16383);
}

// Gaussian
double Gaussian(const double u, const double std, const double x) {
  return (1.0 / std::sqrt(2 * M_PI * std * std)) *
         std::exp(-(x - u) * (x - u) / (2 * std * std));
}

// 2-dimension Gaussian
double Gaussian2d(const double u1, const double u2, const double std1,
                  const double std2, const double x1, const double x2,
                  const double rho) {
  return (1.0 / 2 * M_PI * std1 * std2 * std::sqrt(1 - rho * rho)) *
         std::exp(-((x1 - u1) * (x1 - u1) / (std1 * std1) +
                    (x2 - u2) * (x2 - u2) / (std2 * std2) -
                    2 * rho * (x1 - u1) * (x2 - u2) / (std1 * std2)) /
                  (2 * (1 - rho * rho)));
}

// Sigmoid
double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

Eigen::Vector2d RotateVector2d(const Eigen::Vector2d &v_in,
                               const double theta) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  auto x = cos_theta * v_in.x() - sin_theta * v_in.y();
  auto y = sin_theta * v_in.x() + cos_theta * v_in.y();

  return {x, y};
}

double interps(const std::vector<double> &y, const std::vector<double> &x,
               const double &x_interp) {
  if (y.size() == 0) {
    return 0.0;
  } else if (y.size() == 1) {
    return y[0];
  } else {
    double s = x_interp;
    for (int j = 0; j < (int)y.size() - 1; j++) {
      if (s >= x[j] && s <= x[j + 1]) {
        return y[j] + (y[j + 1] - y[j]) / (x[j + 1] - x[j]) * (s - x[j]);
        break;
      } else if (j == (static_cast<int>(y.size()) - 2)) {
        return y[j + 1];
      }
    }
  }
  return 0.0;
}

void interpsVector(const std::vector<double> &y, const std::vector<double> &x,
                   const std::vector<double> &x_interps,
                   std::vector<double> &y_interps) {
  assert(y.size() == x.size());
  for (std::size_t i = 0; i < x_interps.size(); i++) {
    if (y.size() == 0) {
      y_interps.push_back(0.);
    } else if (y.size() == 1) {
      y_interps.push_back(y[0]);
    } else {
      double s = x_interps[i];
      for (int j = 0; j < (int)y.size() - 1; j++) {
        if (s >= x[j] && s <= x[j + 1]) {
          y_interps.push_back(y[j] + (y[j + 1] - y[j]) / (x[j + 1] - x[j]) *
                                         (s - x[j]));
          break;
        } else if (j == (static_cast<int>(y.size()) - 2)) {
          y_interps.push_back(y[j + 1]);
        }
      }
    }
  }
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

/**
 * @brief 计算当前位置对轨迹上的投影点
 * @param point1 投影的目标区间起始点
 * @param point2 投影的目标区间结束点
 * @param point 当前位置点
 * @return Pose2D 投影点
 */
Pose2D calc_projection_point(const Pose2D &point1, const Pose2D &point2,
                             const Pose2D &point) {
  Pose2D projection_point;
  // k为投影点在p1 p2点间的比例位置，∈(0,1)
  Vec2d base_vec(point2.x - point1.x, point2.y - point1.y),
      dir_vec(point.x - point1.x, point.y - point1.y);
  base_vec.Normalize();
  double k = dir_vec.InnerProd(base_vec) /
             std::hypot(point2.x - point1.x, point2.y - point1.y);
  k = std::min(std::max(0.0, k), 1.0);
  projection_point.x = point1.x + (point2.x - point1.x) * k;
  projection_point.y = point1.y + (point2.y - point1.y) * k;
  projection_point.theta = std::atan2(
      std::sin(point1.theta) * (1.0 - k) + std::sin(point2.theta) * k,
      std::cos(point1.theta) * (1.0 - k) + std::cos(point2.theta) * k);   
  return projection_point;
}

LineSegment2d tf2d(const Pose2D &local_frame, const LineSegment2d &line) {
  return LineSegment2d(tf2d(local_frame, line.start()),
                       tf2d(local_frame, line.end()));
}

Box2d tf2d(const Pose2D &local_frame, const Box2d &box) {
  return Box2d(tf2d(local_frame, box.center()),
               NormalizeAngle(box.heading() - local_frame.theta), box.length(),
               box.width());
}

Vec2d tf2d(const Pose2D &local_frame, const Vec2d &point) {
  double x_local, y_local;

  rotate2d(point.x() - local_frame.x, point.y() - local_frame.y,
           -local_frame.theta, 0.0, 0.0, x_local, y_local);
  return Vec2d(x_local, y_local);
}

Pose2D tf2d(const Pose2D &local_frame, const Pose2D &pose) {
  Pose2D Point_local;
  rotate2d(pose.x - local_frame.x, pose.y - local_frame.y, -local_frame.theta,
           0.0, 0.0, Point_local.x, Point_local.y);
  Point_local.theta = NormalizeAngle(pose.theta - local_frame.theta);
  return Point_local;
}

PathPoint tf2d(const Pose2D &local_frame, const PathPoint &pose) {
  PathPoint Point_local = pose;
  rotate2d(pose.x - local_frame.x, pose.y - local_frame.y, -local_frame.theta, 0.0, 0.0,
           Point_local.x, Point_local.y);
  Point_local.theta = NormalizeAngle(pose.theta - local_frame.theta);
  return Point_local;
}

LineSegment2d tf2d_inv(const Pose2D &local_frame,
                       const LineSegment2d &line_local) {
  return LineSegment2d(tf2d_inv(local_frame, line_local.start()),
                       tf2d_inv(local_frame, line_local.end()));
}

Box2d tf2d_inv(const Pose2D &local_frame, const Box2d &box_local) {
  return Box2d(tf2d_inv(local_frame, box_local.center()),
               NormalizeAngle(box_local.heading() + local_frame.theta),
               box_local.length(), box_local.width());
}

Vec2d tf2d_inv(const Pose2D &local_frame, const Vec2d &point_local) {
  double x_global, y_global;
  rotate2d(point_local.x(), point_local.y(), local_frame.theta, local_frame.x,
           local_frame.y, x_global, y_global);
  return Vec2d(x_global, y_global);
}

Pose2D tf2d_inv(const Pose2D &local_frame, const Pose2D &p_local) {
  Pose2D p_global;
  rotate2d(p_local.x, p_local.y, local_frame.theta, local_frame.x,
           local_frame.y, p_global.x, p_global.y);
  p_global.theta = NormalizeAngle(p_local.theta + local_frame.theta);
  return p_global;
}

PathPoint tf2d_inv_without_normalize_angle(const Pose2D &local_frame,
                                           const PathPoint &p_local) {
  PathPoint p_global = p_local;
  rotate2d(p_local.x, p_local.y, local_frame.theta, local_frame.x, local_frame.y,
           p_global.x, p_global.y);
  p_global.theta = p_local.theta + local_frame.theta;
  return p_global;
}

PathPoint tf2d_inv(const Pose2D &local_frame, const PathPoint &p_local) {
  PathPoint p_global = p_local;
  rotate2d(p_local.x, p_local.y, local_frame.theta, local_frame.x, local_frame.y,
           p_global.x, p_global.y);
  p_global.theta = NormalizeAngle(p_local.theta + local_frame.theta);
  return p_global;
}

inline void trans_rot_2d(double x, double y, double x_offset, double y_offset,
                         double &x_local, double &y_local) {
  x_local = x + x_offset;
  y_local = y + y_offset;
}

inline void rotate2d(double lx, double ly, double theta, double ox, double oy,
                     double &gx, double &gy) {
  double cos_a = cos(theta);
  double sin_a = sin(theta);
  gx = ox + lx * cos_a - ly * sin_a;
  gy = oy + lx * sin_a + ly * cos_a;
}

double variation_limiter(double current_value, double target_val, double limiter) {
  double result = target_val;
  if (double diff = target_val - current_value; std::abs(diff) > 1e-3) {
    diff = Clamp(diff, -limiter, limiter);
    result = current_value + diff;
  }
  return result;
}

} // namespace planning_math
} // namespace msquare

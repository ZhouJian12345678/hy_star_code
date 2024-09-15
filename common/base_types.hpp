#pragma once

#include <limits>
#include "common/math/box2d.hpp"
#include "common/math/line_segment2d.hpp"
#include "common/math/polygon2d.hpp"
#include "common/math/vec2d.hpp"
//#include "common/thread/thread_extension.h"
//#include "common/fault_code.h"

namespace msquare {

struct Point2D {
  double x = 0.0;
  double y = 0.0;

  Point2D() = default;
  Point2D(double xx, double yy) : x(xx), y(yy) {}
  void clear() {
    x = 0.0;
    y = 0.0;
  }
};

struct Point3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Point3D() = default;
  Point3D(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
  void clear() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }
};

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 0.0;

  Quaternion() = default;
  Quaternion(double xx, double yy, double zz, double ww)
      : x(xx), y(yy), z(zz), w(ww) {}
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

  Pose2D() = default;
  Pose2D(double xx, double yy, double tt) : x(xx), y(yy), theta(tt) {}
};

struct SLPoint {
  double s_;
  double l_;

  double s() const {
    return s_;
  }
  double l() const {
    return l_;
  }
  void set_s(const double s) {
    s_ = s;
  }
  void set_l(const double l) {
    l_ = l;
  }
};

struct SLBoundary {
  double start_s_;
  double end_s_;
  double start_l_;
  double end_l_;

  std::vector<SLPoint> boundary_point_;

  double start_s() const {
    return start_s_;
  }
  double end_s() const {
    return end_s_;
  }
  double start_l() const {
    return start_l_;
  }
  double end_l() const {
    return end_l_;
  }

  void set_start_s(const double s) {
    start_s_ = s;
  }
  void set_end_s(const double s) {
    end_s_ = s;
  }
  void set_start_l(const double l) {
    start_l_ = l;
  }
  void set_end_l(const double l) {
    end_l_ = l;
  }

  const std::vector<SLPoint> &boundary_point() const {
    return boundary_point_;
  }
  void add_boundary_point(const SLPoint &sl_point) {
    boundary_point_.emplace_back(sl_point);
  }

  void clear() {
    start_s_ = 0.0;
    end_s_ = 0.0;
    start_l_ = 0.0;
    end_l_ = 0.0;
    boundary_point_.clear();
  }
};

enum class DriveDirection {
  CCW = -2,
  BACKWARD = -1,
  NONE = 0,
  FORWARD = 1,
  CW = 2,
};

struct YawRotation {
  enum class Type {
    NONE = 0,
    ORIGIN = 1,               // vot
    REAR_AXLE_CENTER = 2,
    FL_WHEEL = 3,             // front left
    FR_WHEEL = 4,             // front right
    BL_WHEEL = 5,             // back left
    BR_WHEEL = 6,             // back right
    ANY_POINT = 7,
  };
  Type type = Type::NONE;

  // RFU coordinate
  Pose2D center;

  // target rotation delta theta [rad]
  double theta;

  // target rotation rate [rad/s]
  double rate;
};

struct PathPoint {
  // coordinates
  double x = std::numeric_limits<double>().infinity();
  double y = std::numeric_limits<double>().infinity();
  double z = std::numeric_limits<double>().infinity();

  double theta = 0.;

  // accumulated distance from beginning of the path
  double s = std::numeric_limits<double>().infinity();
  // direction on the x-y plane
  DriveDirection direction = DriveDirection::NONE;
  // sign along with direction of cw & ccw
  YawRotation rotation;

  // curvature on the x-y plane
  double kappa = 0.;
  // curvature radius on the x-y plane
  double rho = std::numeric_limits<double>().infinity();

  // derivative of kappa w.r.t s.
  double dkappa = 0.;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa = 0.;

  PathPoint(double x_ = 0, double y_ = 0, double z_ = 0, double theta_ = 0,
            double s_ = 0, DriveDirection dir = DriveDirection::NONE)
      : x(x_), y(y_), z(z_), theta(theta_), s(s_) , direction(dir) {}
};

struct TrajectoryPoint {
  // path point
  PathPoint path_point;

  // linear velocity
  double v = 0.;  // in [m/s]
  // linear acceleration
  double a = 0.;
  // relative time from beginning of the trajectory
  double relative_time = 0.;

  double steer = 0.;

  TrajectoryPoint(double x_ = 0, double y_ = 0, double z_ = 0,
                  double theta_ = 0, double s_ = 0, double v_ = 0,
                  double a_ = 0, double relative_time_ = 0, double steer_ = 0)
      : path_point(PathPoint(x_, y_, z_, theta_, s_)),
        v(v_),
        a(a_),
        relative_time(relative_time_),
        steer(steer_){};
  TrajectoryPoint &operator=(const Pose2D &pose) {
    path_point.x = pose.x;
    path_point.y = pose.y;
    path_point.theta = pose.theta;
    return *this;
  }
};

enum class GearState : int {
  NONE = 0,
  PARK = 1,
  REVERSE = 2,
  NEUTRAL = 3,
  DRIVE = 4,
};

enum class RotateDirection {
  UNKOWN = 0,
  CW = 2,
  CCW = 1
};

enum class RotationState : int {
  STANDBY = 0,
  ROTATING = 1,
  ROTATED = 2,
  ROTATE_FAILED = 3
};

enum class RotationAvailable : int {
  NO_AVAILABLE = 0,
  AVAILABLE = 1
};

enum class ChassisType {
  UNKOWN = 0,
  ACC_TYPE = 1,
  VS_TYPE = 2
};

enum class DrivingMode {
  AUTO = 0,
  MANUAL,
};

enum class ObjectType {
  NOT_KNOW = 0,
  PEDESTRIAN = 1,
  OFO = 2,
  COUPE = 3,  // 轿跑
  TRANSPORT_TRUNK = 4,
  BUS = 5,
  ENGINEER_TRUCK = 6,
  TRICYCLE = 7,
  CONE_BUCKET = 8,
  STOP_LINE = 9,
  GATE = 10,  // 消防门
  FREESPACE = 11
};

enum class BrakeMode {  // 刹车模式
  NO_BRAKE = 0,
  EMERGENCY_BRAKE = 1,
  COMFORT_BRAKE = 2,
  STOP_ROTATION = 3,
};

enum class ParkingSlotType : int {  // 车位类型
  UNKNOW = 0,                       // 未知
  PERPENDICULAR = 1,                // 垂直
  PARALLEL = 2,                     // 平行
  OBLIQUE = 3,                      // 斜向
};

enum class ParkingSlotScene {   // 车位场景
  UNKNOW = 0,                   // 未知
  NORMAL,                       // 常规
  CUT_ROAD,                     // 断头路
  NARROW_ROAD,                  // 窄路
  SLIM_PARA,                    // 平行小侧方
  OTHER,                        // 其他
};

enum class ParkOutType {
  // Sequence indicates priority
  UNDEFINED = 0,
  FRONT_LEFT = 1,
  FRONT_RIGHT = 2,
};

enum class GroundLineType : short {  // 地面线类型
  UNKNOWN = 0,                       // 未知
  WALL = 1,                          // 墙
  PILLAR = 2,                        // 柱子
  FENCE = 3,                         // 栅栏
  STEP = 4,                          // 台阶
  SPECIAL = 5,                       // 特殊

  // uss类型
  USS_UNKNOWN = 100,
  USS_WALL = 101,
  USS_PILLAR = 102,
  USS_FENCE = 103,
  USS_STEP = 104,
  USS_SPECIAL = 105,
};

enum class ObsType : short {  // 被point、box、polygon共用
  UNKNOWN = 0,                // 未知
  NON_GROUNDING_POINT = 1,    // 非接地点
  ROAD_SURFACE_POINT = 2,     // 路面点
  STATIC_BORDER = 3,          // 静态边界
  DYNAMIC_BORDER = 4,         // 动态边界
  PILLAR = 5,                 // 立柱
  CAR = 6,                    // 汽车
  WHEEL_STOP = 7,             // 轮挡
  CONE_BUCKET = 8,            // 锥桶
  PEDESTRIAN = 9,             // 行人
  OTHER = 8,                  // 其他

};

enum class SensorType : short {
  UNKNOWN = 0,               // 未知
  FUSION_SENSOR = 1,         // 前融合
  VISION_SENSOR = 2,         // 纯视觉
};

struct PointObstacle {
  uint64_t id;
  Point3D position;
  double v = 0.;
  ObsType type;
  SensorType sensor;
};

struct LineObstacle {
  uint64_t id;
  std::vector<Point3D> pts{};
  double v = 0.;
  GroundLineType type = GroundLineType::UNKNOWN;
};

struct LineSegmentObstacle {
  // int id = -1;
  std::vector<planning_math::LineSegment2d> line_segs;
  std::vector<planning_math::Vec2d> points;

  GroundLineType type = GroundLineType::UNKNOWN;
  double v = 0.;
};

struct BoxObstacle {
  BoxObstacle(const planning_math::Vec2d &center, const double heading,
              const double length, const double width)
      : box(planning_math::Box2d(center, heading, length, width)) {}
  BoxObstacle() {}
  uint64_t id;
  planning_math::Box2d box;
  bool is_static = false;
  Point3D velocity;
  double linear_velocity;
  ObsType type;
};

struct PolygonObstacle {
  PolygonObstacle(std::vector<planning_math::Vec2d> &points)
      : polygon(planning_math::Polygon2d(points)) {}
  PolygonObstacle() {}
  int id = -1;
  planning_math::Polygon2d polygon;
  double v = 0.;
  ObsType type;
};

struct InputObstacleInfo {
  std::vector<PointObstacle> point;
  std::vector<LineObstacle> line;
  std::vector<BoxObstacle> box;
  std::vector<PolygonObstacle> polygon;
  void clear() {
    point.clear();
    line.clear();
    box.clear();
    polygon.clear();
  }
};

struct GroundLineInfo {
  std::vector<PointObstacle> point;
  std::vector<LineObstacle> line;
  void clear() {
    point.clear();
    line.clear();
  }
};

struct WheelStop {
  bool available = false;
  Point2D point1;
  Point2D point2;
  double wheel_stop_depth = 0.;
};

struct CollisionInfo {
  bool is_collision = false;
  enum class Type {
    NONE = 0,
    ROLLING_OVER = 1,
    BODY_COLLISION = 2,
    BOTH = 3,
  };
  Type type = Type::NONE;

  void set_collision(const Type _type) {
    switch (_type) {
      case Type::NONE:
        is_collision = false;
        type = Type::NONE;
        break;
      case Type::ROLLING_OVER:
        is_collision = true;
        switch (type) {
          case Type::NONE:
            type = Type::ROLLING_OVER;
            break;
          case Type::BODY_COLLISION:
            type = Type::BOTH;
            break;
          default:
            break;
        }
        break;
      case Type::BODY_COLLISION:
        is_collision = true;
        switch (type) {
          case Type::NONE:
            type = Type::BODY_COLLISION;
            break;
          case Type::ROLLING_OVER:
            type = Type::BOTH;
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }
  }
};

// 泊车场景信息
struct ParkingLotDetectionInfo {
  bool is_available = false;     // 是否可用
  bool is_empty = true;
  ParkingSlotScene scene = ParkingSlotScene::UNKNOW;  // 场景类型
  ParkingSlotType type = ParkingSlotType::UNKNOW;
  std::vector<Point2D> corners{};
  double road_boundary = 0.;
  WheelStop wheel_stop;
};

// struct TransCoorBase {  // only for conver_data.h
//   double trans_coor_x = 0.;
//   double trans_coor_y = 0.;
//   double trans_theta = 0.;
// };

// 泊车轨迹信息
struct ParkingTrajectoryData {
  std::vector<TrajectoryPoint> trajectory_point{};    // 泊车跟踪轨迹点
  std::vector<TrajectoryPoint> goal_point{};          // 泊车关键点
  TrajectoryPoint target_point;                       // 目标
  std::vector<planning_math::Vec2d> tpoints_debug{};  // T型区域角点 sim debug only
  BrakeMode brake_mode = BrakeMode::NO_BRAKE;         // 刹车模式
  GearState target_gear = GearState::NONE;
};

// 控制状态信息
struct ControlState {
  ThreadState state = ThreadState::Stop;
  PncCode error_code = PncCode::OK;
  bool control_response_steering_zeroing = false;  // 方向盘归零标志, 当前是否使用了？
  bool handshake_request = false;     // 用于反馈e4 enable状态
};

struct ParkingStatus {
  enum Status : int {
    UNKNOWN = 0,
    STAND_BY,
    PREPLAN,
    WORKING,
    FINISHED,
    FAILED,
  };
  Status type = UNKNOWN;

  double counting_time_stamp = 0.;
};

enum class ActionType {
  NONE = -1,
  STOP = 0,
  START = 1,
};

struct DecisionParkData {
  int available;
  int charge_property;
  uint64_t track_id;
  int32_t empty_votes;
  int park_scene_type;  // 0:unknown, 1:断头路，2：窄通道，3：小侧方，4：预留
  std::vector<Point3D> points_fusion;
  std::vector<float> points_fusion_confidence;
  std::vector<uint8_t> points_fusion_type;
  std::vector<Point3D> local_points_fusion;
  std::vector<float> local_points_fusion_confidence;
  std::vector<uint8_t> local_points_fusion_type;
  std::vector<Point2D> points_fusion_in_image;
  std::vector<float> points_fusion_in_image_confidence;
  std::vector<uint8_t> points_fusion_in_image_type;
  std::vector<Point3D> local_wheel_stop_points;
};

struct DecisionParkResult {
  int available;
	uint16_t status_code;
  std::vector<DecisionParkData> parking_slot_data;
  uint64_t ego_parking_slot;
  std::vector<std::string> reserved_info;
  bool is_planning_processing;
  int64_t locked_target_parking_slot_id;
};

}  // namespace msquare

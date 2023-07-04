// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OCC_GRIDMAP__RIGID_TRANSFORM_HPP_
#define OCC_GRIDMAP__RIGID_TRANSFORM_HPP_

#include <Eigen/Dense>
#include <Eigen/Core>

#include <string>

#include "proto/transform.pb.h"

namespace transform
{

template<typename FloatType>
inline FloatType getYaw(const Eigen::Quaternion<FloatType> & q)
{
  FloatType yaw;
  FloatType sqw;
  FloatType sqx;
  FloatType sqy;
  FloatType sqz;
  sqx = q.x() * q.x();
  sqy = q.y() * q.y();
  sqz = q.z() * q.z();
  sqw = q.w() * q.w();
  FloatType sarg = -2 * (q.x() * q.z() - q.w() * q.y()) / (sqx + sqy + sqz + sqw);
  if (sarg <= -0.99999) {
    yaw = -2 * atan2(q.y(), q.x());
  } else if (sarg >= 0.99999) {
    yaw = 2 * atan2(q.y(), q.x());
  } else {
    yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
  }
  return yaw;
}

template<typename FloatType>
FloatType NormalizeAngleDifference(FloatType difference)
{
  const FloatType kPi = FloatType(M_PI);
  while (difference > kPi) {difference -= 2. * kPi;}
  while (difference < -kPi) {difference += 2. * kPi;}
  return difference;
}

template<typename FloatType>
class Rigid2
{
public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<FloatType>;

  Rigid2()
  : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
  Rigid2(const Vector & translation, const Rotation2D & rotation)
  : translation_(translation), rotation_(rotation) {}
  Rigid2(const Vector & translation, const FloatType angle)
  : translation_(translation), rotation_(angle) {}
  void operator=(const Rigid2<FloatType> & D)
  {
    translation_ = D.translation();
    rotation_ = D.rotation();
  }
  static Rigid2 Rotation(const FloatType angle)
  {
    return Rigid2(Vector::Zero(), angle);
  }

  static Rigid2 Rotation(const Rotation2D & rotation)
  {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Translation(const Vector & vector)
  {
    return Rigid2(vector, Rotation2D::Identity());
  }

  static Rigid2<FloatType> Identity() {return Rigid2<FloatType>();}

  template<typename OtherType>
  Rigid2<OtherType> cast() const
  {
    return Rigid2<OtherType>(
      translation_.template cast<OtherType>(),
      rotation_.template cast<OtherType>());
  }

  const Vector & translation() const {return translation_;}

  Rotation2D rotation() const {return rotation_;}

  double normalized_angle() const
  {
    return NormalizeAngleDifference(rotation().angle());
  }

  Rigid2 inverse() const
  {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation);
    return Rigid2(translation, rotation);
  }

  std::string DebugString() const
  {
    std::string debug_string;
    debug_string = "{ t: [" + std::to_string(translation().x()) + " , " +
      std::to_string(translation().y()) + " ], r: [ " +
      std::to_string(rotation().angle()) + " ]";
    return debug_string;
  }

private:
  Vector translation_;
  Rotation2D rotation_;
};  // class Rigid2D

template<typename FloatType>
Rigid2<FloatType> operator*(
  const Rigid2<FloatType> & lhs,
  const Rigid2<FloatType> & rhs)
{
  return Rigid2<FloatType>(
    lhs.rotation() * rhs.translation() + lhs.translation(),
    lhs.rotation() * rhs.rotation());
}

template<typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
  const Rigid2<FloatType> & rigid,
  const typename Rigid2<FloatType>::Vector & point)
{
  return rigid.rotation() * point + rigid.translation();
}

using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;

template<typename FloatType>
class Rigid3
{
public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  Rigid3()
  : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector & translation, const Quaternion & rotation)
  : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector & translation, const AngleAxis & rotation)
  : translation_(translation), rotation_(rotation) {}
  void operator=(const Rigid3<FloatType> & D)
  {
    translation_.x() = D.translation().x();
    translation_.y() = D.translation().y();
    translation_.z() = D.translation().z();
    rotation_.w() = D.rotation().w();
    rotation_.x() = D.rotation().x();
    rotation_.y() = D.rotation().y();
    rotation_.z() = D.rotation().z();
  }
  static Rigid3 Rotation(const AngleAxis & angle_axis)
  {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion & rotation)
  {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector & vector)
  {
    return Rigid3(vector, Quaternion::Identity());
  }

  static Rigid3<FloatType> Identity() {return Rigid3<FloatType>();}

  template<typename OtherType>
  Rigid3<OtherType> cast() const
  {
    return Rigid3<OtherType>(
      translation_.template cast<OtherType>(),
      rotation_.template cast<OtherType>());
  }

  const Vector & translation() const {return translation_;}
  const Quaternion & rotation() const {return rotation_;}

  Rigid3 inverse() const
  {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  std::string DebugString() const
  {
    std::string debug_string;
    debug_string = "{ t: [" + std::to_string(translation().x()) + " , " +
      std::to_string(translation().y()) + " , " +
      std::to_string(translation().z()) + " ], q: [ " +
      std::to_string(rotation().w()) + " , " +
      std::to_string(rotation().x()) + " , " +
      std::to_string(rotation().y()) + " , " +
      std::to_string(rotation().z()) + " ]";
    return debug_string;
  }

private:
  Vector translation_;
  Quaternion rotation_;
};  // class Rigid3D

template<typename FloatType>
FloatType GetAngle(const Rigid3<FloatType> & transform)
{
  return FloatType(2) * std::atan2(
    transform.rotation().vec().norm(),
    std::abs(transform.rotation().w()));
}

template<typename FloatType>
Rigid3<FloatType> operator*(
  const Rigid3<FloatType> & lhs,
  const Rigid3<FloatType> & rhs)
{
  return Rigid3<FloatType>(
    lhs.rotation() * rhs.translation() + lhs.translation(),
    (lhs.rotation() * rhs.rotation()).normalized());
}

template<typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
  const Rigid3<FloatType> & rigid,
  const typename Rigid3<FloatType>::Vector & point)
{
  return rigid.rotation() * point + rigid.translation();
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

template<typename FloatType>
static inline Rigid2<FloatType> ToRigid2(const Rigid3<FloatType> & rigid3)
{
  Eigen::Matrix<FloatType, 2, 1> translation(
    rigid3.translation()[0], rigid3.translation()[1]);
  FloatType yaw = getYaw(rigid3.rotation());
  return Rigid2<FloatType>(translation, yaw);
}

template<typename T>
Rigid3<T> ToRigid3(const Rigid2<T> & transform)
{
  return Rigid3<T>(
    {transform.translation().x(), transform.translation().y(), T(0)},
    Eigen::AngleAxis<T>(
      transform.rotation().angle(),
      Eigen::Matrix<T, 3, 1>::UnitZ()));
}

struct TimeRigid3d
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Rigid3d pose;
  int64_t timestamp;
};

static inline
TimeRigid3d Interpolate(
  const TimeRigid3d & start,
  const TimeRigid3d & end,
  const int64_t time)
{
  const double duration = static_cast<double>(end.timestamp - start.timestamp);
  const double interpolate_time = static_cast<double>(time - start.timestamp);
  const double factor = interpolate_time / duration;
  const Eigen::Vector3d origin =
    start.pose.translation() + (end.pose.translation() - start.pose.translation()) * factor;
  const Eigen::Quaterniond rotation =
    Eigen::Quaterniond(start.pose.rotation())
    .slerp(factor, Eigen::Quaterniond(end.pose.rotation()));   // 四元数球面插值
  return TimeRigid3d{Rigid3d(origin, rotation), time};
}

// Conversions between Eigen and proto.
Rigid2d ToRigid2(const proto::Rigid2d & transform);
Eigen::Vector2d ToEigen(const proto::Vector2d & vector);
Eigen::Vector3d ToEigen(const proto::Vector3d & vector);
Eigen::Quaterniond ToEigen(const proto::Quaterniond & quaternion);
proto::Rigid2d ToProto(const Rigid2d & transform);
proto::Rigid3d ToProto(const Rigid3d & rigid);
Rigid3d ToRigid3(const proto::Rigid3d & rigid);
proto::Vector2d ToProto(const Eigen::Vector2d & vector);
proto::Vector3d ToProto(const Eigen::Vector3d & vector);
proto::Quaterniond ToProto(const Eigen::Quaterniond & quaternion);

struct Eigentf
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  transform::Rigid3d imu2baselink;
  transform::Rigid3d laser2baselink;
  transform::Rigid2d laser2imu;
  transform::Rigid2d odom2imu;
  Eigentf()
  : imu2baselink(transform::Rigid3d::Identity()),
    laser2baselink(transform::Rigid3d::Identity())
  {}
};

}  // namespace transform

#endif  // OCC_GRIDMAP__RIGID_TRANSFORM_HPP_

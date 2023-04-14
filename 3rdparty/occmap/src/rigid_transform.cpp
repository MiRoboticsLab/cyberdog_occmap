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

#include "occ_gridmap/rigid_transform.hpp"

namespace transform
{

Rigid2d ToRigid2(const proto::Rigid2d & transform)
{
  return Rigid2d(
    {transform.translation().x(), transform.translation().y()},
    transform.rotation());
}

Eigen::Vector2d ToEigen(const proto::Vector2d & vector)
{
  return Eigen::Vector2d(vector.x(), vector.y());
}


Eigen::Vector3d ToEigen(const proto::Vector3d & vector)
{
  return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

Eigen::Quaterniond ToEigen(const proto::Quaterniond & quaternion)
{
  return Eigen::Quaterniond(
    quaternion.w(), quaternion.x(), quaternion.y(),
    quaternion.z());
}

proto::Rigid2d ToProto(const transform::Rigid2d & transform)
{
  proto::Rigid2d proto;
  proto.mutable_translation()->set_x(transform.translation().x());
  proto.mutable_translation()->set_y(transform.translation().y());
  proto.set_rotation(transform.rotation().angle());
  return proto;
}


proto::Rigid3d ToProto(const transform::Rigid3d & rigid)
{
  proto::Rigid3d proto;
  *proto.mutable_translation() = ToProto(rigid.translation());
  *proto.mutable_rotation() = ToProto(rigid.rotation());
  return proto;
}

transform::Rigid3d ToRigid3(const proto::Rigid3d & rigid)
{
  return transform::Rigid3d(
    ToEigen(rigid.translation()),
    ToEigen(rigid.rotation()));
}

proto::Vector2d ToProto(const Eigen::Vector2d & vector)
{
  proto::Vector2d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  return proto;
}

proto::Vector3d ToProto(const Eigen::Vector3d & vector)
{
  proto::Vector3d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  return proto;
}

proto::Quaterniond ToProto(const Eigen::Quaterniond & quaternion)
{
  proto::Quaterniond proto;
  proto.set_w(quaternion.w());
  proto.set_x(quaternion.x());
  proto.set_y(quaternion.y());
  proto.set_z(quaternion.z());
  return proto;
}


}  // namespace transform

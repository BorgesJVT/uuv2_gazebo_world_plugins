// Copyright (c) 2016 The UUV Simulator Authors. All rights reserved.
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

/// \file UnderwaterCurrentPlugin.hpp
/// \brief Plugin for the underwater world

#ifndef UUV2_GAZEBO_WORLD_PLUGINS__UNDERWATERCURRENTPLUGIN_HPP_
#define UUV2_GAZEBO_WORLD_PLUGINS__UNDERWATERCURRENTPLUGIN_HPP_

#include <map>
#include <cmath>
#include <string>

#include "gazebo/gazebo.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "sdf/sdf.hh"

#include "uuv2_gazebo_world_plugins/GaussMarkovProcess.hpp"

namespace uuv2_gazebo_world_plugins
{

class GZ_PLUGIN_VISIBLE UnderwaterCurrentPlugin : public gazebo::WorldPlugin
{
  /// \brief Constructor

public:
  UnderwaterCurrentPlugin();

  /// Destructor

public:
  virtual ~UnderwaterCurrentPlugin();

  // Documentation inherited.

public:
  void Load(
    gazebo::physics::WorldPtr _world,
    sdf::ElementPtr _sdf);

  /// \brief Update the simulation state.

public:
  void Update();

  /// \brief Publish current velocity and the pose of its frame

protected:
  void PublishCurrentVelocity();

  /// \brief Update event

protected:
  gazebo::event::ConnectionPtr updateConnection;

  /// \brief Pointer to world

protected:
  gazebo::physics::WorldPtr world;

  /// \brief Pointer to sdf

protected:
  sdf::ElementPtr sdf;

  /// \brief True if the sea surface is present

protected:
  bool hasSurface;

  /// \brief Pointer to a node for communication

protected:
  gazebo::transport::NodePtr node;

  /// \brief Map of publishers

protected:
  std::map<std::string, gazebo::transport::PublisherPtr> publishers;

  /// \brief Current velocity topic

protected:
  std::string currentVelocityTopic;

  /// \brief Namespace for topics and services

protected:
  std::string ns;

  /// \brief Gauss-Markov process instance for the current velocity

protected:
  GaussMarkovProcess currentVelModel;

  /// \brief Gauss-Markov process instance for horizontal angle model

protected:
  GaussMarkovProcess currentHorzAngleModel;

  /// \brief Gauss-Markov process instance for vertical angle model

protected:
  GaussMarkovProcess currentVertAngleModel;

  /// \brief Last update time stamp

protected:
  gazebo::common::Time lastUpdate;

  /// \brief Current linear velocity vector

protected:
  ignition::math::Vector3d currentVelocity;
};

}  // namespace uuv2_gazebo_world_plugins
#endif  // UUV2_GAZEBO_WORLD_PLUGINS__UNDERWATERCURRENTPLUGIN_HPP_

// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
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

/// \file gauss_markov_process.hpp
/// \brief Implementation of a Gauss-Markov process model

#ifndef UUV2_GAZEBO_WORLD_PLUGINS__GAUSSMARKOVPROCESS_HPP_
#define UUV2_GAZEBO_WORLD_PLUGINS__GAUSSMARKOVPROCESS_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace uuv2_gazebo_world_plugins
{

class GZ_PLUGIN_VISIBLE GaussMarkovProcess
{ 
  /// \brief Constructor
  public: GaussMarkovProcess();

  /// \brief Resets the process parameters
  void Reset();

  /// \brief Sets all the necessary parameters for the computation
  /// \param _mean Mean value
  /// \param _min Minimum limit
  /// \param _max Maximum limit
  /// \param _mu Process constant
  /// \param _noise Amplitude for the Gaussian white noise
  /// \return True, if all parameters were valid
  public: bool SetModel(double _mean, double _min, double _max,
        double _mu = 0, double _noise = 0);

  /// \brief Set mean process value
  /// \param[in] _mean New mean value
  /// \return true if value inside the limit range
  public: bool SetMean(double _mean);

  /// \brief Process variable
  public: double var;

  /// \brief Mean process value
  public: double mean;

  /// \brief Minimum limit for the process variable
  public: double min;

  /// \brief Maximum limit for the process variable
  public: double max;

  /// \brief Process constant, if zero, process becomes a random walk
  public: double mu;

  /// \brief Gaussian white noise amplitude
  public: double noiseAmp;

  /// \brief Timestamp for the last update
  public: double lastUpdate;

  /// \brief Update function for a new time stamp
  /// \param[in] _time Current time stamp
  public: double Update(double _time);

  /// \brief Print current model paramters
  public: void Print();

};

}  // namespace uuv2_gazebo_world_plugins
#endif  // UUV2_GAZEBO_WORLD_PLUGINS__GAUSSMARKOVPROCESS_HPP_

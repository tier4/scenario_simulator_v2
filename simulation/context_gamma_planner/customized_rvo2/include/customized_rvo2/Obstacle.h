/*
 * Obstacle.h
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 *
 * modified by Kotaro Yoshimoto <kotaro.yoshimoto@tier4.jp>
 */

#ifndef CUSTOMIZED_RVO2__OBSTACLE_H_
#define CUSTOMIZED_RVO2__OBSTACLE_H_

/**
 * \file       Obstacle.h
 * \brief      Contains the Obstacle class.
 */

#include <memory>

#include "Math.h"

namespace RVO
{
/**
 * \brief      Defines static obstacles in the simulation.
 */
class Obstacle
{
public:
  using SharedPtr = std::shared_ptr<Obstacle>;
  /**
   * \brief      Constructs a static obstacle instance.
   */
  Obstacle();
  Vector2 point_;
  Obstacle::SharedPtr next_obstacle_ = nullptr;
  Obstacle::SharedPtr prev_obstacle_ = nullptr;
  Vector2 unit_dir_;
  bool is_convex_;
  size_t id_;

  friend class AgentKdTree;
  friend class ObstacleKdTree;
};
}  // namespace RVO

#endif  // CUSTOMIZED_RVO2__OBSTACLE_H_

/*
* Math.cpp
* copied and modified from Vector2.h in RVO2 Library
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

#include "customized_rvo2/Math.h"

#include <algorithm>
#include <vector>

bool RVO::linearProgram1(
  const std::vector<Line> & lines, size_t line_no, float radius, const Vector2 & opt_velocity,
  bool direction_opt, Vector2 & result)
{
  const float dot_product = lines[line_no].point * lines[line_no].direction;
  const float discriminant = sqr(dot_product) + sqr(radius) - absSq(lines[line_no].point);

  if (discriminant < 0.0f) {
    /* Max speed circle fully invalidates line line_no. */
    return false;
  }

  const float sqrt_discriminant = std::sqrt(discriminant);
  float t_left = -dot_product - sqrt_discriminant;
  float t_right = -dot_product + sqrt_discriminant;

  for (size_t i = 0; i < line_no; ++i) {
    const float denominator = det(lines[line_no].direction, lines[i].direction);
    const float numerator = det(lines[i].direction, lines[line_no].point - lines[i].point);

    if (std::fabs(denominator) <= RVO_EPSILON) {
      /* Lines line_no and i are (almost) parallel. */
      if (numerator < 0.0f) {
        return false;
      } else {
        continue;
      }
    }

    const float t = numerator / denominator;

    if (denominator >= 0.0f) {
      /* Line i bounds line line_no on the right. */
      t_right = std::min(t_right, t);
    } else {
      /* Line i bounds line line_no on the left. */
      t_left = std::max(t_left, t);
    }
    if (t_left > t_right) {
      return false;
    }
  }

  if (direction_opt) {
    /* Optimize direction. */
    if (opt_velocity * lines[line_no].direction > 0.0f) {
      /* Take right extreme. */
      result = lines[line_no].point + t_right * lines[line_no].direction;
    } else {
      /* Take left extreme. */
      result = lines[line_no].point + t_left * lines[line_no].direction;
    }
  } else {
    /* Optimize closest point. */
    const float t = lines[line_no].direction * (opt_velocity - lines[line_no].point);

    if (t < t_left) {
      result = lines[line_no].point + t_left * lines[line_no].direction;
    } else if (t > t_right) {
      result = lines[line_no].point + t_right * lines[line_no].direction;
    } else {
      result = lines[line_no].point + t * lines[line_no].direction;
    }
  }

  return true;
}
size_t RVO::linearProgram2(
  const std::vector<Line> & lines, float radius, const Vector2 & opt_velocity, bool direction_opt,
  Vector2 & result)
{
  if (direction_opt) {
    /*
     * Optimize direction. Note that the optimization velocity is of unit
     * length in this case.
     */
    result = opt_velocity * radius;
  } else if (absSq(opt_velocity) > sqr(radius)) {
    /* Optimize closest point and outside circle. */
    result = normalize(opt_velocity) * radius;
  } else {
    /* Optimize closest point and inside circle. */
    result = opt_velocity;
  }

  for (size_t i = 0; i < lines.size(); ++i) {
    if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
      /* Result does not satisfy constraint i. Compute new optimal result. */
      const Vector2 temp_result = result;

      if (!linearProgram1(lines, i, radius, opt_velocity, direction_opt, result)) {
        result = temp_result;
        return i;
      }
    }
  }

  return lines.size();
}
void RVO::linearProgram3(
  const std::vector<Line> & lines, size_t num_obst_lines, size_t begin_line, float radius,
  Vector2 & result)
{
  float distance = 0.0f;

  for (size_t i = begin_line; i < lines.size(); ++i) {
    if (det(lines[i].direction, lines[i].point - result) > distance) {
      /* Result does not satisfy constraint of line i. */
      std::vector<Line> proj_lines(
        lines.begin(), lines.begin() + static_cast<ptrdiff_t>(num_obst_lines));

      for (size_t j = num_obst_lines; j < i; ++j) {
        Line line;

        float determinant = det(lines[i].direction, lines[j].direction);

        if (std::fabs(determinant) <= RVO_EPSILON) {
          /* Line i and line j are parallel. */
          if (lines[i].direction * lines[j].direction > 0.0f) {
            /* Line i and line j point in the same direction. */
            continue;
          } else {
            /* Line i and line j point in opposite direction. */
            line.point = 0.5f * (lines[i].point + lines[j].point);
          }
        } else {
          line.point = lines[i].point +
                       (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) *
                         lines[i].direction;
        }

        line.direction = normalize(lines[j].direction - lines[i].direction);
        proj_lines.push_back(line);
      }

      const Vector2 temp_result = result;

      if (
        linearProgram2(
          proj_lines, radius, Vector2(-lines[i].direction.y(), lines[i].direction.x()), true,
          result) < proj_lines.size()) {
        /* This should in principle not happen.  The result is by definition
         * already in the feasible region of this linear program. If it fails,
         * it is due to small floating point error, and the current result is
         * kept.
         */
        result = temp_result;
      }

      distance = det(lines[i].direction, lines[i].point - result);
    }
  }
}

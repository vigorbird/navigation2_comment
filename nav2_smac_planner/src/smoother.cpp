// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

#include <chrono>
#include <memory>
#include <vector>

#include "angles/angles.h"

#include "tf2/utils.hpp"

#include "nav2_smac_planner/smoother.hpp"

namespace nav2_smac_planner
{
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT

Smoother::Smoother(const SmootherParams & params)
{
  tolerance_ = params.tolerance_;
  max_its_ = params.max_its_;
  data_w_ = params.w_data_;
  smooth_w_ = params.w_smooth_;
  is_holonomic_ = params.holonomic_;
  do_refinement_ = params.do_refinement_;
  refinement_num_ = params.refinement_num_;
}

void Smoother::initialize(const double & min_turning_radius)
{
  min_turning_rad_ = min_turning_radius;
  state_space_ = std::make_unique<ompl::base::DubinsStateSpace>(min_turning_rad_);
}

bool Smoother::smooth(
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const double & max_time)
{
  // by-pass path orientations approximation when skipping smac smoother
  if (max_its_ == 0) {
    return false;
  }

  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time;
  bool success = true, reversing_segment;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;
  //1.分段
  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);//将机器人的path找出尖点并分段！

  //2.遍历上面的每一段
  for (unsigned int i = 0; i != path_segments.size(); i++) {
    if (path_segments[i].end - path_segments[i].start > 10) {
      // Populate path segment
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      // Make sure we're still able to smooth with time remaining
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time - duration_cast<duration<double>>(now - start).count();
      refinement_ctr_ = 0;

      // Smooth path segment naively
      const geometry_msgs::msg::Pose start_pose = curr_path_segment.poses.front().pose;
      const geometry_msgs::msg::Pose goal_pose = curr_path_segment.poses.back().pose;
      //2.1 核心实现函数！！！！！
      bool local_success = smoothImpl(curr_path_segment, //输入 输出变量
                                      reversing_segment, //输出变量
                                      costmap, //输入变量
                                      time_remaining);//输入变量
      success = success && local_success;

      // Enforce boundary conditions
      if (!is_holonomic_ && local_success) {
        //这两个函数的目的是：
        enforceStartBoundaryConditions(start_pose, curr_path_segment, costmap, reversing_segment);
        enforceEndBoundaryConditions(goal_pose, curr_path_segment, costmap, reversing_segment);
      }

      // Assemble the path changes to the main path
      std::copy(
        curr_path_segment.poses.begin(),
        curr_path_segment.poses.end(),
        path.poses.begin() + path_segments[i].start);
    }
  }

  return success;
}

//
bool Smoother::smoothImpl(
  nav_msgs::msg::Path & path,
  bool & reversing_segment,
  const nav2_costmap_2d::Costmap2D * costmap,
  const double & max_time)
{
  steady_clock::time_point a = steady_clock::now();
  rclcpp::Duration max_dur = rclcpp::Duration::from_seconds(max_time);

  int its = 0;
  double change = tolerance_;
  const unsigned int & path_size = path.poses.size();
  double x_i, y_i, y_m1, y_ip1, y_i_org;
  unsigned int mx, my;

  nav_msgs::msg::Path new_path = path;
  nav_msgs::msg::Path last_path = path;

  //如果你每次平滑迭代变化的仍旧是很大，那么需要再平滑。
  while (change >= tolerance_) {
    its += 1;
    change = 0.0;

    // Make sure the smoothing function will converge
    //1.如果迭代次数太多了也会失败！！！！
    if (its >= max_its_) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "Number of iterations has exceeded limit of %i.", max_its_);
      path = last_path;
      updateApproximatePathOrientations(path, reversing_segment);//保证运动轨迹和path上的位姿保持一致！！！
      return false;
    }

    // Make sure still have time left to process
    //2.如果计算时间太久也会失败！！！！
    steady_clock::time_point b = steady_clock::now();
    rclcpp::Duration timespan(duration_cast<duration<double>>(b - a));
    if (timespan > max_dur) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "Smoothing time exceeded allowed duration of %0.2f.", max_time);
      path = last_path;
      updateApproximatePathOrientations(path, reversing_segment);
      return false;
    }

    //
    for (unsigned int i = 1; i != path_size - 1; i++) {

      //这个平滑做的太简单了！！！！可以说很粗糙需要被替换！！！！
      // 这段代码的作用是对路径中的每一个中间点（不包括首尾点）进行平滑处理
      // 具体来说，对每个点的x和y坐标分别进行如下操作：
      // j=0时处理x坐标，j=1时处理y坐标
      for (unsigned int j = 0; j != 2; j++) {
        // x_i是原始路径path中第i个点的第j维（x或y）坐标
        x_i = getFieldByDim(path.poses[i], j);

        // y_i是当前平滑路径new_path中第i个点的第j维坐标
        y_i = getFieldByDim(new_path.poses[i], j);
        // y_m1是new_path中前一个点的第j维坐标
        y_m1 = getFieldByDim(new_path.poses[i - 1], j);
        // y_ip1是new_path中后一个点的第j维坐标
        y_ip1 = getFieldByDim(new_path.poses[i + 1], j);
        // 保存当前y_i用于后续计算变化量
        y_i_org = y_i;

        // 平滑算法核心：结合原始点和相邻点对当前点进行调整
        // data_w_ * (x_i - y_i)：让当前点向原始点靠拢，防止偏离太远
        // smooth_w_ * (y_ip1 + y_m1 - 2*y_i)：让当前点向相邻点的均值靠拢，实现平滑
        y_i += data_w_ * (x_i - y_i) + smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
        // 将调整后的y_i写回new_path
        setFieldByDim(new_path.poses[i], j, y_i); // y_i是设定的值
        // 统计本次迭代的总变化量，用于判断是否收敛
        change += abs(y_i - y_i_org);
      }

      // validate update is admissible, only checks cost if a valid costmap pointer is provided
      float cost = 0.0;
      if (costmap) {
        costmap->worldToMap(
          getFieldByDim(new_path.poses[i], 0),
          getFieldByDim(new_path.poses[i], 1),
          mx, my);
        cost = static_cast<float>(costmap->getCost(mx, my));
      }

      //平滑之后的坐标有障碍物在上面，直接返回函数！宣告平滑失败！！
      if (cost > MAX_NON_OBSTACLE_COST && cost != UNKNOWN_COST) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("SmacPlannerSmoother"),
          "Smoothing process resulted in an infeasible collision. "
          "Returning the last path before the infeasibility was introduced.");
        path = last_path;
        updateApproximatePathOrientations(path, reversing_segment);
        return false;
      }
    }

    last_path = new_path;
  }

  // Let's do additional refinement, it shouldn't take more than a couple milliseconds
  // but really puts the path quality over the top.
  if (do_refinement_ && refinement_ctr_ < refinement_num_) {
    refinement_ctr_++;
    smoothImpl(new_path, reversing_segment, costmap, max_time);
  }
  
  //保证运动轨迹和path上的位姿保持一致！！！
  updateApproximatePathOrientations(new_path, reversing_segment);
  path = new_path;
  return true;
}//end function smoothImpl




double Smoother::getFieldByDim(
  const geometry_msgs::msg::PoseStamped & msg, const unsigned int & dim)
{
  if (dim == 0) {
    return msg.pose.position.x;
  } else if (dim == 1) {
    return msg.pose.position.y;
  } else {
    return msg.pose.position.z;
  }
}

void Smoother::setFieldByDim(
  geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
  const double & value)
{
  if (dim == 0) {
    msg.pose.position.x = value;
  } else if (dim == 1) {
    msg.pose.position.y = value;
  } else {
    msg.pose.position.z = value;
  }
}

//详见算法实现文档！！！
// 这个函数的作用是将路径（path）按照运动方向的变化（如前进/后退切换点、原地转向点）分割成若干段（PathSegment）。
// 对于非全向（非holonomic）机器人，路径中遇到“尖点”（即运动方向发生反转，或出现原地旋转）的位置会被识别出来，
// 并以这些点为分界，将整条路径划分为多个方向一致的子段。
// 返回值是每个子段的起止索引（PathSegment），便于后续对每一段分别进行平滑处理或特殊处理。
std::vector<PathSegment> Smoother::findDirectionalPathSegments(const nav_msgs::msg::Path & path)
{
  std::vector<PathSegment> segments;
  PathSegment curr_segment;
  curr_segment.start = 0;

  // If holonomic, no directional changes and
  // may have abrupt angular changes from naive grid search
  if (is_holonomic_) {
    curr_segment.end = path.poses.size() - 1;
    segments.push_back(curr_segment);
    return segments;
  }//只有是全向机器人才会进入这个条件

  // Iterating through the path to determine the position of the cusp
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = path.poses[idx].pose.position.x - path.poses[idx - 1].pose.position.x;
    double oa_y = path.poses[idx].pose.position.y - path.poses[idx - 1].pose.position.y;
    double ab_x = path.poses[idx + 1].pose.position.x - path.poses[idx].pose.position.x;
    double ab_y = path.poses[idx + 1].pose.position.y - path.poses[idx].pose.position.y;

    // Checking for the existence of cusp, in the path, using the dot product.
    double dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }//这好像是一个bug！！！！！

    // Checking for the existence of a differential rotation in place.
    double cur_theta = tf2::getYaw(path.poses[idx].pose.orientation);
    double next_theta = tf2::getYaw(path.poses[idx + 1].pose.orientation);
    double dtheta = angles::shortest_angular_distance(cur_theta, next_theta);
    if (fabs(ab_x) < 1e-4 && fabs(ab_y) < 1e-4 && fabs(dtheta) > 1e-4) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }
  }

  curr_segment.end = path.poses.size() - 1;
  segments.push_back(curr_segment);
  return segments;
}//findDirectionalPathSegments function 结束！！！！


// 该函数的核心作用是根据路径点的分布，近似地为路径上的每个点分配合适的朝向（orientation），
// 以保证机器人沿路径运动时，其朝向与路径的切线方向一致。
// 对于非全向机器人，还会判断当前路径段是否为倒车段（reversing_segment），
// 并在倒车时自动将朝向反转180度，从而确保朝向与实际运动方向一致。
void Smoother::updateApproximatePathOrientations(
  nav_msgs::msg::Path & path,
  bool & reversing_segment)
{
  double dx, dy, theta, pt_yaw;
  reversing_segment = false;

  // Find if this path segment is in reverse
  dx = path.poses[2].pose.position.x - path.poses[1].pose.position.x;
  dy = path.poses[2].pose.position.y - path.poses[1].pose.position.y;
  theta = atan2(dy, dx);
  pt_yaw = tf2::getYaw(path.poses[1].pose.orientation);
  if (!is_holonomic_ && fabs(angles::shortest_angular_distance(pt_yaw, theta)) > M_PI_2) {
    reversing_segment = true;
  }

  // Find the angle relative the path position vectors
  for (unsigned int i = 0; i != path.poses.size() - 1; i++) {
    dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    theta = atan2(dy, dx);

    // If points are overlapping, pass
    if (fabs(dx) < 1e-4 && fabs(dy) < 1e-4) {
      continue;
    }

    // Flip the angle if this path segment is in reverse
    if (reversing_segment) {
      theta += M_PI;  // orientationAroundZAxis will normalize
    }

    path.poses[i].pose.orientation = orientationAroundZAxis(theta);
  }
}//

//找到哪个epansion的新path的路径最短
unsigned int Smoother::findShortestBoundaryExpansionIdx(const BoundaryExpansions & boundary_expansions)
{
  // Check which is valid with the minimum integrated length such that
  // shorter end-points away that are infeasible to achieve without
  // a loop-de-loop are punished
  double min_length = 1e9;
  int shortest_boundary_expansion_idx = 1e9;
  for (unsigned int idx = 0; idx != boundary_expansions.size(); idx++) {
    if (boundary_expansions[idx].expansion_path_length<min_length &&
      !boundary_expansions[idx].in_collision &&
      boundary_expansions[idx].path_end_idx>0.0 &&
      boundary_expansions[idx].expansion_path_length > 0.0)
    {
      min_length = boundary_expansions[idx].expansion_path_length;
      shortest_boundary_expansion_idx = idx;
    }
  }

  return shortest_boundary_expansion_idx;
}

//详见算法实现文档
//对expansion节点到起点的其他节点进行插值再拟合。保证生成的新的节点满足dubins曲线要求。
void Smoother::findBoundaryExpansion(
  const geometry_msgs::msg::Pose & start,//整个segment的起点
  const geometry_msgs::msg::Pose & end,//expansion点的位姿
  BoundaryExpansion & expansion,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  //一定要特别的注意！这里的state_space_使用的是dubins曲线！
  static ompl::base::ScopedState<> from(state_space_), to(state_space_), s(state_space_);

  //
  from[0] = start.position.x;
  from[1] = start.position.y;
  from[2] = tf2::getYaw(start.orientation);

  to[0] = end.position.x;
  to[1] = end.position.y;
  to[2] = tf2::getYaw(end.orientation);

  //计算起点到expansion的距离
  double d = state_space_->distance(from(), to());//这个计算的不是直线距离，是dubins距离！！！！！！
  // If this path is too long compared to the original, then this is probably
  // a loop-de-loop, treat as invalid as to not deviate too far from the original path.
  // 2.0 selected from prinicipled choice of boundary test points
  // r, 2 * r, r * PI, and 2 * PI * r. If there is a loop, it will be
  // approximately 2 * PI * r, which is 2 * PI > r, PI > 2 * r, and 2 > r * PI.
  // For all but the last backup test point, a loop would be approximately
  // 2x greater than any of the selections.
  // 如果起点到expansion的距离d大于原始路径长度的2倍，说明该expansion点很可能需要绕远路（如出现回环、绕圈等不合理路径），
  // 这会导致平滑路径严重偏离原始路径，影响规划效果，因此直接返回，认为该expansion无效。
  if (d > 2.0 * expansion.original_path_length) {
    return;
  }

  std::vector<double> reals;
  double theta(0.0), x(0.0), y(0.0);
  double x_m = start.position.x;
  double y_m = start.position.y;

  // Get intermediary poses
  for (double i = 0; i <= expansion.path_end_idx; i++) {
    //在segemnt原始起点和expansion节点之间进行插值，
    state_space_->interpolate(from(), to(), i / expansion.path_end_idx, s());
    reals = s.reals();
    // Make sure in range [0, 2PI)
    theta = (reals[2] < 0.0) ? (reals[2] + 2.0 * M_PI) : reals[2];
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    x = reals[0];
    y = reals[1];

    // Check for collision
    //判断这个节点是不是碰撞了
    unsigned int mx, my;
    costmap->worldToMap(x, y, mx, my);
    if (static_cast<float>(costmap->getCost(mx, my)) >= INSCRIBED_COST) {
      expansion.in_collision = true;
    }

    // Integrate path length
    expansion.expansion_path_length += hypot(x - x_m, y - y_m);
    x_m = x;
    y_m = y;

    // Store point
    expansion.pts.emplace_back(x, y, theta);
  }
}

//详见算法实现文档
//从起点开始遍历，找到超过累计距离超过4个阈值的对应节点。
template<typename IteratorT>
BoundaryExpansions Smoother::generateBoundaryExpansionPoints(IteratorT start, IteratorT end)
{
  std::vector<double> distances = {
    min_turning_rad_,  // Radius
    2.0 * min_turning_rad_,  // Diameter
    M_PI * min_turning_rad_,  // 50% Circumference
    2.0 * M_PI * min_turning_rad_  // Circumference
  };

  BoundaryExpansions boundary_expansions;//本质上是一个vector
  boundary_expansions.resize(distances.size());
  double curr_dist = 0.0;
  double x_last = start->pose.position.x;
  double y_last = start->pose.position.y;
  geometry_msgs::msg::Point pt;
  unsigned int curr_dist_idx = 0;

  for (IteratorT iter = start; iter != end; iter++) {
    pt = iter->pose.position;
    curr_dist += hypot(pt.x - x_last, pt.y - y_last);//这是一个std函数！计算二维或三维空间中两点之间的欧几里得距离。
    x_last = pt.x;
    y_last = pt.y;

    if (curr_dist >= distances[curr_dist_idx]) {
      // 这里 iter 和 start 都是 std::vector<geometry_msgs::msg::PoseStamped>::iterator 类型，
      // 它们的减法（iter - start）表示从 start 到 iter 之间的元素个数，即当前点在路径中的索引（下标）。
      boundary_expansions[curr_dist_idx].path_end_idx = iter - start;
      boundary_expansions[curr_dist_idx].original_path_length = curr_dist;
      curr_dist_idx++;
    }

    if (curr_dist_idx == boundary_expansions.size()) {
      break;
    }
  }

  return boundary_expansions;
}//generateBoundaryExpansionPoints

// 该函数的作用是对路径的起点进行边界条件约束，使路径起点附近的一段轨迹满足运动学可行性（如最小转弯半径等约束），
//通过生成一系列起点附近的边界扩展点，寻找一条最短且运动学可行的曲线段，并用这段曲线替换原路径起点附近的点，从而保证路径起点的平滑性和可行性。
void Smoother::enforceStartBoundaryConditions(
  const geometry_msgs::msg::Pose & start_pose,
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const bool & reversing_segment)
{
  // Find range of points for testing
  //1.从起点开始遍历，找到超过累计距离超过4个阈值的对应节点boundary_expansions
  BoundaryExpansions boundary_expansions = generateBoundaryExpansionPoints<PathIterator>(path.poses.begin(), path.poses.end());

  // Generate the motion model and metadata from start -> test points
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
      BoundaryExpansion & expansion = boundary_expansions[i];
      if (expansion.path_end_idx == 0.0) {
        continue;
      }

      if (!reversing_segment) {
        //对expansion节点到起点的其他节点进行插值再拟合。保证生成的新的节点满足dubins曲线要求。
        findBoundaryExpansion( start_pose, //输入变量
                              path.poses[expansion.path_end_idx].pose, //输入变量
                              expansion, // input output变量
                              costmap);//输入变量
      } else {
        findBoundaryExpansion( path.poses[expansion.path_end_idx].pose, start_pose, expansion,  costmap);
      }
  }

  // Find the shortest kinematically feasible boundary expansion
  //找到哪个epansion的新path的路径最短
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }

  // Override values to match curve
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  // 4.使用这个最短的explansion，更新原始path上的节点的位姿。
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path.poses[i].pose.position.x = best_expansion.pts[i].x;
    path.poses[i].pose.position.y = best_expansion.pts[i].y;
    path.poses[i].pose.orientation = orientationAroundZAxis(best_expansion.pts[i].theta);
  }
}//end function enforceStartBoundaryConditions



void Smoother::enforceEndBoundaryConditions(
  const geometry_msgs::msg::Pose & end_pose,
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const bool & reversing_segment)
{
  // Find range of points for testing
  BoundaryExpansions boundary_expansions =
    generateBoundaryExpansionPoints<ReversePathIterator>(path.poses.rbegin(), path.poses.rend());

  // Generate the motion model and metadata from start -> test points
  unsigned int expansion_starting_idx;
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
    BoundaryExpansion & expansion = boundary_expansions[i];
    if (expansion.path_end_idx == 0.0) {
      continue;
    }
    expansion_starting_idx = path.poses.size() - expansion.path_end_idx - 1;
    if (!reversing_segment) {
      findBoundaryExpansion(path.poses[expansion_starting_idx].pose, end_pose, expansion, costmap);
    } else {
      findBoundaryExpansion(end_pose, path.poses[expansion_starting_idx].pose, expansion, costmap);
    }
  }

  // Find the shortest kinematically feasible boundary expansion
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }

  // Override values to match curve
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  expansion_starting_idx = path.poses.size() - best_expansion.path_end_idx - 1;
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path.poses[expansion_starting_idx + i].pose.position.x = best_expansion.pts[i].x;
    path.poses[expansion_starting_idx + i].pose.position.y = best_expansion.pts[i].y;
    path.poses[expansion_starting_idx + i].pose.orientation = orientationAroundZAxis(
      best_expansion.pts[i].theta);
  }
}

}  // namespace nav2_smac_planner

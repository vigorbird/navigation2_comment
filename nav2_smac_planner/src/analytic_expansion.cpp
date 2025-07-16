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

#include <algorithm>
#include <vector>
#include <memory>

#include "nav2_smac_planner/analytic_expansion.hpp"

namespace nav2_smac_planner
{

template<typename NodeT>
AnalyticExpansion<NodeT>::AnalyticExpansion(
  const MotionModel & motion_model,
  const SearchInfo & search_info,
  const bool & traverse_unknown,
  const unsigned int & dim_3_size)
: _motion_model(motion_model),
  _search_info(search_info),
  _traverse_unknown(traverse_unknown),
  _dim_3_size(dim_3_size),
  _collision_checker(nullptr)
{
}

template<typename NodeT>
void AnalyticExpansion<NodeT>::setCollisionChecker(
  GridCollisionChecker * collision_checker)
{
  _collision_checker = collision_checker;
}

template<typename NodeT>
typename AnalyticExpansion<NodeT>::NodePtr AnalyticExpansion<NodeT>::tryAnalyticExpansion(
  const NodePtr & current_node,
  const NodeVector & coarse_check_goals,//如果只有一个goal，那么coarse check goal包含一个元素，fine不包括
  const NodeVector & fine_check_goals,
  const CoordinateVector & goals_coords,//如果是hybrid A* ,那么这个坐标包括的是xy和theta
  const NodeGetter & getter, //根据索引生成node，并添加到graph中
  int & analytic_iterations,//这是一个输入输出的变量！！！ 初始输入的时候0
  int & closest_distance)//这是一个输入输出的变量！！！ 初始输入的时候max_limits
{
  // This must be a valid motion model for analytic expansion to be attempted
  if (_motion_model == MotionModel::DUBIN || 
    _motion_model == MotionModel::REEDS_SHEPP ||
    _motion_model == MotionModel::STATE_LATTICE)
  {
    // See if we are closer and should be expanding more often
    //1.根据index计算得到angle、x和y
    const Coordinates node_coords =
      NodeT::getCoords(current_node->getIndex(), 
                      _collision_checker->getCostmap()->getSizeInCellsX(), 
                      _dim_3_size);//这个参数是angle_quantization

    AnalyticExpansionNodes current_best_analytic_nodes;
    NodePtr current_best_goal = nullptr;
    NodePtr current_best_node = nullptr;
    float current_best_score = std::numeric_limits<float>::max();
    //2.计算启发函数分数！！！
    closest_distance = std::min( closest_distance,
                                static_cast<int>(NodeT::getHeuristicCost(node_coords, goals_coords)));
    // We want to expand at a rate of d/expansion_ratio,
    // but check to see if we are so close that we would be expanding every iteration
    // If so, limit it to the expansion ratio (rounded up)
    //4.计算想要迭代的次数
    // 这句代码的意思是：计算本次分析性扩展（analytic expansion）期望执行的迭代次数desired_iterations。
    // 具体做法是：用当前距离closest_distance除以分析性扩展比例_analytic_expansion_ratio，得到一个建议的迭代次数；
    // 但如果距离很近导致结果小于扩展比例本身，则至少取扩展比例（向上取整）作为最小迭代次数。
    // 也就是说，desired_iterations = max(距离/扩展比例, 扩展比例本身)，保证扩展不会过于频繁。
    int desired_iterations = std::max(
      static_cast<int>(closest_distance / _search_info.analytic_expansion_ratio),
      static_cast<int>(std::ceil(_search_info.analytic_expansion_ratio)));

    // If we are closer now, we should update the target number of iterations to go
    analytic_iterations = std::min(analytic_iterations, desired_iterations);

    // Always run the expansion on the first run in case there is a
    // trivial path to be found
    if (analytic_iterations <= 0) {
          // Reset the counter and try the analytic path expansion
          analytic_iterations = desired_iterations;
          bool found_valid_expansion = false;

          // First check the coarse search resolution goals
          //5.遍历coarse的目标
          for (auto & current_goal_node : coarse_check_goals) {
            //5.1
            AnalyticExpansionNodes analytic_nodes =  getAnalyticPath( current_node, 
                                                                      current_goal_node, 
                                                                      getter,
                                                                      current_node->motion_table.state_space);//!!!!!!!!!!!!
            if (!analytic_nodes.nodes.empty()) {
              found_valid_expansion = true;
              NodePtr node = current_node;
              //5.2
              float score = refineAnalyticPath(node, current_goal_node, getter, analytic_nodes);//!!!!!!!!!
              // Update the best score if we found a better path
              if (score < current_best_score) {//current_best_score初始值最大是max
                current_best_analytic_nodes = analytic_nodes;
                current_best_goal = current_goal_node;
                current_best_score = score;
                current_best_node = node;//这是一局部变量
              }
            }
          }

          // perform a final search if we found a goal
          //好像只有一个目标的话，我们不会进入这个条件！！！
          if (found_valid_expansion) {
            for (auto & current_goal_node : fine_check_goals) {
              AnalyticExpansionNodes analytic_nodes =
                getAnalyticPath(
                current_node, current_goal_node, getter,
                current_node->motion_table.state_space);
              if (!analytic_nodes.nodes.empty()) {
                NodePtr node = current_node;
                float score = refineAnalyticPath(node, 
                                                current_goal_node, 
                                                getter, 
                                                analytic_nodes);
                // Update the best score if we found a better path
                if (score < current_best_score) {
                  current_best_analytic_nodes = analytic_nodes;
                  current_best_goal = current_goal_node;
                  current_best_score = score;
                  current_best_node = node;
                }
              }
            }
          }
    }

    if (!current_best_analytic_nodes.nodes.empty()) {
      //6.根据最好的结果解析解结果，进行设置！
      return setAnalyticPath(current_best_node, 
                            current_best_goal,
                            current_best_analytic_nodes);
    }
    analytic_iterations--;
  }

  // No valid motion model - return nullptr
  return NodePtr(nullptr);
}//end functon tryAnalyticExpansion


//计算rs曲线改变方向的次数，向前走为1 ，向后走为负1
template<typename NodeT>
int AnalyticExpansion<NodeT>::countDirectionChanges(
  const ompl::base::ReedsSheppStateSpace::ReedsSheppPath & path)
{
  const double * lengths = path.length_;
  int changes = 0;
  int last_dir = 0;
  for (int i = 0; i < 5; ++i) {
    if (lengths[i] == 0.0) {
      continue;
    }

    int currentDirection = (lengths[i] > 0.0) ? 1 : -1;
    if (last_dir != 0 && currentDirection != last_dir) {
      ++changes;
    }
    last_dir = currentDirection;
  }

  return changes;
}

//
template<typename NodeT>
typename AnalyticExpansion<NodeT>::AnalyticExpansionNodes AnalyticExpansion<NodeT>::getAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal,
  const NodeGetter & node_getter,
  const ompl::base::StateSpacePtr & state_space)
{
  static ompl::base::ScopedState<> from(state_space), to(state_space), s(state_space);
  from[0] = node->pose.x;
  from[1] = node->pose.y;
  from[2] = node->motion_table.getAngleFromBin(node->pose.theta);
  to[0] = goal->pose.x;
  to[1] = goal->pose.y;
  to[2] = node->motion_table.getAngleFromBin(goal->pose.theta);

  //1.起点到终点的RS曲线距离
  float d = state_space->distance(from(), to());

  auto rs_state_space = dynamic_cast<ompl::base::ReedsSheppStateSpace *>(state_space.get());
  //2.计算rs生成的曲线改变方向的次数
  int direction_changes = 0;
  if (rs_state_space) {
    //计算rs曲线改变方向的次数，向前走为1 ，向后走为负1
    direction_changes = countDirectionChanges(rs_state_space->reedsShepp(from.get(), to.get()));
  }

  // A move of sqrt(2) is guaranteed to be in a new cell
  static const float sqrt_2 = sqrtf(2.0f);

  // If the length is too far, exit. This prevents unsafe shortcutting of paths
  // into higher cost areas far out from the goal itself, let search to the work of getting
  // close before the analytic expansion brings it home. This should never be smaller than
  // 4-5x the minimum turning radius being used, or planning times will begin to spike.
  //3.rs计算得到的距离不能太短也不能太近，否则直接返回
  if (d > _search_info.analytic_expansion_max_length || d < sqrt_2) {
    return AnalyticExpansionNodes();
  }

  //3.计算要走过多少个栅格！！！
  unsigned int num_intervals = static_cast<unsigned int>(std::floor(d / sqrt_2));//要走过多少个grid栅格

  AnalyticExpansionNodes possible_nodes;
  // When "from" and "to" are zero or one cell away,
  // num_intervals == 0
  possible_nodes.nodes.reserve(num_intervals);  // We won't store this node or the goal
  std::vector<double> reals;
  double theta;

  // Pre-allocate
  NodePtr prev(node);
  uint64_t index = 0;
  NodePtr next(nullptr);
  float angle = 0.0;
  Coordinates proposed_coordinates;
  bool failure = false;
  std::vector<float> node_costs;
  node_costs.reserve(num_intervals);

  // Check intermediary poses (non-goal, non-start)
  //4.遍历走过的栅格
  for (float i = 1; i <= num_intervals; i++) {
    //4.1 根据百分比进行插值，得到每个node的位姿
    state_space->interpolate(from(), to(), i / num_intervals, s());
    reals = s.reals();
    // Make sure in range [0, 2PI)
    theta = (reals[2] < 0.0) ? (reals[2] + 2.0 * M_PI) : reals[2];
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    angle = node->motion_table.getAngle(theta);//theta / bin_size;

    // Turn the pose into a node, and check if it is valid
    // 4.2 根据位姿计算得到这个node对应的全局地图的唯一索引
    index = NodeT::getIndex(
      static_cast<unsigned int>(reals[0]),
      static_cast<unsigned int>(reals[1]),
      static_cast<unsigned int>(angle));
    // Get the node from the graph
    //4.3 向graph中添加这个节点
    /*
    node_getter函数具体的实现
     NodeGetter neighborGetter =
    [&, this](const uint64_t & index, NodePtr & neighbor_rtn) -> bool
    {
      if (index >= max_index) {
        return false;
      }

      neighbor_rtn = addToGraph(index);//新建index这个节点并向graph添加，graph本质上就是一个哈希表
      return true;
    };
    */
    if (node_getter(index, next)) {
      //4.4 将新生成节点的坐标信息赋值给 next
      Coordinates initial_node_coords = next->pose;
      proposed_coordinates = {static_cast<float>(reals[0]), static_cast<float>(reals[1]), angle};
      next->setPose(proposed_coordinates);
      //4.5判断这个节点和障碍物的关系
      if (next->isNodeValid(_traverse_unknown, _collision_checker) && next != prev) {
        // Save the node, and its previous coordinates in case we need to abort
        possible_nodes.add(next, initial_node_coords, proposed_coordinates);
        node_costs.emplace_back(next->getCost());
        prev = next;
      } else {
        // Abort
        next->setPose(initial_node_coords);
        failure = true;
        break;
      }
    } else {
      // Abort
      failure = true;
      break;
    }
  }

  //5.如果没有失败 
  if (!failure) {
    // We found 'a' valid expansion. Now to tell if its a quality option...
    const float max_cost = _search_info.analytic_expansion_max_cost;
    auto max_cost_it = std::max_element(node_costs.begin(), node_costs.end());
    if (max_cost_it != node_costs.end() && *max_cost_it > max_cost) {
      // If any element is above the comfortable cost limit, check edge cases:
      // (1) Check if goal is in greater than max_cost space requiring
      //  entering it, but only entering it on final approach, not in-and-out
      // (2) Checks if goal is in normal space, but enters costed space unnecessarily
      //  mid-way through, skirting obstacle or in non-globally confined space
      // 这段代码的目的是检查路径上的代价（cost）是否存在“先从高代价区出来后又重新进入高代价区”的情况。
      // 具体做法是：从路径终点（goal）向起点遍历每个节点的cost（node_costs是反向遍历），
      // 一旦发现cost小于等于max_cost，说明已经离开了高代价区，标记cost_exit_high_cost_region为true。
      // 如果之后又遇到cost大于max_cost且已经标记过cost_exit_high_cost_region，说明路径中间又进入了高代价区，
      // 这种情况会被判定为failure（即路径不被接受）。
      bool cost_exit_high_cost_region = false;
      for (auto iter = node_costs.rbegin(); iter != node_costs.rend(); ++iter) {
        const float & curr_cost = *iter;
        if (curr_cost <= max_cost) {
          cost_exit_high_cost_region = true;
        } else if (curr_cost > max_cost && cost_exit_high_cost_region) {
          failure = true;
          break;
        }
      }

      // (3) Handle exception: there may be no other option close to goal
      // if max cost is set too low (optional)
      //算法不想轻易认输，再尝试一个条件看看是否满足，如果满足了，就成功了
      // 这段代码的目的是：如果路径被判定为failure（即路径中存在不合理的高代价区穿越），
      // 但有两个特殊条件同时满足时，可以“放宽限制”，允许这条路径通过。
      // 具体条件是：
      // 1. 路径长度d小于2倍的π乘以最小转弯半径（即距离目标点很近，通常是最后的转弯或调整阶段）；
      // 2. 配置参数_analytic_expansion_max_cost_override为true（允许在特殊情况下放宽最大代价限制）。
      // 如果这两个条件都满足，则将failure重新置为false，允许这条路径被接受。
      if (failure) {
        if (d < 2.0f * M_PI * goal->motion_table.min_turning_radius &&
          _search_info.analytic_expansion_max_cost_override)
        {
          failure = false;
        }
      }
    }
  }

  // Reset to initial poses to not impact future searches
  for (const auto & node_pose : possible_nodes.nodes) {
    const auto & n = node_pose.node;
    n->setPose(node_pose.initial_coords);
  }

  if (failure) {
    return AnalyticExpansionNodes();
  }

  possible_nodes.setDirectionChanges(direction_changes);
  return possible_nodes;
}//end fucntion getAnalyticPath!!!!!!!!!!!!!!!!!!!






template<typename NodeT>
float AnalyticExpansion<NodeT>::refineAnalyticPath(
  NodePtr & node,
  const NodePtr & goal_node,
  const NodeGetter & getter,
  AnalyticExpansionNodes & analytic_nodes)
{
  NodePtr test_node = node;
  // 这段代码的作用是：尝试通过回溯父节点（每次回溯5个父节点，最多回溯8次，即最多回溯40个节点），
  // 以不同的起点重新进行解析路径（analytic path）扩展，寻找是否存在更优的解析路径。
  // 具体流程如下：
  // 1.1每次循环检查当前节点的父节点链是否足够长（至少有5层父节点），否则跳出循环。
  // 1.2. 如果可以回溯，则将test_node回溯到5个父节点之前的位置。
  // 1.3. 以新的test_node为起点，调用getAnalyticPath尝试生成一条到goal_node的解析路径。
  // 1.4. 如果新生成的解析路径为空，则跳出循环。
  // 1.5. 如果新路径的方向变化次数比当前最优路径多，则跳过本次循环（不更新）。
  // 1.6. 否则，认为新路径更优，更新analytic_nodes和node为当前结果。
  AnalyticExpansionNodes refined_analytic_nodes;
  for (int i = 0; i < 8; i++) {
    // Attempt to create better paths in 5 node increments, need to make sure
    // they exist for each in order to do so (maximum of 40 points back).
    if (test_node->parent && 
      test_node->parent->parent &&
      test_node->parent->parent->parent &&
      test_node->parent->parent->parent->parent &&
      test_node->parent->parent->parent->parent->parent)
    {
      test_node = test_node->parent->parent->parent->parent->parent;
      // print the goals pose
      refined_analytic_nodes = getAnalyticPath( test_node, 
                                                goal_node, 
                                                getter,
                                                test_node->motion_table.state_space);
      if (refined_analytic_nodes.nodes.empty()) {
        break;
      }
      if (refined_analytic_nodes.direction_changes > analytic_nodes.direction_changes) {
        // If the direction changes are worse, we don't want to use this path
        continue;
      }
      analytic_nodes = refined_analytic_nodes;
      node = test_node;//要十分注意这句话！！
    } else {
      break;
    }
  }

  // The analytic expansion can short-cut near obstacles when closer to a goal
  // So, we can attempt to refine it more by increasing the possible radius
  // higher than the minimum turning radius and use the best solution based on
  // a scoring function similar to that used in traversal cost estimation.
  //将解析算出来的path，得到一个分数。
  auto scoringFn = [&](const AnalyticExpansionNodes & expansion) {
      if (expansion.nodes.size() < 2) {
        return std::numeric_limits<float>::max();
      }

      float score = 0.0;
      float normalized_cost = 0.0;
      // Analytic expansions are consistently spaced
      const float distance = hypotf( expansion.nodes[1].proposed_coords.x - expansion.nodes[0].proposed_coords.x,
                                     expansion.nodes[1].proposed_coords.y - expansion.nodes[0].proposed_coords.y);
      const float & weight = expansion.nodes[0].node->motion_table.cost_penalty;
      for (auto iter = expansion.nodes.begin(); iter != expansion.nodes.end(); ++iter) {
        normalized_cost = iter->node->getCost() / 252.0f;
        // Search's Traversal Cost Function
        score += distance * (1.0 + weight * normalized_cost);
      }
      return score;
    };


  //2.然后开始不停地增加最小转弯半径，然后寻求获取更好的分数
  float original_score = scoringFn(analytic_nodes);
  float best_score = original_score;
  float score = std::numeric_limits<float>::max();
  float min_turn_rad = node->motion_table.min_turning_radius;
  const float max_min_turn_rad = 4.0 * min_turn_rad;  // Up to 4x the turning radius
  //不停的增加 min_turn_rad
  while (min_turn_rad < max_min_turn_rad) {
        // 为什么要不停地增加最小转弯半径进行搜索最优值？
        // 这是因为在某些情况下，较大的最小转弯半径可以生成更平滑、方向变化更少、代价更低的路径。
        // 通过逐步增加最小转弯半径，可以在不同的转弯约束下多次尝试，找到综合得分最优的路径。
        min_turn_rad += 0.5;  // 在栅格坐标系下，每次增加半个cell
        ompl::base::StateSpacePtr state_space;
        if (node->motion_table.motion_model == MotionModel::DUBIN) {
          state_space = std::make_shared<ompl::base::DubinsStateSpace>(min_turn_rad);
        } else {
          state_space = std::make_shared<ompl::base::ReedsSheppStateSpace>(min_turn_rad);
        }
        refined_analytic_nodes = getAnalyticPath(node, goal_node, getter, state_space);
        score = scoringFn(refined_analytic_nodes);

        // Normal scoring: prioritize lower cost as long as not more directional changes
        if (score <= best_score &&
          refined_analytic_nodes.direction_changes <= analytic_nodes.direction_changes)
        {
          analytic_nodes = refined_analytic_nodes;
          best_score = score;
          continue;
        }

        // Special case: If we have a better score than original (only) and less directional changes
        // the path quality is still better than the original and is less operationally complex
        if (score <= original_score &&
          refined_analytic_nodes.direction_changes < analytic_nodes.direction_changes)
        {
          analytic_nodes = refined_analytic_nodes;
          best_score = score;
        }
  }//end while 循环！！！

  return best_score;
}//end function refineAnalyticPath！！！！！




//总结：
//该函数的核心作用是将分析扩展得到的节点序列，串联成一条合法的路径，并处理节点的父子关系、位姿、访问状态等，确保路径的唯一性和正确性，避免节点复用带来的问题。
//将选择好的expansion node一个个取出来，设置父子节点，最终将goal这个节点输出。
template<typename NodeT>
typename AnalyticExpansion<NodeT>::NodePtr AnalyticExpansion<NodeT>::setAnalyticPath(
  const NodePtr & node,
  const NodePtr & goal_node,
  const AnalyticExpansionNodes & expanded_nodes)
{
  _detached_nodes.clear();
  // Legitimate final path - set the parent relationships, states, and poses
  NodePtr prev = node;
  for (const auto & node_pose : expanded_nodes.nodes) {
    auto n = node_pose.node;
    cleanNode(n);//让这个node的_motion_primitive指针变成空指针
    if (n->getIndex() != goal_node->getIndex()) {
      if (n->wasVisited()) {
        _detached_nodes.push_back(std::make_unique<NodeT>(-1));//std::list = _detached_nodes
        n = _detached_nodes.back().get();
      }
      n->parent = prev;
      n->pose = node_pose.proposed_coords;
      n->visited();
      prev = n;
    }
  }
  
  if (goal_node != prev) {
    goal_node->parent = prev;
    cleanNode(goal_node);
    goal_node->visited();
  }
  return goal_node;
}




template<>
void AnalyticExpansion<NodeLattice>::cleanNode(const NodePtr & node)
{
  node->setMotionPrimitive(nullptr);
}

template<typename NodeT>
void AnalyticExpansion<NodeT>::cleanNode(const NodePtr & /*expanded_nodes*/)
{
}

template<>
typename AnalyticExpansion<Node2D>::AnalyticExpansionNodes AnalyticExpansion<Node2D>::
getAnalyticPath(
  const NodePtr &,
  const NodePtr &,
  const NodeGetter &,
  const ompl::base::StateSpacePtr &)
{
  return AnalyticExpansionNodes();
}

template<>
float AnalyticExpansion<Node2D>::refineAnalyticPath(
  NodePtr &,
  const NodePtr &,
  const NodeGetter &,
  AnalyticExpansionNodes &)
{
  return std::numeric_limits<float>::max();
}

template<>
typename AnalyticExpansion<Node2D>::NodePtr AnalyticExpansion<Node2D>::setAnalyticPath(
  const NodePtr &,
  const NodePtr &,
  const AnalyticExpansionNodes &)
{
  return NodePtr(nullptr);
}

template<>
typename AnalyticExpansion<Node2D>::NodePtr AnalyticExpansion<Node2D>::tryAnalyticExpansion(
  const NodePtr &,
  const NodeVector &,
  const NodeVector &,
  const CoordinateVector &,
  const NodeGetter &, int &,
  int &)
{
  return NodePtr(nullptr);
}

template class AnalyticExpansion<Node2D>;
template class AnalyticExpansion<NodeHybrid>;
template class AnalyticExpansion<NodeLattice>;

}  // namespace nav2_smac_planner

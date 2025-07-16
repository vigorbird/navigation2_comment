// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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

#include <omp.h>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <limits>
#include <type_traits>
#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include "nav2_smac_planner/a_star.hpp"
using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

template<typename NodeT>
AStarAlgorithm<NodeT>::AStarAlgorithm(
  const MotionModel & motion_model,
  const SearchInfo & search_info)
: _traverse_unknown(true),
  _is_initialized(false),
  _max_iterations(0),
  _terminal_checking_interval(5000),
  _max_planning_time(0),
  _x_size(0),
  _y_size(0),
  _search_info(search_info),
  _start(nullptr),
  _goal_manager(GoalManagerT()),
  _motion_model(motion_model)
{
  _graph.reserve(100000);
}

template<typename NodeT>
AStarAlgorithm<NodeT>::~AStarAlgorithm()
{
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::initialize(
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations,
  const int & terminal_checking_interval,
  const double & max_planning_time,
  const float & lookup_table_size,
  const unsigned int & dim_3_size)
{
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
  _terminal_checking_interval = terminal_checking_interval;
  _max_planning_time = max_planning_time;
  if (!_is_initialized) {
    NodeT::precomputeDistanceHeuristic(lookup_table_size, _motion_model, dim_3_size, _search_info);
  }
  _is_initialized = true;
  _dim3_size = dim_3_size;
  _expander = std::make_unique<AnalyticExpansion<NodeT>>(
    _motion_model, _search_info, _traverse_unknown, _dim3_size);
}

template<>
void AStarAlgorithm<Node2D>::initialize(
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations,
  const int & terminal_checking_interval,
  const double & max_planning_time,
  const float & /*lookup_table_size*/,
  const unsigned int & dim_3_size)
{
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
  _terminal_checking_interval = terminal_checking_interval;
  _max_planning_time = max_planning_time;

  if (dim_3_size != 1) {
    throw std::runtime_error("Node type Node2D cannot be given non-1 dim 3 quantization.");
  }
  _dim3_size = dim_3_size;
  _expander = std::make_unique<AnalyticExpansion<Node2D>>(
    _motion_model, _search_info, _traverse_unknown, _dim3_size);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setCollisionChecker(GridCollisionChecker * collision_checker)
{
  _collision_checker = collision_checker;
  _costmap = collision_checker->getCostmap();
  unsigned int x_size = _costmap->getSizeInCellsX();
  unsigned int y_size = _costmap->getSizeInCellsY();

  clearGraph();

  if (getSizeX() != x_size || getSizeY() != y_size) {
    _x_size = x_size;
    _y_size = y_size;
    NodeT::initMotionModel(_motion_model, _x_size, _y_size, _dim3_size, _search_info);//非常重要的函数！！！!!!
  }
  _expander->setCollisionChecker(_collision_checker);
}

//
template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::addToGraph(
  const uint64_t & index)
{
  auto iter = _graph.find(index);
  if (iter != _graph.end()) {
    return &(iter->second);
  }

  return &(_graph.emplace(index, NodeT(index)).first->second);
}

template<>
void AStarAlgorithm<Node2D>::setStart(
  const float & mx,
  const float & my,
  const unsigned int & dim_3)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero starting dim 3.");
  }
  _start = addToGraph(
    Node2D::getIndex(
      static_cast<unsigned int>(mx),
      static_cast<unsigned int>(my),
      getSizeX()));
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setStart(
  const float & mx,
  const float & my,
  const unsigned int & dim_3)
{
  _start = addToGraph(
    NodeT::getIndex(
      static_cast<unsigned int>(mx),
      static_cast<unsigned int>(my),
      dim_3));
  _start->setPose(Coordinates(mx, my, dim_3));
}

template<>
void AStarAlgorithm<Node2D>::populateExpansionsLog(
  const NodePtr & node,
  std::vector<std::tuple<float, float, float>> * expansions_log)
{
  Node2D::Coordinates coords = node->getCoords(node->getIndex());
  expansions_log->emplace_back(
    _costmap->getOriginX() + ((coords.x + 0.5) * _costmap->getResolution()),
    _costmap->getOriginY() + ((coords.y + 0.5) * _costmap->getResolution()),
    0.0);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::populateExpansionsLog(
  const NodePtr & node,
  std::vector<std::tuple<float, float, float>> * expansions_log)
{
  typename NodeT::Coordinates coords = node->pose;
  expansions_log->emplace_back(
    _costmap->getOriginX() + ((coords.x + 0.5) * _costmap->getResolution()),
    _costmap->getOriginY() + ((coords.y + 0.5) * _costmap->getResolution()),
    NodeT::motion_table.getAngleFromBin(coords.theta));
}

template<>
void AStarAlgorithm<Node2D>::setGoal(
  const float & mx,
  const float & my,
  const unsigned int & dim_3,
  const GoalHeadingMode & /*goal_heading_mode*/,
  const int & /*coarse_search_resolution*/)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero goal dim 3.");
  }
  _goal_manager.clear();
  auto goal = addToGraph(
    Node2D::getIndex(
      static_cast<unsigned int>(mx),
      static_cast<unsigned int>(my),
      getSizeX()));

  goal->setPose(Node2D::Coordinates(mx, my));
  _goal_manager.addGoal(goal);

  _coarse_search_resolution = 1;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setGoal(
  const float & mx,
  const float & my,
  const unsigned int & dim_3,
  const GoalHeadingMode & goal_heading_mode,
  const int & coarse_search_resolution)
{
  // Default to minimal resolution unless overridden for ALL_DIRECTION
  _coarse_search_resolution = 1;

  _goal_manager.clear();
  Coordinates ref_goal_coord(mx, my, static_cast<float>(dim_3));

  if (!_search_info.cache_obstacle_heuristic ||
    _goal_manager.hasGoalChanged(ref_goal_coord))
  {
    if (!_start) {
      throw std::runtime_error("Start must be set before goal.");
    }

    //整个代码就这里使用了resetObstacleHeuristic函数
    NodeT::resetObstacleHeuristic(
      _collision_checker->getCostmapROS(), _start->pose.x, _start->pose.y, mx, my);//mx 和 my是目标点！！！！
  }

  _goal_manager.setRefGoalCoordinates(ref_goal_coord);

  unsigned int num_bins = NodeT::motion_table.num_angle_quantization;
  // set goal based on heading mode
  switch (goal_heading_mode) {
    case GoalHeadingMode::DEFAULT: {
        // add a single goal node with single heading
        auto goal = addToGraph(
          NodeT::getIndex(
            static_cast<unsigned int>(mx),
            static_cast<unsigned int>(my),
            dim_3));
        goal->setPose(typename NodeT::Coordinates(mx, my, static_cast<float>(dim_3)));
        _goal_manager.addGoal(goal);
        break;
      }

    case GoalHeadingMode::BIDIRECTIONAL: {
        // Add two goals, one for each direction
        // add goal in original direction
        auto goal = addToGraph(
          NodeT::getIndex(
            static_cast<unsigned int>(mx),
            static_cast<unsigned int>(my),
            dim_3));
        goal->setPose(typename NodeT::Coordinates(mx, my, static_cast<float>(dim_3)));
        _goal_manager.addGoal(goal);

        // Add goal node in opposite (180°) direction
        unsigned int opposite_heading = (dim_3 + (num_bins / 2)) % num_bins;
        auto opposite_goal = addToGraph(
          NodeT::getIndex(
            static_cast<unsigned int>(mx),
            static_cast<unsigned int>(my),
            opposite_heading));
        opposite_goal->setPose(
          typename NodeT::Coordinates(mx, my, static_cast<float>(opposite_heading)));
        _goal_manager.addGoal(opposite_goal);
        break;
      }

    case GoalHeadingMode::ALL_DIRECTION: {
        // Set the coarse search resolution only for all direction
        _coarse_search_resolution = coarse_search_resolution;

        // Add goal nodes for all headings
        for (unsigned int i = 0; i < num_bins; ++i) {
          auto goal = addToGraph(
            NodeT::getIndex(
              static_cast<unsigned int>(mx),
              static_cast<unsigned int>(my),
              i));
          goal->setPose(typename NodeT::Coordinates(mx, my, static_cast<float>(i)));
          _goal_manager.addGoal(goal);
        }
        break;
      }
    case GoalHeadingMode::UNKNOWN:
      throw std::runtime_error("Goal heading is UNKNOWN.");
  }
}//end function setGoal!!!!



template<typename NodeT>
bool AStarAlgorithm<NodeT>::areInputsValid()
{
  // Check if graph was filled in
  if (_graph.empty()) {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // Check if points were filled in
  if (!_start || _goal_manager.goalsIsEmpty()) {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // remove invalid goals
  _goal_manager.removeInvalidGoals(getToleranceHeuristic(), _collision_checker, _traverse_unknown);

  // Check if ending point is valid
  if (_goal_manager.getGoalsSet().empty()) {
    throw nav2_core::GoalOccupied("Goal was in lethal cost");
  }

  // Note: We do not check the if the start is valid because it is cleared
  return true;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::createPath(
  CoordinateVector & path, int & iterations,
  const float & tolerance,
  std::function<bool()> cancel_checker,
  std::vector<std::tuple<float, float, float>> * expansions_log)
{

  steady_clock::time_point start_time = steady_clock::now();
  _tolerance = tolerance;
  _best_heuristic_node = {std::numeric_limits<float>::max(), 0};//是一个pair，存储是float 和 index
  //1.清空优先队列！！
  clearQueue();

  if (!areInputsValid()) {
    return false;
  }

  //2.设置粗细目标点
  //如果你只有一个目标，那么这个目标一定是粗目标点
  //在 Hybrid A* 或 Lattice 路径规划算法中，通常会有多个目标点_goals_state（比如终点有一定容忍区间或多目标任务）。
  //将所有有效的目标点，按照指定的分辨率分为“粗粒度目标”和“细粒度目标”，以便在分析性扩展（如Hybrid A的直接连线尝试）时，
  //先对粗粒度目标做快速尝试，再对细粒度目标做补充优化，从而提升路径规划的效率和成功率。
  NodeVector coarse_check_goals, fine_check_goals;
  _goal_manager.prepareGoalsForAnalyticExpansion(coarse_check_goals, //输出变量
                                                  fine_check_goals,//输出变量
                                                _coarse_search_resolution);//输入变量

  // 0) Add starting point to the open set
  //3.向优先队列中添加当前起始点
  addNode(0.0, getStart());//getStart返回的数据类型是NodePtr
  getStart()->setAccumulatedCost(0.0);//设置这个node的成员变量值 _accumulated_cost

  // Optimization: preallocate all variables
  NodePtr current_node = nullptr;
  NodePtr neighbor = nullptr;
  NodePtr expansion_result = nullptr;
  float g_cost = 0.0;
  NodeVector neighbors;
  int approach_iterations = 0;
  NeighborIterator neighbor_iterator;
  int analytic_iterations = 0;
  int closest_distance = std::numeric_limits<int>::max();

  // Given an index, return a node ptr reference if its collision-free and valid
  const uint64_t max_index = static_cast<uint64_t>(getSizeX()) *
    static_cast<uint64_t>(getSizeY()) *
    static_cast<uint64_t>(getSizeDim3());
    
    //定义了一个lambada函数，新建index这个节点并向graph添加
  NodeGetter neighborGetter =
    [&, this](const uint64_t & index, NodePtr & neighbor_rtn) -> bool
    {
      if (index >= max_index) {
        return false;
      }

      neighbor_rtn = addToGraph(index);//新建index这个节点并向graph添加，graph本质上就是一个哈希表
      return true;
    };


  //开始了最大的while循环！！！！！！！！！！！！！！！！！！1
  while (iterations < getMaxIterations() && !_queue.empty()) {
        
      // Check for planning timeout and cancel only on every Nth iteration
        if (iterations % _terminal_checking_interval == 0) {
          if (cancel_checker()) {
            throw nav2_core::PlannerCancelled("Planner was cancelled");
          }
          std::chrono::duration<double> planning_duration = std::chrono::duration_cast<std::chrono::duration<double>>(steady_clock::now() - start_time);
          if (static_cast<double>(planning_duration.count()) >= _max_planning_time) {
            return false;
          }
        }

        // 1) Pick Nbest from O s.t. min(f(Nbest)), remove from queue
        //4.1从优先队列中取出node，然后将这个node从优先队列中pop出去。
        current_node = getNextNode();

        // Save current node coordinates for debug
        if (expansions_log) {
          populateExpansionsLog(current_node, expansions_log);
        }

        // We allow for nodes to be queued multiple times in case
        // shorter paths result in it, but we can visit only once
        // Also a chance to perform last-checks necessary.
        //4.2 判断这个node是否被访问过，如果被访问过了则直接contine
        if (onVisitationCheckNode(current_node)) {
          continue;
        }

        iterations++;

        // 2) Mark Nbest as visited
        //4.3 设置这个node被访问过了
        current_node->visited();//设置这个node被访问过了

        // 2.1) Use an analytic expansion (if available) to generate a path
        //4.4 
        expansion_result = nullptr;
        expansion_result = _expander->tryAnalyticExpansion( current_node, 
                                                            coarse_check_goals, //两种不同的目标，如果只有一个目标，那么coarse只有一个元素，fine_check_goals没有元素
                                                            fine_check_goals,
                                                            _goal_manager.getGoalsCoordinates(), //终点的坐标，如果是hybrid A*，那么就是xy和theta。
                                                            neighborGetter, //向graph添加节点的函数
                                                            analytic_iterations, 
                                                            closest_distance);//这是一个输出变量会被更新！！
        if (expansion_result != nullptr) {
          current_node = expansion_result;
        }

        // 3) Check if we're at the goal, backtrace if required
        if (_goal_manager.isGoal(current_node)) {//表示已经到达了终点
          return current_node->backtracePath(path);//遍历之前的所有path，取出最优路径
        } else if (_best_heuristic_node.first < getToleranceHeuristic()) {//getToleranceHeuristic返回的是tolerance值
          // Optimization: Let us find when in tolerance and refine within reason
          approach_iterations++;
          if (approach_iterations >= getOnApproachMaxIterations()) {//迭代次数太多，也是需要直接返回的!!!!
            return _graph.at(_best_heuristic_node.second).backtracePath(path);
          }
        }

        // 4) Expand neighbors of Nbest not visited
        //4.7 清空neighbour，并且根据当前节点获取新的neighbour
        neighbors.clear();
        //
        // 这个函数的作用是获取当前节点 current_node 的所有可行邻居节点，并将它们存入 neighbors 向量中。
        // neighborGetter 用于根据索引获取邻居节点指针，_collision_checker 用于碰撞检测，
        // _traverse_unknown 表示是否允许遍历未知区域。
        current_node->getNeighbors(neighborGetter, _collision_checker, _traverse_unknown, neighbors);

        for (neighbor_iterator = neighbors.begin();
          neighbor_iterator != neighbors.end(); ++neighbor_iterator)
        {
          neighbor = *neighbor_iterator;

          // 4.1) Compute the cost to go to this node
          //a.走到这个neighbour所耗费的cost
          g_cost = current_node->getAccumulatedCost() + current_node->getTraversalCost(neighbor);//!!!!!

          // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
          if (g_cost < neighbor->getAccumulatedCost()) {
            neighbor->setAccumulatedCost(g_cost);
            neighbor->parent = current_node;

            // 4.3) Add to queue with heuristic cost
            addNode(g_cost + getHeuristicCost(neighbor), neighbor);
          }
        }
  }//结束了while循环！！！！

  // std::pair<float, uint64_t> _best_heuristic_node;
  //5.
  if (_best_heuristic_node.first < getToleranceHeuristic()) {
    // If we run out of search options, return the path that is closest, if within tolerance.
    return _graph.at(_best_heuristic_node.second).backtracePath(path);
  }

  return false;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getStart()
{
  return _start;
}

//从优先队列中最顶层取出 node，然后将这个node从优先队列中pop出去
//然后更新这个node的信息
//最后将这个node返回
template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::getNextNode()
{
  NodeBasic<NodeT> node = _queue.top().second;
  _queue.pop();
  node.processSearchNode();//如果这个node没有被访问过，则更新这个node的信息
  return node.graph_node_ptr;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::addNode(const float & cost, NodePtr & node)
{
  NodeBasic<NodeT> queued_node(node->getIndex());
  queued_node.populateSearchNode(node);//使用node中存储的信息，更新queued_node的成语对象
  _queue.emplace(cost, queued_node);//构建一个node，向优先队列中压入！！！！
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getHeuristicCost(const NodePtr & node)
{
  const Coordinates node_coords = NodeT::getCoords(node->getIndex(), getSizeX(), getSizeDim3());
  float heuristic = NodeT::getHeuristicCost(node_coords, _goal_manager.getGoalsCoordinates());
  if (heuristic < _best_heuristic_node.first) {
    _best_heuristic_node = {heuristic, node->getIndex()};//整个代码就这里更新了_best_heuristic_node
  }

  return heuristic;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::onVisitationCheckNode(const NodePtr & current_node)
{
  return current_node->wasVisited();
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearQueue()
{
  NodeQueue q;
  std::swap(_queue, q);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearGraph()
{
  Graph g;
  std::swap(_graph, g);
  _graph.reserve(100000);
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getMaxIterations()
{
  return _max_iterations;
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getOnApproachMaxIterations()
{
  return _max_on_approach_iterations;
}

template<typename NodeT>
float & AStarAlgorithm<NodeT>::getToleranceHeuristic()
{
  return _tolerance;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeX()
{
  return _x_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeY()
{
  return _y_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeDim3()
{
  return _dim3_size;
}

template<typename NodeT>
unsigned int AStarAlgorithm<NodeT>::getCoarseSearchResolution()
{
  return _coarse_search_resolution;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::GoalManagerT AStarAlgorithm<NodeT>::getGoalManager()
{
  return _goal_manager;
}

// Instantiate algorithm for the supported template types
template class AStarAlgorithm<Node2D>;
template class AStarAlgorithm<NodeHybrid>;
template class AStarAlgorithm<NodeLattice>;

}  // namespace nav2_smac_planner

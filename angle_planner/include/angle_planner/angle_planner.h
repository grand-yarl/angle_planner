 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <dynamic_reconfigure/server.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include <tf/tf.h>
 #include <tf/transform_datatypes.h>
 #include "que.h"
 #include "step.h"
 #include "std_msgs/String.h"
 #include <iostream>
 #include <fstream>
 #include <unistd.h>
 
 #include <angle_planner/AnglePlannerConfig.h>
 
 using std::string;
 using namespace std;

 #ifndef GLOBAL_PLANNER_CPP
 #define GLOBAL_PLANNER_CPP

 namespace angle_planner {

 class AnglePlanner : public nav_core::BaseGlobalPlanner {
 protected:
     
  costmap_2d::Costmap2DROS *costmap_ros_;
  costmap_2d::Costmap2D *costmap_;
  
  unsigned long int bound_x, bound_y;
  double*** Orientation;
  int costmap_critical_;
  double orientation_critical_;
  double orientation_coeff_;
  bool use_16_;
  dynamic_reconfigure::Server<angle_planner::AnglePlannerConfig> *dsrv_;
    
  void reconfigureCB(angle_planner::AnglePlannerConfig &config, uint32_t level);
  
  double get_orientation(int prev_x, int prev_y, int next_x, int next_y);

  void create_orientation(unsigned int lim_x, unsigned int lim_y);
  
  void put_orientation();
  
  step Dijkstra_search(int start_x, int start_y, int goal_x, int goal_y);
  
  que neighbours(int current_x, int current_y);
  
  double step_cost(int prev_x, int prev_y, int next_x, int next_y);
  
  double heuristic(int current_x, int current_y, int goal_x, int goal_y);
  
  std::vector<point> structPath(step Came_From, int start_x, int start_y, int goal_x, int goal_y);
     
  void put_orientation(const std_msgs::String::ConstPtr& msg);
  
 public:

  AnglePlanner();
  
  ~AnglePlanner();
  
  AnglePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
  
  };
 };
 #endif

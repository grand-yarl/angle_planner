#include <pluginlib/class_list_macros.h>
#include <angle_planner/angle_planner.h>

//register this planner as a BaseAnglePlanner plugin
PLUGINLIB_EXPORT_CLASS(angle_planner::AnglePlanner, nav_core::BaseGlobalPlanner)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

using namespace std;

//Default Constructor
namespace angle_planner {
    
 AnglePlanner::AnglePlanner (){}

 AnglePlanner::AnglePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
 {
    initialize(name, costmap_ros);
 }

 void AnglePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
 {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    unsigned int bound_x = costmap_->getSizeInCellsX(), bound_y = costmap_->getSizeInCellsY();
   
    ros::NodeHandle nh("~/" + name);
   
    dsrv_ = new dynamic_reconfigure::Server<angle_planner::AnglePlannerConfig>(nh);
    dynamic_reconfigure::Server<angle_planner::AnglePlannerConfig>::CallbackType cb = boost::bind(&AnglePlanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
 }

 void AnglePlanner::reconfigureCB(angle_planner::AnglePlannerConfig &config, uint32_t level)
 {
    costmap_critical_ = config.costmap_critical;
    orientation_critical_ = config.orientation_critical;
    orientation_coeff_ = config.orientation_coeff;
    use_astar_ = config.use_astar;
 }

 step AnglePlanner::Dijkstra_search(int start_x, int start_y, int goal_x, int goal_y)
 {
    
    que Frontier;
    Frontier.put(start_x, start_y, 0);
    que Cost_so_Far;
    Cost_so_Far.put(start_x, start_y, 0);
    
    step Came_From;
    Came_From.put(NULL, NULL, start_x, start_y);
    point current;
    int current_x, current_y, op = 0;
    double priority;
    ROS_INFO("Start cell: %d    %d", start_x, start_y);
    sleep(1);
    ROS_INFO("Goal cell: %d    %d", goal_x, goal_y);
    sleep(1);
    
    while (!Frontier.is_empty())
    {
        current = Frontier.pop_min();
        current_x = current.coord[0];
        current_y = current.coord[1];
        use_16_ = buffer_use_16;
        if (current.cost >= 10e9)
        {
            Came_From.clear();
            break;
        }
        
        if (current_x == goal_x && current_y == goal_y)
        {
            ROS_INFO("Path is available, now it will be constructed");
            break;
        }
        
        if (heuristic(current_x, current_y, goal_x, goal_y) < sqrt(2) + 0.01)
        {
            use_16_ = false;
        }
        
        op++;
        que N = neighbours(current_x, current_y);
        point next;
        while (!N.is_empty())
        {
            double new_cost;
            next = N.pop_min();
            if (costmap_->getCost(next.coord[0], next.coord[1]) >= costmap_critical_ || get_orientation(current_x, current_y, next.coord[0], next.coord[1]) >= orientation_critical_) new_cost = 10e9;
            else new_cost = Cost_so_Far.take_cost(current_x, current_y) + costmap_->getCost(current_x, current_y) + step_cost(current_x, current_y, next.coord[0], next.coord[1]) + orientation_coeff_ * get_orientation(current_x, current_y, next.coord[0], next.coord[1]);
            if (Cost_so_Far.where_el(next.coord[0], next.coord[1]) == false || new_cost < Cost_so_Far.take_cost(next.coord[0], next.coord[1]))
            {
                if (Cost_so_Far.where_el(next.coord[0], next.coord[1]) == false) Cost_so_Far.put(next.coord[0], next.coord[1], new_cost);
                else Cost_so_Far.update(next.coord[0], next.coord[1], new_cost);
                if (use_astar_ == true) priority = new_cost + heuristic(next.coord[0], next.coord[1], goal_x, goal_y);
                else priority = new_cost;
                Frontier.put(next.coord[0], next.coord[1], priority);
                if (Came_From.where_el(next.coord[0], next.coord[1]) == false) Came_From.put(current_x, current_y, next.coord[0], next.coord[1]);
                else Came_From.update(current_x, current_y, next.coord[0], next.coord[1]);
            }
        }
    }
    return Came_From;
 }
  
 std::vector<point> AnglePlanner::structPath(step Came_From, int start_x, int start_y, int goal_x, int goal_y)
 {
     point current;
     std::vector<point> Path;
     
     current.coord[0] = goal_x;
     current.coord[1] = goal_y;
     current.cost = 0;
     
     if (Came_From.is_empty())
     {
         current.coord[0] = start_x;
         current.coord[1] = start_y;
         current.cost = 0;
         ROS_WARN("Can't reach goal point. There is no safe path");
         Path.push_back(current);
         return Path;
     }
     
     while (!(current.coord[0] == start_x && current.coord[1] == start_y))
     {
     	 
         Path.push_back(current);
         int prev_x = current.coord[0], prev_y = current.coord[1];
         Came_From.get_prev(current.coord[0], current.coord[1], current.coord);
     }
     ROS_INFO("Path ready");
     return Path;
 }
 
 que AnglePlanner::neighbours(int current_x, int current_y)
 {
     que neighbour;
     
     if (use_16_ == true)
     {
     	int i = current_x, j = current_y + 2;
     	for (int number = 0; number < 16; number++)
	{
		 if (i < bound_x && j < bound_y && i > 0 && j > 0) neighbour.put(i, j, 0);
		 if (j == current_y + 2 && i != current_x + 2)
		 {
		     i++;
		     continue;
		 }
		 if (i == current_x + 2 && j != current_y - 2)
		 {
		     j--;
		     continue;
		 }
		 if (j == current_y - 2 && i != current_x - 2)
		 {
		     i--;
		     continue;
		 }
		 if (i == current_x - 2 && j != current_y + 2)
		 {
		     j++;
		     continue;
		 }
	 }
     }
     else
     {
             int i = current_x, j = current_y + 1;
	     for (int number = 0; number < 8; number++)
	     {
		 if (i < bound_x && j < bound_y && i > 0 && j > 0) neighbour.put(i, j, 0);
		 if (j == current_y + 1 && i != current_x + 1)
		 {
		     i++;
		     continue;
		 }
		 if (i == current_x + 1 && j != current_y - 1)
		 {
		     j--;
		     continue;
		 }
		 if (j == current_y - 1 && i != current_x - 1)
		 {
		     i--;
		     continue;
		 }
		 if (i == current_x - 1 && j != current_y + 1)
		 {
		     j++;
		     continue;
		 }
	     }
     }
     return neighbour;
 }
 
 double AnglePlanner::heuristic(int current_x, int current_y, int goal_x, int goal_y)
 {
     double current_dx = (double)current_x;
     double current_dy = (double)current_y;
     double goal_dx = (double)goal_x;
     double goal_dy = (double)goal_y;
     return sqrt(((goal_dx - current_dx)*(goal_dx - current_dx)) + ((goal_dy - current_dy)*(goal_dy - current_dy)));
 }
 
 double AnglePlanner::step_cost(int prev_x, int prev_y, int next_x, int next_y)
 {
     if (use_16_ == true)
     {
        
     	if (((next_x - prev_x) == 2 || (next_x - prev_x) == -2) && (next_y == prev_y)) return costmap_->getCost((next_x + prev_x) / 2, next_y) + 2;
     	if ((next_x == prev_x) && ((next_y - prev_y) == 2 || (next_y - prev_y) == -2)) return costmap_->getCost(next_x ,(next_y + prev_y) / 2) + 2;
     	if (((next_x - prev_x) == 2 || (next_x - prev_x) == -2) && ((next_y - prev_y) == 2 || (next_y - prev_y) == -2)) return costmap_->getCost((next_x + prev_x) / 2,(next_y + prev_y) / 2) + sqrt(8);
     	if (((next_x - prev_x) == 2 || (next_x - prev_x) == -2) && ((next_y - prev_y) == 1 || (next_y - prev_y) == -1)) return (costmap_->getCost((next_x + prev_x) / 2, prev_y) + costmap_->getCost((next_x + prev_x) / 2, next_y)) / 2 + sqrt(6);
     	if (((next_x - prev_x) == 1 || (next_x - prev_x) == -1) && ((next_y - prev_y) == 2 || (next_y - prev_y) == -2)) return (costmap_->getCost(next_x,(next_y + prev_y) / 2) + costmap_->getCost(prev_x,(next_y + prev_y) / 2)) / 2 + sqrt(6);
     }
     else
     {
     	if (((next_x - prev_x) == 1 || (next_x - prev_x) == -1) && ((next_y - prev_y) == 1 || (next_y - prev_y) == -1)) return sqrt(2);
     	else return 1;
     }
 }
 
 double AnglePlanner::get_orientation(int prev_x, int prev_y, int next_x, int next_y)
 {
     if (prev_x > bound_x || prev_y > bound_y)  ROS_WARN("Cells are out of restrict bounds");
     
     for (int i = 0; i < Areas.size(); i++)
     {
     	if (prev_x >= Areas[i].min_x && prev_y >= Areas[i].min_y && prev_x <= Areas[i].max_x && prev_y <= Areas[i].max_y)
     	{
     		if (use_16_ == true)
     		{
		     if (next_x - prev_x == 2)
		     {
			 if (next_y - prev_y == 0) return Areas[i].restr_16[0];
			 if (next_y - prev_y == 1) return Areas[i].restr_16[1];
			 if (next_y - prev_y == 2) return Areas[i].restr_16[2];
			 if (next_y - prev_y == -1) return Areas[i].restr_16[15];
			 if (next_y - prev_y == -2) return Areas[i].restr_16[14];
		     }
		     if (next_x - prev_x == 1)
		     {
			 if (next_y - prev_y == 2) return Areas[i].restr_16[3];
			 if (next_y - prev_y == -2) return Areas[i].restr_16[13];
		     }
		     if (next_x - prev_x == 0)
		     {
			 if (next_y - prev_y == 2) return Areas[i].restr_16[4];
			 if (next_y - prev_y == -2) return Areas[i].restr_16[12];
		     }
		     if (next_x - prev_x == -1)
		     {
			 if (next_y - prev_y == 2) return Areas[i].restr_16[5];
			 if (next_y - prev_y == -2) return Areas[i].restr_16[11];
		     }
		     if (next_x - prev_x == -2)
		     {
			 if (next_y - prev_y == 0) return Areas[i].restr_16[8];
			 if (next_y - prev_y == 1) return Areas[i].restr_16[7];
			 if (next_y - prev_y == 2) return Areas[i].restr_16[6];
			 if (next_y - prev_y == -1) return Areas[i].restr_16[9];
			 if (next_y - prev_y == -2) return Areas[i].restr_16[10];
		     }
		}
     		else
     		{
		     if (next_x - prev_x == 1)
		     {
			 if (next_y - prev_y == 0) return Areas[i].restr_8[0];
			 if (next_y - prev_y == 1) return Areas[i].restr_8[1];
			 if (next_y - prev_y == -1) return Areas[i].restr_8[7];
		     }
		     if (next_x - prev_x == 0)
		     {
			 if (next_y - prev_y == 1) return Areas[i].restr_8[2];
			 if (next_y - prev_y == -1) return Areas[i].restr_8[6];
		     }
		     if (next_x - prev_x == -1)
		     {
			 if (next_y - prev_y == 0) return Areas[i].restr_8[4];
			 if (next_y - prev_y == 1) return Areas[i].restr_8[3];
			 if (next_y - prev_y == -1) return Areas[i].restr_8[5];
		     }
		}
	}
     }
     return 0;
 }

void AnglePlanner::get_format(const std_msgs::Bool::ConstPtr& format)
{
    buffer_use_16 = format->data;
    int number = 8;
    if (buffer_use_16 == true) number = 16;
    ROS_INFO("Got format [%d]", number);
}

void AnglePlanner::put_orientation(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Got file [%s]", msg->data.c_str());
    string filepath = msg->data.c_str();

    ifstream orient_file(filepath);

    int curr_line = 0;
    vector<int> lines;
    string line;
    double min_x, min_y, max_x, max_y;
    restr_area R;
    double restr;
    bool have_already = false;
    
    while (getline(orient_file, line))
    {
    	curr_line++;
    	int p = 0;
    	while (line[p] != '\0')
    	{
    		if (line[p] == '#') break;
    		if (line[p] == ' ') p++;
    		if ((line[p] >= '0' && line[p] <= '9') || line[p] == '-')
    		{
    			lines.push_back(curr_line);
    			break;
    		}
    	}
    }
    
    orient_file.clear();
    orient_file.seekg(0);
    
    for (int i = 1; i <= curr_line; i++)
    {
    	if (lines.size() == 0) break;
    	if (i != lines[0]) 
    	{
    	   getline(orient_file, line);
    	   continue;
    	}
    	lines.erase(lines.begin());
    
    	orient_file >> min_x;
    	orient_file >> min_y;
    	orient_file >> max_x;
    	orient_file >> max_y;
    	
    	costmap_->worldToMap(min_x, min_y, R.min_x, R.min_y);
    	costmap_->worldToMap(max_x, max_y, R.max_x, R.max_y);
    	
    	if (R.max_x > bound_x || R.max_y > bound_y)
    	{
    	   ROS_WARN("Bounds for orientation restrictions are out of map bounds in line %d file %s. This restrictions will not be applied", i, filepath);
    	   continue;
    	}
    	
    	if (buffer_use_16 == true) 
    	{
    	   for (int n = 0; n < 16; n++)
    	   {
    	   	orient_file >> R.restr_16[n];   
    	   }
    	   for (int j = 0; j < Areas.size(); j++)
    	   {
    	   	if (R.min_x == Areas[j].min_x && R.min_y == Areas[j].min_y && R.max_x == Areas[j].max_x && R.max_y == Areas[j].max_y) have_already = true; 
    	   }
    	}
    	else 
    	{
    	   for (int n = 0; n < 8; n++)
    	   {
    	   	orient_file >> R.restr_8[n];   
    	   }
    	   for (int j = 0; j < Areas.size(); j++)
    	   {
    	   	if (R.min_x == Areas[j].min_x && R.min_y == Areas[j].min_y && R.max_x == Areas[j].max_x && R.max_y == Areas[j].max_y) have_already = true; 
    	   }
    	}
    	
    	if (have_already == false) Areas.push_back(R);
    }
    cout << Areas.size();
}

 bool AnglePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
 {
 
    ros::NodeHandle n;
    
    ros::Subscriber sub1 = n.subscribe("orientation_format", 1000, &AnglePlanner::get_format, this);
    ros::Subscriber sub2 = n.subscribe("orientation_file", 1000, &AnglePlanner::put_orientation, this);
    
    float start_world_x = start.pose.position.x;
    float start_world_y = start.pose.position.y;
    
    float goal_world_x = goal.pose.position.x;
    float goal_world_y = goal.pose.position.y;
    
    unsigned int start_map_x, start_map_y, goal_map_x, goal_map_y;
    
    costmap_->worldToMap(start_world_x, start_world_y, start_map_x, start_map_y);
    costmap_->worldToMap(goal_world_x, goal_world_y, goal_map_x, goal_map_y);
    
    std::vector<point> Path = structPath(Dijkstra_search(start_map_x, start_map_y, goal_map_x, goal_map_y), start_map_x, start_map_y, goal_map_x, goal_map_y);
    
    point current;
    double current_world_x, current_world_y; 
    double previous_world_x = start_world_x, previous_world_y = start_world_y;
    geometry_msgs::PoseStamped current_pose = goal;
    
    while (!Path.empty())
    {
        current = Path[Path.size() - 1];
        Path.erase(Path.end() - 1);
        costmap_->mapToWorld(current.coord[0], current.coord[1], current_world_x, current_world_y);
        current_pose.pose.position.x = current_world_x;
        current_pose.pose.position.y = current_world_y;
        current_pose.pose.position.z = 0;
        
        double angle = atan2(current_world_y - previous_world_y, current_world_x - previous_world_x);
        current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        
        plan.push_back(current_pose);
        
        previous_world_x = current_world_x;
        previous_world_y = current_world_y;
    }

    return true;
}

};



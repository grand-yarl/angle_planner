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

 AnglePlanner::~AnglePlanner()
 {
    for (int i = 0; i < bound_x; i++)
    {
    	for (int j = 0; j < bound_y; j++)
    	{
    		delete [] Orientation[i][j];
    	}
    	delete [] Orientation[i];
    }
    delete [] Orientation;
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
   
    create_orientation(bound_x, bound_y);
    
 }

 void AnglePlanner::reconfigureCB(angle_planner::AnglePlannerConfig &config, uint32_t level)
 {
    costmap_critical_ = config.costmap_critical;
    orientation_critical_ = config.orientation_critical;
    orientation_coeff_ = config.orientation_coeff;
    use_16_ = config.use_16;
 }

 step AnglePlanner::Dijkstra_search(int start_x, int start_y, int goal_x, int goal_y)
 {
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("orientation_file", 1000, &AnglePlanner::put_orientation, this);
    
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
        op++;
        
        que N = neighbours(current_x, current_y);
        point next;
        while (!N.is_empty())
        {
            double new_cost;
            next = N.pop_min();
            if (costmap_->getCost(next.coord[0], next.coord[1]) >= costmap_critical_ || get_orientation(current_x, current_y, next.coord[0], next.coord[1]) >= orientation_critical_) new_cost = 10e9;
            else new_cost = Cost_so_Far.take_cost(current_x, current_y) + costmap_->getCost(current_x, current_y) + orientation_coeff_ * get_orientation(current_x, current_y, next.coord[0], next.coord[1]) + step_cost(current_x, current_y, next.coord[0], next.coord[1]);
            if (Cost_so_Far.where_el(next.coord[0], next.coord[1]) == false || new_cost < Cost_so_Far.take_cost(next.coord[0], next.coord[1]))
            {
                if (Cost_so_Far.where_el(next.coord[0], next.coord[1]) == false) Cost_so_Far.put(next.coord[0], next.coord[1], new_cost);
                else Cost_so_Far.update(next.coord[0], next.coord[1], new_cost);
                priority = new_cost + heuristic(current_x, current_y, goal_x, goal_y);
                Frontier.put(next.coord[0], next.coord[1], priority);
                if (Came_From.where_el(next.coord[0], next.coord[1]) == false) Came_From.put(current_x, current_y, next.coord[0], next.coord[1]);
                else Came_From.update(current_x, current_y, next.coord[0], next.coord[1]);
            }
        }
    }
    
    return Came_From;
 }
 
 que AnglePlanner::neighbours(int current_x, int current_y)
 {
     int i = current_x, j = current_y + 1;
     que neighbour;
     
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
     return neighbour;
 }

 double AnglePlanner::step_cost(int prev_x, int prev_y, int next_x, int next_y)
 {
    if (abs(prev_x - next_x) == 1 && abs(prev_y - next_y) == 1) return sqrt(2);
    else return 1;
 }
 
 double AnglePlanner::heuristic(int current_x, int current_y, int goal_x, int goal_y)
 {
     double current_dx = (double)current_x;
     double current_dy = (double)current_y;
     double goal_dx = (double)goal_x;
     double goal_dy = (double)goal_y;
     return sqrt(((goal_dx - current_dx)*(goal_dx - current_dx)) + ((goal_dy - current_dy)*(goal_dy - current_dy)));
 }
 
 
 double AnglePlanner::get_orientation(int prev_x, int prev_y, int next_x, int next_y)
 {
     if (prev_x > bound_x || prev_y > bound_y)  ROS_WARN("Cells are out of restrict bounds");
     
     if (next_y - prev_y == 1)
     {
         if (next_x - prev_x == 0) return Orientation[prev_x][prev_y][0];
         if (next_x - prev_x == -1) return Orientation[prev_x][prev_y][1];
         if (next_x - prev_x == 1) return Orientation[prev_x][prev_y][7];
     }
     if (next_y - prev_y == 0)
     {
         if (next_x - prev_x == -1) return Orientation[prev_x][prev_y][2];
         if (next_x - prev_x == 1) return Orientation[prev_x][prev_y][6];
     }
     if (next_y - prev_y == -1)
     {
         if (next_x - prev_x == 0) return Orientation[prev_x][prev_y][4];
         if (next_x - prev_x == -1) return Orientation[prev_x][prev_y][3];
         if (next_x - prev_x == 1) return Orientation[prev_x][prev_y][5];
     }
     return 0;
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

void AnglePlanner::create_orientation(unsigned int lim_x, unsigned int lim_y)
{
    Orientation = new double**[lim_x];
    for (unsigned long int i = 0; i < lim_x; i++)
    {
    	Orientation[i] = new double*[lim_y];
    	for (unsigned long int j = 0; j < lim_y; j++)
    	{
    		Orientation[i][j] = new double[8];
    		for (int m = 0; m < 8; m++)  Orientation[i][j][m] = 0;
    	}
    }
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
    unsigned int min_map_x, min_map_y, max_map_x, max_map_y;
    double restr;
    
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
    	
    	costmap_->worldToMap(min_x, min_y, min_map_x, min_map_y);
    	costmap_->worldToMap(max_x, max_y, max_map_x, max_map_y);
    	//cout << bound_x << " " << bound_y << "\n";
    	
    	if (max_map_x > bound_x || max_map_y > bound_y)
    	{
    	   ROS_WARN("Bounds for orientation restrictions are out of map bounds in line %d file %s. This restrictions will not be applied", i, filepath);
    	   continue;
    	}
    	
    	
    	for (int n = 0; n < 8; n++)
    	{
    	   orient_file >> restr;
    	   for (unsigned int i = min_map_x; i < max_map_x; i++)
    	   {
     	    	for (unsigned int j = min_map_y; j < max_map_y; j++)
     	    	{
     	    		Orientation[i][j][n] = restr;
     	    	}
    	   }   
    	}
    	
    	
    }

}

 bool AnglePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
 {
    
    
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



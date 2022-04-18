
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
using namespace std;

bool check_format(char* filepath)
{
    bool file_right = true; //this flag means errors in file
    
    ifstream orient_file(filepath);
    
    if (!orient_file.is_open())
    {
        ROS_ERROR("File %s can't be opened", filepath);
        return false;
    }
    
    int curr_line = 0;
    vector<int> lines; //this vector is used for storing number of lines with data
    string line;
    
    double min_x, min_y, max_x, max_y, restr;
    
    
    
    while (getline(orient_file, line))
    {
        curr_line++;
        int p = 0;
        int number_count = 0;
        	
        while (line[p] != '\0')
        {
            if (line[p] == '#')  //the symbol # is used for commenting
            { 
            	break;
            }
            
            if ((line[p] >= '0' && line[p] <= '9') || line[p] == '.' || line[p] == '-')
            {
            	 if (line[p] == '.' && line[p-1] == ' ')   //there should be digit before point
            	 {
            	 	ROS_ERROR("Invalid format of data in line %d file %s", curr_line, filepath);
            	 	break;
                	file_right = false;
            	 }
            	 
            	 if (line[p] == '-' && ((line[p-1] != ' ' && p != 0) || line[p+1] < '0' || line[p+1] > '9'))   //minus should be before digits
            	 {
            	 	ROS_ERROR("Invalid format of data in line %d file %s", curr_line, filepath);
            	 	break;
                	file_right = false;
            	 }
            	 
                if (line[p+1] == ' ' || line[p+1] == '#' || line[p+1] == '\0') number_count++;  //number are counted, if only there are #, space and end of line after them
                p++;
                continue;
            }
            
            if (line[p] == ' ') //numbers should be divided by space
            {
                p++;
                continue;
            }
            else
            {
                ROS_ERROR("Invalid format of data in line %d file %s", curr_line, filepath);
                file_right = false;
                break;
            }
        }
        
        if (number_count != 12 && number_count != 0) //the quantity of numbers should be 12
        {
            ROS_ERROR("Invalid number of double in line %d file %s", curr_line, filepath);
            file_right = false;
        }
        if (number_count == 12) lines.push_back(curr_line);
        
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
    	if (min_x >= max_x || min_y >= max_y)
    	{
    	    ROS_ERROR("Error of coordinate bounds in line %d file %s. The coordinate min_x and min_y should be lower than max_x and max_y", i, filepath);
    	    file_right = false;
    	}
    	
    	for (int j = 0; j < 8; j++)
    	{
    	    orient_file >> restr;
    	    if (restr < 0 || restr > 1)
    	    {
    	    	ROS_ERROR("Error of restiction format in line %d file %s. Restriction values should be double from 0 to 1", i, filepath);
    	    	file_right = false;
    	    }
    	}
    	getline(orient_file, line);
    }
    
    return file_right;
}


int main (int argc, char **argv) { 
    int i, ch;
 
    if (argc != 4) {
      ROS_ERROR("Invalid number %d of parameters for node ", argc);
      return 1;
    }
 
    
    ros::init(argc, argv, "orientation_pub");
    
    ros::NodeHandle n;
    
    ros::Publisher orientation_pub = n.advertise<std_msgs::String>("orientation_file", 1000);
    
    ros::Rate loop_rate(0.5);

    while (ros::ok())
    {
    
    	if (check_format(argv[1]) == false)
    	{
    	    return 1;
    	}
    
    	std_msgs::String msg;

    	std::stringstream ss;
    
    
    	ss << argv[1];
    	msg.data = ss.str();

    	ROS_INFO("%s", msg.data.c_str());

    	orientation_pub.publish(msg);

    	ros::spinOnce();

    	loop_rate.sleep();
    }
    return 0;
}

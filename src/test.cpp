#include <stdio.h>
#include <thread>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <smb_planner_msgs/PlannerService.h>

#include <vector>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    vector<geometry_msgs::PoseStamped> way_points;
    geometry_msgs::PoseStamped goalpose;
    vector<double> x={1,1,0,0};
    vector<double> y={0,1,0,1};
    for(int i=0;i<x.size();i++){
        goalpose.pose.position.x=0.0;
        goalpose.pose.position.y=0.0;
        way_points.push_back(goalpose);
    }
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<smb_planner_msgs::PlannerService>("/compute_global_path");
    smb_planner_msgs::PlannerService srv;
    //srv.request =1;
    if (client.call(srv))
    {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
    return 1;
    }

    return 0;
    for(auto &p:way_points){
    }
}

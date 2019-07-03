#include <stdio.h>
#include <thread>
#include <math.h>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <smb_planner_msgs/PlannerService.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <vector>

using namespace std;


int main(int argc, char **argv)
{
    // waypoints
    //vector<double> x={-16.0,-16.0,-13.0,-21.0,-3.0};
    //vector<double> y={54.0,45.0,63.0,63.0,54.0};
    // test
    //vector<double> x={-13.0,-2.0,-9.0};
    //vector<double> y={10.0,19.0,0.0};
    vector<double> x={13.0};
    vector<double> y={0.0};

    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    ros::Time::init();
    printf("node initialized _ wait 10 sec\n");
    ros::Duration(10.0).sleep();

    ros::ServiceClient client = n.serviceClient<smb_planner_msgs::PlannerService>("/compute_global_path");
    ros::ServiceClient trig = n.serviceClient<std_srvs::Empty>("/trigger_local_planner");
    smb_planner_msgs::PlannerService srv;
    std_srvs::Empty trig_srv;
    ros::Publisher traj_pub=n.advertise<nav_msgs::Path>("/mpc_trajectory",1);
    nav_msgs::Path path;

    geometry_msgs::PoseStamped pose;
    double time=0.0;
    ros::Time t=ros::Time(time);
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";
    double x_r,y_r,th_r;
    for(int i=0;i<x.size();++i){
        printf("point #%d \n",i);
        srv.request.goal_pose.pose.position.x=x[i];
        srv.request.goal_pose.pose.position.y=y[i];
        if (client.call(srv)){
            if(srv.response.success==false)
                printf("point #%d is unreachable\n",i);
                continue;
        }
        else{
            printf("Service call failed\n");
            continue;
        }
        trig.call(trig_srv);
        ros::Duration(40.0).sleep();
        printf("rotation\n");
        x_r=srv.response.x;
        y_r=srv.response.y;
        th_r=srv.response.th;
        time=0.0;
        for(double th=0;th<6.28;th+=0.4){
            ros::Time t=ros::Time(time);
            time+=5.0;
            pose.header.stamp=t;
            pose.header.frame_id="map";
            pose.pose.position.x=x_r;
            pose.pose.position.y=y_r;
            pose.pose.position.z=0.0 ;
            Eigen::Quaterniond quat(Eigen::AngleAxis<double>(fmod(th_r+th,6.28), Eigen::Vector3d(0,0,1)));
            pose.pose.orientation.x=quat.x();
            pose.pose.orientation.y=quat.y();
            pose.pose.orientation.z=quat.z();
            pose.pose.orientation.w=quat.w();
            path.poses.push_back(pose);
        }
        traj_pub.publish(path);
        ros::Duration(80.0).sleep();
    }
}

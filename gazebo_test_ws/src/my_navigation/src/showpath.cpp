#include <ros/ros.h> 
#include <ros/console.h> 
#include <nav_msgs/Path.h> 
#include <std_msgs/String.h> 
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        pub_=n_.advertise<nav_msgs::Path>("my_trajectory",1, true);
        sub_=n_.subscribe("/odom",1,&SubscribeAndPublish::callback,this);
    }
    void callback(const nav_msgs::Odometry input)
    {
        geometry_msgs::PoseStamped pose;

        pose.header.stamp=input.header.stamp;
        pose.header.frame_id="odom";
        pose.pose.position.x=input.pose.pose.position.x;
        pose.pose.position.y=input.pose.pose.position.y;
        pose.pose.position.z=input.pose.pose.position.z;
        pose.pose.orientation.x=input.pose.pose.orientation.x;
        pose.pose.orientation.y=input.pose.pose.orientation.y;
        pose.pose.orientation.z=input.pose.pose.orientation.z;
        pose.pose.orientation.w=input.pose.pose.orientation.w;

        path.header.stamp=input.header.stamp;
        path.header.frame_id="odom";
        path.poses.push_back(pose);
        pub_.publish(path);
    }
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Time time;
    nav_msgs::Path path;

};
main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");
    SubscribeAndPublish SAP;
    ros::spin();
    return 0;
}

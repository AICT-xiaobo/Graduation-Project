#include<ros/ros.h>
#include<ros/console.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>

double real_x = 0;
double real_z = 0;
double goal_x = 0;
double goal_z = 0;
double err = 0;
double last_err = 0;
double speed_x_out = 0;
double speed_z_out = 0;
double Kp = 7;
double Ki = 1.5;
double Kp_z = 4;
double Kd_z = 3;
double increment = 0;
double z_err = 0;
double z_err_last = 0;
/*
void carspeedCallback(const nav_msgs::Odometry& car)
{
        real_x = car.twist.twist.linear.x;
        real_z = car.twist.twist.angular.z;
   //	speed_z_out = goal_z;
	err = goal_x - real_x;
	increment = Kp*(err - last_err)+Ki*err;
        
	if(speed_x_out>=3&&increment>0)
		increment = 0;

	if(speed_x_out<=-3&&increment<0)
		increment = 0;

	speed_x_out += increment;
	last_err = err;
	z_err = goal_z - real_z;
        speed_z_out = Kp_z*z_err+Kd_z*z_err_last;
	z_err_last = z_err;
        if(speed_z_out>15)
	speed_z_out = 15;
	if(speed_z_out<-15)
	speed_z_out = -15;	
	ROS_INFO("goal:%f,real:%f,out:%f",goal_x,real_x,speed_x_out);
	ROS_INFO("goal_z:%f,real_z:%f,out_z:%f",goal_z,real_z,speed_z_out);

}*/
void goalspeedCallback(const geometry_msgs::Twist& goal)
{
	goal_x = goal.linear.x;
	goal_z = goal.angular.z;
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"mypid");
	ros::NodeHandle  a,b,c;
//	ros::Subscriber car_speed_sub = a.subscribe("/odom",10,carspeedCallback);
	
	ros::Subscriber goal_speed_sub = b.subscribe("/my/cmd_vel",10,goalspeedCallback);
	ros::Publisher goal_speed_pub = c.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	ros::Rate loop_rate(10.0);
	while(ros::ok())
	{
		geometry_msgs::Twist my_vel;
		
		my_vel.linear.x = goal_x;
		my_vel.angular.z = goal_z*1.2;
		goal_speed_pub.publish(my_vel);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

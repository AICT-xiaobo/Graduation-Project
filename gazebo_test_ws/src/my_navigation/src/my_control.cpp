#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

double Path_x[200] = {0.0},Path_y[200] = {0.0};
double dx = 0.0 ,dy = 0.0,dO = 0.0;
double A = 0.0 ,B = 0.0;
double sheer_speed = 0.0;
double Motor_speed = 0.0; 
double g_fP,g_fD; //7,10 0.6m//9 15 0.65
double err = 0,last_err = 0;
// x  =  cosdO  sindO * dx  
// y    -sindO  cosdO   dy
double My_point_x[10] = {0.0};
double My_point_y[10] = {0.0};
double My_point_z[10] = {0.0};
double Dis_point[200];
double goal_x = 5.4;
double goal_y = -5.4;
double car_x = 0;
double car_y = 0;
double Lfc = 0.3;
double linear = 0;
void pathInfoCallback(const nav_msgs::Path& msg)
{	
	int Lfc_index;
    double dx,dy;
    int size = msg.poses.size();
    for(int i = 0;i < size;i++)
    {
        dx = msg.poses[i].pose.position.x - car_x;
        dy = msg.poses[i].pose.position.y - car_y;
        Path_x[i] = A*dx + B*dy;
        Path_y[i] = -B*dx + A*dy;
        Dis_point[i] = sqrt(Path_x[i]*Path_x[i] + Path_y[i]*Path_y[i]);
        if(Dis_point[i] >= Lfc)
        {
            Lfc_index = i;
            break;
        }
        else
            Lfc_index = i;
 	}



			 My_point_x[0] = msg.poses[Lfc_index].pose.position.x;
			 My_point_y[0] = msg.poses[Lfc_index].pose.position.y;
             My_point_z[0] = msg.poses[Lfc_index].pose.position.z;
/*
             My_point_x[1] = msg.poses[point_B].pose.position.x;
             My_point_y[1] = msg.poses[point_B].pose.position.y;
	         My_point_z[1] = msg.poses[point_B].pose.position.z;
			 
			 My_point_x[2] = msg.poses[point_C].pose.position.x;
			 My_point_y[2] = msg.poses[point_C].pose.position.y;
			 My_point_z[2] = msg.poses[point_C].pose.position.z;
  */           
			 err = Path_y[Lfc_index];
			 double different;
			 different = err - last_err;
             sheer_speed = g_fP*err+g_fD*different;
			
			last_err = err;
			if(sheer_speed>4)//4 0.6
				sheer_speed = 4;
			if(sheer_speed<-4)
				sheer_speed = -4;
			Motor_speed = linear;
			
			double goal_car_dis;
			 goal_car_dis = sqrt((car_x-goal_x)*(car_x-goal_x)+(car_y-goal_y)*(car_y-goal_y));
			 if(goal_car_dis < 0.4)//0.3
			 {
			 	sheer_speed = 0;
				Motor_speed = 0;
				ROS_INFO("goal received!!!");
			 }
}

void goalPoseCallback(const geometry_msgs::PoseStamped& goal)
{
    goal_x = goal.pose.position.x;
    goal_y = goal.pose.position.y;
}

int main(int argc,char **argv)
{
	ros::init (argc,argv,"my_control");
	ros::NodeHandle ctrl,m,n,po1,po2,po3,a,b,c,d;
	ros::NodeHandle pn("~");
	pn.param("g_fP", g_fP, 7.0);
	pn.param("g_fD", g_fD, 10.0);
	pn.param("linear", linear, 0.5);
	pn.param("Lfc", Lfc,0.8);
	ROS_INFO("[param] g_fP: %lf", g_fP);
	ROS_INFO("[param] g_fD: %lf", g_fD);
	ros::Subscriber goal_path = ctrl.subscribe("/car_localPath", 1, pathInfoCallback);//move_base/GlobalPlanner/plan
    ros::Publisher goal_speed_pub = m.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	ros::Publisher goal_point1_pub = po1.advertise<geometry_msgs::PointStamped>("/my_point1",1);
	//ros::Publisher goal_point2_pub = po2.advertise<geometry_msgs::PointStamped>("/my_point2",1);
    //ros::Publisher goal_point3_pub = po3.advertise<geometry_msgs::PointStamped>("/my_point3",1);
	ros::ServiceClient states_client = b.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	ros::Subscriber goal_pose_sub = b.subscribe("/move_base/current_goal",1,goalPoseCallback);
	geometry_msgs::PointStamped this_pose1_stamped;
	geometry_msgs::PointStamped this_pose2_stamped;
    geometry_msgs::PointStamped this_pose3_stamped;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
    last_time = ros::Time::now();
	ros::Rate loop_rate(10.0);
	while (ros::ok())
	{
		gazebo_msgs::GetModelState model_states;
		model_states.request.model_name = "robot";
 		model_states.request.relative_entity_name = "world";
		states_client.call(model_states);
		car_x = model_states.response.pose.position.x;
		car_y = model_states.response.pose.position.y;
		float x = model_states.response.pose.orientation.x;
		float y = model_states.response.pose.orientation.y;
		float z = model_states.response.pose.orientation.z;
		float w = model_states.response.pose.orientation.w;
		double tmp,yaw;
		tf::Quaternion q(x,y,z,w);
		tf::Matrix3x3 quaternion(q);
		quaternion.getRPY(tmp,tmp,yaw);
		dO = yaw;
		A = cos(yaw);
		B = sin(yaw);
//		ROS_INFO("YAW:%lf",yaw);
		geometry_msgs::Twist my_vel;

		my_vel.linear.x = Motor_speed;  //m/s
		my_vel.angular.z = sheer_speed; //rad/s
		goal_speed_pub.publish(my_vel);
		 current_time = ros::Time::now();
	this_pose1_stamped.header.stamp = current_time;
	//this_pose2_stamped.header.stamp = current_time;
	//this_pose3_stamped.header.stamp = current_time;
	this_pose1_stamped.header.frame_id = "map";
	//this_pose2_stamped.header.frame_id = "odom";
	//this_pose3_stamped.header.frame_id = "odom";
   	this_pose1_stamped.point.x = My_point_x[0];
	this_pose1_stamped.point.y = My_point_y[0];
	this_pose1_stamped.point.z = My_point_z[0];
 /*
	this_pose2_stamped.point.x = My_point_x[1];
	this_pose2_stamped.point.y = My_point_y[1];
	this_pose2_stamped.point.z = My_point_z[1];

	this_pose3_stamped.point.x = My_point_x[2];
	this_pose3_stamped.point.y = My_point_y[2];
	this_pose3_stamped.point.z = My_point_z[2];
*/
    goal_point1_pub.publish(this_pose1_stamped);
  //  goal_point2_pub.publish(this_pose2_stamped);
   // goal_point3_pub.publish(this_pose3_stamped);
    ros::spinOnce();
    last_time = current_time;	
		loop_rate.sleep();
	}
	return 0;
}

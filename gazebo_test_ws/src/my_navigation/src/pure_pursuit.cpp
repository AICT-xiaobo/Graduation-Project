#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
double KA,KB,dO;//坐标转换系数
//模型位置
double car_x = 0;
double car_y = 0;
gazebo_msgs::GetModelState model_states;
void getState_yaw()
{
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
	KA = cos(yaw);
	KB = sin(yaw);

}
double Lfc = 0.8;//前视距离
double L = 0.15;  // 车辆轴距，单位：m
//double car_L = 0.15;//m 前轮到base_link的距离
double dt = 0.1;//S
double Path_x[200] = {0.0},Path_y[200] = {0.0};//前视路径
double Dis_point[200];//前视路径距离
double sheer_speed = 0.0;//角速度
double Motor_speed = 0.0;//线速度i
double linear = 0.0;//目标速度
//目标点位置
double goal_x = 0;
double goal_y = 0;
//前瞻点位置 rviz 显示用
double My_point_x[10] = {0.0};
double My_point_y[10] = {0.0};
double My_point_z[10] = {0.0};

void pathInfoCallback(const nav_msgs::Path& msg)
{
	int Lfc_index;
	double dx,dy;
	int size = msg.poses.size();
	for(int i = 0;i < size;i++)
	{
		dx = msg.poses[i].pose.position.x - car_x;
		dy = msg.poses[i].pose.position.y - car_y;
		Path_x[i] = KA*dx + KB*dy;
		Path_y[i] = -KB*dx + KA*dy;
		Dis_point[i] = sqrt(Path_x[i]*Path_x[i] + Path_y[i]*Path_y[i]);
		if(Dis_point[i] >= Lfc)
		{
			Lfc_index = i;
			break;
		}
		else
			Lfc_index = i;
	}
	double point_x = Path_x[Lfc_index],point_y = Path_y[Lfc_index];
	if(point_x)//防止分母为零
	{	
		int flag = 0;
		if(point_x < 0)
		{
			point_x = -point_x;//取第三四象限关于y轴的镜像

			flag = 1;
		}

		double etc = atan(point_y/point_x);
		if(flag)
		{
			if(point_y > 0)
					etc = 3.1415926 - etc;
			if(point_y < 0)
					etc = -etc - 3.1415926;
		}
	    double delta = atan2(2.0*L*sin(etc)/Lfc,1.0);
//		double path_R = car_L/(point_y/point_x);//
//		Motor_speed = 1;//
//		sheer_speed = Motor_speed*2/path_R;//
	    sheer_speed = delta/dt;
		if(sheer_speed >= 10)
				sheer_speed = 10;
		if(sheer_speed <= -10)
				sheer_speed = -10;
		Motor_speed = linear;
		ROS_INFO("sheer:%lf",sheer_speed);
	}
	//前瞻点位置 rviz 显示用
	My_point_x[0] = msg.poses[Lfc_index].pose.position.x;
    My_point_y[0] = msg.poses[Lfc_index].pose.position.y;
    My_point_z[0] = msg.poses[Lfc_index].pose.position.z;
	

	double goal_car_dis;
	goal_car_dis = sqrt((car_x-goal_x)*(car_x-goal_x)+(car_y-goal_y)*(car_y-goal_y));
	if(goal_car_dis < 0.4)
	{
		sheer_speed = 0;
		Motor_speed = 0;
		ROS_INFO("goal received!!!");
	}

}

void goalPoseCallback(const geometry_msgs::PoseStamped& goal)
{
	//获取rviz发布的目标点
	goal_x = goal.pose.position.x;
	goal_y = goal.pose.position.y;
}

int main(int argc,char **argv)
{
	ros::init (argc,argv,"pure_pursuit");
	ros::NodeHandle a,b,c,e,f,g;
	ros::Subscriber goal_path = a.subscribe("/car_localPath", 1, pathInfoCallback);
	ros::ServiceClient states_client = b.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	ros::Subscriber goal_pose_sub = c.subscribe("/move_base/current_goal",1,goalPoseCallback);
	ros::Publisher goal_point1_pub = f.advertise<geometry_msgs::PointStamped>("/my_point1",1);
	ros::Publisher goal_speed_pub = g.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	ros::NodeHandle pn("~");
	pn.param("Lfc", Lfc,0.8);
	pn.param("linear", linear,0.5);
//	pn.param("Motor_speed", Motor_speed,0.5);

	ROS_INFO("[param] Lfc: %f", Lfc);

	geometry_msgs::PointStamped this_pose1_stamped;
//发布点用
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate loop_rate(10.0);
	while (ros::ok())
	{
		//获取模型位置服务用
        model_states.request.model_name = "robot";
        model_states.request.relative_entity_name = "world";
        states_client.call(model_states);
	    
		getState_yaw();
		//发布速度指令
		geometry_msgs::Twist my_vel;
		my_vel.linear.x = Motor_speed;  //m/s
		my_vel.angular.z = sheer_speed; //rad/s
		goal_speed_pub.publish(my_vel);
		//发布前瞻点
		current_time = ros::Time::now();
		this_pose1_stamped.header.stamp = current_time;
		this_pose1_stamped.header.frame_id = "map";
		this_pose1_stamped.point.x = My_point_x[0];
		this_pose1_stamped.point.y = My_point_y[0];
		this_pose1_stamped.point.z = My_point_z[0];
		goal_point1_pub.publish(this_pose1_stamped);
		last_time = current_time;
		//接受话题用
		ros::spinOnce();
		//延时用
		loop_rate.sleep();

	}

	return 0;
}

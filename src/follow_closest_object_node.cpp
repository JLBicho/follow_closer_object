/*
	Code to target the closest object with scan

	Author: Jose Luis Millan Valbuena
*/

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class FollowClosestObject{
public:
	FollowClosestObject() : ac("move_base", true), tf2_(buffer_), target_frame("map"), tf2_filter_(point_sub_, buffer_, target_frame, 10, 0){

		// Subscribers
		point_sub_.subscribe(nh_, "closest_object", 10);
		tf2_filter_.registerCallback( boost::bind(&FollowClosestObject::msgCallback, this, _1));
		scan_sub_ = nh_.subscribe("/scan", 10, &FollowClosestObject::scanCallback, this);

		// Publishers
		target_map_pub_ = nh_.advertise<geometry_msgs::PointStamped>("closest_object_map", 10);
		target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("closest_object", 10);
		target_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("closest_object_marker", 10);

		// Initialization
		target_marker_.type = visualization_msgs::Marker::CUBE;
		target_marker_.action = visualization_msgs::Marker::ADD;

		target_marker_.pose.position.z = 0.07;
		target_marker_.pose.orientation.x = 0.0;
		target_marker_.pose.orientation.y = 0.0;
		target_marker_.pose.orientation.z = 0.0;
		target_marker_.pose.orientation.w = 1.0;

		target_marker_.scale.x = 0.1;
		target_marker_.scale.y = 0.1;
		target_marker_.scale.z = 0.1;

		target_marker_.color.a = 1.0;
		target_marker_.color.r = 0.0;
		target_marker_.color.g = 0.0;
		target_marker_.color.b = 1.0;

	}

	~FollowClosestObject(){
		ROS_INFO("Closing follow_closest_object");
	}
	
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
		float min_dist = 100;
		int index = -1;
		for (int i = 0; i < msg->ranges.size(); i++){
			if(msg->ranges[i]<min_dist && msg->ranges[i] != 0.0){
				min_dist = msg->ranges[i];
				index = i;
			}
		}

		double angle, x, y, x_goal, y_goal;
		angle = msg->angle_min + index*msg->angle_increment;
		x = min_dist*cos(angle);
		y = min_dist*sin(angle);

		x_goal = (min_dist-0.3)*cos(angle);
		y_goal = (min_dist-0.3)*sin(angle);
		
		target_position_.header = msg->header;
		target_position_.point.x = x_goal;
		target_position_.point.y = y_goal;

		target_pub_.publish(target_position_);
	}

	void msgCallback(const geometry_msgs::PointStamped::ConstPtr& point_ptr){
		try{
			buffer_.transform(*point_ptr, point_out_, target_frame);
			ROS_INFO("point in map = %f, %f\n",point_out_.point.x, point_out_.point.y);
			target_map_pub_.publish(point_out_);
			ROS_INFO("yaw = %f\n",atan2(point_ptr->point.y,point_ptr->point.x)*180/M_PI);
			sendTarget(atan2(point_ptr->point.y,point_ptr->point.x));
		}
		catch(tf2::TransformException &ex){
			ROS_ERROR("Failed %s\n", ex.what());
		}
	}

	void sendTarget(float yaw){

		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = point_out_.point.x;
		goal.target_pose.pose.position.y = point_out_.point.y;
		
		tf2::Quaternion q;
		q.setRPY(0,0,yaw);

		geometry_msgs::Quaternion q_msg;
		
		q.normalize();
		tf2::convert(q,q_msg);

		goal.target_pose.pose.orientation.x = q_msg.x;
		goal.target_pose.pose.orientation.y = q_msg.y;
		goal.target_pose.pose.orientation.z = q_msg.z;
		goal.target_pose.pose.orientation.w = q_msg.w;

		std::cout<<q_msg.x<<", "<<q_msg.y<<", "<<q_msg.z<<", "<<q_msg.w<<std::endl;

		ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Hooray, the base moved");
		}
		else{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}

	}
	
private:
	ros::NodeHandle nh_;
	ros::Publisher target_pub_;
	ros::Publisher target_map_pub_;
	ros::Publisher target_marker_pub_;
	ros::Subscriber scan_sub_;

	geometry_msgs::PointStamped target_position_;
	geometry_msgs::PointStamped point_out_;

	MoveBaseClient ac;

	visualization_msgs::Marker target_marker_;
	
	std::string target_frame;

	tf2_ros::Buffer buffer_;
	tf2_ros::TransformListener tf2_;
	geometry_msgs::TransformStamped transform;
	message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
	tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;
};


int main(int argc, char **argv){
	ros::init(argc, argv, "closest_object");
	ROS_INFO("Inicializando closest_object_node");

	FollowClosestObject fco;

	ros::spin();
	return 0;
}
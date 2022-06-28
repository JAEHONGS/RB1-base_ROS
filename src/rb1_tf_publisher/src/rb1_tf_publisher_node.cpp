#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#define PI 3.1415926

ros::Time cur_broadcasting_time_;

tf::StampedTransform rb1_base_base_link_to_velodyne;
tf::StampedTransform rb1_base_base_link_to_front_laser_link;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(50);

    tf::TransformBroadcaster broadcaster;

    tf::Transform tf_rb1_base_base_link_to_velodyne = tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0.0, 0.0, 0.85));
    tf::Transform tf_rb1_base_base_link_to_front_laser_link = tf::Transform(tf::Quaternion(0,0,0,0),tf::Vector3(0.22, 0.0, 0.2));  // quaternion ==> roll -180'

    tf_rb1_base_base_link_to_velodyne.setRotation(tf::createQuaternionFromRPY(0,0,0));
    tf_rb1_base_base_link_to_front_laser_link.setRotation(tf::createQuaternionFromRPY(-3.141592,0,0));

    while (n.ok())
    {
        cur_broadcasting_time_ = ros::Time::now();

        rb1_base_base_link_to_velodyne = tf::StampedTransform(tf_rb1_base_base_link_to_velodyne, cur_broadcasting_time_, "rb1_base_base_link", "velodyne");
        rb1_base_base_link_to_front_laser_link = tf::StampedTransform(tf_rb1_base_base_link_to_front_laser_link, cur_broadcasting_time_, "rb1_base_base_link", "front_laser_link");

        broadcaster.sendTransform(rb1_base_base_link_to_velodyne);
        broadcaster.sendTransform(rb1_base_base_link_to_front_laser_link);
        //ROS_ERROR("TF Checking");

        r.sleep();
    }

}

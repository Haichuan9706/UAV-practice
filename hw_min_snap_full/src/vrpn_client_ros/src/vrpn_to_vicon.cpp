#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>

Eigen::Vector3d pos_drone_mocap; 
Eigen::Quaterniond q_mocap;

ros::Publisher vicon_pub;

void mocap_publish();
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_mocap = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vrpn_to_vicon");
    ros::NodeHandle nh("~");

    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/uav/pose", 100, mocap_cb);

    vicon_pub = nh.advertise<geometry_msgs::TransformStamped>("/vicon/fast3_drone/fast3_drone", 10);

    ros::Rate rate(20.0);

    while (ros::ok())
    {

        ros::spinOnce();

        mocap_publish();

        rate.sleep();
    }

    return 0;
}
void mocap_publish()
{
    geometry_msgs::TransformStamped mocap_msg;

    mocap_msg.child_frame_id = "base";
    mocap_msg.transform.translation.x = pos_drone_mocap[0];
    mocap_msg.transform.translation.y = pos_drone_mocap[1];
    mocap_msg.transform.translation.z = pos_drone_mocap[2];


    mocap_msg.transform.rotation.x = q_mocap.x();
    mocap_msg.transform.rotation.y = q_mocap.y();
    mocap_msg.transform.rotation.z = q_mocap.z();
    mocap_msg.transform.rotation.w = q_mocap.w();

    mocap_msg.header.stamp = ros::Time::now();
    vicon_pub.publish(mocap_msg);
}

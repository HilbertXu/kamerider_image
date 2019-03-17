#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init (argc, argv, "tf_talker");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin (tf::Vector3(0,0,0.673));
    tf::Quaternion q;
    q.setRPY (0,1.20427,0);
    transform.setRotation(q);
    while (ros::ok())
    {
        broadcaster.sendTransform (tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/astra_depth_frame"));
        rate.sleep();
    }
    return 0;
}
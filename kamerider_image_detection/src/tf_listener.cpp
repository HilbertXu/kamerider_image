#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <iostream>
#include <fstream>
using namespace std;

const int MAX_PATH=260;
char Buffer[MAX_PATH];
using namespace tf;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Itf_listener");
	ros::NodeHandle node;

	tf::TransformListener listener;

	ros::Rate rate(1);
	
	while (node.ok())
	{
		tf::StampedTransform CurrTf;
		
		try{
		  listener.lookupTransform("/astra_depth_optical_frame", "/base_link",ros::Time(0), CurrTf);
		}
		catch (tf::TransformException &ex) {
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		  continue;
		}
		tf::Vector3 T=CurrTf.getOrigin();
		tf::Matrix3x3 R=CurrTf.getBasis();
		printf("%.17g\t%.17g\t%.17g\t%.17g\n" \
						"%.17g\t%.17g\t%.17g\t%.17g\n" \
						"%.17g\t%.17g\t%.17g\t%.17g\n" \
						"0\t0\t0\t1\n",
						R[0][0],R[0][1],R[0][2],T[0],
						R[1][0],R[1][1],R[1][2],T[1],
						R[2][0],R[2][1],R[2][2],T[2]);
		cout<<"--------------------------------------"<<endl;
		// ofstream Txt(argv[1],ios::app);
		// Txt<<Buffer;
		// Txt.close();
		 rate.sleep();
	}
	return 0;
};

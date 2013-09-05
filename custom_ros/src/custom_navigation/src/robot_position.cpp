#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <pcl/registration/icp.h>
#include <angles/angles.h>
using namespace std;



ros::Publisher pointCloudPublisher;





int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, "robot_position");
    ros::NodeHandle n;

    ros::Publisher pub1 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_1/amcl_pose", 100);

    double x = 0.0;
    double y = 0.0;

    ros::Rate rate(20);
    while(ros::ok()){
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;

        x+=0.005;
        //y+=0.005;

        pub1.publish(msg);
        cout << "x = " << x << "  y = " << y << endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


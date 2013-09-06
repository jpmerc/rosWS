#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <pcl/registration/icp.h>
#include <angles/angles.h>
using namespace std;


int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, "robot_position");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    string robot_name = "";
    nh.param("robot_name",robot_name, std::string("robot_0"));

    ros::Publisher pub1 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/" + robot_name +"/amcl_pose", 100);

    double x = 0.0;
    double y = 0.0;

    //cout << "Robot " + robot_name + " amcl_pose simulation is started" << endl;
    ROS_INFO("Robot %s amcl_pose simulation is started \n", robot_name.c_str());

    ros::Rate rate(20);
    while(ros::ok()){
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;

        if(robot_name == "robot_0"){
           x+=0.005;
        }
        else if(robot_name == "robot_1"){
            x-=0.005;
        }
        else if(robot_name == "robot_2"){
            y+=0.005;
        }
        else if(robot_name == "robot_3"){
            y-=0.005;
        }
        else if(robot_name == "robot_4"){
            x+=0.005;
            y+=0.005;
        }
        else if(robot_name == "robot_5"){
            x-=0.005;
            y-=0.005;
        }


        pub1.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


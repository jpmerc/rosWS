#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include <pcl/registration/icp.h>
#include <angles/angles.h>
using namespace std;


string robot_name = "";
ros::Subscriber sub_robot0;
ros::Subscriber sub_robot1;
ros::Subscriber sub_robot2;
ros::Subscriber sub_robot3;
ros::Subscriber sub_robot4;
ros::Subscriber sub_robot5;
ros::Publisher pointCloudPublisher;
vector<pcl::PointXYZ> vRobot_0;
vector<pcl::PointXYZ> vRobot_1;
vector<pcl::PointXYZ> vRobot_2;
vector<pcl::PointXYZ> vRobot_3;
vector<pcl::PointXYZ> vRobot_4;
vector<pcl::PointXYZ> vRobot_5;


//pcl::PointCloud<pcl::PointXYZ> pointcloud;

//Function declarations
int getRobotID();
void initSubscribersAndPublishers(ros::NodeHandle *node);
pcl::PointCloud<pcl::PointXYZ> createPointCloud();
void AddRobotPositionToCloud(pcl::PointCloud<pcl::PointXYZ> *pointcloud,const pcl::PointXYZ &pos);


//=================================== CALLBACKS ===================================
void robot_0_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose){
    pcl::PointXYZ pt;
    pt.x = pose->pose.pose.position.x;
    pt.y = pose->pose.pose.position.y;
    pt.z = 0;
    vRobot_0.push_back(pt);
}
void robot_1_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose){
    pcl::PointXYZ pt;
    pt.x = pose->pose.pose.position.x;
    pt.y = pose->pose.pose.position.y;
    pt.z = 0;
    vRobot_1.push_back(pt);
}
void robot_2_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose){
    pcl::PointXYZ pt;
    pt.x = pose->pose.pose.position.x;
    pt.y = pose->pose.pose.position.y;
    pt.z = 0;
    vRobot_2.push_back(pt);
}
void robot_3_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose){
    pcl::PointXYZ pt;
    pt.x = pose->pose.pose.position.x;
    pt.y = pose->pose.pose.position.y;
    pt.z = 0;
    vRobot_3.push_back(pt);
}
void robot_4_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose){
    pcl::PointXYZ pt;
    pt.x = pose->pose.pose.position.x;
    pt.y = pose->pose.pose.position.y;
    pt.z = 0;
    vRobot_4.push_back(pt);
}
void robot_5_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose){
    pcl::PointXYZ pt;
    pt.x = pose->pose.pose.position.x;
    pt.y = pose->pose.pose.position.y;
    pt.z = 0;
    vRobot_5.push_back(pt);
}
//=================================== FUNCTIONS ===================================
int getRobotID(){
    char c = robot_name.at(6);
    int id =  atoi(&c);
    //cout << "robot id = " << id << endl;
    return id;
}

void initSubscribersAndPublishers(ros::NodeHandle &node){
    // Subscribes to every robot
    int id = getRobotID();
    if(id != 0) sub_robot0 = node.subscribe("/robot_0/amcl_pose", 100, robot_0_callback);
    if(id != 1) sub_robot1 = node.subscribe("/robot_1/amcl_pose", 100, robot_1_callback);
    if(id != 2) sub_robot2 = node.subscribe("/robot_2/amcl_pose", 100, robot_2_callback);
    if(id != 3) sub_robot3 = node.subscribe("/robot_3/amcl_pose", 100, robot_3_callback);
    if(id != 4) sub_robot4 = node.subscribe("/robot_4/amcl_pose", 100, robot_4_callback);
    if(id != 5) sub_robot5 = node.subscribe("/robot_5/amcl_pose", 100, robot_5_callback);
    pointCloudPublisher = node.advertise<sensor_msgs::PointCloud2>("/" + robot_name  + "/robots_sensor", 100);
}

pcl::PointCloud<pcl::PointXYZ> createPointCloud(){
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    if(!vRobot_0.empty()){
        pcl::PointXYZ pos = vRobot_0.front();
        vRobot_0.erase(vRobot_0.begin());
        AddRobotPositionToCloud(&pointcloud,pos);
    }
    if(!vRobot_1.empty()){
        pcl::PointXYZ pos = vRobot_1.front();
        vRobot_1.erase(vRobot_1.begin());
        AddRobotPositionToCloud(&pointcloud,pos);
    }
    if(!vRobot_2.empty()){
        pcl::PointXYZ pos = vRobot_2.front();
        vRobot_2.erase(vRobot_2.begin());
        AddRobotPositionToCloud(&pointcloud,pos);
    }
    if(!vRobot_3.empty()){
        pcl::PointXYZ pos = vRobot_3.front();
        vRobot_3.erase(vRobot_3.begin());
        AddRobotPositionToCloud(&pointcloud,pos);
    }
    if(!vRobot_4.empty()){
        pcl::PointXYZ pos = vRobot_4.front();
        vRobot_4.erase(vRobot_4.begin());
        AddRobotPositionToCloud(&pointcloud,pos);
    }
    if(!vRobot_5.empty()){
        pcl::PointXYZ pos = vRobot_5.front();
        vRobot_5.erase(vRobot_5.begin());
        AddRobotPositionToCloud(&pointcloud,pos);
    }
    return pointcloud;
}

void AddRobotPositionToCloud(pcl::PointCloud<pcl::PointXYZ> *pointcloud,const pcl::PointXYZ &pos){
    double radius = 0.18;
    double angle = 0;
    while(angle < 360){
        pcl::PointXYZ pt;
        pt.x = pos.x + radius*cos(angles::from_degrees(angle));
        pt.y = pos.y + radius*sin(angles::from_degrees(angle));
        pt.z = 0;
        pointcloud->push_back(pt);
        angle += 2;
    }
}

void publishPointCloud(const pcl::PointCloud<pcl::PointXYZ> &pointcloud){
    sensor_msgs::PointCloud2 msg;
//    for(int i = 0; i< pointcloud.size(); i++){
//        cout << "data = " << pointcloud.at(i) << endl;
//    }

    pcl::toROSMsg(pointcloud,msg);
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    pointCloudPublisher.publish(msg);
}

int main(int argc, char **argv)
{
    //Init
    ros::init(argc, argv, "robot_obstacle");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    //Sets to which topic to listen to
    private_node.param("robot_name",robot_name, std::string("robot_0"));
    initSubscribersAndPublishers(node);




    ros::Rate rate(20);
    while(ros::ok()){
        pcl::PointCloud<pcl::PointXYZ> pc = createPointCloud();
        publishPointCloud(pc);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


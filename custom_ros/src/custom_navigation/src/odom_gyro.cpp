#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
using namespace std;


vector<nav_msgs::Odometry> odom_vector;
vector<sensor_msgs::Imu> gyro_vector;

bool odom_lock = false;
bool gyro_lock = false;
bool firstOdomMessage = true;
bool thereIsIMUData = false;


string base_frame = "odom_gyro";
string child_frame = "base_footprint";
int spinFrequency = 30;

double x = 0.0;
double y = 0.0;
double dt = 0.0;
double vx = 0.0;
geometry_msgs::Quaternion orientationQuaternion = tf::createQuaternionMsgFromYaw(0.0);
double theta = 0.0;
double angularVelocity = 0.0;
ros::Time current_time;
ros::Time last_time;

void odom_callback(const nav_msgs::OdometryConstPtr& odom){

    //Probleme ici pour les callbacks, je devrais recuperer les premiers elements et non les derniers---possible bug
    odom_lock = true;
    odom_vector.push_back(*odom);
    // printf("x = %.4f y = %.4f \n",odom->twist.twist.linear.x,odom->twist.twist.linear.y);
    //    cout << "xVel = " << odom->twist.twist.linear.x << "   yVel = " << odom->twist.twist.linear.y << endl;
    odom_lock = false;
}

void gyro_callback(const sensor_msgs::ImuConstPtr& gyro){
    gyro_lock = true;
    gyro_vector.push_back(*gyro);
    // printf("ang. velocity = %.4f \n",gyro->angular_velocity.z);
    //    cout << "angular velocity = " <<gyro->angular_velocity.z << endl;
    gyro_lock = false;
}

void printOdomMessage(nav_msgs::Odometry *odom){
    cout << " ---------------ODOMETRY MESSAGE---------------" << endl;
    cout << "Timestamp = " << std::setprecision (15) << odom ->header.stamp.toSec() << std::setprecision (6) <<  endl;
    cout << "child framd id : " << odom->child_frame_id << endl;
    cout << "header frame id : " << odom->header.frame_id << endl;
    cout << "Orientation z : " << odom->pose.pose.orientation.z << endl;
    cout << "Orientation w : " << odom->pose.pose.orientation.w << endl;
    cout << "Vitesse rotation z = " << odom->twist.twist.angular.z << endl;
    cout << "covariance = " << odom->pose.covariance.at(0) << endl;
    //cout << " ----------------------------------------------" << endl;
}

void printTransformMessage(geometry_msgs::TransformStamped *odom_trans){
    cout << " ------------------TF MESSAGE------------------" << endl;
    cout << "Timestamp = " << std::setprecision (15) << odom_trans ->header.stamp.toSec() << std::setprecision (6) << endl;
    cout << "Tx : " << odom_trans->transform.translation.x << endl;
    cout << "Ty : " << odom_trans->transform.translation.y << endl;
    cout << "Orientation z : " <<  odom_trans->transform.rotation.z << endl;
    cout << "Orientation w : " <<  odom_trans->transform.rotation.w << endl;
    //cout << " ----------------------------------------------" << endl;
}


void createTransform(geometry_msgs::TransformStamped *odom_trans){
    odom_trans->child_frame_id = child_frame;
    odom_trans->header.frame_id = base_frame;
    odom_trans->header.stamp = current_time;
    odom_trans->transform.rotation =  orientationQuaternion;
    odom_trans->transform.translation.x = x;
    odom_trans->transform.translation.y = y;
}

void createOdomMessage(nav_msgs::Odometry *msg){
    msg->child_frame_id = child_frame;
    msg->header.frame_id = base_frame;
    msg->pose.pose.orientation = orientationQuaternion;
    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->twist.twist.angular.z = angularVelocity;
}

void updateData(nav_msgs::Odometry *odom_msg, sensor_msgs::Imu *gyro){
    current_time = odom_msg->header.stamp;

    //dt initialized in case robot is already moving
    if(firstOdomMessage){
        dt = 1000000/spinFrequency;
        firstOdomMessage = false;
    }

    else{
        dt = (current_time - last_time).toSec();
    }

    //Translation (can only move in X axis)
    vx = odom_msg->twist.twist.linear.x;
    double delta_x = (vx * cos(theta)) * dt;
    double delta_y = (vx * sin(theta)) * dt;
    x += delta_x;
    y += delta_y;

    //Orientation
    if(thereIsIMUData){
        orientationQuaternion = gyro->orientation;
        angularVelocity = gyro->angular_velocity.z;
        theta = tf::getYaw(orientationQuaternion);
    }
    else{
        angularVelocity = 0.0;
    }


    last_time = current_time;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "odom_gyro");
    ros::NodeHandle n;
    ros::Subscriber odom_subscriber;
    ros::Subscriber gyro_subscriber;
    odom_subscriber = n.subscribe("/odom", 1, odom_callback);
    gyro_subscriber = n.subscribe("/adcParams/gyroscope", 1, gyro_callback);
    ros::Publisher odom_gyro_publisher = n.advertise<nav_msgs::Odometry>("/odom_gyro", 100);

    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_transform;
    odom_transform.header.frame_id = base_frame;
    odom_transform.child_frame_id = child_frame;

    while(ros::ok()){
        nav_msgs::Odometry odom_msg;
        sensor_msgs::Imu imu_msg;

        if(odom_vector.size() > 0){
            if(true){
                odom_msg = odom_vector.front();
                odom_vector.erase(odom_vector.begin());

                //Rotation part
                if(gyro_vector.size() > 0 ){
                    thereIsIMUData = true;
                    imu_msg = gyro_vector.front();
                    gyro_vector.erase(gyro_vector.begin());
                }
                else{
                    thereIsIMUData = false;
                }

                //updates global variables
                updateData(&odom_msg,&imu_msg);

                //creates odometry topic "odom_gyro"
                createOdomMessage(&odom_msg);

                //creates tf (odom_gyro -> base_footprint) message
                createTransform(&odom_transform);

                //Console print
                printOdomMessage(&odom_msg);
                printTransformMessage(&odom_transform);

                //Publish the TF and the topic
                odom_broadcaster.sendTransform(odom_transform);
                odom_gyro_publisher.publish(odom_msg);
            }
        }

        else{
            //cout << "vector is empty!" << endl;
        }

        ros::spinOnce();
        usleep(1000000/spinFrequency);
    }

    return 0;
}



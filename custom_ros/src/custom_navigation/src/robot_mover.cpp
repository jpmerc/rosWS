/* Author : Jean-Philippe Mercier, Université Laval */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <vector>
#include <boost/array.hpp>
#include <angles/angles.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_srvs/Empty.h>
#include <ros/service.h>
#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
using namespace std;

double PI = 3.14159265359;

void setPoseCovarianceMatrix(boost::array<double,36> &myArray, double x, double y, double Rz);
bool isAnglesInDegrees();
void setInitialPose(bool degrees);
void randomizePose();
void findInitialPose();
void rotateInPlace(double seconds);
void setFinalPose(bool degrees,bool Position);
void print_amcl_pose();
void move_robot_amcl();
void move_robot_odom();
vector<double> askUserForPosition(bool degrees,bool Position);
vector<double> askUserForDisplacement();
void waitForDisplacement();

ros::Publisher initPosePublisher;
ros::Publisher finalPosePublisher;
ros::Publisher robotVelocityCommandsPublisher;

ros::Subscriber amcl_pose_subscriber;
double amcl_CovX = 100000;
double amcl_CovY = 100000;
double amcl_CovRz = 100000;
double amcl_X = 0;
double amcl_Y = 0;
double amcl_Rz = 0;

double odom_X = 0;
double odom_Y = 0;
double odom_Rz = 0;

ros::Subscriber moveBase_status_subscriber;
ros::Subscriber odom_gyro_subscriber;

vector<actionlib_msgs::GoalStatus> Move_Base_Status;

void printOptions(){
    cout << endl;
    cout << "Select the option you want!" << endl;
    cout << "0) Quit" << endl;
    cout << "1) Get pose and precision" << endl;
    cout << "2) Randomize position in map" << endl;
    cout << "3) Set initial pose" << endl;
    cout << "4) Find initial pose" << endl;
    cout << "5) Set destination in map (x,y,θ)" << endl;
    cout << "6) Move robot in map (Δx,Δy,Δθ)" << endl;
    cout << "7) Move robot (forward/rotation with amcl)" << endl;
    cout << "8) Move robot (forward/rotation with odometry)" << endl;
}

int parseOptions(){
    int option = -1;
    cin >> (option);

    if(option == 0){
        return -1;
    }
    else if(option == 1){
        print_amcl_pose();
    }
    else if(option == 2){
        // Set random position
        randomizePose();
    }
    else if(option == 3){
        // Set initial pose
        bool isInDegrees = isAnglesInDegrees();
        setInitialPose(isInDegrees);
    }
    else if(option == 4){
        // Find initial Pose
        findInitialPose();
    }
    else if(option == 5){
        // Set destination (global)
        bool isInDegrees = isAnglesInDegrees();
        setFinalPose(isInDegrees,true);
    }
    else if(option == 6){
        // Set destination (differential)
        bool isInDegrees = isAnglesInDegrees();
        setFinalPose(isInDegrees,false);
    }
    else if(option == 7){
        // Move robot
        move_robot_amcl();
    }
    else if(option == 8){
        move_robot_odom();
    }
    else{
        cout << "Invalid Option!" << endl;
    }
    return 0;
}

bool isAnglesInDegrees(){
    int data;
    cout << "Orientation in : " << endl;
    cout << "1) Degrees" << endl;
    cout << "2) Radians" << endl;
    cout << "Your choice : ";
    cin >> data;
    if(data == 1) return true;
    else if(data == 2) return false;
    else return true;
}

geometry_msgs::PoseWithCovarianceStamped createPoseWithCovarianceMessage(double x,double y,double theta,double covX,double covY,double covRz){

    geometry_msgs::PoseWithCovarianceStamped msg;

    //Set Header
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    //Set Pose
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;

    setPoseCovarianceMatrix(msg.pose.covariance,covX,covY,covRz);

    //Set Orientation
    geometry_msgs::Quaternion orientationQuaternion = tf::createQuaternionMsgFromYaw(theta);
    msg.pose.pose.orientation = orientationQuaternion;

    return msg;
}

void setInitialPose(bool degrees){
    vector<double> lActualPose = askUserForPosition(degrees,true);
    double x = lActualPose[0];
    double y = lActualPose[1];
    double theta = lActualPose[2];

    double covRz = angles::from_degrees(10);
    geometry_msgs::PoseWithCovarianceStamped msg = createPoseWithCovarianceMessage(x,y,theta,0.2,0.2,covRz);

    //Publish Message on Topic
    initPosePublisher.publish(msg);
}

void findInitialPose(){
    //    double covRz = PI;
    //    geometry_msgs::PoseWithCovarianceStamped msg = createPoseWithCovarianceMessage(0,0,0,4,4,covRz);
    //    initPosePublisher.publish(msg);
    randomizePose();
    rotateInPlace(60);
    print_amcl_pose();
}

void randomizePose(){
    std_srvs::Empty srv;
    if(ros::service::call("global_localization", srv)){
        cout << "Pose randomization succeeded!" << endl;
    }
    else{
        cout << "Pose randomization failed!" << endl;
    }
}

void setPoseCovarianceMatrix(boost::array<double,36> &myArray, double x, double y, double Rz){
    for(int i=0; i<36; i++){
        myArray[i] = 0.0;
    }
    myArray[0] = x;
    myArray[7] = y;
    myArray[35] = Rz;
}

bool continueProgram(){
    cout << endl;
    cout << "Do you want to continue the program?" << endl;
    cout << "0) No" << endl;
    cout << "1) Yes" << endl;
    int answer = 0;
    cin >> answer;
    if(answer == 1) return true;
    else return false;
}

//seconds is the maximum time the robot will rotate in place to try to localize itself
void rotateInPlace(double seconds){
    geometry_msgs::Twist lTwist;
    lTwist.linear.x = 0;
    lTwist.linear.y = 0;
    lTwist.linear.z = 0;
    lTwist.angular.x = 0;
    lTwist.angular.y = 0;
    lTwist.angular.z = -1.5;

    ros::Time lTime1 = ros::Time::now();
    double angleCov = angles::from_degrees(10);
    ros::Rate rate(20);

    while(true && ros::ok()){
        //ros::spinOnce();
        cout << "amcl_covX = " << amcl_CovX << "  amcl_covY = " << amcl_CovY << endl;

        ros::Time lTime2 = ros::Time::now();
        double lTimeDiff = (lTime2 - lTime1).toSec();
        if(lTimeDiff >= seconds) break;

        if(amcl_CovX<=0.05 && amcl_CovY<=0.05 && amcl_CovRz<=angleCov) break;

        robotVelocityCommandsPublisher.publish(lTwist);
        rate.sleep();
    }
}


geometry_msgs::PoseStamped createPoseMessage(double x,double y,double theta){

    geometry_msgs::PoseStamped msg;

    //Set Header
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    //Set Pose
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0.0;

    //Set Orientation
    geometry_msgs::Quaternion orientationQuaternion = tf::createQuaternionMsgFromYaw(theta);
    msg.pose.orientation = orientationQuaternion;

    return msg;
}

void setFinalPose(bool degrees,bool Position){
    vector<double> lActualPose = askUserForPosition(degrees,Position);
    double x = lActualPose[0];
    double y = lActualPose[1];
    double theta = lActualPose[2];

    if(!Position){
        x += amcl_X;
        y += amcl_Y;
        theta += amcl_Rz;
    }

    geometry_msgs::PoseStamped msg = createPoseMessage(x,y,theta);

    //Publish Message on Topic
    finalPosePublisher.publish(msg);
    waitForDisplacement();
}

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose){
    amcl_CovX = pose->pose.covariance[0];
    amcl_CovY = pose->pose.covariance[7];
    amcl_CovRz = pose->pose.covariance[35];

    amcl_X = pose->pose.pose.position.x;
    amcl_Y = pose->pose.pose.position.y;
    amcl_Rz = tf::getYaw(pose->pose.pose.orientation);
}

void print_amcl_pose(){
    ros::spinOnce();
    cout << "Pose :  x = " << amcl_X << "  y = " << amcl_Y << "  θ = " << amcl_Rz << endl;
    cout << "Precision :  x = " << amcl_CovX << "m  y = " << amcl_CovY << "m  θ = " << amcl_CovRz <<"rads" <<endl;
}

void odom_gyro_callback(const nav_msgs::OdometryConstPtr &odom){
    odom_X = odom->pose.pose.position.x;
    odom_Y = odom->pose.pose.position.y;
    odom_Rz = tf::getYaw(odom->pose.pose.orientation);
    //cout << "X = " << odom_X << "  Y = " << odom_Y << "  Rz = " << angles::to_degrees(odom_Rz) << endl;
}



void move_robot_amcl(){
    vector<double> lActualPose = askUserForDisplacement();
    double typeOfMovement = lActualPose[0];

    double finalPoseX = 0.0;
    double finalPoseY = 0.0;
    double finalTheta = 0.0;

    bool sendCommand = false;

    if(typeOfMovement == 0){
        //Forward movement
        double fwd = lActualPose[1];
        if(fwd!=0) sendCommand = true;
        finalPoseX = amcl_X + fwd*cos(amcl_Rz);
        finalPoseY = amcl_Y + fwd*sin(amcl_Rz);
        finalTheta = amcl_Rz;
    }
    else if(typeOfMovement == 1){
        //Rotation
        double rot = lActualPose[1];
        if(rot!=0) sendCommand = true;
        finalPoseX = amcl_X;
        finalPoseY = amcl_Y;
        finalTheta = amcl_Rz + rot;
    }

    if(amcl_CovX>0.05 && amcl_CovY>0.05 && amcl_CovRz>angles::from_degrees(10)){
        cout << "WARNING : You should to try to localize the robot more precisely in the map before moving." << endl;
        //rotateInPlace(60);
    }

    if(sendCommand){
        geometry_msgs::PoseStamped msg = createPoseMessage(finalPoseX,finalPoseY,finalTheta);

        //Publish Message on Topic
        finalPosePublisher.publish(msg);
        waitForDisplacement();
    }

}

void move_robot_odom(){
    //At the end the rate drops, because there is a delay between the movement of the robot and when data is received.
    //To increase the precision of the movement, the rate is decreased when approching the target.
    //****TODO**** Check if the delay is only when the program is used on the base station or on the robot computer too.

    vector<double> lActualPose = askUserForDisplacement();
    double typeOfMovement = lActualPose[0];

    geometry_msgs::Twist lTwist;
    lTwist.linear.x = 0;
    lTwist.linear.y = 0;
    lTwist.linear.z = 0;
    lTwist.angular.x = 0;
    lTwist.angular.y = 0;
    lTwist.angular.z = 0;

    //Forward movement
    if(typeOfMovement == 0){      
        double distanceToTravel = lActualPose[1];
        if(distanceToTravel == 0) return;
        double xi = odom_X;
        double yi = odom_Y;
        double rzi = odom_Rz;

        double deltaX = 0;
        double deltaY = 0;
        double travelledDistance = 0;
        double distanceRemaining = 0;
        bool loop = true;

        ros::Rate rate(5);
        double velocity = 0.05;

        while(ros::ok() && loop){
            deltaX = odom_X - xi;
            deltaY = odom_Y - yi;
            travelledDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
            distanceRemaining = abs(distanceToTravel) - travelledDistance;

            //Set next velocity
            if(distanceRemaining <= 0.01 && distanceRemaining >= -0.01){
                sleep(1);
                deltaX = odom_X - xi;
                deltaY = odom_Y - yi;
                travelledDistance = sqrt(deltaX*deltaX + deltaY*deltaY);
                distanceRemaining = abs(distanceToTravel) - travelledDistance;
                if(distanceRemaining <= 0.01 && distanceRemaining >= -0.01){
                   loop = false;
                }
            }
            else if(distanceRemaining < -0.01){
                velocity = -0.03;
            }
            else if(distanceRemaining < 0.05){
                velocity = 0.03;
                rate = 1;
            }
            else if(distanceRemaining < 0.1){
                velocity = 0.05;
                rate = 2;
            }
            else if(distanceRemaining < 0.20){
                //if(velocity > 0.2) velocity = 0.2;
                //if(velocity >0.05) velocity -= 0.01;
                velocity = 0.1;
            }
            else{
                if(velocity<0.5) velocity += 0.005;
            }


            //Send velocity command
            lTwist.linear.x = velocity;
            if(distanceToTravel < 0){
                lTwist.linear.x = -1 * velocity;
            }
            robotVelocityCommandsPublisher.publish(lTwist);
            rate.sleep();

            cout << "Distance remaining = " << distanceRemaining << "  Velocity = " << velocity << endl;
        }

        double expectedX = xi + distanceToTravel * cos(rzi) ;
        double expectedY = yi + distanceToTravel * sin(rzi) ;
        cout << endl;
        cout << "Expected Position : (" << expectedX << " , " << expectedY << ")" << endl;
        cout << "Final Position    : (" << odom_X << " , " << odom_Y << ")" << endl;
    }

    else if(typeOfMovement == 1){
        //Rotation
        double rotationToDo = lActualPose[1];
        if(rotationToDo == 0) return;
        if(rotationToDo > angles::from_degrees(180) || rotationToDo < angles::from_degrees(-180)){
            rotationToDo = angles::normalize_angle(rotationToDo);
            //cout << "normalized angle = " << angles::to_degrees(rotationToDo) << endl;
        }
        bool negAngle = false;
        if(rotationToDo < 0) negAngle = true;
        double rzi = odom_Rz;
        double expectedRz = rzi + rotationToDo;
        ros::Rate rate(4);
        double velocity = 0.2;

        double deltaRz = 0;
        double rotationRemaining = 0;

        while(ros::ok()){
            //TODO NE FONCTIONNE PAS CORRECTEMENT POUR DES GRANDS ANGLES (Ros switch entre angles positifs et negatifs lorsque +- 180 degrés)

            //deltaRz = angles::normalize_angle_positive(odom_Rz) - angles::normalize_angle_positive(rzi);
            deltaRz = angles::shortest_angular_distance(rzi,odom_Rz);
            cout << "  OdomRZ = " << odom_Rz << endl;
            rotationRemaining = abs(rotationToDo) - abs(deltaRz);
            cout << "deltaRz = " << angles::to_degrees(deltaRz) << "  Rotation remaining = " << angles::to_degrees(rotationRemaining);

            if(rotationRemaining < angles::from_degrees(2) && rotationRemaining >= angles::from_degrees(-2)){
                sleep(1);
                deltaRz = angles::shortest_angular_distance(rzi,odom_Rz);
                rotationRemaining = abs(rotationToDo) - abs(deltaRz);
                if(rotationRemaining < angles::from_degrees(2) && rotationRemaining >= angles::from_degrees(-2)){
                   // cout << "BREAK!!!" << endl;
                    break;
                }
            }
            else if(rotationRemaining < angles::from_degrees(-1)){
                velocity = -0.3;
                //cout << "vel < -1" << endl;
            }
            else if(rotationRemaining < angles::from_degrees(10)){
                //if(velocity > 0.3) velocity = 0.3;
                //if(velocity > 0.2) velocity -= 0.01;
                velocity = 0.3;
                rate = 1;
            }
            else if(rotationRemaining < angles::from_degrees(20)){
                //if(velocity > 0.4) velocity = 0.4;
                //if(velocity > 0.3) velocity -= 0.01;
                velocity = 0.4;
                rate = 2;
            }
            else{
                if(velocity < 1.5) velocity += 0.005;
            }
            //cout << "Rotation remaining = " << angles::to_degrees(rotationRemaining) << "  Velocity = " << velocity << endl;

            //Set velocity
            lTwist.angular.z = velocity;
            if(negAngle) lTwist.angular.z = -1 * velocity;
            cout << "  Velocity = " << lTwist.angular.z << endl;
            robotVelocityCommandsPublisher.publish(lTwist);
            rate.sleep();
        }

        cout << endl;
        cout << "Initial Angle  : " << angles::to_degrees(rzi) << endl;
        cout << "Expected Angle : " << angles::to_degrees(expectedRz) << endl;
        cout << "Final Angle    : " << angles::to_degrees(odom_Rz) << endl;
    }
}

vector<double> askUserForPosition(bool degrees,bool Position){
    vector<double> lVector;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    string string_x;
    string string_y;
    string string_z;

    string unit = "(rads)";
    if(degrees) unit = "(degrees)";

    if(Position){
        string_x = "Enter X : ";
        string_y = "Enter Y : ";
        string_z = "Enter θ " + unit + " : ";
    }
    else{
        string_x = "Enter ΔX : ";
        string_y = "Enter ΔY : ";
        string_z = "Enter Δθ " + unit + " : ";
    }


    cout << string_x;
    cin >> x;
    cout << string_y;
    cin >> y;
    cout << string_z;
    cin >> theta;

    if(degrees){
        theta = angles::from_degrees(theta);
    }

    lVector.push_back(x);
    lVector.push_back(y);
    lVector.push_back(theta);

    return lVector;
}

vector<double> askUserForDisplacement(){
    vector<double> lVector;
    double data;

    int choice;
    cout << "Do you want a :" << endl;
    cout << "1) Forward displacement" << endl;
    cout << "2) Rotation" << endl;
    cout << "Your choice : ";
    cin >> choice;

    if(choice == 1){
        lVector.push_back(0); //means a forward displacement
        cout << "Enter distance of displacement : " ;
        cin >> data;
        lVector.push_back(data);
    }
    else if(choice == 2){
        lVector.push_back(1); //means a rotation
        bool degrees = isAnglesInDegrees();
        string unit = "(rads)";
        if(degrees) unit = "(degrees)";

        cout << "Enter rotation " << unit << " : " ;
        cin >> data;
        if(degrees){
            data = angles::from_degrees(data);
        }
        lVector.push_back(data);
    }
    else{
        lVector.push_back(-1);
    }

    return lVector;
}

void waitForDisplacement(){
    cout << "Robot is moving..." ;
    bool continueLoop = true;

    //If necessary, loop until we get 1 active status. Then we can wait for it to finish.

    while(continueLoop){
        for(int i=0; i<Move_Base_Status.size(); i++){
            int status = Move_Base_Status[i].status;
            //cout << "status = " << status << endl;
            //pending or active
            if(status == 0 || status == 1){
                cout << ".";
            }
            //Succeeded
            else if(status == 3){
                cout << "Finished!" << endl;
                continueLoop = false;
                break;
            }
            else{
                cout << "Cancelled!" << endl;
                continueLoop = false;
                break;
            }
        }
        sleep(2);
    }
    print_amcl_pose();
}

void move_base_status_callback(const actionlib_msgs::GoalStatusArrayConstPtr &statusArray){
    Move_Base_Status = statusArray -> status_list;
}

void spinFunction(){
    ros::Rate r(50);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_mover");
    ros::NodeHandle n;

    //Node publishers
    initPosePublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);
    robotVelocityCommandsPublisher = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 100);
    finalPosePublisher = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);

    //Node subscribers
    amcl_pose_subscriber = n.subscribe("amcl_pose", 1, amcl_pose_callback);
    moveBase_status_subscriber = n.subscribe("move_base/status", 1, move_base_status_callback);
    odom_gyro_subscriber =  n.subscribe("odom_gyro", 1, odom_gyro_callback);

    //Thread to execute the callbacks
    boost::thread spinThread(spinFunction);


    cout << "Welcome in the robot_mover program." << endl;

    //Options
    bool toContinue = true;
    while(ros::ok() && toContinue){
        printOptions();
        int lStatus = parseOptions();
        if(lStatus == -1) break;
        //toContinue = continueProgram();
    }

    spinThread.interrupt();

    return 0;
}




//Examples
/*
header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: /map
pose:
  pose:
    position:
      x: 0.108927965164
      y: -0.190945148468
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0208527155944
      w: 0.999782558486
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
*/



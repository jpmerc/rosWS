/* Author : Jean-Philippe Mercier, Université Laval */

#include <ros/ros.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>

using namespace std;
string robot_name;

void randomizePose(){
    std_srvs::Empty srv;
    if(ros::service::call("global_localization", srv)){
        cout << "Pose randomization succeeded!" << endl;
    }
    else{
        cout << "Pose randomization failed!" << endl;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_position");
    randomizePose();
    return 0;
}




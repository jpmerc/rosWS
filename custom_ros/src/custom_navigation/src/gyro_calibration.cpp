#include <ros/ros.h>
#include <custom_navigation/M_ADC.h>
#include <vector>
using namespace std;

int calib_size = 0;
double calib_sum = 0.0;
double calib_sum4x = 0.0;
bool CALIBRATION_MODE = true;

double AVG1X = 0.0;
double AVG4X = 0.0;

void dataAdcReceived(const custom_navigation::M_ADC::Ptr& msg){
    calib_sum += msg->canal5[0];
    calib_sum4x += msg->canal6[0];
    calib_size++;
    if(calib_size > 100){
        CALIBRATION_MODE = false;
        AVG1X = calib_sum / calib_size;
        AVG4X = calib_sum4x / calib_size;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gyro_calibration");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("D_ADC/data", 1000, dataAdcReceived);

    while(CALIBRATION_MODE && ros::ok()){
        ros::spinOnce();
    }

    cout << "AVG1X = " << AVG1X << "  AVG4X = " << AVG4X << endl;
    return 0;
}


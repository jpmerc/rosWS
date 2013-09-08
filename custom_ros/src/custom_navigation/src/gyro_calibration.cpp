#include <ros/ros.h>
#include <custom_navigation/M_ADC.h>
#include <vector>
#include <boost/thread.hpp>
using namespace std;


struct ADCmessages{
    double time;
    double data1x;
    double data4x;
};

int calib_size = 0;
double calib_sum = 0.0;
double calib_sum4x = 0.0;
bool CALIBRATION_REST_MODE = false;
bool CALIBRATION_MOVEMENT_MODE = false;
double AVG1X = 0.0;
double AVG4X = 0.0;
ros::Rate *rate;
string textWhileWaitingUserInput = "";
vector<ADCmessages> vRestCalibration;
vector<ADCmessages> vMoveCalibration;
vector<double> vGain1x;
vector<double> vGain4x;
double AVG_GAIN1X = 0.0;
double AVG_GAIN4X = 0.0;

void calibrateAtRest();
void restCalibration(double &data1x,double &data4x);
void calibrateRevolutions();
int getNumberOfCalibrations();
void movementCalibration();
void calculateGains();
void pressAnyKey();
void waitUser();
void pressAnyKey();
void spinFunction();

//=================================== CALLBACKS ===================================
void dataAdcReceived(const custom_navigation::M_ADC::Ptr& msg){
    struct ADCmessages data;
    data.time = msg->header.stamp.toSec();
    data.data1x = msg->canal5[0];
    data.data4x = msg->canal6[0];

    if(CALIBRATION_REST_MODE){
        vRestCalibration.push_back(data);
    }
    else if(CALIBRATION_MOVEMENT_MODE){
        vMoveCalibration.push_back(data);
    }
}

//=================================== FUNCTIONS ===================================

void calibrateAtRest(){
    cout << "The program will be doing calibration when the robot is at rest, so do not move the robot during this step." << endl;
    textWhileWaitingUserInput = "When ready to start rest calibration of the robot, press enter! ";
    waitUser();
    CALIBRATION_REST_MODE = true;
    cout << "Starting rest calibration" << endl;
    while(CALIBRATION_REST_MODE && ros::ok()){
        if(vRestCalibration.size() > 0){
            struct ADCmessages lData = vRestCalibration.front();
            vRestCalibration.erase(vRestCalibration.begin());
            restCalibration(lData.data1x,lData.data4x);
        }
        (*rate).sleep();
    }
}

void restCalibration(double &data1x,double &data4x){
    calib_sum += data1x;
    calib_sum4x += data4x;
    calib_size++;
    if(calib_size > 150){
        AVG1X = calib_sum / calib_size;
        AVG4X = calib_sum4x / calib_size;
        CALIBRATION_REST_MODE = false;
    }
}

void calibrateRevolutions(){
    int NumberOfMoveCalib = getNumberOfCalibrations();
    for(int i=0; i<NumberOfMoveCalib; i++){
        if(ros::ok()){
            textWhileWaitingUserInput = "Press enter to begin rotation calibration. ";
            waitUser();
            textWhileWaitingUserInput = "Starting to record data. Turn the robot by 360 degrees and press enter when it is completed. ";
            boost::thread myThread(pressAnyKey);
            movementCalibration();
            myThread.join();
        }
    }
    calculateGains();
}

int getNumberOfCalibrations(){
    int NumberOfMoveCalib;
    cout << "How many revolutions you want to do for calibrating the gyroscope (minimum of 3 recommended) ? " << endl;
    cin >> NumberOfMoveCalib;
    getchar();
    return NumberOfMoveCalib;
}

void movementCalibration(){
    CALIBRATION_MOVEMENT_MODE = true;
    bool firstCalibrationTime = true;
    double oldtime = 0.0;
    double dt = 0.0;
    double buffer1X = 0.0;
    double buffer4X = 0.0;
    while(CALIBRATION_MOVEMENT_MODE){
        if(vMoveCalibration.size() > 0){
            struct ADCmessages lData = vMoveCalibration.front();
            vMoveCalibration.erase(vMoveCalibration.begin());

            if(firstCalibrationTime){
                dt = 0.05;
                firstCalibrationTime = false;
            }
            else{
                dt = lData.time - oldtime;
            }
            oldtime = lData.time;
            double temp1x = (lData.data1x - AVG1X) * dt;
            double temp4x = (lData.data4x - AVG4X) * dt;
            buffer1X += temp1x;
            buffer4X += temp4x;
            //cout << "-> (" << lData.data1x << " - " << AVG1X << ") * " << dt <<" = " << temp1x;
            //cout << "  buffer1x = " << buffer1X << endl;
        }
    }
    //cout << "Gain 1x = " << 360/buffer1X << endl;
    //cout << "Gain 4x = " << 360/buffer1X << endl;
    if(buffer1X != 0) vGain1x.push_back(360/buffer1X);
    if(buffer4X != 0) vGain4x.push_back(360/buffer4X);
}

void calculateGains(){
    AVG_GAIN1X = 0.0;
    AVG_GAIN4X = 0.0;
    double size1x = vGain1x.size();
    double size4x = vGain1x.size();

    for(int i=0; i<size1x; i++){
        AVG_GAIN1X += abs(vGain1x.at(i));
    }
    if(size1x > 0){
       AVG_GAIN1X = AVG_GAIN1X / size1x;
    }
    for(int i=0; i<size4x; i++){
        AVG_GAIN4X += abs(vGain4x.at(i));
    }
    if(size4x > 0){
       AVG_GAIN4X = AVG_GAIN4X / size4x;
    }

    //cout << "size = " << size1x << "  AVG_GAIN1X = " << AVG_GAIN1X << endl;
}

void waitUser(){
    boost::thread myThread(pressAnyKey);
    myThread.join();
}

void printResults(){
    cout << "AVG1X = " << AVG1X << "  AVG4X = " << AVG4X << endl;
    cout << "GAIN1X = " << AVG_GAIN1X << "  GAIN4X = " << AVG_GAIN4X << endl;
}

//=================================== THREADS ===================================
void pressAnyKey(){
    cout << textWhileWaitingUserInput << endl;
    getchar();
    CALIBRATION_MOVEMENT_MODE = false;
}

void spinFunction(){
    ros::Rate r(20);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gyro_calibration");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("robot_0/D_ADC/data", 1000, dataAdcReceived);
    rate = new ros::Rate(20);

    boost::thread spinThread(spinFunction);
    calibrateAtRest();
    printResults();
    calibrateRevolutions();
    printResults();
    spinThread.~thread();

    return 0;
}


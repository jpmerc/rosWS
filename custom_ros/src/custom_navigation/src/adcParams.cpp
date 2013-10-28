#include <ros/ros.h>
#include <custom_navigation/S_ADC.h>
#include <custom_navigation/M_ADC.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
using namespace std;

vector<double> calculateRotation(double data1X, double data4X);

struct dataStruct{
    std_msgs::Header header;
    double time;
    double data1X;
    double data4X;
};

vector<dataStruct> dataVector;

//double AVG1X = 1.2369862;
//double AVG4X = 1.2122734;

// LE BON POUR ROBOT double AVG1X = 1.24124;
double AVG4X = 1.2183;
double AVG1X = 0.0;

double GAIN1X = 0.0;
double GAIN4X = 106.7593;
//double VARIANCE = 0.0195; //1 LSB
double VARIANCE = 0.000195; //1 LSB

//Matrices de covariance
boost::array<double,9> orientation_variance = {0,0,0,0,0,0,0,0,VARIANCE};
boost::array<double,9> angular_vel_variance = {0,0,0,0,0,0,0,0, VARIANCE};
boost::array<double,9> linear_accel_variance = {-1,0,0,0,0,0,0,0,0};

sensor_msgs::Imu imu;


double gTimeDiff = 0.0;
double gTime = 0.0;
bool gFirstTimeStamp = true;
double gOrientation1X = 0.0;
double gOrientation4X = 0.0;
double gRot1X = 0.0;
double gRot4X = 0.0;

double PI = 3.14159265359;

bool CALIBRATION_REST_MODE = true;
int calib_size = 0;
double calib_sum = 0.0;

bool isFirstNonZeroAngle = true;


//frequency : frequency in HZ
bool setParameters(int frequency){
    ros::NodeHandle n;

    //To get data from channels 5-6 only
    boost::array<bool,8> channels;
    for(int i = 0; i<8; i++){
        if(i<=3 || i>=6){
            channels[i] = false;
        }
        else{
            channels[i] = true;
        }
    }

    //Create a request to service
    ros::ServiceClient client = n.serviceClient<custom_navigation::S_ADC>("D_ADC/params");
    custom_navigation::S_ADC srv;
    srv.request.adcBufferSize = 1000;
    srv.request.adcChannels = channels;
    srv.request.adcFreqAcq = frequency;
    srv.request.adcFreqPoll = frequency;
    srv.request.subscribe = true;

    //make the request (return false if the requests does not work)
    if (client.call(srv))
    {
        ROS_INFO("ADC Parameters are set : %i", (int) srv.response.ret);
        return true;
    }

    else{
        ROS_ERROR("Failed to set ADC Parameters");
        return false;
    }
}



void updateData(){
    struct dataStruct myStruct = dataVector.front();
    dataVector.erase(dataVector.begin());

    imu.header.frame_id = myStruct.header.frame_id;
    imu.header.stamp = myStruct.header.stamp;

    if(gFirstTimeStamp){
        gTime = myStruct.time;
        gFirstTimeStamp = false;
    }

    double deltaT = myStruct.time - gTime;
    gTime = myStruct.time;

    vector<double> rotation = calculateRotation(myStruct.data1X,myStruct.data4X);
    gRot1X = rotation[0];
    gRot4X = rotation[1];

    gOrientation1X += (gRot1X * deltaT);
    gOrientation4X += (gRot4X * deltaT);




}

//Calcul de la rotation en degré
vector<double> calculateRotation(double data1X, double data4X){
    double lRot1x = GAIN1X * (data1X - AVG1X);
    double lRot4x = GAIN4X * (data4X - AVG4X);

    if(lRot1x < 1.0 && lRot1x > -1.0){
        lRot1x = 0.0;
        lRot4x = 0.0;
        isFirstNonZeroAngle = true;
    }
    else if(lRot4x < 1.0 && lRot4x > -1.0){
        lRot1x = 0.0;
        lRot4x = 0.0;
        isFirstNonZeroAngle = true;
    }
    else{
        if(isFirstNonZeroAngle){
            lRot1x = 0.0;
            lRot4x = 0.0;
            isFirstNonZeroAngle = false;
        }
    }

    vector<double> rot;
    rot.push_back(lRot1x);
    rot.push_back(lRot4x);
    return rot;
}


void dataAdcReceived(const custom_navigation::M_ADC::Ptr& msg){
    int size = (int)msg->timestamps.size();
    msg->header.frame_id = "odom_gyro";

    for (int i = 0; i < size; i++) {
        struct dataStruct myStruct;
        myStruct.header = msg->header;
        myStruct.time = msg->timestamps[i];
        myStruct.data1X = msg->canal5[i];
        myStruct.data4X = msg->canal6[i];
        dataVector.push_back(myStruct);
    }
}

void publishData(ros::Publisher *pub){
    double radAngle1X = gOrientation1X * PI / 180;
    double radRot1x = gRot1X * PI / 180;

    geometry_msgs::Quaternion quat1X = tf::createQuaternionMsgFromYaw(radAngle1X);

    geometry_msgs::Vector3 angular_velocity;
    angular_velocity.x = 0;
    angular_velocity.y = 0;
    angular_velocity.z = radRot1x;

    imu.orientation = quat1X;
    imu.angular_velocity = angular_velocity;

    pub->publish(imu);

}

void printDataToConsole(){
    cout << "Orientation (degrés) = " << gOrientation1X << endl;
    //cout << "Gain = " << GAIN1X << "  average = " << AVG1X << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "adcParams");

    //End the program if setParameters did not work
    if(!setParameters(20)){
        return -1;
    }

    ros::NodeHandle n;

    //Loading parameters
    ros::NodeHandle nh("~");
    nh.param("avg",AVG1X,1.24124);
    nh.param("gain",GAIN1X,424.0293);

    ros::Subscriber sub = n.subscribe("D_ADC/data", 1000, dataAdcReceived);
    ros::Publisher gyro_publisher = n.advertise<sensor_msgs::Imu>("adcParams/gyroscope", 1000);

    //Set covariance since it doesnt change in time
    imu.orientation_covariance = orientation_variance;
    imu.angular_velocity_covariance = angular_vel_variance;
    imu.linear_acceleration_covariance = linear_accel_variance;

    int freq = 100;
    double lSleep = 1000000/freq;
    while(ros::ok()){

        if(dataVector.size() > 0){
            updateData();
            publishData(&gyro_publisher);
            printDataToConsole();
        }
        ros::spinOnce();
        usleep(lSleep);
    }

    return 0;
}


#include <iostream>
#include <string>
#include "sigint.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "serial/serial.h"

#define loop_rate_value 100

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "imu_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);
    ros::NodeHandle nh;
    ros::NodeHandle n_param ("~");

    ros::Rate loop_rate(100);

    // Params management:
    int baud;
    bool pub_rviz_tf;
    std::string port, pub_imu_name, pub_mag_name, frame_id;

    n_param.param<std::string>("port", port, "/dev/ttyACM0");
    n_param.param<std::string>("frame_id", frame_id, "imu_link");
    n_param.param("baud", baud, 115200);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1);

    // port, baudrate, timeout in milliseconds
    serial::Serial razor(port, baud, serial::Timeout::simpleTimeout(1000));
    // Make sure serial port was closed before
    razor.close();
    // Then open it again
    razor.open();
    // Inits serial connection and tests if it worked
    if(razor.isOpen())
        ROS_INFO("Razor Serial port is connected");
    else
    {
        ROS_ERROR("Could not connect to razor");
        return -1;
    }

    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;
    /*
    Orientation covariance estimation:
    Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
    Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
    Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
    cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
    i.e. variance in yaw: 0.0025
    Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
    static roll/pitch error of 0.8%, owing to gravity orientation sensing
    error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
    so set all covariances the same.
    */
    imu_msg.orientation_covariance = {
        0.0025 , 0 , 0,
        0, 0.0025, 0,
        0, 0, 0.0025};

    /*
    # Angular velocity covariance estimation:
    # Observed gyro noise: 4 counts => 0.28 degrees/sec
    # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
    # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
    */
    imu_msg.angular_velocity_covariance = {
        0.02, 0 , 0,
        0, 0.02, 0,
        0, 0 , 0.02};

    /*
    # linear acceleration covariance estimation:
    # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
    # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
    # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
    */
    imu_msg.linear_acceleration_covariance = {
        0.04, 0, 0,
        0, 0.04, 0,
        0, 0, 0.04};

    mag_msg.magnetic_field_covariance = {
        0.05, 0, 0,
        0, 0.05, 0,
        0, 0, 0.05};

    ros::Time stamp;
    std::string data, token;
    int seq = 0;
    double accel_factor = 9.806;    // sensor reports accel as 1G (9.8m/s^2). Convert to m/s^2.
    double deg2rad = M_PI/180;

    while(ros::ok() && razor.isOpen())
    {
        stamp = ros::Time::now();
        // It reads one line of data
        data = razor.readline();

        std::istringstream ss(data);
        int dataCount = 0;

        while(std::getline(ss, token, ','))
        {
            switch(dataCount)
            {
                case 0:
                    break;
                case 1:
                    imu_msg.linear_acceleration.x = std::atof(token.c_str())* accel_factor;
                    break;
                case 2:
                    imu_msg.linear_acceleration.y = std::atof(token.c_str()) * accel_factor;
                    break;
                case 3:
                    imu_msg.linear_acceleration.z = std::atof(token.c_str()) * accel_factor;
                    break;
                case 4:
                    imu_msg.angular_velocity.x = std::atof(token.c_str())*deg2rad;
                    break;
                case 5:
                    imu_msg.angular_velocity.y = std::atof(token.c_str())*deg2rad;
                    break;
                case 6:
                    imu_msg.angular_velocity.z = std::atof(token.c_str())*deg2rad;
                    break;
                case 7:
                    mag_msg.magnetic_field.x = std::atof(token.c_str());
                    break;
                case 8:
                    mag_msg.magnetic_field.y = std::atof(token.c_str());
                    break;
                case 9:
                    mag_msg.magnetic_field.z = std::atof(token.c_str());
                    break;
            }
            dataCount++;
        }

        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = frame_id;
        imu_msg.header.seq = seq;
        imu_pub.publish(imu_msg);

        mag_msg.header.frame_id = frame_id;
        mag_msg.header.stamp = stamp;
        mag_msg.header.seq = seq;
        mag_pub.publish(mag_msg);

        seq++;

        loop_rate.sleep();
        ros::spinOnce();
    }

    if(!razor.isOpen())
    {
        ROS_ERROR("Serial port stopped working");
        razor.close();
        return -1;
    }

    razor.close();

    return 0;
}
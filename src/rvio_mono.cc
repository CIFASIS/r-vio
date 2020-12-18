/**
* This file is part of R-VIO.
*
* Copyright (C) 2019 Zheng Huai <zhuai@udel.edu> and Guoquan Huang <ghuang@udel.edu>
* For more information see <http://github.com/rpng/R-VIO> 
*
* R-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* R-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with R-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>

#include <opencv2/core/core.hpp>

#include "rvio/System.h"


class ImageGrabber
{
public:
    ImageGrabber(RVIO::System* pSys) : mpSys(pSys) {
#ifdef SAVE_TIMES
        num_tracked_frames_ = 0;
        f_track_times_.open("tracking_times_start.txt");
        f_track_times_ << std::fixed;
        f_track_times_ << std::setprecision(6);
#endif
    }

    ~ImageGrabber() {
#ifdef SAVE_TIMES
        f_track_times_.close();
#endif
    }
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    RVIO::System* mpSys;

#ifdef SAVE_TIMES
    int num_tracked_frames_;
    std::ofstream f_track_times_;
#endif

};


class ImuGrabber
{
public:
    ImuGrabber(RVIO::System* pSys) : mpSys(pSys) {}

    void GrabImu(const sensor_msgs::ImuConstPtr& msg);

    RVIO::System* mpSys;
};


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    static int lastseq = -1;
    if ((int)msg->header.seq!=lastseq+1 && lastseq!=-1)
        ROS_INFO("Image message drop! curr seq: %d expected seq: %d.", msg->header.seq, lastseq+1);

#ifdef SAVE_TIMES
    num_tracked_frames_++;
    auto const t = std::chrono::system_clock::now().time_since_epoch().count();
    f_track_times_ << num_tracked_frames_ << " " << t / 1e09 << std::endl;
#endif

    lastseq = msg->header.seq;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSys->MonoVIO(cv_ptr->image, cv_ptr->header.stamp.toSec());
}


void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr& msg)
{
    static int lastseq = -1;
    if ((int) msg->header.seq!=lastseq+1 && lastseq!=-1)
        ROS_DEBUG("IMU message drop! curr seq: %d expected seq: %d.", msg->header.seq, lastseq+1);
    lastseq = msg->header.seq;

    Eigen::Vector3d angular_velocity;
    tf::vectorMsgToEigen(msg->angular_velocity, angular_velocity);

    Eigen::Vector3d linear_acceleration;
    tf::vectorMsgToEigen(msg->linear_acceleration, linear_acceleration);

    double currtime = msg->header.stamp.toSec();

    RVIO::ImuData* pData = new RVIO::ImuData();
    pData->AngularVel = angular_velocity;
    pData->LinearAccel = linear_acceleration;
    pData->Timestamp = currtime;

    static double lasttime = -1;
    if (lasttime!=-1)
        pData->TimeInterval = currtime-lasttime;
    else
        pData->TimeInterval = 0;
    lasttime = currtime;

    mpSys->PushImuData(pData);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rvio");
    ros::start();

    RVIO::System Sys(argv[1]);

    ImageGrabber igb1(&Sys);
    ImuGrabber igb2(&Sys);

    ros::NodeHandle nodeHandler;
    ros::Subscriber image_sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb1);
    ros::Subscriber imu_sub = nodeHandler.subscribe("/imu", 100, &ImuGrabber::GrabImu, &igb2);

    ros::spin();

    ros::shutdown();

    return 0;
}

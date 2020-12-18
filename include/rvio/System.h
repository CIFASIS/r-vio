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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>

#include <boost/thread.hpp>

#include <tf/transform_broadcaster.h>

#include "Tracker.h"
#include "Updater.h"
#include "PreIntegrator.h"
#include "SensorDatabase.h"

#ifdef SAVE_TIMES
#include <fstream>
#endif


namespace RVIO
{

class System
{
public:

    System(const std::string& strSettingsFile);

    ~System();

    void initialize(const Eigen::Vector3d& w, const Eigen::Vector3d& a, const int nImuData, const bool bEnableAlignment);

    void MonoVIO(const cv::Mat& im, const double& timestamp);

    void PushImuData(ImuData* data) { mpSensorDatabase->PushImuData(data); }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    double mnImuRate;
    double mnCamTimeOffset;

    int mnSlidingWindowSize;
    int mnMinCloneStates;

    bool mbEnableAlignment;
    bool mbRecordOutputs;

    bool mbIsMoving;
    bool mbIsReady;

    double mnThresholdAngle;
    double mnThresholdDispl;

    // Gravity
    double mnGravity;

    // Sigma{g,wg,a,wa}
    double msigmaGyroNoise;
    double msigmaGyroBias;
    double msigmaAccelNoise;
    double msigmaAccelBias;

    // State and covariance
    Eigen::VectorXd xkk;
    Eigen::MatrixXd Pkk;

    // Handlers
    Tracker* mpTracker;
    Updater* mpUpdater;
    PreIntegrator* mpPreIntegrator;
    SensorDatabase* mpSensorDatabase;

    ros::Publisher mPosePub;

    // Interact with rviz
    ros::NodeHandle mSystemNode;
    ros::Publisher mPathPub;
    tf::TransformBroadcaster mTfPub;

#ifdef SAVE_TIMES
    int num_tracked_frames_;
    std::ofstream f_track_times_;
#endif

};

} // namespace RVIO

#endif

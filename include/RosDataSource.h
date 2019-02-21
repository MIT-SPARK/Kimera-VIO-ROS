/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RosDataSource.h
 * @brief  ROS wrapper
 * @author Yun Chang
 */

#pragma once

#include <string>
#include <functional>
#include <opencv2/core/core.hpp>
#include "datasource/DataSource.h"
#include "StereoImuSyncPacket.h"
#include "StereoFrame.h"
#include "VioFrontEndParams.h"
#include "ImuFrontEnd.h"

namespace VIO {
// Create Class to Manage Message Streams from ROS
	class RosDataProvider { // TODO Inherit from DataProvider?
	public:
    // Stereo VIO Pipeline
    VioPipeline stereoPipeline_;

    // Pose Estimate Publisher
    ros::Publisher posePub_;
    ros::Publisher odomPub_;

    // PIM Estimate Publisher
    ros::Publisher pimPub_;

    // IMU Filtered Data Publisher
    ros::Publisher imuPub_;

    // Velocity Estimate Publisher
    ros::Publisher velPub_;

    // IMU Bias Estimate Publishers
    ros::Publisher biasAccPub_;
    ros::Publisher biasGyrPub_;

    // Frontend and Backend Loggers
    ros::Publisher frontendPub_;
    ros::Publisher backendPub_;

    // Last Pose, Velocity and IMU Bias Estimate
    Pose3 poseEstimate_;
    Vector3 velocityEstimate_;
    Vector6 imubiasEstimate_;

    // Pre-Integration Publisher Objects
    gtsam::NavState pim_navstate_lpf_;
    int64_t pim_publisher_counter_ = 0; 

    // Declare Frame Variables
    // @ TODO: Sandro --> Remove, this is only for dataset evaluation (not live)
    int camera_counter_ = 0;

    // Initialize Pipeline
    void initializePipeline(ros::Publisher& posePub,
                            ros::Publisher& odomPub,
                            ros::Publisher& velPub,
                            ros::Publisher& biasAccPub,
                            ros::Publisher& biasGyrPub,
                            ros::Publisher& pimPub, 
                            ros::Publisher& imuPub,
                            ros::Publisher& frontendPub,
                            ros::Publisher& backendPub);

    // Callback to Get IMU Data
    void callbackIMU(const sensor_msgs::ImuConstPtr& msgIMU);

    // Callback to Get Camera Data and Process Frame
    void callbackCamAndProcessStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                const sensor_msgs::ImageConstPtr& msgRight);

    // Publisher to Publish Pose Estimates
    void publishPoseEstimate(long long sec, long long nsec);

    // Publisher to Publish Odometry Estimates
    void publishOdometryEstimate(long long sec, long long nsec);

    // Publisher to Publish PIM Odometry Estimates
    void publishPIMOdometryEstimate(long long sec, long long nsec, 
                                gtsam::NavState pim_navstate);

    // Low-Pass Filter for PIM Odometry Estimate
    gtsam::NavState lowPassFilterPIM(gtsam::NavState pim_navstate);

    // Publisher to Publish IMU Bias Estimates
    void publishBiasEstimate(long long sec, long long nsec);

    // Frontend Logger
    void frontendLogger();

    // Backend Logger
    void backendLogger();

    // ROS Spin with Max. Wait for Testing MATLAB
    void spinAndShutdown(double wait_time_max);

    // Lock Mutex
    bool checkAndLock() const {
        return mutex_.try_lock();
    }

    // Unlock Mutex
    inline void unlock() const {
        mutex_.unlock();
    }

	protected:

    // Mutex to Protect Cam Callbacks (Drop Frame If Used)
    mutable std::mutex mutex_;
	};
}; // End of VIO Namespace 
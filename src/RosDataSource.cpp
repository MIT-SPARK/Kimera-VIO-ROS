/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RosDataSource.cpp
 * @brief  ROS wrapper
 * @author Yun Chang
 */
#include "RosDataSource.h"

namespace VIO {

RosDataProvider::RosDataProvider(): 
			it_(nh_cam_),
			left_img_subscriber_(it_, left_camera_topic_, 1), // Image subscriber (left)
			right_img_subscriber_(it_, right_camera_topic_, 1), // Image subscriber (right)
			sync(sync_pol(10), left_img_subscriber_, right_img_subscriber_) {

	// Parse calibration info for camera and IMU 

  sync.registerCallback(boost::bind(&RosDataProvider::callbackCamAndProcessStereo, this, _1, _2) );
}

} // End of VIO namespace

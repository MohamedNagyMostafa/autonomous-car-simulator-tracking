#ifndef TRACKING_3D_POINTCLOUDCALIBRATION_H
#define TRACKING_3D_POINTCLOUDCALIBRATION_H

#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>
#include "../datatype/data_structure.cpp"
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>
/**
 * @class @PointCloudCalibration responsible for calibration between Camera and LiDAR sensors,
 *  and assign PointClouds for 2D detected objects.
 */
class PointCloudCalibration {

private:
    /// Calibration Matrices ///
    cv::Mat P_rect_00;
    cv::Mat R_rect_00;
    cv::Mat RT;
    tf::TransformListener *tfListener;

public:
    /**
     * @PointCloudCalibration constructor to load calibration parameters from files, and prepare
     * calibration matrices.
     * @param calibrationPath calibration files path.
     */
    explicit PointCloudCalibration(const std::string &calibrationPath);

    PointCloudCalibration();

    ~PointCloudCalibration();

    /**
     * Launch calibration between LiDAR and Camera given 3D PointClouds, and assign individual PointClouds for
     * detected objects based of certain shrink factor.
     * @param pointClouds PointCloud to be projected into 2D dimension.
     * @param objects Detected objects to assign PointClouds to them.
     * @param shrinkFactor Shrink factor to avoid other objects located in the detected bounding box.
     */
    void run(pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds, std::vector<Object *> &objects, float shrinkFactor = 0);

    void runSensorProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr pointClouds, std::vector<Object *> &objects,
                             float shrinkFactor);
};

#endif //TRACKING_3D_POINTCLOUDCALIBRATION_H

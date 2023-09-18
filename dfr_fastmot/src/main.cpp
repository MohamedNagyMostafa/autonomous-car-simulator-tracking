#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>
#include "streamer_module/ImageStream.h"
#include "streamer_module/LidarStream.h"
#include "datatype/data_structure.cpp"
#include "visualizer_module/PointCloudVisualization.h"
#include "visualizer_module/FrameVisualization.h"
#include "filter_module/PointCloudFiltering.h"
#include "object_detection_module/ObjectDetection.h"
#include "calibration_module/PointCloudCalibration.h"
#include "memory_module/MemoryManagement.h"
#include "association_module/ObjectAssociation.h"
#include "tracking_module/KF_Tracking.h"
#include "util.cpp"
#include <ros/ros.h>
#include "dfr_fastmot/TrackingMessage.h"
#include "sensor_msgs/Image.h"

using namespace std;


int main(int args, char** argv) {
    /**
     * Setup initial settings for the system.
     */

    // Stream Settings
    const string CURRENT_STREAM = "0019";
    const string CURRENT_STREAM_PATH = CURRENT_STREAM + "/";

    // Camera Images Settings
    // TODO: Change BASE_DIR to KITTI dataset directory on your machine.
    const string BASE_DIR               = "/home/mohamed-nagy/Desktop/Thesis Project/dataset/";

    const string IMAGE_DATASET_PATH     = BASE_DIR + "/Camera/training/image_02/";
    const string IMAGE_PREFIX           = ImageStream::ImageType::IMAGE_PNG; /// Image extension.
    const int FPS                       = 1000;       /// 10 images per second.
    const int ONE_SECOND                = 1000;

    // LiDAR PointCloud Settings
    const string POINTCLOUD_DATASET_PATH    = BASE_DIR +  "/LiDAR/training/velodyne/";
    const string POINTCLOUD_FILE_PREFIX     = LidarStream::FilePrefix::BIN_EXTENSION;

    // Calibration Settings
    const string CALIB_BASE_PATH            = BASE_DIR + "/LiDAR/training/calib/";
    const string CALIB_FILE_PATH            = CURRENT_STREAM + ".txt";

    // IMU Sensor Settings
    const string IMU_DATASET_PATH           = BASE_DIR + "/IMU/training/oxts/";
    const string IMU_FILE_PREFIX            = CURRENT_STREAM + ".txt";

    // Evaluation Settings
    const string LABEL_PATH                 = BASE_DIR +  "/training/label_02/";
    const string LABEL_FILE_PREFIX          = CURRENT_STREAM + ".txt";

    // Object Detection Settings
    const float YOLO_DETECTION_CONFIDENCE_THR               = 0.4;
    const float YOLO_DETECTION_NMS_THR                      = 0.6;
    const vector<ObjectDetection::Class> DETECTION_CLASSES  = {  ObjectDetection::Class::CAR, ObjectDetection::Class::TRUCK};
    const float SHRINK_FACTOR               = 0.1; /// shrink 10% of the bounding box to exclude most irrelevant point clouds

    // Kalman Filter Settings
    const float ALPHA   = 0.9;
    const float BETA    = 0.9;
    const float GAMMA   = 0.1;
    const float DELTA_T = 1./FPS;

    /// Visualization Settings
    const bool imageStreamVisual    = true;
    const bool lidarStreamVisual    = true;

    /// Forms Title ///
    const std::string imgStreamTitle            = "Camera Stream";
    const std::string lidarStreamTitle          = "LiDAR Stream";
    const std::string imgWithDetectionTitle     = "Camera Stream: Detection";

    /// Paths ///
    const string IMG_MAIN_PATH      = IMAGE_DATASET_PATH + CURRENT_STREAM_PATH;
    const string LIDAR_MAIN_PATH    = POINTCLOUD_DATASET_PATH + CURRENT_STREAM_PATH;

    auto* dataHolder = new DataFrame();

    /// Initialization ///
    ros::init(args, argv, "dfr_fastmot_tracking_data");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(1);  // 1 Hz

    ros::Publisher publisher                = nodeHandle.advertise<dfr_fastmot::TrackingMessage>("tracking_data", 10);

    // Initialize data streamers for sensors data.
    ImageStream imgStreamer(nodeHandle, dataHolder);

    LidarStream lidarStreamer(nodeHandle, dataHolder);



//        // Initialize viewer.
   PointCloudVisualization pointCloudVisualization(lidarStreamTitle);

    // Initialize object detection.
    ObjectDetection objectDetection(ObjectDetection::DetectorType::YOLO_DETECTION,
                                    YOLO_DETECTION_CONFIDENCE_THR,
                                    YOLO_DETECTION_NMS_THR,
                                    ObjectDetection::Device::GPU,
                                    DETECTION_CLASSES,
                                    false);

    // Initialize calibration for projection.
    PointCloudCalibration pointCloudCalibration;



    // Initialize memory
    MemoryManagement memoryManagement(true);

    // Initialize Kalman Filter.
    KF_Tracking KF_tracker(ALPHA, BETA, GAMMA, DELTA_T);

    int streamId = 0;

    /// Streaming ...///
    float sum = 0;

    while (ros::ok())
    {

//
//        std::cout<<"=========================\nNew data is reading, stream id: "<< streamId<<std::endl;
//
//        /// Filtering ...///
//        std::cout<<"\n(1) Point Cloud Filtering: Begin Voxel"<<std::endl;
//// TODO: Uncomment to do pointcloud filtering.
////        PointCloudFiltering::run(dataHolder->pointClouds,
////                                 dataHolder->pointCloudsFiltered,
////                                 PointCloudFiltering::FilterType::VOXEL_FILTERING);
//
//        std::cout<<"\n(1) Point Cloud Filtering: Begin Cropping"<<std::endl;
//
////        PointCloudFiltering::run(dataHolder->pointClouds,
////                                 dataHolder->pointCloudsFiltered,
////                                 PointCloudFiltering::FilterType::CROP_FILTERING);
//
//        std::cout<<"\n(1) Point Cloud Filtering: End Cropping"<<std::endl;
//
//        /// Object Detection ...///
        if(!dataHolder->imageFrame->empty())
        {
            std::cout<<"\n(2) Object Detection: Begin"<<std::endl;
            objectDetection.run(*(dataHolder->imageFrame), dataHolder->objects);
            std::cout << "\n(2) Object Detection: End" << std::endl;


            std::cout<<"\n(1) Point Cloud Filtering: Begin Voxel"<<std::endl;
            PointCloudFiltering::run(dataHolder->pointClouds,
                         dataHolder->pointCloudsFiltered,
                         PointCloudFiltering::FilterType::VOXEL_FILTERING);
            std::cout<<"\n(1) Point Cloud Filtering: End Voxel"<<std::endl;


            std::cout << "\n(3) Point Cloud Calibration & Projection: Begin" << std::endl;
            pointCloudCalibration.runSensorProjection(dataHolder->pointCloudsFiltered,
                                      dataHolder->objects,
                                      SHRINK_FACTOR);
            std::cout << "\n(3) Point Cloud Calibration & Projection: End" << std::endl;


            std::cout << "\n(4) Filtering & assigning point clouds to objects: begin" << std::endl;
            PointCloudFiltering::run(dataHolder->objects,PointCloudFiltering::FilterType::OBJECT_FILTERING);
            std::cout << "\n(4) Filtering & assigning point clouds to objects: end" << std::endl;


            std::cout << "\n(6) Object Association: Begin" << std::endl;
            auto t_start = std::chrono::high_resolution_clock::now();
            std::map<int, int> matches;
            std::cout << "# objects is " << memoryManagement.getSavedObjects().size() << std::endl;
            ObjectAssociation::runAssociation(dataHolder->objects,
                                              memoryManagement.getSavedObjects(),
                                              matches,
                                              {
                                                      ObjectAssociation::AssociationType::IMAGE_2D_ASSOCIATIONIoU,
                                                      ObjectAssociation::AssociationType::LIDAR_3D_ASSOCIATION},
                                              {100, 5.,},
                                              false
            );
            std::cout << "\n(6) Object Association: End" << std::endl;


            std::cout << "\n(7) Memory Management: Begin" << std::endl;
            /// Memory Update ..///
            memoryManagement.updateAssociatedObjects(matches, dataHolder->objects);
            std::cout << "\n(7) Memory Management: End" << std::endl;


            std::cout << "\n(8) Kalman Filter Update: Begin" << std::endl;
            KF_tracker.run({KF_Tracking::Type::KALMAN_FILTER_2D, KF_Tracking::Type::KALMAN_FILTER_3D_LIDAR},
                           memoryManagement.getSavedObjects());
            std::cout << "\n(8) Kalman Filter Update: End" << std::endl;

            auto t_end = std::chrono::high_resolution_clock::now();

            std::cout << "\n(8) Visualization: Begin" << std::endl;
            cv::Mat frameWithBoundingBox;
            FrameVisualization::addBoundingBoxes(*dataHolder->imageFrame,
                                                 frameWithBoundingBox,
                                                 dataHolder->objects,
                                                 cv::Scalar(0, 255, 0));

            FrameVisualization::addPointClouds(frameWithBoundingBox,
                                               frameWithBoundingBox,
                                               dataHolder->objects,
                                               true);

            FrameVisualization::run(frameWithBoundingBox,
                                        ONE_SECOND / FPS,
                                        imgStreamTitle);



            if(!dataHolder->pointClouds->empty())
            {
                // 3D PointCloud Representation
                pointCloudVisualization.simulatePointCloud(dataHolder->pointClouds,
                                                           lidarStreamTitle + to_string(streamId));
                pointCloudVisualization.simulate_3DBoundingBox(dataHolder->objects, streamId);

                pointCloudVisualization.showViewer();
            }


            std::cout << "\n(8) Visualization: End" << std::endl;

            dataHolder->reset();
            memoryManagement.updateObjectStatus();

            sum += std::chrono::duration<double, std::milli>(t_end - t_start).count();

        }
//
//
//            // TODO: Comment this line to block detection from YOLO.
////        Helper::addPretrainedDetection2D(streamId, dataHolder, CURRENT_STREAM);
//
//            /// PointCloud Projection ...///

//
//            /// 3D Object BoundingBox ...///

//
//
//            /// Object Association ..///


//
//
////            /// Visualization ///

////

//
////            /// Add BoundingBoxes to visualized frame.

////
////            /// Add PointClouds to visualized frame.

//
//

//            //Helper::saveTrackingData(streamId, dataHolder->objects, CURRENT_STREAM);
//
////        for(auto object: dataHolder->objects)
////        {
////            if(!object->_3dBoxLiDAR)
////            {
////                continue;
////            }
////            dfr_fastmot::TrackingMessage msg;
////            msg.frame_id    = streamId;
////            msg.tracking_id = object->trackId;
////            msg.location_3d = {
////                    static_cast<float>(object->_3dBoxLiDAR->x_min + (object->_3dBoxLiDAR->x_max -  object->_3dBoxLiDAR->x_min)/2.),
////                    static_cast<float>(object->_3dBoxLiDAR->y_min + (object->_3dBoxLiDAR->y_max -  object->_3dBoxLiDAR->y_min)/2.),
////                    static_cast<float>(object->_3dBoxLiDAR->z_min + (object->_3dBoxLiDAR->z_max -  object->_3dBoxLiDAR->z_min)/2.)
////
////                    };
////          if(ros::ok())
////          {
////
////              publisher.publish(msg);
////          }
////        }
//
//            // Clear frame information

////        if(streamId == 0) cv::waitKey(0);
////        cv::waitKey(0);
            streamId++;
//
////        imgNext = imgStreamer.hasNext() ;
////        lNext  = lidarStreamer.hasNext();
//
        ros::spinOnce();
    }

    std::cout<<"time in seconds" << sum /1000.;
    std::cout<<"stream end "<<streamId<<std::endl;
    /// Free Used MemoryManagement ///
    delete dataHolder;

    return 0;

}



#include "LidarStream.h"

LidarStream::LidarStream(ros::NodeHandle& nodeHandle, DataFrame *dataFrame)
{
    this->dataFrame = dataFrame;

    // Sensor Settings
    liderSubscriber = nodeHandle.subscribe("/catvehicle/lidar_points", 10, &LidarStream::lidarCallback, this);

}

void LidarStream::lidarCallback(const sensor_msgs::PointCloud::ConstPtr& cloudMsg )
{
    sensor_msgs::PointCloud2  pointCloud2Msg;
    sensor_msgs::convertPointCloudToPointCloud2(*cloudMsg,pointCloud2Msg);

    pcl::fromROSMsg(pointCloud2Msg, *dataFrame->pointClouds);

}

/**
 * @LidarStream constructor, initialize attributes.
 * @param path Directory of LiDAR PointCloud streams.
 * @param dataFrame Data holder for PointClouds for a certain stream.
 * @param prefix PointClouds file extension. It should be provided by @FilePrefix.
 * @param bVisual Monitoring the internal procedures of PointCloud reading,memory allocation, and storing.
 */
LidarStream::LidarStream(std::string path, DataFrame *dataFrame, std::string prefix, bool bVisual)
{
    // Check given extension.
    LidarStream::FilePrefix::acceptable(prefix);

    mainPath        = path;
    this->dataFrame = dataFrame;
    filePrefix      = prefix;
    visual          = bVisual;
    currentStreamId = 0;
}

LidarStream::~LidarStream()
{
    dataFrame->pointClouds;
}

/**
 * Read PointCloud stream from the assigned directory, allocate 4 Mb for an individual stream. PointClouds in
 * the stream are stored in @LidarPoint from @DataFrame.
 * @return True: if the given directory contains a stream of PointClouds, otherwise returns false.
 *
 */
bool LidarStream::hasNext()
{
    std::ostringstream nextRead;
    nextRead << std::setfill(_FIXED_INDEX) << std::setw(_REPEATED_WIDTH) << currentStreamId;
    currentStreamId++;

    std::string path = mainPath + nextRead.str() + filePrefix;
    /// MemoryManagement Allocation ///
    float* memoryPtr = (float*) malloc(_MEMORY_ALLOCATED_SLOTS * _MEMORY_ALLOCATED_SLOTS_SIZE);

    /// Pointers ///
    float* xPtr = memoryPtr + 0;
    float* yPtr = memoryPtr + 1;
    float* zPtr = memoryPtr + 2;
    float* rPtr = memoryPtr + 3;

    if(visual)  std::cout<<"PointCloud stream loading is trigger: [... PROCESSING]"<<std::endl;
    /// Load PointCloud ///
    FILE *stream;

    bool exist = Helper::isFileExists(path);
    if (exist)
    {
        if(visual)  std::cout<<"* A PointCloud file is found, start processing [... LOADING ...]"<< std::endl;

        stream = fopen(path.c_str(), "rb");
        unsigned long num = fread(memoryPtr, sizeof(float), _MEMORY_ALLOCATED_SLOTS, stream)/4;

        for(int32_t i = 0; i < num; i++)
        {
            dataFrame->pointClouds->push_back(pcl::PointXYZ(*xPtr, *yPtr, *zPtr));
            // Shift pointers to next input
            xPtr+=4;
            yPtr+=4;
            zPtr+=4;
            rPtr+=4;
        }
        if(visual)  std::cout<<"* PointCloud file is successfully loaded. [DONE]!"<<std::endl;

        fclose(stream);
    }
    else
    {
        // Directory is incorrect.
        if(visual)  std::cout<<"* No stream exist."<<std::endl;
    }

    return exist;
}

const std::string LidarStream::FilePrefix::BIN_EXTENSION = ".bin";
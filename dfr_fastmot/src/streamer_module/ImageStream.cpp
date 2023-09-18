#include "ImageStream.h"


ImageStream::ImageStream(ros::NodeHandle& nodeHandle,  DataFrame* dataFrame)
{
    this->nodeHandle = nodeHandle;
    this->dataFrame  = dataFrame;

    this->cameraSensorSubscriber  = nodeHandle.subscribe("/catvehicle/triclops/triclops/left/image", 10, & ImageStream::cameraSubscriberCallback, this);

}


void ImageStream::cameraSubscriberCallback(const sensor_msgs::Image& msg)
{
    // Image frame "catvehicle/triclops_link"

    cv::Mat img_data = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    *dataFrame->imageFrame  = img_data;
}

/**
 * ImageStream constructor, initialize attributes.
 * @param streamMainPath Original directory of the image stream dataset.
 * @param dataframe A pointer data holder to store the recent streamed image.
 * @param prefix Image extension, expected input should be from @struct ImageStream::DataType.
 * @param bVisual To monitor the internal procedures.
 */
ImageStream::ImageStream(std::string streamMainPath, DataFrame* dataframe, std::string prefix, bool bVisual)
{
    // Verify given extension.
    ImageStream::ImageType::acceptable(prefix);

    mainPath        = streamMainPath;
    currentStreamId = 0;
    imagePrefix     = prefix;
    visual          = bVisual;
    dataFrame       = dataframe;

}

ImageStream::~ImageStream()
{

}

/**
 * This function use the provided path to read an image stream in order and store it into dataframe pointer
 * until no images are exist.
 * @return True: No file exist. False: Image file is read correctly.
 */
bool ImageStream::hasNext()
{
    std::ostringstream nextRead;
    nextRead << std::setfill(_FIXED_INDEX) << std::setw(_REPEATED_WIDTH) << currentStreamId;
    currentStreamId++;

    // File path formation.
    std::string path = mainPath + nextRead.str() + imagePrefix;

    // Check file existence.
    bool isExist = Helper::isFileExists(path);

    if(isExist)
    {
        *dataFrame->imageFrame = cv::imread(path);

        if(visual)  std::cout<< "==== Image Stream Reader Trigger - [ ... LOADING ...]\n* An image is found, [DONE]."<<std::endl;
    }
    else
    {
        if(visual)  std::cout << "* No image is found, checkout the provided path."<<std::endl;
    }
    return isExist;
}



const std::string ImageStream::ImageType::IMAGE_PNG = ".png";
const std::string ImageStream::ImageType::IMAGE_JPG = ".jpg";

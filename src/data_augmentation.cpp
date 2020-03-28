//------------------------------------------------------------------------------
// @file: data_augmentation.cpp
// @created on: March 18th, 2019
// @author: Ivana Collado
//
// @brief:Contains the implementations for the Data Augmentation class.
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "data_augmentation.h"

// CLASS FUNCTION IMPLEMENTATION  ----------------------------------------------
DataAugmentation::DataAugmentation(){
  // Empty Body
}

DataAugmentation::~DataAugmentation(){
  // Empty body
}

// FUNCTIONS -----------------------------------------------------------------
void DataAugmentation::Read(const std::string &path){
  //Read image from path
  in_ = cv::imread( path, 1 );
  if (! in_.data ) 
    {
        std::cout << "Could not open or find the image.\n";
        //return -1; // unsuccessful
    }
  cv::namedWindow("Original",CV_WINDOW_NORMAL); 
  cv::imshow("Original",in_);
}

void DataAugmentation::AverageFilter(const int &kernel){
  out_ = in_.clone();
  cv::blur( in_, out_, cv::Size( kernel, kernel ), cv::Point(-1,-1) );
  cv::namedWindow("Average Filter",CV_WINDOW_NORMAL); 
  cv::imshow("Average Filter",out_);
  cv::imwrite("../../imgs/average.png",out_);
}

void DataAugmentation::Rotation(const double angle){
  // Get rotation matrix for rotating the image around its center in pixel coordinates
  cv::Point2f center((in_.cols-1)/2.0, (in_.rows-1)/2.0);
  cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
  // Determine bounding rectangle, center not relevant
  cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), in_.size(), angle).boundingRect2f();
  // Adjust transformation matrix
  rot.at<double>(0,2) += bbox.width/2.0 - in_.cols/2.0;
  rot.at<double>(1,2) += bbox.height/2.0 - in_.rows/2.0;
  cv::Mat dst;
  cv::warpAffine(in_, dst, rot, bbox.size());
  // Resize the image in case it get larger when rotating
  cv::resize(dst, dst, cv::Size(in_.cols, in_.rows), 0, 0, CV_INTER_LINEAR);
  cv::imshow("Rotated Image", dst);
  cv::imwrite("../../imgs/rotated.png", dst);
}
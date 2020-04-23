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

void DataAugmentation::Hue(const unsigned char min_hue, const unsigned char max_hue, const int step){
  cv::Mat dst_, Hsv2Bgr;
  cv::cvtColor(in_, dst_, CV_BGR2HSV);
  int i,j;
  // Apply Hue changes
  for(int hue_ = min_hue; hue_ <= max_hue; hue_ += step){
    for(j=0; j < dst_.rows; j++){
      for(i=0; i < dst_.cols; i++){
        // cv::Vec3b --> Vec<uchar,3> Uchar type vector of 3 elements
        // uchar --> typedef unsigned char (an unsigned 1 byte integer)
        dst_.at<cv::Vec3b>(j,i)[0] = hue_;
      }
    }
    cv::cvtColor(dst_, Hsv2Bgr, CV_HSV2BGR);
    cv::imshow("BGR_hue: "+std::to_string(hue_), Hsv2Bgr);
    cv::imwrite("../../imgs/BGR_hue:"+ std::to_string(hue_)+".png", Hsv2Bgr);
  }
}
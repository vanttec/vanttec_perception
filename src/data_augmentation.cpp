//------------------------------------------------------------------------------
// @file: data_augmentation.cpp
// @created on: March 18th, 2020
// @modified: May 18th, 2020
// @author: Ivana Collado
// @mail: ivanacollado@gmail.com
// @co-author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
// @brief: Contains the implementations for the Data Augmentation class.
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

void DataAugmentation::Hue(const u_char min_hue, const u_char max_hue, u_char step){
  out_ = in_.clone();
  cv::Mat HSV;
  cv::cvtColor(in_, HSV, CV_BGR2HSV);
  int i,j;
  // Apply Hue changes
  for(int hue = min_hue; hue <= max_hue; hue += step){
    for(j=0; j < HSV.rows; j++){
      for(i=0; i < HSV.cols; i++){
        // cv::Vec3b --> Vec<uchar,3> Uchar type vector of 3 elements
        // uchar --> typedef unsigned char (an unsigned 1 byte integer)
        HSV.at<cv::Vec3b>(j,i)[0] = hue;
      }
    }
    cv::cvtColor(HSV, out_, CV_HSV2BGR);
    cv::imshow("BGR_hue: "+std::to_string(hue), out_);
    cv::imwrite("../../imgs/BGR_hue:"+ std::to_string(hue)+".png", out_);
  }
}
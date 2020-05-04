//------------------------------------------------------------------------------
// @file: data_augmentation.cpp
// @created on: March 18th, 2019
// @author: Ivana Collado
// @co-author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
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

void DataAugmentation::GaussianBlur(const int &kernel){
  out_ = in_.clone();
  cv::GaussianBlur( in_, out_, cv::Size( kernel, kernel ), 0, 0 );
  cv::namedWindow("Gaussian Blur Filter",CV_WINDOW_NORMAL); 
  cv::imshow("Gaussian Blur Filter",out_);
  cv::imwrite("../../imgs/gaussian.png",out_);
}
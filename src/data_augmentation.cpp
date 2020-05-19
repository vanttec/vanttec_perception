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

void DataAugmentation::Scaling_ROI(const float ratio){
  if(ratio < 1){
    cv::Mat temp = in_.clone();
    cv::Size size;
    cv::Point offset;
    // Width and  height of ROI
    int width = std::round(ratio*in_.cols);
    int height = std::round(ratio*in_.rows);
    for(int i=0; i<=2; i++){
      // Get offset of the 3 ROI's based on the original image
      offset.x = std::round(in_.cols*(1-ratio)/2);
      offset.y = std::round(in_.rows*(1-ratio)/2*i);
      size.height = height;
      size.width = width;
      //Crop the ROI from original image
      cv::Rect ROI(offset, size);
      out_ = temp(ROI);  
      //Resize ROI back to the original image
      cv::resize(out_, out_, cv::Size(in_.rows, in_.cols), 
                                                    0, 0, CV_INTER_LINEAR);
      cv::namedWindow("Cropped image",CV_WINDOW_NORMAL); 
      cv::imshow("Cropped image",out_);
      switch (i){
      case 0:
        cv::imwrite("../../imgs/scale_upper_x_"+std::to_string(ratio)+".png",
                                                                        out_);
        break;
      case 1:
      cv::imwrite("../../imgs/scale_middle_x_"+std::to_string(ratio)+".png",
                                                                       out_);
        break;
      case 2:
      cv::imwrite("../../imgs/scale_lower_x_"+std::to_string(ratio)+".png",
                                                                      out_);
        break;
      default:
        break;
      }
    }
  }
}
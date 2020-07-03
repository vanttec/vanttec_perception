//------------------------------------------------------------------------------
// @file: data_augmentation.cpp
// @created on: March 18th, 2020
// @modified: July 3rd, 2020
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
  // Empty
}

DataAugmentation::~DataAugmentation(){
  // Close directory stream pointer
  closedir(dir_pointer_);
}

// FUNCTIONS -------------------------------------------------------------------
cv::Mat DataAugmentation::GetIn(){
  return in_;
}

void DataAugmentation::SetIn(cv::Mat in){
  in_ = in;
}

cv::Mat DataAugmentation::GetOut(){
  return out_;
}

void DataAugmentation::SetOut(cv::Mat out){
  out_ = out;
}

dirent* DataAugmentation::GetEntry(){
  return entry_;
}

void DataAugmentation::GetNextEntry(){
  // Read next directory entry
  entry_ = readdir(dir_pointer_);
}

void DataAugmentation::SetDirectory(const std::string path){
  // Save input directory path
  dir_path_ = path;
  dir_pointer_ = opendir(path.c_str());
  // Get first file in the directory
  entry_ = readdir(dir_pointer_);
}

void DataAugmentation::ReadEntry(const std::string extension){
  std::string filename;
  std::size_t found;
  // Gey entry_ name
  filename = entry_ -> d_name;
  // std::cout<<filename<<"\n";
  // Find filename extension
  found = filename.find(extension);
  // If the the file has the desired extension
  if(found != std::string::npos){
    //Read image from path
    in_ = cv::imread(dir_path_+filename, cv::IMREAD_COLOR);
  } else {
      std::cout << "Could not open the image.\n";
      // So we don't save thrash
      in_.data = NULL;
  }
}

void DataAugmentation::Save(const std::string path, const std::string extension, const int img_number){
  if(out_.data != NULL){
    cv::imwrite(path+std::to_string(img_number)+extension, out_);
  }
}

void DataAugmentation::ShowIn(){
  cv::namedWindow("Input: ",CV_WINDOW_NORMAL); 
  cv::imshow("Input: ", in_);
}

void DataAugmentation::ShowOut(){
  cv::namedWindow("Output: ",CV_WINDOW_NORMAL); 
  cv::imshow("Output: ", out_);
}

void DataAugmentation::GaussianBlur(const int &kernel){
  out_ = in_.clone();
  cv::GaussianBlur( in_, out_, cv::Size( kernel, kernel ), 0, 0 );
}

void DataAugmentation::Hue(const int hue){
  cv::Mat HSV;
  cv::Mat BGR;
  out_ = in_.clone();
  cv::cvtColor(in_, HSV, CV_BGR2HSV);
  // Apply Hue changes
  for(int j=0; j < HSV.rows; j++){
    for(int i=0; i < HSV.cols; i++){
      // cv::Vec3b --> Vec<uchar,3> Uchar type vector of 3 elements
      // uchar --> typedef unsigned char (an unsigned 1 byte integer)
      HSV.at<cv::Vec3b>(j,i)[0] = hue;
    }
  }
  cv::cvtColor(HSV, out_, CV_HSV2BGR);
}

void DataAugmentation::SaltPepper(const float percentage){
  out_ = in_.clone();
  int total = percentage * out_.cols * out_.rows;
  int column_pixel;
  int row_pixel;
  int color;
  for(int i = 0; i<total; i++){
    //Choose a random pixel between 0 and (out_.cols-1)
    //and 0 and (out_.rows-1)
    column_pixel = rand()%out_.cols;
    row_pixel = rand()%out_.rows;
    //Randomly change pixel color to black or white
    color = rand()%2 ? 255:0;
    //Apply color
    if(out_.channels() == 1){
        out_.at<uchar>(row_pixel,column_pixel) = color;
    }
    else if(out_.channels() == 3){
        out_.at<cv::Vec3b>(row_pixel,column_pixel)[0] = color;
        out_.at<cv::Vec3b>(row_pixel,column_pixel)[1] = color;
        out_.at<cv::Vec3b>(row_pixel,column_pixel)[2] = color;
    }
  }
}

std::vector<cv::Mat> DataAugmentation::ScalingROI(const float ratio){
  std::vector<cv::Mat> img = {};
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
      img.push_back(temp(ROI));  
      //Resize ROI back to the original image
      cv::resize(img.back(), img.back(), cv::Size(in_.rows, in_.cols), 
                                                    0, 0, CV_INTER_LINEAR);
    }
  }
  return img;
}

void DataAugmentation::ContrastBrightness(const float contrast, const int brightness){
  // Full explanation can be found in: 
  // https://docs.opencv.org/3.4/d3/dc1/tutorial_basic_linear_transform.html
  int i;
  int j;
  int c;
  out_ = cv::Mat::zeros( in_.size(), in_.type() ) ;
  for(j = 0; j < in_.rows; j++ ) {
      for(i = 0; i < in_.cols; i++ ) {
          for(c = 0; c < in_.channels(); c++ ) {
              out_.at<cv::Vec3b>(j,i)[c] =
              cv::saturate_cast<uchar>( contrast*in_.at<cv::Vec3b>(j,i)[c] + brightness);
          }
      }
  }
}
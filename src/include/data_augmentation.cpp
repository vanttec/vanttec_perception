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
#include <time.h>

// CLASS FUNCTION IMPLEMENTATION  ----------------------------------------------
DataAugmentation::DataAugmentation(){
  // Random seed
  srand(time(NULL));
}

DataAugmentation::~DataAugmentation(){
  // Empty body
}

// FUNCTIONS -------------------------------------------------------------------
cv::Mat DataAugmentation::getIN(){
  return in_;
}

void DataAugmentation::setIN(cv::Mat in){
  in_ = in;
}

std::vector<cv::Mat> DataAugmentation::getOUT(){
  return out_;
}

void DataAugmentation::setOUT(std::vector<cv::Mat> out){
  out_ = out;
}

void DataAugmentation::PopBack(){
  out_.pop_back();
}


void DataAugmentation::Read(const std::string &path){
  //Read image from path
  in_ = cv::imread(path, 1);
  if (! in_.data ) 
    {
        std::cout << "Could not open or find the image.\n";
        //return -1; // unsuccessful
    }
  // cv::namedWindow("Original",CV_WINDOW_NORMAL); 
  // cv::imshow("Original",in_);
}

void DataAugmentation::GaussianBlur(const int &kernel){
  out_.push_back(in_.clone());
  cv::GaussianBlur( in_, out_.back(), cv::Size( kernel, kernel ), 0, 0 );
  // cv::namedWindow("Gaussian Blur Filter",CV_WINDOW_NORMAL); 
  // cv::imshow("Gaussian Blur Filter",out_);
  // cv::imwrite("../../Filtered_imgs/gaussian.jpg",out_);
}

void DataAugmentation::Hue(const int hue){
  cv::Mat HSV, BGR;
  cv::cvtColor(in_, HSV, CV_BGR2HSV);
  int i,j;
  // Apply Hue changes
  for(j=0; j < HSV.rows; j++){
    for(i=0; i < HSV.cols; i++){
      // cv::Vec3b --> Vec<uchar,3> Uchar type vector of 3 elements
      // uchar --> typedef unsigned char (an unsigned 1 byte integer)
      HSV.at<cv::Vec3b>(j,i)[0] = hue;
    }
  }
  out_.push_back(BGR);
  cv::cvtColor(HSV, out_.back(), CV_HSV2BGR);
  // cv::imshow("BGR_hue: "+std::to_string(hue), out_[k++]);
  // cv::imwrite("../../Filtered_imgs/BGR_hue:"+ std::to_string(hue)+".jpg", out_);
}

void DataAugmentation::SaltPepper(const float percentage){
  out_.push_back(in_.clone());
  int total = percentage * out_.back().cols * out_.back().rows;
  int column_pixel, row_pixel, color;
  for(int i = 0; i<total; i++){
    //Choose a random pixel between 0 and (out_.cols-1)
    //and 0 and (out_.rows-1)
    column_pixel = rand()%out_.back().cols;
    row_pixel = rand()%out_.back().rows;
    //Randomly change pixel color to black or white
    color = rand()%2 ? 255:0;
    //Apply color
    if(out_.back().channels() == 1){
        out_.back().at<uchar>(row_pixel,column_pixel) = color;
    }
    else if(out_.back().channels() == 3){
        out_.back().at<cv::Vec3b>(row_pixel,column_pixel)[0] = color;
        out_.back().at<cv::Vec3b>(row_pixel,column_pixel)[1] = color;
        out_.back().at<cv::Vec3b>(row_pixel,column_pixel)[2] = color;
    }
  }
  // cv::namedWindow("Salt_and_Pepper",CV_WINDOW_KEEPRATIO);
  // cv::imshow("Salt_and_Pepper",out_.back());
  // cv::imwrite("../../Filtered_imgs/salt_pepper.jpg",out_.back());
}

void DataAugmentation::Scaling_ROI(const float ratio){
  if(ratio < 1){
    cv::Mat temp = in_.clone(), img;
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
      img = temp(ROI);  
      //Resize ROI back to the original image
      cv::resize(img, img, cv::Size(in_.rows, in_.cols), 
                                                    0, 0, CV_INTER_LINEAR);
      // cv::namedWindow("Cropped image",CV_WINDOW_NORMAL); 
      // cv::imshow("Cropped image",out_);
      switch (i){
      case 0:
      // cv::imwrite("../../Filtered_imgs/scale_upper_x_"+std::to_string(ratio)+".jpg",
      //                                                                   out_);
        out_.push_back(img);
        break;
      case 1:
      // cv::imwrite("../../Filtered_imgs/scale_middle_x_"+std::to_string(ratio)+".jpg",
      //                                                                  out_);
        out_.push_back(img);
        break;
      case 2:
      // cv::imwrite("../../Filtered_imgs/scale_lower_x_"+std::to_string(ratio)+".jpg",
      //                                                                 out_);
        out_.push_back(img);
        break;
      default:
        break;
      }
    }
  }
}

void DataAugmentation::ContrastBrightness(const double contrast, const int brightness){
  // Full explanation can be found in: 
  // https://docs.opencv.org/3.4/d3/dc1/tutorial_basic_linear_transform.html
  int i,j,c;
  out_.push_back( cv::Mat::zeros( in_.size(), in_.type() ) );
  for(j = 0; j < in_.rows; j++ ) {
      for(i = 0; i < in_.cols; i++ ) {
          for(c = 0; c < in_.channels(); c++ ) {
              out_.back().at<cv::Vec3b>(j,i)[c] =
              cv::saturate_cast<uchar>( contrast*in_.at<cv::Vec3b>(j,i)[c] + brightness);
          }
      }
  }
  // cv::namedWindow("Brightness_value: "+std::to_string(brightness),CV_WINDOW_NORMAL); 
  // cv::imshow("Brightness_value: "+std::to_string(brightness), out_);
  // cv::imwrite("../../Filtered_imgs/brightness:"+ std::to_string(brightness)+".jpg", out_);
}

void DataAugmentation::ReadDirectory(const std::string path, std::vector<std::string>& images){
  DIR* dirp = opendir(path.c_str());
  struct dirent * dp;
  std::string file;
  std::size_t found;
  int i = 1;
  while ((dp = readdir(dirp)) != NULL) {
    file = dp->d_name;
    found = file.find(".png");
    if(found != std::string::npos){
      images.push_back(dp->d_name);
      // std::cout<<std::to_string(i++) + " image: "<<dp->d_name<<std::endl;
    }
  }
  closedir(dirp);
}

void DataAugmentation::Save(const std::string path){
  for(int j=0; j<out_.size(); j++)
    cv::imwrite(path+"robosub2020_"+std::to_string(j + 1)+".jpg", out_[j]);
}
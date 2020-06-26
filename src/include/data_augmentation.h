//------------------------------------------------------------------------------
// @file: data_augmentation.h
// @created on: March 18th, 2020
// @modified: May 18th, 2020
// @author: Ivana Collado
// @mail: ivanacollado@gmail.com
// @co-author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
// @brief: This file contains the definition for the Data Augmentation class. 
//------------------------------------------------------------------------------

// IFNDEF ----------------------------------------------------------------------
#ifndef DATA_AUGMENTATION_
#define DATA_AUGMENTATION_

// INCLUDES --------------------------------------------------------------------
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <dirent.h>
#include <iostream>
#include <sys/types.h>
#include <time.h>

// CLASS DECLARATION -----------------------------------------------------------
class DataAugmentation {
public:

  // CONSTRUCTOR AND DESTRUCTOR ------------------------------------------------
  // Constructor. 
  DataAugmentation();
  // Destructor.
  ~DataAugmentation();

  // FUNCTIONS -----------------------------------------------------------------
  // Get in_ (input raw image)
  //
  // No params
  cv::Mat GetIn();
  // Set in_ (input raw image)
  //
  // Mat[in]: OpenCV Matrix
  void SetIn(cv::Mat);
  // Get out_ (output processed image)
  //
  // No params
  std::vector<cv::Mat> GetOut();
  // Set out_ (output processed image)
  //
  // vec[in]: Vector of OpenCV Matrices
  void SetOut(std::vector<cv::Mat>);
  // Delete last element from out_
  //
  // No params
  void PopBack();
  // Reads input Image.
  // 
  // @param path[in]: Path to the input image.
  void Read(const std::string &path);
  // Applies GaussianBlur Filter to image
  // 
  // @param kernel[in]: Size of kernel to be applied.
  void GaussianBlur(const int &kernel);
  // Applies changes to the hue value
  //
  // @param hue[in]: hue value
  void Hue(const int hue);
  // Applies Salt and pepper noise to image
  // 
  // @param percentage[in]: percentage of the image to be covered with noise
  void SaltPepper(const float percentage);
  // Crops, resizes and saves 3 images of lower, middle and upper ROI's
  //
  // @param  ratio[in]: reduction of the original image. MUST BE < 1
  void ScalingROI(const float ratio);
  // Change brightness of the image
  //
  // @param contrast[in]: contrast control (0,2] recommended, 1 doesn't present
  //                      any change
  // @param brightness[in]: brightness control [-100,100] recommended
  void ContrastBrightness(const float contrast, const int brightness);
  // Read directory contents
  //
  // @param path[in]: directory path
  // @param images[in]: saved images
  void ReadDirectory(const std::string path, std::vector<std::string>& images);
  // Save combination
  //
  // path[in]: location where the images will be saved
  void Save(const std::string path);

private:
  // MEMBERS -------------------------------------------------------------------
  //Input image Matrix
  cv::Mat in_;
  //Output Matrix vector
  std::vector<cv::Mat> out_;
};
#endif
//------------------------------------------------------------------------------
// @file: data_augmentation.h
// @created on: March 18th, 2019
// @author: Ivana Collado
//
// @brief: This file contains the definition for the Data Augmentation class. 
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

// CLASS DECLARATION -----------------------------------------------------------
class DataAugmentation {
public:

  // CONSTRUCTOR AND DESTRUCTOR ------------------------------------------------
  // Constructor. 
  DataAugmentation();
  // Destructor.
  ~DataAugmentation();

  // FUNCTIONS -----------------------------------------------------------------
  // Reads input Image.
  // 
  // @param path[in]: Path to the input image.
  void Read(const std::string &path);

  // Applies Average Filter to image
  // 
  // @param kernel[in]: Saze of kernel to be applied.
  void AverageFilter(const int &kernel);

  // Applies changes to the hue value
  //
  // @param min_hue[in]: lower hue value
  // @param max_hue[in]: upper hue value
  // @param step[in]: increments in hue
  void Hue(const unsigned char min_hue, const unsigned char max_hue, const int step);

private:
  // MEMBERS -------------------------------------------------------------------
  //Input image Matrix
  cv::Mat in_;
  //Output image Matrix
  cv::Mat out_;
};
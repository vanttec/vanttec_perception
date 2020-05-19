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

  // Crops, resizes and saves 3 images of lower, middle and upper ROI's
  //
  // @param  ratio[in]: reduction of the original image. MUST BE < 1
  void Scaling_ROI(const float ratio);

private:
  // MEMBERS -------------------------------------------------------------------
  //Input image Matrix
  cv::Mat in_;
  //Output image Matrix
  cv::Mat out_;
};
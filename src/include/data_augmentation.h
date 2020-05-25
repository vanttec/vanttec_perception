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

  // Applies GaussianBlur Filter to image
  // 
  // @param kernel[in]: Size of kernel to be applied.
  void GaussianBlur(const int &kernel);

  // Applies changes to the hue value
  //
  // @param min_hue[in]: lower hue value
  // @param max_hue[in]: upper hue value
  // @param step[in]: increments in hue range
  void Hue(const u_char min_hue, const u_char max_hue, const u_char step);
  // Applies Salt and pepper noise to image
  // 
  // @param percentage[in]: percentage of the image to be covered with noise
  void SaltPepper(const float percentage);
  // Crops, resizes and saves 3 images of lower, middle and upper ROI's
  //
  // @param  ratio[in]: reduction of the original image. MUST BE < 1
  void Scaling_ROI(const float ratio);
  // Change brightness of the image
  //
  // @param contrast[in]: contrast control (0,2] recommended, 1 doesn't present
  //                      a change
  // @param brightness[in]: brightness control [-100,100] recommended
  void ContrastBrightness(const double contrast, const int brightness);
  // Make combinations of filters
  //
  // No params
  void CombiningFilters();
  void read_directory(const std::string& name, std::vector<std::string>& v);

private:
  // MEMBERS -------------------------------------------------------------------
  //Input image Matrix
  cv::Mat in_;
  //Output image Matrix
  cv::Mat out_;
};
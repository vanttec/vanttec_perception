//------------------------------------------------------------------------------
// @file: data_augmentation.h
// @created on: March 18th, 2020
// @modified: July 3rd, 2020
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
#include <queue>
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
  // Set out_ to in_
  //
  // No params
  void SetOut2In();
  // Set in_ to out_
  //
  // No params
  void SetIn2Out();
  // Read directory contents
  //
  // @param dir_path[in]: directory path
  // @param extension[in]: image extension (.jpg, .png, etc.)
  // @param images[in]: saved images
  void ReadDirectory(const std::string &dir_path, const std::string &extension, std::queue<std::string> &images);
  // Reads input Image.
  // 
  // @param path[in]: Path to the input image.
  void ReadImage(const std::string &input_image);
  // Save combination of filters
  //
  // @param path[in]: location where the images will be saved
  // @param extension[in]: desired image extension
  void Save(const std::string &path, const std::string &extension);
  // Show in_ image
  //
  // No params
  void ShowIn();
  // Show out_ image
  //
  // No params
  void ShowOut();
  // Applies GaussianBlur Filter to image
  // 
  // @param kernel[in]: Size of kernel to be applied.
  void GaussianBlur(const int &kernel);
  // Applies changes to the hue value
  //
  // @param hue[in]: hue value
  void Hue(const int &hue);
  // Applies Salt and pepper noise to image
  // 
  // @param percentage[in]: percentage of the image to be covered with noise
  void SaltPepper(const float &percentage);
  // Crops, resizes and saves 3 images of lower, middle and upper ROI's
  //
  // @param ratio[in]: reduction of the original image. MUST BE < 1
  // @param ROI_number[in]: reference to which of the ROI's will be 
  //                        generated. 0: UPPER, 1: MIDDLE, 2: LOWER
  void ScalingROI(const float &ratio, const int &ROI_number);
  // Change brightness of the image
  //
  // @param contrast[in]: contrast control (0,2] recommended, 1 doesn't present
  //                      any change
  // @param brightness[in]: brightness control [-80,80] recommended
  void ContrastBrightness(const float &contrast, const int &brightness);

private:
  // MEMBERS -------------------------------------------------------------------
  // Input image Matrix
  cv::Mat in_;
  // Output image Matrix
  cv::Mat out_;
};
#endif
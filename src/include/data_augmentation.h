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
  // @param in[in]: OpenCV Matrix
  void SetIn(cv::Mat in);
  // Get out_ (output processed image)
  //
  // No params
  cv::Mat GetOut();
  // Set out_ (output processed image)
  //
  // @param out[in]: Vector of OpenCV Matrices
  void SetOut(cv::Mat out);
  // Get entry_ pointer
  //
  // No params
  dirent* GetEntry();
  // Get next entry_ of the directory
  //
  // No params
  void GetNextEntry();
  // Reads each entry looking for desired image.
  // 
  // @param extension[in]: desired image extension (.jpg, .png, etc.).
  void ReadEntry(const std::string extension);
  // Set input image directory
  //
  // @param path[in]: input directory path
  void SetDirectory(const std::string path);
  // Save combination of filters
  //
  // @param path[in]: location where the images will be saved
  // @param extension[in]: desired image extension
  // @param img_number[in]: image number
  void Save(const std::string path, const std::string extension, const int img_number);
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
  void Hue(const int hue);
  // Applies Salt and pepper noise to image
  // 
  // @param percentage[in]: percentage of the image to be covered with noise
  void SaltPepper(const float percentage);
  // Crops, resizes and saves 3 images of lower, middle and upper ROI's
  //
  // @param  ratio[in]: reduction of the original image. MUST BE < 1
  // @return std::vector<cv::Mat>[out]
  std::vector<cv::Mat> ScalingROI(const float ratio);
  // Change brightness of the image
  //
  // @param contrast[in]: contrast control (0,2] recommended, 1 doesn't present
  //                      any change
  // @param brightness[in]: brightness control [-100,100] recommended
  void ContrastBrightness(const float contrast, const int brightness);

private:
  // MEMBERS -------------------------------------------------------------------
  // Input image Matrix
  cv::Mat in_;
  // Output image Matrix
  cv::Mat out_;
  // Directory stream pointer
  DIR* dir_pointer_;
  // Directory file pointer
  dirent* entry_;
  // Input directory path
  std::string dir_path_;
};
#endif
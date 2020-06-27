// INCLUDES --------------------------------------------------------------------
#include <time.h>

#include "include/data_augmentation.h"

// MAIN PROGRAM ----------------------------------------------------------------
int main( int argc, char** argv ){
  DataAugmentation data;
  int combination;
  int kernel_size;
  int brightness;
  int min_hue;
  int max_hue;
  int hue;
  int max_noise;
  int min_noise;
  float noise_percentage;
  int max_scaling;
  int min_scaling;
  float scaling_ratio;
  double contrast;
  std::vector<std::string> images;
  srand(time(NULL));

  data.ReadDirectory("../imgs/",images);

  for(int i=0; i<images.size(); i++){
      data.Read("../imgs/" + images[i]);

      // Define parameters
      combination = rand()%7 + 1;
      contrast = 1.0;
      brightness = (rand()%101) * (rand()%2 ? 1:-1);
      min_hue = 0;
      max_hue = 105;
      hue = rand()% (max_hue - min_hue + 1) + min_hue;
      max_noise = 4;
      min_noise = 1;
      noise_percentage = (float) (rand()% max_noise + min_noise)/10;
      max_scaling = 95;
      min_scaling = 60;
      scaling_ratio = (float) ((rand()% (max_scaling - min_scaling + 1)) +
                                                        min_scaling)/100;
      kernel_size = 2 * (rand()%13) + 1;

      std::cout<<"Combinacion: " + std::to_string(combination)<<"\n";
      std::cout<<"Contrast: "<<contrast<<" Brightness: "<<brightness<<"\n";
      std::cout<<"min_hue: "<< min_hue 
              <<" max_hue: "<< max_hue
              <<" hue: "<< hue <<"\n";
      std::cout<<"noise_percentage: "<<noise_percentage<<"\n";
      std::cout<<"scaling_ratio (reduction): "<<1 - scaling_ratio<<"\n";
      std::cout<<"kernel_size: "<<kernel_size<<"\n";

      switch (combination){
      case 1:
          // Noise + scaling
          data.SaltPepper(noise_percentage);
          data.setIN(data.getOUT().back());
          data.PopBack();
          data.Scaling_ROI(scaling_ratio);
          break;
      case 2:
          // Scaling + Gaussian blur
          data.GaussianBlur(kernel_size);
          data.setIN(data.getOUT().back());
          data.PopBack();
          data.Scaling_ROI(scaling_ratio);
          break;
      case 3:
          // Gaussian blur + hue
          data.GaussianBlur(kernel_size);
          data.setIN(data.getOUT().back());
          data.PopBack();
          data.Hue(hue);
          break;
      case 4:
          // Brightness + hue
          data.ContrastBrightness(contrast, brightness);
          data.setIN(data.getOUT().back());
          data.PopBack();
          data.Hue(hue);
          break;
      case 5:
          // Brightness + noise
          data.ContrastBrightness(contrast, brightness);
          data.setIN(data.getOUT().back());
          data.PopBack();
          data.SaltPepper(noise_percentage);
          break;
      case 6:
          // Scaling + hue
          data.Hue(hue);
          data.setIN(data.getOUT().back());
          data.PopBack();
          data.Scaling_ROI(scaling_ratio);
          break;
      case 7:
          // Brightness + hue + blur
          data.ContrastBrightness(contrast, brightness);
          data.setIN(data.getOUT().back());
          data.PopBack();
          data.GaussianBlur(kernel_size);
          data.setIN(data.getOUT().back());
          data.PopBack();
          data.Hue(hue);
          break;
      default:
          break;
      }
  }
  data.Save("../Filtered_imgs/");
  return 0;
}
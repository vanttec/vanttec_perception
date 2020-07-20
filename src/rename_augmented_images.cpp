//------------------------------------------------------------------------------
// @file: rename_augmented_images.cpp
// @created on: July 19th, 2020
// @author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
// @brief: Rename images from test and train directories
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "include/data_augmentation.h"

// MAIN PROGRAM ----------------------------------------------------------------
int main( int argc, char** argv ){
    // DataAugmentation object
    DataAugmentation data;
    // Class path
    std::string class_path = "../../Classes/Badge/";
    // File within directory
    std::string file = "";
    // Desired extension
    std::string extension = ".jpg";
    // Class
    std::string img_class = "badge_";
    // Save name
    std::string save_name = "robosub2020_"+img_class;
    // Image number
    int img_num = 0;
    // Pointer to class directory
    DIR* class_pointer = opendir(class_path.c_str());
    // Pointer to directory entry
    dirent* entry;

    // Read files in train directory
    while(NULL != (entry = readdir(class_pointer))){
        // Gey entry name
        file = entry->d_name;
        // If file length is larger than length of "." or ".." directories
        if(file.length() > 2){
            // Read entry
            data.ReadImage(class_path+file);
            data.SetIn2Out();
            // Save image with desired name and extension
            data.Save(class_path+save_name+std::to_string(++img_num),extension);
            // Remove old image
            remove((class_path+file).c_str()); 
        }
    }
    closedir(class_pointer);
    return 0;
}


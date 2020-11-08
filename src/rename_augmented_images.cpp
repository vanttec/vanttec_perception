//------------------------------------------------------------------------------
// @file: rename_augmented_images.cpp
// @created: July 19th, 2020
// @modified: August 7th, 2020
// @author: Sebastian Martínez
// @mail: sebas.martp@gmail.com
// @brief: Rename images from test and train directories
//  note: If the desired new name is an existing file, the function may either 
//      fail or override the existing file, depending on the specific system and 
//      library implementation.
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "include/data_augmentation.h"

// MAIN PROGRAM ----------------------------------------------------------------
int main( int argc, char** argv ){
    // Class path
    std::string class_path = "../../imgs/";
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
    // Initialize and point pointers to NULL
    char* old_name = NULL;
    char* new_name = NULL;
    // Pointer to class directory
    DIR* class_pointer = opendir(class_path.c_str());
    // Pointer to directory entry
    dirent* entry;

    // Read files in train directory
    while(NULL != (entry = readdir(class_pointer))){
        // Get entry name
        file = entry->d_name;
        // If file length is larger than length of "." or ".." directories
        if(file.length() > 2){
            // Create array
            old_name = new char[(class_path+file).length()+1]; 
            // Copy the old name
            strcpy(old_name,(class_path+file).c_str());
            // Create array
            new_name = new char[(class_path+save_name+std::to_string(++img_num)
                                + extension).length()+1];
            // Copy the new name
            strcpy(new_name, (class_path+save_name+std::to_string(img_num)
                                + extension).c_str());
            // If something goes wrong...
            if(rename(old_name, new_name) != 0){
                std::cout << "Error: " << strerror(errno) << std::endl;
            }
            // Delete arrays
            delete[] old_name, new_name;
        }
    }
    closedir(class_pointer);

    return 0;
}


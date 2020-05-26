void DataAugmentation :: read_directory(const std::string& name, std::vector<std::string>& v)
{
    DIR* dirp = opendir(name.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        v.push_back(dp->d_name);
        std::cout<<"image"<<dp->d_name<<std:endl;
    }
    closedir(dirp);
}

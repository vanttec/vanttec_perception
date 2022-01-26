def imgname(x,cntr):
    '''
        @name: imgname
        @brief: function to define the name of a image/point cloud file
        @param: x: current counter value
        @return: string with the name of the image/point cloud file
    '''

    return{
        '1': '00000'+str(cntr),
        '2': '0000'+str(cntr),
        '3': '000'+str(cntr),
        '4': '00'+str(cntr),
        '5': '0'+str(cntr),
        '6': str(cntr)
    }[x]


with open("/home/rauldds/catkin_ws/src/vttc/Real_Dataset/label_2/000000.txt") as f:
    lines = f.readlines()
    for i in range(399):
        txtname = imgname(str(len(str(i))),i)
        with open("/home/rauldds/catkin_ws/src/vttc/Real_Dataset/label_2/"+txtname+".txt", "w") as f1:
            #print(lines)
            f1.writelines(lines)
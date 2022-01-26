import random
to_delete=[2,3,8,9,10,11,12,13,14]
document=""
for i in range (100):
    if i<10:
        direc="00000"+str(i)
    else:
        direc= "0000"+str(i)
    directorio = "/home/rauldds/catkin_ws/src/vttc/dataset/label_rgb_detect/"+direc+".txt"
    try:
        f = open(directorio, "r")
        datastr = " "
        for x in f:
            my_list=x.split(" ")
            new_list = [val for n, val in enumerate(my_list) if n not in to_delete]
            new_list[1]=str(random.randrange(9500,9999)/10000)
            dataline = "dataset/KITTI/object/training/image_2/"+direc+".png "+datastr.join(new_list)+"\n"
            document+=dataline
    except:
        continue

#print(document)
document=document.replace("Buouy","1")
document=document.replace("MarkerDos","3")
document=document.replace("Marker","2")
file = open("rgb_detection_train.txt", "w") 
file.write(document) 
file.close() 
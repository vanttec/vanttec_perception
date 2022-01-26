import numpy as py
def imgname(x):
    return{
        '1': '00000'+x,
        '2': '0000'+x,
        '3': '000'+x,
        '4': '00'+x,
        '5': '0'+x,
        '6': x
    }[x]
c=2
name= imgname(str(len(str(c))))
print(name)
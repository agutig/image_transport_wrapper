
import numpy as np


max_weight = 6#1080 #dctImage.columns
max_height =  3#1920 #dctImage.rows

dct_image = np.zeros((max_weight,max_height))

n = 0 #filas
m = 0 #columnas

zigzag_array = []
dir = "pos"


while dir != "stop":

    zigzag_array.append(dct_image[m][n])
    if dir == "pos":

        if m < max_weight - 1:
            m +=1

            if n == 0:
                dir = "neg"

            else:
                n = n-1
        
        else:
            n +=1
            dir = "neg"

    elif dir == "neg":

        if n < max_height - 1:
            n +=1

            if m == 0:
                dir = "pos"
                if m == max_weight -1 and n == max_height -1: 
                    dir = "stop"
            else:
                m = m-1

        else:
            m +=1
            dir = "pos"


    if m == max_weight - 1 and n== max_height - 1: 
        dir = "stop"
        zigzag_array.append(dct_image[m][n])



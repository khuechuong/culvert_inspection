import os

import numpy as np
from PIL import Image
from matplotlib import pyplot as plt
#dir = "data/Masks/"
dir_in = "/home/aralab/Segmentation/dataset/rust_anno/"
dir = "/home/aralab/Segmentation/dataset/mask_rust/"
# dir = "data/Masks_test/"
image_names = [f for f in os.listdir(dir_in) if '.png' in f]
#print(image_names)

r_ = []
g_ = []
b_ = []

for name in image_names:
    img = Image.open(dir_in + name).convert('RGB')
    # plt.figure(figsize=(10, 10))
    # plt.imshow(img)
    # plt.show()
    pixels = img.load()
#print(name)
    for i in range(img.size[0]):    # for every col:
        for j in range(img.size[1]):    # For every row
            r, g, b = pixels[i,j]
            r_.append(r)
            g_.append(g)
            b_.append(b)

        # break
            if r > 230 and g == 0 and b == 0:
                pixels[i,j] = (1,1,1)
            else:
                pixels[i, j] = (0, 0, 0)
    # plt.figure(figsize=(10, 10))
    # plt.imshow(img)
    # plt.title(name)
    # plt.show()
    # img.show()
# break
    img.save(dir+name)
#print("Red unique: {}".format(np.unique(r_)))
#print("Blue unique: {}".format(np.unique(b_)))
#print("Green unique: {}".format(np.unique(g_)))

import sys
import os
from PIL import Image

filepath = "/home/htqkhanh/Downloads/non_sign"


count = 0
# Loop through all provided arguments
for filename in os.listdir(filepath):
    left = 32
    upper = 0
    right = 64
    lower = 32
    widthCrop = 0 #320/32 = 10 img
    heightCrop = 0  #240/32 = 7 img
    
    if "." not in filename:
        continue
    ending = filename.split(".")[1]
    if ending not in ["jpg", "gif", "png"]:
        continue

    # try:
    #     # Attempt to open an image file
    #     image = Image.open(os.path.join(filepath, filename))
    # except IOError:
    #     # Report error, and then skip to the next argument
    #     print("Problem opening", filepath, ":")
    #     continue
    

    # image = image.crop((left, upper, right, lower))
    # image.save(os.path.join("/home/htqkhanh/Downloads/cropped", str(count) + '.png'))
    # count += 1

    # while(widthCrop < 10):
    #     print(left," ", upper, " ",right, " ",lower)
    #     image = image.crop((left, upper, right, lower))
    #     left += 32
    #     right += 32
    #     widthCrop += 1
    #     image.save(os.path.join("/home/htqkhanh/Downloads/cropped", str(count) + '.png'))
    #     count += 1

    # Perform operations on the image here
    while(True):
        if(widthCrop < 10 and heightCrop < 7):
            image = Image.open(os.path.join(filepath, filename))
            print(left," ", upper, " ",right, " ",lower)
            image = image.crop((left, upper, right, lower))
            left += 32
            right += 32
            widthCrop += 1
            image.save(os.path.join("/home/htqkhanh/Downloads/cropped", str(count) + '.png'))
            count += 1
        elif(widthCrop == 10 and heightCrop < 7):
            image = Image.open(os.path.join(filepath, filename))
            left = 0
            upper += 32
            right = 32
            lower += 32
            widthCrop = 0
            heightCrop += 1
            print(left," ", upper, " ",right, " ",lower)
            image = image.crop((left, upper, right, lower))
            left += 32
            right += 32
            widthCrop += 1
            image.save(os.path.join("/home/htqkhanh/Downloads/cropped", str(count) + '.png'))
            count += 1
        else:
            break
    

    # Split our origional filename into name and extension 
    #name, extension = os.path.splitext(filename)

    # Save the image as "(origional_name)_thumb.jpg
    #print(name + '_.png')
    
# Image Preprocessing

# Importing the libraries
import numpy as np
import PIL.Image as Image
import cv2

# Preprocessing the Images

class PreprocessImage:
    
    def __init__(self, height = 64, width = 64, grayscale = True):
        self.img_size = (height, width)
        self.grayscale = grayscale

    def debug_show(self, img):
        cv2.imshow('frame', img)
        while not cv2.waitKey(1) or not 0xFF == ord('q'):
            None


    def process(self, img):
        if self.grayscale:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        img = Image.fromarray(img)
        img = img.resize(self.img_size)

        img = np.array(img)

        #img = img.mean(-1, keepdims = True)
        #img = np.transpose(img, )
        img = img.astype('float32') / 255.

        #self.debug_show(img)

        return np.array(img)

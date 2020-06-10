#!/usr/bin/env python
import sys

# Import ROS
import rospy

# OpenCV module
import cv2

# Numpy
import numpy as np
#Import colorthief for dominant colors
from colorthief import ColorThief
# 1 objetivo: template matching con el siguiente


class Tracker():

    # Init tracker. Receives the first image
    # and the col and row coordinates (u,v) of the upper-left (ul)
    # and lower-right (lr) corners of the initial bounding-box (bb)
    def __init__(self, image, bb_ul, bb_lr):

        # Initial template
        self.template = image[bb_ul[1]:bb_lr[1], bb_ul[0]:bb_lr[0], :]
        self.h = self.template.shape[0]
        self.w = self.template.shape[1]
        self.downscale_ratio = 0.75
        self.upscale_ratio = 1.25
        self.multiply_factor = 1.75
        
        # Position of the upper-left corner of the bounding box
        self.upper_left = bb_ul

        # Size of the template (width, height)
        self.template_width = bb_lr[0] - bb_ul[0]
        self.template_height = bb_lr[1] - bb_ul[1]
        self.size = (self.template_width, self.template_height)
        # Distance cost value

        img_width = image.shape[0]
        img_height = image.shape[1]
        #print img_width,img_height
        #Multiply factor where to draw the mask to look for the object

        # Center position of the template (u,v)
        self.position = (bb_ul[0] + self.template_width / 2.0, bb_ul[1] + self.template_height / 2.0)
        self.mask = self.create_circular_mask(img_height,img_width,center=self.position, radius=self.template_width * self.multiply_factor)
        self.masked_img = image.copy()
        self.masked_img[~self.mask] = 0
        cv2.imshow('mascara',self.masked_img)

        #Downscale image to look for bigger object
        self.img_downscaled = self.resize_img(image,factor=self.downscale_ratio)

        #Upscale image to look for smaller object
        self.img_upscaled = self.resize_img(image, factor=self.upscale_ratio)
        
        cv2.rectangle(image, bb_ul, bb_lr, 255, 2)

        cv2.imwrite("aux.jpg",image)
        #THIS IS NOT USED, THIS WILL HELP USING COLOR INFORMATION FROM THE TEMPLATE
        #color_thief = ColorThief("aux.jpg")
        # get the dominant color
        #dominant_color = color_thief.get_color(quality=1)
        #print "color dominante:", dominant_color
        cv2.imshow('img_downscaled',self.img_downscaled)
        cv2.imshow('masked_up',self.img_upscaled)
        cv2.imshow('result', image)
        cv2.imshow('template', self.template)
        cv2.waitKey(0)

    # This is the function to fill. You can also modify the class and add additional
    # helper functions and members if needed
    # It should return, in this order, the u (col) and v (row) coordinates of the top left corner
    # the width and the height of the bounding box
    # TODO: MULTIPLY WEIGHT EJEMPLO: MULTIPLICAR POR 1 - LA DISTANCIA NORMALIZADA DE MANERA QUE DECREZCA SEGUN SE ALEJA
    def track(self, image):

        # Fill here the function
        res = cv2.matchTemplate(self.masked_img, self.template, cv2.TM_CCOEFF_NORMED)
        res_down = cv2.matchTemplate(self.img_downscaled, self.template, cv2.TM_CCORR_NORMED)
        res_up = cv2.matchTemplate(self.img_upscaled, self.template, cv2.TM_CCORR_NORMED)

        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        best_loc = max_loc
        best_val = max_val
        
        '''min_val_down, max_val_down, min_loc_down, max_loc_down = cv2.minMaxLoc(res_down)
        if best_val < max_val_down:
            best_loc = max_loc_down
            best_val = max_val_down
            print "Encontrado en resolucion disminuida"

        min_val_up,max_val_up,min_loc_up,max_loc_up = cv2.minMaxLoc(res_up)
        
        if best_val < max_val_up:
            best_loc = max_loc_up
            best_val = max_val_up
            print "Encontrado en resolucion aumentada"
            '''

        img_width = image.shape[0]
        img_height = image.shape[1]

        self.position = (best_loc[0] + self.template_width / 2.0, best_loc[1] + self.template_height / 2.0)
        self.mask = self.create_circular_mask(img_height,img_width,center=self.position, radius=self.template_width * self.multiply_factor)
        self.masked_img = image.copy()
        self.masked_img[~self.mask] = 0

        #Downscale image to look for bigger object
        self.img_downscaled = self.resize_img(image,factor=self.downscale_ratio)

        #Upscale image to look for smaller object
        self.img_upscaled = self.resize_img(image, factor=self.upscale_ratio)

        return best_loc[0], best_loc[1], self.w, self.h


    #Creates a circular mask centered and with a radius defined.
    def create_circular_mask(self, h, w, center=None, radius=None):

        if center is None:  # use the middle of the image
            center = (int(w/2), int(h/2))
        if radius is None:  # use the smallest distance between the center and image walls
            radius = min(center[0], center[1], w-center[0], h-center[1])

        Y, X = np.ogrid[:w, :h]
        dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)

        mask = dist_from_center <= radius
        return mask
    def resize_img(self,image, factor=0.75):
        if factor > 1:
            #Uses interpolation recommended to enlarge image
            image = cv2.resize(image,(0,0),fx=factor,fy=factor,interpolation=cv2.INTER_CUBIC)
        else:
            #Uses interpolation recommended to reduce image
            image  = cv2.resize(image,(0,0),fx=factor,fy=factor,interpolation=cv2.INTER_AREA)

        img_width = image.shape[0]
        img_height = image.shape[1]

        mask = self.create_circular_mask(img_height,img_width,center= np.array(self.position)*factor, radius=self.template_width * self.multiply_factor)

        image[~mask] = 0
        return image

        

 # rosrun vision_challenge vision_challenge.py imagenes_challenge/pelota 496 419 536 461
 # rosrun vision_challenge vision_challenge.py imagenes_challenge/car1 246 162 357 279
 # rosrun vision_challenge vision_challenge.py imagenes_challenge/balt 326 157  371 224
 # rosrun vision_challenge vision_challenge.py imagenes_challenge/balt 341 172 353 193


if __name__ == '__main__':
    # initiliaze node
    rospy.init_node('vision_challenge')

    # Read arguments, removing the added by ROS
    myargv = rospy.myargv(argv=sys.argv)

    folder = myargv[1] + '/%08d.jpg'

    output_file = open('output.txt', 'w')

    # Ball: 496 419 536 461
    # Car1: 246 163 357 280
    # Bolt1: 330 162 368 213
    upper_left = (int(myargv[2]), int(myargv[3]))
    lower_right = (int(myargv[4]), int(myargv[5]))

    print(folder)

    # Open camera device
    cap = cv2.VideoCapture(folder)

    # Capture first frame
    ret, frame = cap.read()

    # Init tracker
    tracker = Tracker(frame, upper_left, lower_right)

    try:

        rate = rospy.Rate(10)  # 10hz

        # Main loop
        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = cap.read()

            # Call the tracker function
            left, top, width, height = tracker.track(frame)

            # Store results in file
            output_file.write(str(left) + ',' + str(top) +
                              ',' + str(width) + ',' + str(height) + '\n')

            # Draw results and show
            cv2.rectangle(frame, (left, top), (left+width, top+height), 255, 2)

            cv2.imshow('result', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

    except rospy.ROSInterruptException:
        print("Shutting down")

    # When everything done, release the capture and destroy the windows
    output_file.close()
    cap.release()
    cv2.destroyAllWindows()

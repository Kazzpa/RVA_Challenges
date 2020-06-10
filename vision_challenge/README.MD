
# Vision Challenge
Vision challenge where the program should track an object that is defined at the first frame, and use the info of that object to track in the following frames
##### Note: upscaling and downscaling images is disabled in code as it isnt working as intended and giving worse results.
-  **Scenario 1:** Ball1: video of a person kicking a ball
- **Scenario 2**: car1: video of a car from behind
- **Scenario 3:** bolt1: video of usain bolt running
## Credits
[Luis Merino](https://github.com/lmercab)  
[David Alejo Teissière](https://github.com/david-alejo)  
[OpenCV](https://docs.opencv.org/master/d6/d00/tutorial_py_root.html)  
# Dependencies:
[Ros kinetic](http://wiki.ros.org/kinetic/Installation)
[Python](https://www.python.org/downloads/)
# Usage
The challenge has three different scenarios and there is a way to execute each one.  
All of them will work with the same python module but will have different parameters to select the folder of the images and the location of the object.  

You must prepare the project folder before launching any scenario: `source ./devel/setup.bash` in your project directory.

Scenario 1:  
```
rosrun vision_challenge vision_challenge.py imagenes_challenge/pelota 496 419 536 461  
```  
Scenario 2:  
```
rosrun vision_challenge vision_challenge.py imagenes_challenge/car1 246 162 357 279   
```
Scenario 3:  
```
rosrun vision_challenge vision_challenge.py imagenes_challenge/balt 326 157  371 224   
```

If you need further help with ros commands check my [Ros Kinetic commands gist](https://gist.github.com/Kazzpa/5e88584811270419eaea57b7e87195df)
# Module
For now there is only this module in the project
- [Vision Challenge](#Vision Challenge)
## Vision Challenge:
[code](vision_challenge/scripts/vision_challenge.py) *CHECK*
##### From the first image it will create a template that will check in each frame the correlation that each image fragment has with this template and for each frame it will give an estimation of where this object can be.

### Parameters:
 -  Image **Template:** Fragment of the first frame where the object is located, that will work as a template to look for in the following frames.  
 -  Float **downscale_ratio:**  Ratio that the image will reduce to, so we can look for the object if it becomes bigger.  
 -  Float **upscale_ratio:** Ratio that the image will enlarge to, so we can look for the object if it becomes bigger.  
 -  Float **template_width:** Width of the template that we will use to set the radius distance to look for in the next frames.
 - Float [] **position:** Position where the object was supossed to be in the last frame, initially in the actual object position. This will set the center of the radius to look for the object.
 - Float [][] **mask:** Matrix which sets as 1 the pixels that are in the radius to the center of position.
 - Image **masked_img:** Image that shows only the pixels in the radius
 - Image **img_downscaled:**  Image according to the masked that is downscaled to downscale_ratio.
 - Image **img_upscaled:** Image according to the masked that is enlarged to upscale_ratio. 
### Functions:
  - **__main__**: It initializes the node and creates the tracker object and loads the folder to load images from.
 - **Tracker(frame, upper_left, lower_right )**:  Creates the template from the first frame and masks the image to the radius, and creates the upscaled image and the downscaled that are masked too.
 - **track(image )**: This function is called every frame and its goal its to create the bounding box of the tracked object, it also creates the mask  for the next frame.  
 - **create_circular_mask( height,width,center,radius )**: Creates a mask of true and false based in the size of the matrix specified by height and width and centers it on the center parameter with the parameter radius.  
 - **resize_img (Image, factor)**:  resizes the image to the factor parameter if its to enlarge it will use the CUBIC interpolation and if its to reduce the size it will use the AREA interpolation, this function will also mask the created image based on the multiply_factor  

## Images TODO: EDIT
 #### Challenge 1,2:
 Challenge 1 being completed, image from rviz  
 ![challenge 1 in rviz](img/challenge1_rviz.png)  
 Challenge 2 being completed, image from gazebo    
 ![challenge 2 in gazebo](img/challenge2_gazebo.png)  
 Challenge 2 being completed, image from rviz  
 ![challenge 2 in rviz](img/challenge2_rviz.png)  
 #### Challenge 3:  
 Challenge 3 being completed, image from gazebo  
 ![challenge 3 in gazebo](img/challenge3_gazebo.png)  
 Challenge 3 being completed, image from rviz  
 ![challenge 3 in rviz](img/challenge3_rviz.png)  
## TODO: 
These are things that were supossed to be working in the program but couldn't be done to the date.  
- Fix looking for object in different scales
- Use color information with colorthief library

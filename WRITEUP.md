# CarND-Capstone
Self-Driving Car Engineer Nanodegree Program

The final project by Marcus Neuert, 21.08.2019

### Project Goals and how I managed them

No. | Criteria | How I solved it 
---|---------|---------------
1|Update Waypoints|Take all waypoints from the current position to the driving direction, but considering the next 100-200 waypoints is more than enough.
2|Send steer and throttle commands to keep the lane and speed limit|Using the twist controller is a good approuch. But parameter tuning is always a never-ending story, regarding unexpected error-handling.
3|Traffic light detection|The pre-trained CNN from YOLO is a very good approach for object-detection on streets, so it was amazing playing around with that. Sometimes it classifies the traffic light as a stop sign, but this is ok for this projects.
4|Object detection (optional)|The YOLOv3 for ROS frame work is ready to recognize a lot of objects, but I limited it as you below.
5|Verify traffic light detection with rosbag|I started the video from Carla and saw that traffic light detection works with slightly different image preprocessing methods. Finally it was very good, as you can see below.

### Testresults from track

##### Simulator on track right after start. [Manual mode, no Camera] 
[track_start_default]: ./imgs/track_start_default.png

##### ... turned camera on, green light, long waypoints. [Manual mode, Camera on] 
[track_start_camera]: ./imgs/track_start_camera.png

##### ... turned camera on, yellow light, long waypoints. [Manual mode, Camera on] 
[track_start_camera_yellow]: ./imgs/track_start_camera_yellow.png

##### ... turned camera on, red light, only few waypoints until stopline. [Manual mode, Camera on] 
[track_start_camera_red]: ./imgs/track_start_camera_red.png

##### vehicle started with green light in autonomous mode. [Autonomous mode, Camera on] 
[track_run_green]: ./imgs/track_run_green.png

### Testresults from parking site

[Traffic Light 1]: ./imgs/YOLO V3_screenshot_20.08.2019.png

[Traffic Light 2]: ./imgs/YOLO V3_screenshot_20.08.2019_2.png

[Traffic Light 3 as Stop Sign]: ./imgs/YOLO v3_screenshot_stopsign.PNG

### Settings for YOLO for ROS 
CarND-Capstone\ros\src\darknet_ros\darknet_ros\config\yolov3.yaml:

detection_classes:

      - person
      - bicycle
      - car
      - motorbike
      - bus
      - train
      - truck
      - traffic light
      - fire hydrant
      - stop sign
      - parking meter
      
### Future improvements
* Use a lighter solution for object detection than YOLO, because it requires really much CPU and GPU
* Tune parameters for steering and throttle
* Handle unexpected errors, in case of sudden lane changes or other vehicles
* Do traffic light detection based on image data, limited to the current lane in order to ignore traffic light for other directions
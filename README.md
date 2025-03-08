# Vision-Pick-Place-Robot
This is package for pincher x100 robot manipulator to perform autonomous pick and placed based on vision based object recognition

Follow the tutorials files given in the tutorials folder to get started with simualtion and using hardware setup

[Tutorials1](tutorials/Tutorial-1.pdf) Gives an idea of installing robotic tool box in the and getting started with robotic arm in matlab.

[Tutorials2](tutorials/Tutorial-2.pdf) Gives an idea of how to use the robotic arm in simualtion to solve inverse kinematics

[Tutorials3](tutorials/Tutorial-3.pdf) Gives an idea of how to setup and insatllations for hardware and getting started with hardware control



To test the codes in virtual environment run 
```bash
Visual_env
```

## Result Visual_env
![Visual_env](https://img.youtube.com/vi/2Wf224BRYD8/0.jpg)(https://youtu.be/2Wf224BRYD8)


To perform pick and place based without camera run 
```bash
pick_place_new
```

To perform pick and place based on camera run
```bash
pick_place_cam
```
## Result Pick and Place with Camera

![Pick and Place with Camera](https://img.youtube.com/vi/fKzTv4b1e78/0.jpg)(https://youtu.be/fKzTv4b1e78)


To perform obstacle avoidance with camera run 
```bash
pick_place_obs_new
```

## Result Obstacle Avoidance with Camera
![Obstacle Avoidance with Camera](https://img.youtube.com/vi/Wv_6YmU4mb8/0.jpg)(https://youtu.be/Wv_6YmU4mb8)

To perform multiple objects pick and place without camera
```bash
pick_place_two_object
```

To perform multiple objects pick and place with camera
```bash
pick_place_two_object_cam
```
## Result multiple objects pick and place with camera
![multiple objects pick and place with camera](https://img.youtube.com/vi/WMds0gxj8UA/0.jpg)(https://youtu.be/WMds0gxj8UA)


## About Environment and Hardware details

### Environment Top View

![Top view Env](Results%20and%20Images/Environment_top_view.PNG)

### Robot: 

The interface of PincherX-100 uses joint variables defined such that the home configura on shown below corresponds to 𝜃 =	𝜃 =	𝜃 =	𝜃 = 0.	This is equivalent to adding a constant offset to the joint variables implied by the DH frames. More specifically, under those DH frames, the same configura on corresponds to  𝜃 = 0, 𝜃 =−1.2341, 𝜃 =+1.2341,𝜃 = 0 radians.

![Robotic Arm Joints](Results%20and%20Images/Robotic%20Arm%20Joints.PNG)

#### Auxiliary func ons (m-files): 

#### get_joint_pos():

Input: None 

Output:  

joint_pos: A 1x4 array of type int containing the current joint positions  

#### set_joint_pos(): 
Input: 

goal_pos: A 1x4 array of type int containing the goal joint positions. 

Output: 

valid: A boolean value which returns True if the goal positions are valid. 

#### closeGripper(): 
Input: 

val: A boolean value which if True closes the gripper and opens the gripper if False. 

Output: None 

#### get_pick_and_place_posi on(): 
Input: None

Output: 

pick_pos: A 4x4 array of type float containing the inital TF of the payload

place_pos: A 4x4 array of type float containing the TF of the goal pose 

obstacle_pos: A 4x4 array of type float containing the TF of the obstacle 
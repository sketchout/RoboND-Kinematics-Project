## Project: Kinematics Pick & Place


**Steps of this project:**  
---

1. Set up your ROS Workspace.
    - Download robo-nd Virtual Machine Image which include Ubuntu + ROS.
    - Run the VM and create "~/create catkin_ws/src folder" that is the ROS workspace.

2. Clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) and Download into the ***src*** directory of ROS Workspace.
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).

5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).

6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[dh_param_all]: ./misc_images/dh_param_all.png


### 5. Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.


1) Define Parameters

Parameter | Names | Definitions
--- | --- | ---
αi−1 | twist angle | Z^i−1 and Z^i measured about X^i−1 in a right-hand sense.
ai−1 | link length | distance from Z^i−1 to Z^i measured along X^i−1 where X^i−1 is perpendicular to both Z^i−1 to Z^i
di | link offset | signed distance from X^i−1 to X^i measured along Z^i
θi | joint angle | angle between X^i−1 to X^i measured about Z^i in a right-hand sense.

2) Get difference(Δ) from the kr210.urdf.xacro  and calculate the DH parameters

![alt text][dh_param_all]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### 6. Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]



## Project: Kinematics Pick & Place


**Steps of this project:**  
---

1. Set up your ROS Workspace.
    - Download robo-nd Virtual Machine Image which include Ubuntu + ROS.
    - Run the VM and create ***"~/catkin_ws/src"*** folder that is the ROS workspace.

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

**Writeup the some steps:**  
---

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
    Δ : joint_1(0, 0, 0.33), joint_2(0.35, 0, 0.42), joint_3(0, 0, 1.25)
    Δ : joint_4(0.96, 0, -0.054), joint_5(0.54, 0, 0), joint_6(0.193, 0, 0)
    Δ : gripper_joint(0.11, 0, 0)
    
3) All DH Parameters

    ![alt text][dh_param_all]

    * DH_Table
        
        Links | alpha0~6 | a0~6  | d1~7 | q1~7
        --- | --- | --- | --- | ---
        0->1 | 0 | 0 | 0.75 | q1
        1->2 | -pi/2. | 0.35 | 0 | -pi/2. + q2
        2->3 | 0 | 1.25 | 0 | q3
        3->4 | -pi/2. | -0.054 | 1.5 | q4
        4->5 | pi/2. | 0 | 0 | q5
        5->6 | -pi/2. | 0 | 0 | q6
        6->EE | 0 | 0 | 0.303 | 0
        
#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

1) Define [DH Transformation Matrix](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/7259f438-36a0-4bc1-ac53-39af669ba3c9) 
    
    - def TF_Matrix(alpha, a, d, q): 
            
2) Create Individual trnasformation matrices

    - T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table) ~
        T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

    And here's another image! 
    
    ![alt text][image2]

### 6. Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


    Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  
    
    
    And just for fun, another example image:
    ![alt text][image3]



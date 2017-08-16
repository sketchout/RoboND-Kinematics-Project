## Project: Kinematics Pick & Place


**Steps of this project:**  
---

1. Set up your ROS Workspace.
    - Download robo-nd Virtual Machine Image which include Ubuntu + ROS.
    - Run the VM and create ***"~/catkin_ws/src"*** folder that is the ROS workspace.

2. Clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) and Download into the ***src*** directory of ROS Workspace.
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
    - demo flag is true in inverse_kinematics.launch

5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
    - Calculate the  DH Parameters

6. Fill in the `IK_server.py` with your Inverse Kinematics code. 
    - To run IK code change demo flag to false and run "rosrun kuka_aram IK_server.py"


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[dh_param_all]: ./misc_images/dh_param_all.png

**Writeup the some steps:**  
---

### 5. Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.


1) Define Parameters Names and  DH Table

    Parameter | Names | Definitions
    --- | --- | ---
    αi−1 | twist angle | Z^i−1 and Z^i measured about X^i−1 in a right-hand sense.
    ai−1 | link length | distance from Z^i−1 to Z^i measured along X^i−1 where X^i−1 is perpendicular to both Z^i−1 to Z^i
    di | link offset | signed distance from X^i−1 to X^i measured along Z^i
    θi | joint angle | angle between X^i−1 to X^i measured about Z^i in a right-hand sense.

    Links | alpha(i-1) | a(i-1)  | d(i) | q(i)
    --- | --- | --- | --- | ---
    0->1 | 0 | 0 | d1 | q1
    1->2 | -pi/2. | a1 | 0 | -pi/2. + q2
    2->3 | 0 | a2 | 0 | q3
    3->4 | -pi/2. | a3 | d4 | q4
    4->5 | pi/2. | 0 | 0 | q5
    5->6 | -pi/2. | 0 | 0 | q6
    6->EE | 0 | 0 | d7 | 0

3) Get difference(Δ) from the kr210.urdf.xacro and calculate the DH parameters
    
    Links | joint(i) relative position | Δ 
    --- | --- | --- 
     0->1 | joint_1 (Δx, Δy, Δz)  |  (0, 0, 0.33)
     1->2 | joint_2 (Δx, Δy, Δz) |   (0.35, 0, 0.42)
     2->3 | joint_3 (Δx, Δy, Δz) |   (0, 0, 1.25)
     3->4 | joint_4 (Δx, Δy, Δz) |   (0.96, 0, -0.054)
     4->5 | joint_5 (Δx, Δy, Δz) |   (0.54, 0, 0)
     5->6 | joint_6 (Δx, Δy, Δz) |   (0.193, 0, 0)
     6->EE | gripper_joint (Δx, Δy, Δz) |   (0.11, 0, 0)
    
3) Calculate difference and get the final DH Parameter table

    * DH_Table
        
        Links | alpha0 ~ alpha6 | a0 ~ a6  | d1 ~ d7 | q1 ~ q7
        --- | --- | --- | --- | ---
        0->1 | 0 | 0 | 0.75(0.34+0.42) | q1
        1->2 | -pi/2. | 0.35(0+0.35) | 0 | -pi/2. + q2
        2->3 | 0 | 1.25(0+1.25) | 0 | q3
        3->4 | -pi/2. | -0.054(0-0.054) | 1.5(0.96+0.54) | q4
        4->5 | pi/2. | 0 | 0 | q5
        5->6 | -pi/2. | 0 | 0 | q6
        6->EE | 0 | 0 | 0.303(0.193+0.11) | 0

    ![alt text][dh_param_all]

        
#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

1) Define [DH Transformation Matrix](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/7259f438-36a0-4bc1-ac53-39af669ba3c9) 
    
    - Define function of the homogeneous transform maxtrix 
    
        TF = Matrix([[cos(q), -sin(q), 0 , 0],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alapha)*d],
                [ 0, 0, 0, 1]])
            
    - Create Individual trnasformation matrices (T0_1 ~ T6_EE)

        - T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table) = TF_Matrix(0, 0, 0.75, q1)
            = [[cos(q1), -sin(q1), 0 , 0],
                [sin(q1)*1, cos(q1)*1, 0, 0 ],
                [0, 0, 1, 1 * 0.75],
                [0, 0, 0, 1]]
        - T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
        - T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
        - T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
        - T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
        - T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
        - T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

    - Extract End-Effector & Rotation Matrix which is the gripper link position and orientation
        postion_EE = [[px], [py], [pz]]
        rotation_EE = RotZ(alpha) * RotY(beta) * RotX(gamma)
    - Get Homogeneoun transform matrix from base_link to gripper_link
        T0_7 = [ [rotation_EE, position_EE],[0, 0 ,0, 1]]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

    - Inverse Position Problem
        - Find Wrist Center Location 
            wx = position_EE[0] - d7 * rotation_EE[0, 0]
            wy = position_EE[1] - d7 * rotation_EE[1, 0]
            wz = position_EE[2] - d7 * rotation_EE[2, 0]
            
        - Find Theta 1-3 (q1 ~ q3)
            q1 = arctan2(wy, wx)
        - Find Theta 4-6 (q4 ~ q6)

### 6. Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

- Fill in the 'IK_server.py"
    - DH Parameter Symbols
    - Joint Angle Symbols
    - Modified DH Parameters
    - Modified DH Transformation Matrix
    - Create Individual Transformation Matrices
    - Extract End-Effector Position and Orientation
    - Find End-Effector Rotation Matrix
    - Find Wrist Center Location
    - Finding Theta 1-3
    - Finding Theta 4-6
- Add the rosrun script to safe_spawner.sh
    $ rosrun kuka_arm IK_server.py

- And test the result with Gazebo and rviz

![alt text][image3]
![alt text][image2]



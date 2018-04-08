## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image4]: ./Writeup_Images/DHParametersSchematic.png
[image5]: ./Writeup_Images/Theta1-3Schematic.png

## [Rubric Points](https://review.udacity.com/#!/rubrics/972/view)
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The figure below shows the different symbolic DH parameters on the kuka arm. Then through looking at the URDF file all the DH parameter values were found. 

![alt text][image4]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 1.25 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

For this inverse kinematics problem to be solvable, it must first be decoupled into an inverse position kinematics and an inverse orientation kinematics. This is done by finding the the wrist center (WC) location. The wrist center, as shown in the diagram above, is where the DH origins for joints 4-6 coincide. Since the End Effector (EE) position is given, finding the WC position parameters are easy.

\begin{equation}
w\_x = p\_x - (d\_7 + l) \bullet n\_x
\end{equation}
\begin{equation}
w\_y = p\_y - (d\_7 + l) \bullet n\_y
\end{equation}
\begin{equation}
w\_z = p\_z - (d\_7 + l) \bullet n\_z
\end{equation}

Where,
Px, Py, Pz = end-effector positions. Wx, Wy, Wz = wrist positions. d6 = from DH table. l = end-effector length

```py
# Find Wrist Center Location
wx = px - (d_7 * R0_6[0, 2])
wy = py - (d_7 * R0_6[1, 2])
wz = pz - (d_7 * R0_6[2, 2])
``` 

Now that WC is found, we can focus on finding theta 1-3 and thetas 4-6 separately. Thetas 1-3 represent the position of WC because joints 1-3 are what control the WC positioning. However, thetas 4-6 represent the orientation of the EE with respect to WC. Thus we have decoupled the Inverse Kinematics IK problem.

##### Theta 1

The image below is courtesy of Wenjin Tao from the Nanodegree program. These diagrams show a clear representation of all the angles and link lengths necessary to calculate thetas 1, 2, and 3.

![alt text][image5]

The WC is projected on the xy-plane. Then using the wx and wy the angle is found as shown below.

```py
theta1 = (atan2(wy, wx)).evalf()
```
##### Theta 2 and 3

To find theta 2 some angles and sides must first be defined. The equations below show how each was derived and their implementation in code. The [law of cosines](http://mathworld.wolfram.com/LawofCosines.html) was used extensively here. It is important to note that betas 2 and 3 have been found using atan2 rather than just acos. That was done to ensure the angles found are in the correct quadrant.

```py
s1 = sqrt(wx**2 + wy**2) - a_1
s2 = wz - d_1
s3 = sqrt(s2**2 + s1**2)
s4 = sqrt(a_3**2 + d_4**2)
beta1 = atan2(s2, s1)

D2 = (a_2**2 + s3**2 - s4**2) / (2 * a_2 * s3)
beta2 = atan2(sqrt(1 - D2**2), D2)
    
D3 = (a_2**2 + s4**2 - s3**2) / (2 * a_2 * s4)
beta3 = atan2(sqrt(1 - D3**2), D3)
    
beta4 = atan2(-a_3, d_4)
```

Since all the needed angles and lengths are now found, thetas 2 and 3 can be easily calculated.

```py
theta2 = ((pi / 2) - beta2 - beta1).evalf()
theta3 = ((pi / 2) - beta4 - beta3).evalf()
```

##### Thetas 4, 5, and 6

To find theta 4-6, we need R3\_6, which is the orientation transformation matrix from joint 3 to joint 6. This matrix includes all three angles, which can then be extracted. Since we are given the roll, pitch, and yaw of the EE, we can create a rotation matrix from the ground to joint 6, R0\_6. However, there is an orientation difference between R0\_6 and the actual orientation of the gripper as defined in the urdf file. This difference needs to be corrected. Once the correction is calculated, it is applied to the extrinsic rotation matrix resulting in R0\_6.

```py
R_z_intrinsic = Matrix([[    cos(pi),    -sin(pi),               0],
                    	[    sin(pi),     cos(pi),               0],
                    	[          0,           0,               1]])
R_y_intrinsic = Matrix([[ cos(-pi/2),           0,      sin(-pi/2)],
                    	[          0,           1,               0],
                    	[-sin(-pi/2),           0,      cos(-pi/2)]])
ZY_intrinsic_rot = R_z_intrinsic * R_y_intrinsic

# Calculate transformation matrix from base link to end-effector
R_z_extrinsic = Matrix([[   cos(yaw),   -sin(yaw),               0],
                 	    [   sin(yaw),    cos(yaw),               0],
                    	[          0,           0,               1]])
R_y_extrinsic = Matrix([[ cos(pitch),           0,      sin(pitch)],
                     	[          0,           1,               0],
                    	[-sin(pitch),           0,      cos(pitch)]])
R_x_extrinsic = Matrix([[          1,           0,               0],
                    	[          0,   cos(roll),      -sin(roll)],
                    	[          0,   sin(roll),      cos(roll)]])
XYZ_extrinsic_rot = R_z_extrinsic * R_y_extrinsic * R_x_extrinsic
R0_6 = XYZ_extrinsic_rot * ZY_intrinsic_rot
```

Since we now have thetas 1-3 a transformation matrix from the ground, 0, to link 3, can be formulated. The code block below shows the initilization of the matrix, and then further down, the matrix is updated with the actual values for thetas 1-3.

To create the transformation matrix, the code below is used.

```py
# Create individual transformation matrices
T0_1 = Transformation_Matrix(q1, alpha0, a0, d1)
T0_1 = T0_1.subs(s)

T1_2 = Transformation_Matrix(q2, alpha1, a1, d2)
T1_2 = T1_2.subs(s)

T2_3 = Transformation_Matrix(q3, alpha2, a2, d3)
T2_3 = T2_3.subs(s)

T0_3 = simplify(T0_1 * T1_2 * T2_3)
```

The orientation matrix of R0\_3 is extracted from T0\_3 and its inverse is found. When R0\_6 is multiplied by the inverse of R0\_3, the result is R3\_6.

```py
R0_3 = T0_3[0:3, 0:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R0_3_inv = R0_3.transpose()
R3_6 = R0_3_inv * R0_6
```

In order to derive theta 4-6 a symbolic matrix of R3\_6 was created. The output is shown below. This matrix is created in `IK_debug.py`.

```py
Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
```

These equations are used to calculate the angles as below.

```py
r13 = R3_6[0, 2]
r33 = R3_6[2, 2]
r23 = R3_6[1, 2]
r21 = R3_6[1, 0]
r22 = R3_6[1, 1]
r12 = R3_6[0, 1]
r32 = R3_6[2, 1]
    
theta5 = (atan2(sqrt(r13**2 + r33**2), r23)).evalf()
theta4 = (atan2(r33, -r13)).evalf()
theta6 = (atan2(-r22, r21)).evalf()
```

The IK problem is solved!

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

##### Techniques Used

Creating a debugging script. I initially created my own, but `IK_debug.py` was equally fantastic. `IK_debug.py` has been a tremendous help in succeeding in this project. Since the simulation takes a very long time to operate, `IK_server.py` grows tiresome to work on directly. So `IK_debug.py` contains a bunch of cases against which the code output can be compared. 

Moving the `sympy` math out of the `for` loop. Creating the necessary transformation matrices takes a long time. Since this creation process needs to happen only once, it is removed from the loop. This reduces the run time significantly.

Including the Forward Kinematics FK in `IK_debug.py`. Since an arm can have multiple orientations, the errors in `IK_debug.py` can be non zero signifying that the angles are incorrect even if the arm is correct. An arm can still reach the correct EE in different orientations. Therefore, by including FK calculations, we can use the calculated angles to see if the correct EE is reached.

##### Reflections

If I were to redo the project from scratch. I would start by immediately creating a debug script to use. I would also take more time in understanding how to use RViz and Gazebo before jumping in. All in all, this has been a tough and tiresome project, but I learned much and more. 10/10 would do again.





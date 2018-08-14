[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic arm - Pick & Place project

in this project, my goal was to use the techniques and code examples presented in the Udacity course to control the KuKa KR210 Robot, to pick up an object from 9 possible locations and place in the bin successfully 8 out 10 times.
The goals / steps of this project are the following:

- Derive the DH Parameter Table for the Kuka KR210 robot
- Create the individual transformation matrices about each joint. 
- Generate a generalized homogeneous transform between base_link and gripper_link
- Derive the equations for individual joint angles

[//]: # (Image References)

[image1]: ./misc_images/kuka_arm_dh.jpg ""
[image2]: ./misc_images/kuka_arm_dh_0.jpg ""
[image3]: ./misc_images/kuka_arm_dh_0.jpg ""
[image4]: ./misc_images/gen_JointT.png ""
[image5]: ./misc_images/Joint_trans.jpg ""
[image6]: ./misc_images/Comb_trans.jpg ""
[image7]: ./misc_images/Theta1.jpg ""
[image8]: ./misc_images/Theta2-3.jpg ""
[image9]: ./misc_images/Grab_1.jpg ""
[image10]: ./misc_images/Grab_2.jpg ""


## Derive the DH Paramater Table. 

![alt text][image1]

Using the kr210.urdf.xacro file the joint offests can be extracted (in meters)

- J0 to J1 : x: 0.000 ; y: 0; z:   0.330
- J1 to J2 : x: 0.350 ; y: 0; z:   0.420
- J2 to J3 : x: 0.000 ; y: 0; z:   1.250
- J3 to J4 : x: 0.960 ; y: 0; z: - 0.054
- J4 to J5 : x: 0.540 ; y: 0; z:   0.000
- J5 to J6 : x: 0.193 ; y: 0; z:   0.000
- J6 to EE : x: 0.110 ; y: 0; z:   0.000

Which can be used with the twist angles to obtain the DH parameters 

![alt text][image2]

![alt text][image3]

Yielding the following DH Table

| Joint Link | α(rad) | a(m)   | d(m)  | θ(rad)   |
|------------|--------|--------|-------|----------|
| 0-1        | 0      | 0      | 0.75  | q1       |
| 1-2        | -pi/2  | 0.35   | 0     | q2 -pi/2 |
| 2-3        | 0      | 1.25   | 0     | q3       |
| 3-4        | -pi/2  | -0.054 | 1.5   | q4       |
| 4-5        | pi/2   | 0      | 0     | q5       |
| 5-6        | -pi/2  | 0      | 0     | q6       |
| 6-EE       | 0      | 0      | 0.303 | 0        |

## Create the Individual Transformation Matrices 
Using the DH parameters and the general expression of the Joint transformation matrix (seen below)

![alt text][image4]

We can derive the following Joint Transformation Matrices 

![alt text][image5]

Yielding the Transformation matrix from base link to the end effector.

![alt text][image6]

## Derive the equations for individual joint angles
Following the methodology presented in the course material and the walkthrough, we first calculate the Wrist Center using the flowing equation

WC = EE - 0.303*ROT_EE[:,2] 

where ROT_EE is the rotation matrix for the end effector which has been compensated for the discrepancy between the DH parameters and gazebo.

Using the WC position, theta1 is relatively straight forward to calculate.

![alt text][image7]

theta1 = atan2(WC[1], WC[0])

Theta2 and Theta3 are calculated using Cosine law and trigonometry.

![alt text][image8]

Using theta1, theta2, and theta3 we can calculate the Rotation Matrix R0_3

R0_3 = R0_1 * R1_2 * R2_3

and then calculate the rotation matrix R3_6

R3_6 = R0_3.inv("LU") * ROT_EE

Using the Rotation Matrix R3_6 we can calculate theta4. theta5 and theta6 using the following equations

theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])

## Final Results
The general movement of the robot is quite inefficient, but it does achieve the goal of pick and place the object into the bin.
See Images Below for Screenshot of working solution.

![alt text][image9]
![alt text][image10]

I also included a video of a single grab operation. 

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/InOza8rKRDs/0.jpg)](https://www.youtube.com/watch?v=InOza8rKRDs)

# Instalation &  Setup
Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.


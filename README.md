======
README	   
======

This document describes how to install the exekutor interface.

		Author: 		Chittaranjan Srinivas Swaminathan
		Instituition: 	Orebro University


###BRIEF DESCRIPTION

Exekutor currently has four capabilities.

1. MoveToExekutor - Uses the ros move_base action server to send robot to a goal on the map.

	Parameter format: 
		
		x y [theta] [xy_tolerance] [yaw_tolerance]
	
2. MoveToSimpleExekutor - Uses only the transform information between odom and base_link frames to move the base.

	Parameter format:
		
		x y [theta] [xy_tolerance] [yaw_tolerance]

3. MoveHandExekutor - Uses JacoROS to move the end effector to a desired pose.

	Parameter format:
	
		  i. cartesian relative del_x, del_y, del_z, del_pitch, del_yaw, del_roll
		 ii. cartesian absolute x, y, z, pitch, yaw, roll
		iii. joints relative del_j1, del_j2, del_j3, del_j4, del_j5, del_j6
		 iv. joints absolute j1, j2, j3, j4, j5, j6	

4. FiddleExekutor - Uses JacoROS to 'fiddle' with jaco's fingers.

	Parameter format:
	
		  i. Open
		 ii. Close
		iii. Absolute finger_1_value finger_2_value finger_3_value
		 iv. Absolute finger_values
		
5. PickUpExekutor - Uses Moveit and PCL table-top segmentation to pick up an object on the table.

	Parameter format: None (currently).
	
NOTE: In the current installation the PickUpExekutor has been omitted.

###INSTALLATION

####STEP 1:
ENSURE the latest version of PEIS is installed.

####STEP 2:
ROS up-to-date?

Do,

sudo apt-get update
sudo apt-get dist-upgrade


p.s. Don't start swearing already. You don't really have to do a dist-upgrade :P .... Just ensure ros-hydro-desktop-full is up-to-date.

####STEP 3:
INSTALL JacoROS.

Assuming you are now in the catkin_workspace/src folder,
Clone the repository using: 
	
	git clone https://github.com/ksatyaki/JacoROS.git

	cd ..
	
	catkin_make
	
	This builds the new package.
	
There is a huge probability that this step will fail.
If it says mono not found, try installing the following packages:
	
	a. mono-complete
	b. mono-gmcs
	c. mono-xbuild
	d. mono-dmcs
	e. libmono

If it still fails then check you /usr/lib directory for the shared object: libmono-2.0.so

If it is there, then in the folder jaco, simply use CMakeLists2.txt after renaming it to CMakeLists.txt and the original file to some other name.
	
Also, do the following:

	cd to the cloned "JacoROS" folder
	sudo cp udev/99-jaco-arm.rules /lib/udev/rules.d/
	
This copies the udev rules for Jaco USB access.

And finally (from inside the same JacoROS folder):

	sudo cp -r Kinova ~

####STEP 4:
INSTALL simple_service

Assuming you are now in the catkin_workspace/src folder,

	git clone https://github.com/ksatyaki/simple_service.git
	
	cd ..
	
	catkin_make
	
	This builds the new package.

####STEP 5:
INSTALL exekutor

Assuming you are now in the catkin_workspace/src folder,

	git clone https://github.com/ksatyaki/exekutor.git
	
	cd ..
	
	catkin_make
	
	This builds the new package.

If this fails because a dependent package can't be found, then ensure that the following packages are installed:

1. ros-hydro-move-base
2. ros-hydro-move-base-msgs
3. ros-hydro-actionlib
4. ros-hydro-actionlib-msgs

####STEP 6:
INSTALL test_exekutor

Unzip test_exekutor to another directory and type:

	cmake .

	make
	
###RUNNING AN EXAMPLE

Assuming your ros environment is set-up,

EXAMPLE:

	Using MoveHandExekutor:
	Run the following in different terminals:
	
	1. roslaunch jaco start.launch
	2. rosrun exekutor unit_test
	3. <path_to_test_exekutor>/bin/test_pseudo_cpm
	4. tupleview
	5. [Optional] rosrun simple_service joint_state_to_peis
	
	In tupleview please key in the desired values under the tuples separated by the dots.
	These will appear under test_pseudo_cpm.
	Once you are done, put the command tuple to ON and link the tuples manually
	to the tuples under unit_test.
	
	To link a meta tuple, select the tuple (under unit test) and 
	select the tuple to link (under test_pseudo_cpm) on the right in tupleview.
	
	===========================================================================	
	CAUTION: Link parameter and state tuples before you link the command tuple.
	===========================================================================


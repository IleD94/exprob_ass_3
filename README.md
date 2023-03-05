# exprob_ass_3

![image](https://user-images.githubusercontent.com/80365922/222982427-8be06d18-7be1-4710-abd2-9185e7ff0a3b.png)


## Introduction 
This project is the third part out of three of the implementation of Cluedo Game in simulated environment. In this part we have implemented the architecture of the game, the real movement of the player, a robot Detective, on gazebo simuator and the perception part, using a camera and some markers to detect in the environment. The architecture is divided in three main part: perception, implemented in the last part using aruco_ros package, action, fully implemented using moveit package and move_base package and planning, that actually is simulated using a final state machine, with the smach ros package. 
The rules of the game are simple. There was a omicide, Mr. Black was killed by someone, somewhere with some weapon. There is a detective, Detective Bot, that goes around (in gazebo envoronment) to find hints to solve the case. Hints can be found looking at some aruco markers positionated on cubes in several places of the environment. In the world simulated there are 6 rooms, each of them has 5 aruco markers that can be found at different heights.
The coordinates of the rooms are: 

- Room1: [-4,-3], 
- Room2: [-4,2], 
- Room3: [-4,7], 
- Room4:[5,-7] , 
- Room5:[5,-3], 
- Room6:[5,1].

To have general information about the game please go to the [first repository](https://github.com/IleD94/exprob_ass_1) of this project.

## Robot model
The robot model is the same described and provided in the [second part](https://github.com/IleD94/exprob_ass_2) of this project. The difference is that here we add also a camera, as it is possible to notice in the urdf, in xacro and gazebo file provided in the urdf folder of this repository. In order to reach better some markers it is possible to change some parameters of the camera, like resolution. But in our experience it was not necessary to win the game.

## MoveIt
Like in the second part of the project, even in this part we have a provided our robot with an arm, in order to catch better the markers. But the poses in this case are different. In [exp_ass3_moveit package](https://github.com/IleD94/exprob_ass_3/tree/main/exp_ass3_moveit), using moveit setup_assistant is possible to find new poses. They are:
1. *zero*: It is used to move from a room to another
2. *investigation* is the position used inside a room to look around to find aruco markers. At some point, if the robot could catch some markers, it starts to move from a room to another keeping the investigation pose.
Below the zero pose:

![zero_pose](https://user-images.githubusercontent.com/80365922/222979884-4b90011d-6ff3-4921-8206-b265f232ae3c.png)

Below the investigation pose:

![investigation_pose](https://user-images.githubusercontent.com/80365922/222979891-8eb79df3-8489-4738-943b-d4cb5e0489b5.png)


## Software Architecture
### Component diagram
![assignment3 drawio](https://user-images.githubusercontent.com/80365922/222980019-e6d07e6c-22a4-4567-b6e1-b9b866cd6ee3.png)
### Nodes
1. *Ontology Settings*: This is a Python script that sets up the Cluedo ontology by adding classes and individuals to the OWL file using the Armor Client API. The script defines a class called DefaultSettings that has several methods. The load_ontology method loads the ontology file into the Armor Server and sets the reasoner. The add_item_into_class method adds individuals to the relevant classes, namely suspects, weapons, and rooms. Finally, the changes_and_apply method saves the changes and applies the reasoner to them.
2. *User_interface*: this node subscribes to the cluedo_ui topic and displays messages, useful for the game, from it to the user.
![image](https://user-images.githubusercontent.com/80365922/222980993-2b43e26f-4301-44c2-9598-89b931e710ac.png)

3. *marker_publish*: This is a C++ ROS node that uses the ArUco library to detect and publish the IDs of ArUco markers visible in a camera image. The node subscribes to the /image topic, which should be of type sensor_msgs/Image, and publishes the detected marker IDs on the /aruco/marker_id topic, which is of type std_msgs/Int32. The node initializes the ArUco marker detector and camera parameters, and then defines a callback function called image_callback that is executed whenever an image message is received on the /image topic. The callback function converts the image message to a cv::Mat object, detects the markers in the image using the mDetector_.detect function, and then publishes the detected marker IDs on the /aruco/marker_id topic.
4. *simulation*: This node was implemented by the prof. Carmine Recchiuto. It manages hints generation and the oracle check of the winning hypothesis and it implements also the environment of the simulation on gazebo.
5. *move_arm*: This node implements the movement of the robotic arm. It can be set in the zero pose or in the investigation pose. The position is chosen by the client of this service, that is in the cluedo_fsm node.
5. *cluedo_fsm*: This node manages all the part of the game using a state machine. It has 4 states: Exploration, LookAround, Query and myOracle.
- **Exploration**: In this state the robot sets its arm to the zero position and using move_base package goes to a chosen randomly room among those in the list. If during the movement it can catch a aruco marker, it stops and goes to the LookAround state to check if the marker was already detected before. This state restart from the point where the robot stop to complete the task of reaching that specific room.
- **LookAround**:  in this state the robot moves the arm to the invastigation pose and start to spin, if the robot reached the room, to detect a marker. If the marker is detected it goes to the Query state to check the completeness and the consistency of the hypothesis, otherwise it continues to spin until it does about 2 rounds.
- **Query**: in this state the robot checks the consistency and the completeness of the hypothesis found, using the armor services. If the hypothesis is consistent and was not checked to the oracle yet.
- **Oracle**: this is the last state of the machine. Here the robot check if the consistent hypothesis found is the winning one or not. If it is, Detective Bot wins the game, otherwise it goes again to the exploration stage.
### State Machine
![image](https://user-images.githubusercontent.com/80365922/222980920-b4fce1bf-a8dd-4914-a87d-7bac968efb4a.png)

## How to install and run
In order to install this package, please clone the repository, in this way:
```
git clone https://github.com/IleD94/exprob_ass_3
```
Please dowload also the model markers that you can find in the branch models of this repository and put that in the file .gazebo in your workspace. To download do:
```
git clone https://github.com/IleD94/exprob_ass_3/tree/models
```
Move to another directory the folder exp_moveit_ass3 and build. Please consider that it could have some conflicts with the moveit setup of the previous part of this project. In this case just put them in different workspace or build them not at the same time. For this reason, unfortunately we had to remove all the dependecies of the third part on the second. The command to build and compile is:
```
catkin_make --only-pkg-with-deps exp_moveit_ass3
```
After that move also the aruco_ros package to another folder in the workspace and build it with:
```
catkin_make --only-pkg-with-deps aruco_ros
```
After that please build and compile also the package of this repository, without the directories described above, with:
```
catkin_make --only-pkg-with-deps exprob_ass_3
```
At this point, to run the project lauch this nodes using the commands below in this order:
```
rosrun exp_ass_3 cluedo_fsm.py
```
```
rosrun aruco_ros marker_publisher
```
```
rosrun exprob_ass_3 OntologySettings.py
```
```
roslaunch exprob_ass_3 my_scripts.launch 2>/dev/null
```
```
roslaunch exprob_ass_3 myass_gaz.launch 2>/dev/null
```
```
rosrun exprob_ass_3 user_interface.py 
```

## Video


https://user-images.githubusercontent.com/80365922/222982019-e6af0b26-29a5-48c6-a406-5bae81f94757.mp4



## Contacts
Author: Ilenia D'Angelo,

email: ileniadangelo94@gmail.com

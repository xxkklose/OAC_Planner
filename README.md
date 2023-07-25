# OAC_Planner
 
This project is a **planning algorithm** for **crawler robots** with swing arms to **cross** and **avoid** obstacles in complex unstructured scenes, the full name is obstacle avoidance and obstacle crossing planner(**OAC_Planner**)

 
## Getting Started
 
### Prerequisites

#### 1.1 Ubuntu and ROS
Ubuntu >= 18.04

ROS >= noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation).

#### 1.2. PCL && Eigen
PCL >= 1.8, Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

#### 1.3. livox_ros_driver(optical)
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

Remarks:

The overall planner is based on the FAST_LIO and LIO_SAM SLAM method to complete the positioning and mapping, and the FAST_LIO is completely based on the Livox series of radars, so if you use the SLAM algorithm in the project or other SLAM algorithms based on Livox radar, you need to install Livox ROS Driver.


#### 1.4 GTSAM

Gtsam >= 4.0.0 Follow [Gtsam Installation](https://github.com/borglab/gtsam).

#### 1.5 Necessary Package

```
sudo apt-get install ros-noetic-ompl
sudo apt-get install ros-noetic-robot-state-publisher*
sudo apt-get install ros-noetic-joint-state-controller*
sudo apt-get install ros-noetic-controller*
sudo apt-get install ros-noetic-velocity-controllers*
sudo apt-get install ros-noetic-eigen*
sudo apt-get install ros-noetic-velodyne*
pip install casadi


sudo apt-get install ros-melodic-geographic-*
sudo apt-get install geographiclib-*
sudo apt-get install libgeographic-*

sudo apt-get install ros-noetic-rviz-visual-tool

```

### Installing
 
#### Step.1

 Clone the repository and catkin_make.
 
```bash
mkdir -p OAC_Planner/src
cd OAC_Planner/src/
catkin_init_workspace
git clone https://github.com/xxkklose/OAC_Planner.git
cd ..
catkin_make
```
 
#### Step.2
 
1. Start with a dataset

```
cd OAC_Planner
source devel/setup.bash
roslaunch planner bringup.launch
```

2. Start with a Livox series Lidar(Defult horizon)

```
cd OAC_Planner
source devel/setup.bash
roslaunch planner horizon.launch
```
 

 <!-- TODO -->
<!-- End with an example of getting some data out of the system or using it for a little demo
 
## Running the tests
 
Explain how to run the automated tests for this system
 
### Break down into end to end tests
 
Explain what these tests test and why
 
```
Give an example
```
 
### And coding style tests
 
Explain what these tests test and why
 
```
Give an example
```
 
## Deployment
 
Add additional notes about how to deploy this on a live system
 
## Built With
 
* [Dropwizard](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [Maven](https://maven.apache.org/) - Dependency Management
* [ROME](https://rometools.github.io/rome/) - Used to generate RSS Feeds
 
## Contributing
 
Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.
 
## Versioning
 
We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 
 
## Authors
 
* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)
 
See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.
 
## License
 
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
 
## Acknowledgments
 
* Hat tip to anyone whose code was used
* Inspiration
* etc -->
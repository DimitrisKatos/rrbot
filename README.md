# Create your first robot arm
In this tutorial we will build a two degrees of freedom robotics arm using xacro methods. The simulated robot arm, rrbot, has two revolote joints and a gripper in the third which is a mesh file. 

Totaly, you will learn the following:
- The advantages of using Xacro in a URDF
- Designing a three-link, two-join robotic arm using Xacro and mesh files.
- Controlling the arm in Gazebo classic using the ros2_control package. 

Before we move forward, in the following link you can find a presentation tha will guide you through this tutorial. In the presentation you can find information about xacro properties, creating a new ROS 2 package and how to set up it. Also gives you information on how to build a two degree robot arm and how to use ros2_control package to control the robot.

- [rrbot package presantation](https://docs.google.com/presentation/d/1dzuV5KVsP0y9m2f1q3UD7XVk2-SVqx-K/edit?usp=drive_web&ouid=106628092038381749227&rtpof=true) Greek language.
- [rrbot package presantation] English language (coming soon).

## Features of Xacro. 
Xacro is the XML macro language for ROS 2. Xacro provides a set of macro operation to replace some statements with sorter macros. Xacro can be used in any XML document, but is most usefull in long and complex URDF files. In Generally, xacro provides you methods that allow you to create more readable XML files for your URDF robot. 
Xacro provides you some advantages in many areas:
- **Properties and property block** : If repeated information is used in a URDF file, the property tag can be used to specify some constants. With the following expression you can create a constant named height and give it the value 0.5:
``` xml 
<xacro:property name=”height” value=”0.5” />
```
- **Simple math** : You can make some mathematicals calculation with the following operation +, -, * and /. The expression must be enclosed in the ${} construct. 

- **Macros** : This is the main feature of Xacro. You can create a macro by using the `<macro>` tag. Macros are helpfull when statements are repeated. 

- **Combining mulitple Xacro files** : Other Xacro file can be included in the main URDF file. You can use the following tag. `<xacro:include filename="$(find package_name)/filename" />`.

- Every xacro file must contain an extra declaration in the second line in the Xacro file. This declaration is vital for the file to parse properly. In every xacro file you must put the following
```xml
 <robot name="name_of_your_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
```

- More information about the xacro properties can be found [here](http://wiki.ros.org/xacro).

## Create, setup and build rrbot package. 

In this chapter, we will create a new ROS 2 packages using Python and setup the package.

Create rrbot package and build it:
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament-python rrbot
cd ~/ros2_ws
colcon build --packages-select rrbot
```

Create some folders in the package:
```
cd ~/ros2_ws/src/rrbot
mkdir urdf launch world meshes config
```

For building the packages in ROS 2 you must informed the setup.py file. In  this file you should make the following changes. At first, add two python libraries. 
``` py
import os
from glob import glob 
``` 
Also you must add the following paths to the data_files Python list.
```py
(os.path.join('share',package_name,'launch'),
         glob(os.path.join('launch','*.launch.py'))),
(os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.xacro'))),
(os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.gazebo'))),
(os.path.join('share',package_name,'worlds'),
         glob(os.path.join('worlds','*.world'))),
(os.path.join('share',package_name,'meshes'),
         glob(os.path.join('meshes','*.dae'))),
(os.path.join('share',package_name,'meshes'),
         glob(os.path.join('meshes','*.png'))),
(os.path.join('share',package_name,'config'),
         glob(os.path.join('config','*.yaml')))
```
The seutp.py file should look like this.

![Poll Mockup](./images/image1.png)

In the following step we will build our package using colcon tool.
```
cd ~/ros2_ws
colcon build --packages-select rrobt
```

## Create your first robot arm using xacro features.

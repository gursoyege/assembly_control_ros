# Assembly control

This is the base ROS package to be used to complete the [HMEE303 project](https://umrob.gitlab.io/control%20architectures/hmee303-project).

## Project organization
- CMakeLists.txt: Root CMake configuration file
- src: Source files and CMake description of the libraries and applications
  - vrep_remote_api: library to communicate with the V-REP simulator 
  - vrep: the node in charge of the communication with the simulator
  - supply_conveyor: example implementation of the supply conveyor
  - camera: example implementation of the camera
- include: Headers files 
  - common: folder containing declarations needed by several components
  - vrep_remote_api: public header files for the vrep_remote_api library
  - vrep: header files for the vrep node
- launch: ROS launch files
- msg: ROS message files
- share: Additional files
  - petri_nets: where to put the TINA files corresponding to the cell modelisation
  - vrep/assembly_cell.ttt: scene to be loaded in V-REP simulating the cell

## Where to work
 - Petri nets go to *share/petri_nets/{without_io,with_io}*
 - Custom input and output messages to go *msg*
 - Each node goes in an *src* subfolder matching its Petri net's name

Don't forget to declare your new nodes in *src/CMakeLists.txt* (you can use the `add_machine` function) and to add them in the *launch/assembly.launch* launch file.

You can also indicate your name(s) and e-mail address(es) in the `maintainer` field of the *package.xml* file.

## How to run the project

First open V-REP and load the *vrep/assembly_cell.ttt* scene.

Then just use the *launch/assembly.launch* launch file:
```
roslaunch assembly_control_ros assembly.launch
```
If V-REP doesn't run locally you can specify its IP address using for example:
```
roslaunch assembly_control_ros assembly.launch vrep_ip:=172.17.0.1
```
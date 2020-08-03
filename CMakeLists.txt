cmake_minimum_required(VERSION 3.2.0)
project(assembly_control_ros)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    message_generation
)

find_package(Threads)

# Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs
)

# The catkin_package macro generates cmake config files for your package
catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
)

# Add the content of src/CMakeLists.txt
add_subdirectory(src)

# Force the generation of a compile_commands.json file to provide autocompletion for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL "" FORCE)
###                         V-REP interface                         ###
if(APPLE)
set(VREP_DEFINITIONS "NON_MATLAB_PARSING;MAX_EXT_API_CONNECTIONS=255;__APPLE__")
elseif(UNIX)
set(VREP_DEFINITIONS "NON_MATLAB_PARSING;MAX_EXT_API_CONNECTIONS=255;__linux")
elseif(WIN32)
set(VREP_DEFINITIONS "NON_MATLAB_PARSING;MAX_EXT_API_CONNECTIONS=255;_WIN32")
endif()

file(GLOB remote_api_FILES ${CMAKE_CURRENT_SOURCE_DIR}/vrep_remote_api/*.c)
add_library(remote_api SHARED ${remote_api_FILES})
target_compile_definitions(remote_api PUBLIC ${VREP_DEFINITIONS})
target_include_directories(remote_api PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include/vrep_remote_api)


file(GLOB vrep_FILES ${CMAKE_CURRENT_SOURCE_DIR}/vrep/*.cpp)
add_executable(vrep ${vrep_FILES})
target_compile_features(vrep PUBLIC cxx_std_14)
target_include_directories(vrep PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include ${catkin_INCLUDE_DIRS})
target_link_libraries(vrep remote_api ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} atomic)
add_dependencies(vrep assembly_control_ros_generate_messages_cpp)

###                         Machines                             ###
function(add_machine name)
    file(GLOB ${name}_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${name}/*.cpp)
    add_executable(${name} ${${name}_FILES})
    target_compile_features(${name} PUBLIC cxx_std_14)
    target_include_directories(${name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include ${catkin_INCLUDE_DIRS})
    target_link_libraries(${name} ${catkin_LIBRARIES})
    add_dependencies(${name} assembly_control_ros_generate_messages_cpp)
endfunction()
    
add_machine(supply_conveyor)
add_machine(camera)
add_machine(evacuation_conveyor)
add_machine(assembly_station)
add_machine(robot)
add_machine(controller)
# add your other nodes here

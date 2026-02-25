cmake_minimum_required(VERSION 3.0.2)
project(first_project)

## Trova le dipendenze necessarie
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

## Dichiara il pacchetto catkin
catkin_package()

## Includi i file di intestazione
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Compila il nodo odometer
add_executable(odometer src/odometer.cpp)

## Collega le librerie necessarie
target_link_libraries(odometer
  ${catkin_LIBRARIES}
)

add_executable(gps_odometer src/gps_odometer.cpp)
target_link_libraries(gps_odometer ${catkin_LIBRARIES})
add_dependencies(gps_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})





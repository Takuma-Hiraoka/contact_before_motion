cmake_minimum_required(VERSION 3.0.2)
project(contact_before_motion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  graph_search
  prioritized_inverse_kinematics_solver2
  global_inverse_kinematics_solver
  ik_constraint2_scfr
  ik_constraint2_distance_field
  ik_constraint2_bullet
  ik_constraint2_body_contact
  choreonoid_contact_candidate_generator
  )

find_package(choreonoid REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS graph_search prioritized_inverse_kinematics_solver2 global_inverse_kinematics_solver ik_constraint2_scfr ik_constraint2_distance_field ik_constraint2_bullet ik_constraint2_body_contact choreonoid_contact_candidate_generator
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME} SHARED
  src/contact_before_motion.cpp
  src/util.cpp
  )

target_link_libraries(${PROJECT_NAME}  PUBLIC
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

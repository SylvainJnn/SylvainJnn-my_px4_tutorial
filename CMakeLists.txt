cmake_minimum_required(VERSION 3.8)
project(my_px4_tutorial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Define the include directories for all targets
include_directories(include)

# Créer la bibliothèque pour OffboardControl
add_library(OffboardControl_lib
  src/OffboardControl.cpp
)
# Lier les bibliothèques nécessaires à la bibliothèque OffboardControl_lib
ament_target_dependencies(OffboardControl_lib
  rclcpp
  px4_msgs
)

# Add executable for test_offboard
add_executable(test_offboard src/test_offboard.cpp)
ament_target_dependencies(test_offboard rclcpp px4_msgs std_srvs)
# install(TARGETS test_offboard DESTINATION lib/${PROJECT_NAME})


# # plus besoin car il n'y a pas de main ? 
# # Add executable for OffboardControl
# add_executable(OffboardControl src/OffboardControl.cpp)
# ament_target_dependencies(OffboardControl rclcpp px4_msgs)
# install(TARGETS OffboardControl DESTINATION lib/${PROJECT_NAME})



# Add executable for jsp
add_executable(jsp src/jsp.cpp)
# Lier la bibliothèque OffboardControl_lib à l'exécutable jsp_node
target_link_libraries(jsp
  OffboardControl_lib
)
ament_target_dependencies(jsp rclcpp px4_msgs)
# install(TARGETS jsp DESTINATION lib/${PROJECT_NAME}) // elle y est en double non ? 


# Installer les fichiers d'en-tête et la bibliothèque
install(TARGETS
  test_offboard
  OffboardControl_lib
  jsp
  DESTINATION lib/${PROJECT_NAME}
)

# je sais pas non plus 
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)



# Install targets
# install(TARGETS test_offboard DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
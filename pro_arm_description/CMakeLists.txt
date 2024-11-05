cmake_minimum_required(VERSION 3.8)
project(pro_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Setup environment hooks that automatically prepend IGN_GAZEBO_RESOURCE_PATH and SDF_PATH when sourcing local_setup.<ext>
ament_environment_hooks(
  "cmake/environment_hooks/ign_gazebo_resource_path.bash"
  "cmake/environment_hooks/sdf_path.bash"
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Install directories
install(DIRECTORY launch models models/meshes_550 models/meshes_900 models/grippers rviz scripts urdf DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(cloud_process)

find_package (ament_cmake_auto REQUIRED) 
ament_auto_find_build_dependencies ()


include_directories(
  ${PROJECT_SOURCE_DIR}/include/cloud_process
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/cloud_process.cpp

)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN cloud_process::PointCloudPreprocessor
  EXECUTABLE cloud_process_node
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()


# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_auto_package()


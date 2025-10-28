cmake_minimum_required(VERSION 3.8)
project(pingpong_gunner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# package.xml から依存関係を読んで find_package
ament_auto_find_build_dependencies()

# ライブラリ関係 ament_target_dependencies
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/core.cpp)

# 実行ファイル関係 add_executable
ament_auto_add_executable(pingpong_gunner_exe
  src/core.cpp)

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

# install
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)



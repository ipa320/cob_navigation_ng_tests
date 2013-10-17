cmake_minimum_required(VERSION 2.8.3)
project(ubisense_tests)

find_package(catkin REQUIRED rostest)

catkin_package()


## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ubisense_tests.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_rostest(launch/test_data_dumps.test)
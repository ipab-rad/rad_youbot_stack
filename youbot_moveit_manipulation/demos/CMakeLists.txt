add_executable(motion_demo src/motion_demo.cpp)
target_link_libraries(motion_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS motion_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
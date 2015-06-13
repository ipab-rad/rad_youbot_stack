add_executable(kinematics_tutorial src/kinematics_tutorial.cpp)
target_link_libraries(kinematics_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS kinematics_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
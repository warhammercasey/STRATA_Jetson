execute_process(COMMAND "/home/strata/git/RADAR/STRATA_Jetson/catkin/build/radar/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/strata/git/RADAR/STRATA_Jetson/catkin/build/radar/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

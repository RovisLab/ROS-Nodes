if (CATKIN_ENABLE_TESTING)
  catkin_download_test_data(ecto_ros_test_data https://github.com/plasmodic/ecto_data/raw/master/t01.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 25b21110c1077cd2e3e1dc9fc0775baa
  )
  catkin_add_nosetests(test_bag.py DEPENDENCIES ecto_ros_test_data)
  catkin_add_nosetests(test_sync_sub.py DEPENDENCIES ecto_ros_test_data)
  catkin_add_nosetests(test_image_sub.py DEPENDENCIES ecto_ros_test_data)
  catkin_add_nosetests(test_reinit.py DEPENDENCIES ecto_ros_test_data)
endif()

foreach(file pubsub_gen_wrap.cmake gen_msg_wrappers.py)
    file(COPY ${PROJECT_SOURCE_DIR}/cmake/${file}
         DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
         USE_SOURCE_PERMISSIONS
    )
endforeach()

install(PROGRAMS ${PROJECT_SOURCE_DIR}/cmake/gen_msg_wrappers.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/
)
install(FILES ${PROJECT_SOURCE_DIR}/cmake/pubsub_gen_wrap.cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/
)

# install the include folder
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ecto_ros/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        COMPONENT main
)

include(pubsub_gen_wrap.cmake)

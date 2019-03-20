add_library(ztx
            Preprocess.cpp
            )
target_link_libraries(ztx ${catkin_LIBRARIES} ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(kcf_track)


find_package(OpenCV  REQUIRED)
find_package(Boost  REQUIRED timer)
find_package(Eigen)
find_package(PCL  REQUIRED COMPONENTS common io)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set( OPENNI_INCLUDES "/opt/OpenNI2/Linux-x86_64/Include")
set( OPENNI_LIB_DIR      "/opt/OpenNI2/Linux-x86_64/Redist")

##set( PCL_INCLUDES "/usr/include/pcl-1.7")
##set( PCL_LIB_DIR      "/usr/lib")
include_directories(include  ##头文件的目录
                    ${Boost_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}/include/
                    ${OPENNI_INCLUDES}
                    ##${PCL_INCLUDES}
                    ${PCL_INCLUDE_DIRS}
                    /usr/include/eigen3/
                    ##"/opt/OpenNI2/Linux-x86_64/Include"
                    )

link_directories ( ${OpenCV_LIB_DIR}  ##so的路径
                   ${OPENNI_LIB_DIR}
                   ${PCL_LIBRARY_DIRS}
                   ## "/opt/OpenNI2/Linux-x86_64/Redist"
)


##INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR}/)


add_executable( TrackTest  src/fhog.cpp src/kcftracker.cpp src/TrackTest.cpp)                                                      
target_link_libraries( TrackTest ${OpenCV_LIBRARIES} ${OpenCV_LIBS} OpenNI2)##so的名称

add_executable( kcf_use_benchmark  src/image_acquisition.cpp src/fhog.cpp src/kcftracker.cpp src/kcf_use_benchmark.cpp)                                                      
target_link_libraries( kcf_use_benchmark ${OpenCV_LIBRARIES} ${OpenCV_LIBS} OpenNI2)##so的名称

add_executable( kcf_rgb_depth  src/fhog.cpp src/kcftracker.cpp src/kcf_rgb_depth.cpp)                                                      
target_link_libraries( kcf_rgb_depth ${OpenCV_LIBRARIES} ${OpenCV_LIBS} OpenNI2)##so的名称

add_executable( kcf_use_class  src/image_acquisition.cpp src/fhog.cpp src/kcftracker.cpp src/kcf_use_class.cpp )                                                      
target_link_libraries( kcf_use_class ${OpenCV_LIBRARIES} ${OpenCV_LIBS} OpenNI2)##so的名称

add_executable(pcl_test  src/image_acquisition.cpp src/pcl.cpp)                                                      
##target_link_libraries( pcl ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )##so的名称
target_link_libraries(pcl_test ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARY_DIRS}/libpcl_io.so ${PCL_LIBRARY_DIRS}/libpcl_common.so ${PCL_LIBRARY_DIRS}/libpcl_visualization.so  /usr/lib/x86_64-linux-gnu/libboost_system.so )##so的名称

add_executable(lccp  src/lccp_pcl.cpp)                                                      
##target_link_libraries( pcl ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} )##so的名称
target_link_libraries(lccp ${OpenCV_LIBRARIES} 
                           ${OpenCV_LIBS} 
                           ${PCL_LIBRARY_DIRS}/libpcl_io.so 
                           ${PCL_LIBRARY_DIRS}/libpcl_common.so 
                           ${PCL_LIBRARY_DIRS}/libpcl_visualization.so 
                           ${PCL_LIBRARY_DIRS}/libpcl_segmentation.so 
                           /usr/lib/x86_64-linux-gnu/libboost_system.so )##so的名称




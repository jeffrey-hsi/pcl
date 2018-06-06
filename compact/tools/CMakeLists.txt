# pcl_compact_process
PCL_ADD_EXECUTABLE(pcl_compact_process "${SUBSYS_NAME}" compact_process.cpp)
target_link_libraries(pcl_compact_process pcl_common pcl_filters pcl_io pcl_octree pcl_compact)

PCL_ADD_EXECUTABLE(pcl_compact_print "${SUBSYS_NAME}" compact_print.cpp)
target_link_libraries(pcl_compact_print pcl_common pcl_filters pcl_io pcl_octree pcl_compact)

if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else(NOT VTK_FOUND)
  set(DEFAULT TRUE)
  set(REASON)
  set(VTK_USE_FILE "${VTK_USE_FILE}" CACHE INTERNAL "VTK_USE_FILE")
  include("${VTK_USE_FILE}")
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
  
  if("${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}" VERSION_GREATER "5.8")
    # pcl_compact_viewer uses some functions not present in vtk 5.8
  
    set(srcs compact_viewer.cpp
           ../src/visualization/camera.cpp
           ../src/visualization/common.cpp
           ../src/visualization/object.cpp
           ../src/visualization/grid.cpp
           ../src/visualization/compact_cloud.cpp
           ../src/visualization/scene.cpp
           ../src/visualization/viewport.cpp)
  
    # pcl_outofcore_viewer
    PCL_ADD_EXECUTABLE_OPT_BUNDLE(pcl_compact_viewer "${SUBSYS_NAME}" ${srcs})
    target_link_libraries(pcl_compact_viewer pcl_common pcl_io pcl_compact pcl_visualization pcl_octree pcl_filters)
  
  endif("${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}" VERSION_GREATER "5.8")
endif(NOT VTK_FOUND)

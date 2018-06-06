set(SUBSYS_NAME compact)
set(SUBSYS_DESC "Point cloud compact library")
set(SUBSYS_DEPS common io filters octree visualization)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
    set(srcs
        src/cJSON.cpp
        src/compact_node_metadata.cpp
        src/compact_base_metadata.cpp
        )

    set(incs
        "include/pcl/${SUBSYS_NAME}/metadata.h"
        "include/pcl/${SUBSYS_NAME}/compact_base_metadata.h"
        "include/pcl/${SUBSYS_NAME}/compact_node_metadata.h"
        "include/pcl/${SUBSYS_NAME}/compact_iterator_base.h"
        "include/pcl/${SUBSYS_NAME}/compact_breadth_first_iterator.h"
        "include/pcl/${SUBSYS_NAME}/compact_depth_first_iterator.h"
        "include/pcl/${SUBSYS_NAME}/boost.h"
        "include/pcl/${SUBSYS_NAME}/cJSON.h"
        "include/pcl/${SUBSYS_NAME}/compact_octree_base.h"
        "include/pcl/${SUBSYS_NAME}/compact_octree_base_node.h"
        "include/pcl/${SUBSYS_NAME}/compact_octree_abstract_node_container.h"
        "include/pcl/${SUBSYS_NAME}/compact_octree_disk_container.h"
        "include/pcl/${SUBSYS_NAME}/compact_octree_ram_container.h"
        "include/pcl/${SUBSYS_NAME}/compact.h"
        "include/pcl/${SUBSYS_NAME}/compact_impl.h"
        )

    set(impl_incs
        "include/pcl/${SUBSYS_NAME}/impl/compact_breadth_first_iterator.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/compact_depth_first_iterator.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/compact_octree_base.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/compact_octree_base_node.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/compact_octree_disk_container.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/compact_octree_ram_container.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/compact_monitor_queue.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/compact_lru_cache.hpp"
        )
    set(visualization_incs
        "include/pcl/${SUBSYS_NAME}/visualization/axes.h"
        "include/pcl/${SUBSYS_NAME}/visualization/camera.h"
        "include/pcl/${SUBSYS_NAME}/visualization/common.h"
        "include/pcl/${SUBSYS_NAME}/visualization/geometry.h"
        "include/pcl/${SUBSYS_NAME}/visualization/grid.h"
        "include/pcl/${SUBSYS_NAME}/visualization/object.h"
        "include/pcl/${SUBSYS_NAME}/visualization/compact_cloud.h"
        "include/pcl/${SUBSYS_NAME}/visualization/scene.h"
        "include/pcl/${SUBSYS_NAME}/visualization/viewport.h"
        )
    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs} ${visualization_incs})
    #PCL_ADD_SSE_FLAGS("${LIB_NAME}")
    target_link_libraries("${LIB_NAME}" pcl_common pcl_visualization ${Boost_SYSTEM_LIBRARY})
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/visualization" ${visualization_incs})

    if(BUILD_tools)
        add_subdirectory(tools)
    endif(BUILD_tools)
    
endif(build)

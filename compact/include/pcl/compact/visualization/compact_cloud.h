#ifndef PCL_COMPACT_COMPACT_CLOUD_H_
#define PCL_COMPACT_COMPACT_CLOUD_H_

#include <stdint.h>

// PCL
//#include <pcl/common/time.h>
//#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL - compact
#include <pcl/compact/compact.h>
#include <pcl/compact/compact_impl.h>
//#include <pcl/compact/impl/monitor_queue.hpp>
#include <pcl/compact/impl/lru_cache.hpp>

// PCL
#include "camera.h"
//#include <pcl/compact/visualization/object.h>

// VTK
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkAppendPolyData.h>
#include <vtkDataSetMapper.h>
//#include <vtkCamera.h>
//#include <vtkCameraActor.h>
//#include <vtkHull.h>
//#include <vtkPlanes.h>
#include <vtkPolyData.h>
//#include <vtkPolyDataMapper.h>
//#include <vtkProperty.h>
#include <vtkSmartPointer.h>

//class Camera;

class CompactCloud : public Object
{
    // Typedefs
    // -----------------------------------------------------------------------------
    typedef pcl::PointXYZ PointT;
//    typedef pcl::compact::CompactOctreeBase<pcl::compact::CompactOctreeDiskContainer<PointT>, PointT> octree_disk;
//    typedef pcl::compact::CompactOctreeBaseNode<pcl::compact::CompactOctreeDiskContainer<PointT>, PointT> octree_disk_node;

    typedef pcl::compact::CompactOctreeBase<> OctreeDisk;
    typedef pcl::compact::CompactOctreeBaseNode<> OctreeDiskNode;
//    typedef pcl::compact::CompactBreadthFirstIterator<> OctreeBreadthFirstIterator;

    typedef boost::shared_ptr<OctreeDisk> OctreeDiskPtr;
    typedef pcl::detail::pagefile_aligned_allocator<PointT> AlignedPointT;



    typedef std::map<std::string, vtkSmartPointer<vtkActor> > CloudActorMap;

  public:

//    typedef std::map<std::string, vtkSmartPointer<vtkPolyData> > CloudDataCache;
//    typedef std::map<std::string, vtkSmartPointer<vtkPolyData> >::iterator CloudDataCacheIterator;


    static boost::shared_ptr<boost::thread> pcd_reader_thread;
    //static MonitorQueue<std::string> pcd_queue;

    struct PcdQueueItem
    {
      PcdQueueItem (std::string pcd_file, float coverage)
      {
       this->pcd_file = pcd_file;
       this->coverage = coverage;
      }

      bool operator< (const PcdQueueItem& rhs) const
      {
       if (coverage < rhs.coverage)
       {
         return true;
       }
       return false;
      }

      std::string pcd_file;
      float coverage;
    };

    typedef std::priority_queue<PcdQueueItem> PcdQueue;
    static PcdQueue pcd_queue;
    static boost::mutex pcd_queue_mutex;
    static boost::condition pcd_queue_ready;

    class CloudDataCacheItem : public LRUCacheItem< vtkSmartPointer<vtkPolyData> >
    {
    public:

      CloudDataCacheItem (std::string pcd_file, float coverage, vtkSmartPointer<vtkPolyData> cloud_data, size_t timestamp)
      {
       this->pcd_file = pcd_file;
       this->coverage = coverage;
       this->item = cloud_data;
       this->timestamp = timestamp;
      }

      virtual size_t
      sizeOf() const
      {
        return item->GetActualMemorySize();
      }

      std::string pcd_file;
      float coverage;
    };


//    static CloudDataCache cloud_data_map;
//    static boost::mutex cloud_data_map_mutex;
    typedef LRUCache<std::string, CloudDataCacheItem> CloudDataCache;
    static CloudDataCache cloud_data_cache;
    static boost::mutex cloud_data_cache_mutex;

    static void pcdReaderThread();

    // Operators
    // -----------------------------------------------------------------------------
    CompactCloud (std::string name, boost::filesystem::path& tree_root);

    // Methods
    // -----------------------------------------------------------------------------
    void
    updateVoxelData ();

    // Accessors
    // -----------------------------------------------------------------------------
    OctreeDiskPtr
    getOctree ()
    {
      return octree_;
    }

    inline vtkSmartPointer<vtkActor>
    getVoxelActor () const
    {
      return voxel_actor_;
    }

    inline vtkSmartPointer<vtkActorCollection>
    getCloudActors () const
    {
      return cloud_actors_;
    }

    void
    setDisplayDepth (int displayDepth)
    {
      if (displayDepth < 0)
      {
        displayDepth = 0;
      }
      else if (static_cast<unsigned int> (displayDepth) > octree_->getDepth ())
      {
        displayDepth = octree_->getDepth ();
      }

      if (display_depth_ != static_cast<uint64_t> (displayDepth))
      {
        display_depth_ = displayDepth;
        updateVoxelData ();
        //updateCloudData();
      }
    }

    int
    getDisplayDepth ()
    {
      return display_depth_;
    }

    uint64_t
    getPointsLoaded ()
    {
      return points_loaded_;
    }

    uint64_t
    getDataLoaded ()
    {
      return data_loaded_;
    }

    Eigen::Vector3d
    getBoundingBoxMin ()
    {
      return bbox_min_;
    }

    Eigen::Vector3d
    getBoundingBoxMax ()
    {
      return bbox_max_;
    }

    void
    setDisplayVoxels (bool display_voxels)
    {
      voxel_actor_->SetVisibility (display_voxels);
    }

    bool
    getDisplayVoxels()
    {
      return voxel_actor_->GetVisibility ();
    }

    void
    setRenderCamera(Camera *render_camera)
    {
      render_camera_ = render_camera;
    }

    int
    getLodPixelThreshold ()
    {
      return lod_pixel_threshold_;
    }

    void
    setLodPixelThreshold (int lod_pixel_threshold)
    {
      if (lod_pixel_threshold <= 1000)
        lod_pixel_threshold = 1000;

      lod_pixel_threshold_ = lod_pixel_threshold;
    }

    void
    increaseLodPixelThreshold ()
    {
      int value = 1000;

      if (lod_pixel_threshold_ >= 50000)
        value = 10000;
      if (lod_pixel_threshold_ >= 10000)
        value = 5000;
      else if (lod_pixel_threshold_ >= 1000)
        value = 100;

      lod_pixel_threshold_ += value;
      std::cout << "Increasing lod pixel threshold: " << lod_pixel_threshold_ << endl;
    }

    void
    decreaseLodPixelThreshold ()
    {
      int value = 1000;
      if (lod_pixel_threshold_ > 50000)
        value = 10000;
      else if (lod_pixel_threshold_ > 10000)
        value = 5000;
      else if (lod_pixel_threshold_ > 1000)
        value = 100;

      lod_pixel_threshold_ -= value;

      if (lod_pixel_threshold_ < 100)
        lod_pixel_threshold_ = 100;
      std::cout << "Decreasing lod pixel threshold: " << lod_pixel_threshold_ << endl;
    }

    virtual void
    render (vtkRenderer* renderer);

  private:

    // Members
    // -----------------------------------------------------------------------------
    OctreeDiskPtr octree_;

    uint64_t display_depth_;
    uint64_t points_loaded_;
    uint64_t data_loaded_;

    Eigen::Vector3d bbox_min_, bbox_max_;

    Camera *render_camera_;

    int lod_pixel_threshold_;

    vtkSmartPointer<vtkActor> voxel_actor_;

    std::map<std::string, vtkSmartPointer<vtkActor> > cloud_actors_map_;
    vtkSmartPointer<vtkActorCollection> cloud_actors_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif

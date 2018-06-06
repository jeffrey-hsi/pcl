/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  $Id$
 *
 */


#include <pcl/compact/compact_base_metadata.h>

#include <pcl/pcl_macros.h>
#include <pcl/exceptions.h>
#include <pcl/console/print.h>

#include <fstream>

#define __PCL_COMPACT_VERSION__ 3

namespace pcl
{
  namespace compact
  {
    CompactOctreeBaseMetadata::CompactOctreeBaseMetadata () 
      : metadata_filename_ ()
      , compact_version_ ()
      , coordinate_system_ ()
      , tree_name_ ()
      , point_type_ ("urp")
      , levels_of_depth_ ()
      , LOD_num_points_ ()
    {
    }
      
    ////////////////////////////////////////////////////////////////////////////////

    CompactOctreeBaseMetadata::CompactOctreeBaseMetadata (const boost::filesystem::path& metadata_filename) 
      : metadata_filename_ (metadata_filename)
      , compact_version_ ()
      , coordinate_system_ ()
      , tree_name_ ()
      , point_type_ ("urp")
      , levels_of_depth_ ()
      , LOD_num_points_ ()
    {
      //read metadata from file and store in fields
      loadMetadataFromDisk ();
    }
    
    ////////////////////////////////////////////////////////////////////////////////
      
    CompactOctreeBaseMetadata::~CompactOctreeBaseMetadata ()
    {
      this->serializeMetadataToDisk ();
    }

    ////////////////////////////////////////////////////////////////////////////////

    CompactOctreeBaseMetadata::CompactOctreeBaseMetadata (const CompactOctreeBaseMetadata& orig) 
      : CompactAbstractMetadata ()
      , metadata_filename_ (orig.metadata_filename_)
      , compact_version_ (orig.compact_version_)
      , coordinate_system_ (orig.coordinate_system_)
      , tree_name_ (orig.tree_name_)
      , point_type_ (orig.point_type_)
      , levels_of_depth_ (orig.levels_of_depth_)
      , LOD_num_points_ (orig.LOD_num_points_)
    {

    }

    ////////////////////////////////////////////////////////////////////////////////

    int
    CompactOctreeBaseMetadata::getCompactVersion () const
    {
      return (compact_version_);
    }
      
    ////////////////////////////////////////////////////////////////////////////////

    void 
    CompactOctreeBaseMetadata::setCompactVersion (const int version)
    {
      compact_version_ = version;
    }
      
    ////////////////////////////////////////////////////////////////////////////////

    boost::filesystem::path
    CompactOctreeBaseMetadata::getMetadataFilename () const
    {
      return (metadata_filename_);
    }
      
    ////////////////////////////////////////////////////////////////////////////////

    void
    CompactOctreeBaseMetadata::setMetadataFilename (const boost::filesystem::path& path_to_metadata)
    {
      metadata_filename_ = path_to_metadata;
    }

    ////////////////////////////////////////////////////////////////////////////////

    void
    CompactOctreeBaseMetadata::serializeMetadataToDisk ()
    {
      // Create JSON object
      boost::shared_ptr<cJSON> idx (cJSON_CreateObject (), cJSON_Delete);
  
      cJSON* name = cJSON_CreateString (tree_name_.c_str ());
      cJSON* version = cJSON_CreateNumber ( __PCL_COMPACT_VERSION__ );
      cJSON* pointtype = cJSON_CreateString (point_type_.c_str ());
      cJSON* lod = cJSON_CreateNumber (static_cast<double> (levels_of_depth_));

      // cJSON does not allow 64 bit ints.  Have to put the points in a double to
      // use this api, will allow counts up to 2^52 points to be stored correctly
      //or split into LSB MSB?
      std::vector<double> lodPoints_db;
      lodPoints_db.insert (lodPoints_db.begin (), LOD_num_points_.begin (), LOD_num_points_.end ());

      cJSON* numpts = cJSON_CreateDoubleArray ( &(lodPoints_db.front ()), static_cast<int>(lodPoints_db.size ()));

      cJSON_AddItemToObject (idx.get (), "name", name);
      cJSON_AddItemToObject (idx.get (), "version", version);
      cJSON_AddItemToObject (idx.get (), "pointtype", pointtype);
      cJSON_AddItemToObject (idx.get (), "lod", lod);
      cJSON_AddItemToObject (idx.get (), "numpts", numpts);
      cJSON_AddStringToObject(idx.get(), "coord_system", coordinate_system_.c_str());

      char* idx_txt = cJSON_Print (idx.get ());

      std::ofstream f (metadata_filename_.string ().c_str (), std::ios::out | std::ios::trunc);
      f << idx_txt;
      f.close ();

      free (idx_txt);
    }

    ////////////////////////////////////////////////////////////////////////////////

    int
    CompactOctreeBaseMetadata::loadMetadataFromDisk ()
    {
      // Open JSON
      std::vector<char> idx_input;
      boost::uintmax_t len = boost::filesystem::file_size (metadata_filename_);
      idx_input.resize (len + 1);

      std::ifstream f (metadata_filename_.string ().c_str (), std::ios::in);
      f.read (&(idx_input.front ()), len);
      idx_input.back () = '\0';

      // Parse JSON
      boost::shared_ptr<cJSON> idx (cJSON_Parse (&(idx_input.front ())), cJSON_Delete);
      cJSON* name = cJSON_GetObjectItem (idx.get (), "name");
      cJSON* version = cJSON_GetObjectItem (idx.get (), "version");
      cJSON* pointtype = cJSON_GetObjectItem (idx.get (), "pointtype");
      cJSON* lod = cJSON_GetObjectItem (idx.get (), "lod");
      cJSON* numpts = cJSON_GetObjectItem (idx.get (), "numpts");
      cJSON* coord = cJSON_GetObjectItem (idx.get (), "coord_system");

      bool parse_failure = false;

      // Validate JSON
      if (!((name) && (version) && (pointtype) && (lod) && (numpts) && (coord)))
      {
        PCL_ERROR ( "[pcl::compact::CompactOctreeBaseMetadata::loadMetadataFromDisk] One of expected metadata fields does not exist in %s\n", metadata_filename_.c_str ());
        parse_failure = true;
      }
      if ((name->type != cJSON_String) || (version->type != cJSON_Number) || (pointtype->type != cJSON_String)
          || (lod->type != cJSON_Number) || (numpts->type != cJSON_Array) || (coord->type != cJSON_String))
      {
        PCL_ERROR ( "[pcl::compact::CompactOctreeBaseMetadata::loadMetadataFromDisk] One of metadata fields does not contain its expected type in %s\n",metadata_filename_.c_str ());
        parse_failure = true;
      }
      if (version->valuedouble != 2.0 && version->valuedouble != 3.0)//only support version 2.0 and 3.0
      {
        PCL_ERROR ( "[pcl::compact::CompactOctreeBaseMetadata::loadMetadataFromDisk] Compact version field (just read version:number = %.1lf) in %s does not match the current supported versions\n",metadata_filename_.c_str (), version->valuedouble);
        parse_failure = true;
      }
      if ((lod->valueint + 1) != cJSON_GetArraySize (numpts))
      {
        PCL_ERROR ( "[pcl::compact::CompactOctreeBaseMetadata::loadMetadataFromDisk] lod:num+1=%d+1 (i.e. height of tree) does not match size of LOD array (%d) in %s\n", lod->valueint, cJSON_GetArraySize (numpts), metadata_filename_.c_str ());
        parse_failure = true;
      }

      if (parse_failure)
      {
        PCL_THROW_EXCEPTION (PCLException, "[pcl::compact::CompactOctreeBaseMetadata::loadMetadataFromDisk] Parse Failure\n");
      }
      

      // Get Data
      LOD_num_points_.resize (lod->valueint + 1);
      for (int i = 0; i < (lod->valueint + 1); i++)
      {
        //cJSON doesn't have explicit 64bit int, have to use double, get up to 2^52
        LOD_num_points_[i] = static_cast<boost::uint64_t> (cJSON_GetArrayItem (numpts, i)->valuedouble );
      }
      levels_of_depth_ = lod->valueint;
      coordinate_system_ = coord->valuestring;

      //return success
      return (1);
    }

    ////////////////////////////////////////////////////////////////////////////////

    int
    CompactOctreeBaseMetadata::loadMetadataFromDisk (const boost::filesystem::path& path_to_metadata)
    {
      this->setMetadataFilename (path_to_metadata);
      return (this->loadMetadataFromDisk ());
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    std::string
    CompactOctreeBaseMetadata::getOctreeName ()
    {
      return (this->tree_name_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    void
    CompactOctreeBaseMetadata::setOctreeName (const std::string& name_arg)
    {
      this->tree_name_ = name_arg;
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    std::string
    CompactOctreeBaseMetadata::getPointType ()
    {
      return (this->point_type_);
    }

    ////////////////////////////////////////////////////////////////////////////////
    
    void
    CompactOctreeBaseMetadata::setPointType (const std::string& point_type_arg)
    {
      this->point_type_ = point_type_arg;
    }
  
    ////////////////////////////////////////////////////////////////////////////////

    std::vector<boost::uint64_t>&
    CompactOctreeBaseMetadata::getLODPoints ()
    {
      return (LOD_num_points_);
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    std::vector<boost::uint64_t>
    CompactOctreeBaseMetadata::getLODPoints () const
    {
      return (LOD_num_points_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    boost::uint64_t
    CompactOctreeBaseMetadata::getLODPoints (const boost::uint64_t& depth_index) const
    {
      return (LOD_num_points_[depth_index]);
    }

    ////////////////////////////////////////////////////////////////////////////////

    void
    CompactOctreeBaseMetadata::setLODPoints (const boost::uint64_t& depth)
    {
      LOD_num_points_.clear ();
      LOD_num_points_.resize (depth);
      assert (LOD_num_points_.size () == this->levels_of_depth_+1);
    }

    ////////////////////////////////////////////////////////////////////////////////

    void
    CompactOctreeBaseMetadata::setLODPoints (std::vector<boost::uint64_t>& lod_points_arg)
    {
      assert (this->LOD_num_points_.size () == lod_points_arg.size ());
      
      this->LOD_num_points_ = lod_points_arg;
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    void
    CompactOctreeBaseMetadata::setLODPoints (const boost::uint64_t& lod_index_arg, const boost::uint64_t& num_points_arg, const bool increment)
    {
      assert (lod_index_arg < LOD_num_points_.size ());

      if (increment)
        LOD_num_points_[lod_index_arg] += num_points_arg;
      else
        LOD_num_points_[lod_index_arg] = num_points_arg;
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    
    void
    CompactOctreeBaseMetadata::setCoordinateSystem (const std::string& coord_sys_arg)
    {
      this->coordinate_system_ = coord_sys_arg;
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    std::string
    CompactOctreeBaseMetadata::getCoordinateSystem () const
    {
      return (this->coordinate_system_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    void
    CompactOctreeBaseMetadata::setDepth (const boost::uint64_t& depth_arg)
    {
      this->levels_of_depth_ = depth_arg;
    }
    
    boost::uint64_t
    CompactOctreeBaseMetadata::getDepth () const
    {
      return (levels_of_depth_);
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    //Protected Member Functions
    ////////////////////////////////////////////////////////////////////////////////
    void
    CompactOctreeBaseMetadata::writeMetadataString (std::vector<char>& buf)
    {
      (void)buf;
      PCL_THROW_EXCEPTION (PCLException, "Not implemented\n");
    }

    ////////////////////////////////////////////////////////////////////////////////

    std::ostream& 
    operator<<(std::ostream& os, const CompactOctreeBaseMetadata& metadata_arg)
    {
      (void) metadata_arg;
      return (os);
    }

    ////////////////////////////////////////////////////////////////////////////////

  }//namespace compact
}//namespace pcl
#undef __PCL_COMPACT_VERSION__

/*********************************************************************
* Copyright (c) 2018, CHC Technology Co., Ltd., All rights reserved
* filename: compact_large_file.h
* created:  2018-06-05  18:21
* author:   Jeffrey
* version:  1.0
* purpose:  
*********************************************************************/
#ifndef __COMPACT_LARGE_FILE_H__
#define __COMPACT_LARGE_FILE_H__

#include <boost/filesystem/path.hpp>

namespace pcl {
namespace compact {

template <typename RecordT>
class CompactLargeFile
{
public:
    CompactLargeFile(const boost::filesystem::path& file);

    RecordT read(int32_t position);

};

}//compact
}//pcl

#endif//__COMPACT_LARGE_FILE_H__

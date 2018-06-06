/*********************************************************************
* Copyright (c) 2018, CHC Technology Co., Ltd., All rights reserved
* filename: pagefile_allocator.h
* created:  2018-06-05  12:03
* author:   Jeffrey
* version:  1.0
* purpose:  
*********************************************************************/

#ifndef __PAGEFILE_ALLOCATOR_H__
#define __PAGEFILE_ALLOCATOR_H__

#include <boost/pool/pool_alloc.hpp>

namespace pcl
{
namespace detail
{
extern void* pcl_mmap(std::size_t size);
extern void pcl_munmap(void* p);

struct pcl_pagefile_allocator
{
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    static const size_type ALIGN_BYTES = 16;
    static const size_type MMAP_SIZE = 10 * 1024 * 1024;

    static char * malloc(const size_type bytes)
    {
        void* original = nullptr;
        size_type mark = 0;
        if (bytes > 10 * 1024 * 1024)
        {
            original = pcl_mmap(bytes + ALIGN_BYTES * 2);
            mark = size_type(-1);
        }
        else
        {
            original = boost::default_user_allocator_malloc_free::malloc(bytes);
        }
        if (original == 0) return 0;
        void *aligned = reinterpret_cast<void*>((reinterpret_cast<std::size_t>(original) & ~(std::size_t(ALIGN_BYTES - 1))) + ALIGN_BYTES * 2);
        *(reinterpret_cast<void**>(aligned) - 1) = original;
        *(reinterpret_cast<void**>(aligned) - 2) = (void*)mark;
        return static_cast<char*>(aligned);
    }

    static void free(char * const block)
    {
        size_type mark = (size_type)(*(reinterpret_cast<void**>(block) - 2));
        void* original = *(reinterpret_cast<void**>(block) - 1);
        if (mark)
        {
            pcl_munmap(original);
        }
        else
        {
            boost::default_user_allocator_malloc_free::free(static_cast<char*>(original));
        }
    }
};

} // detail

template<class T>
class pagefile_allocator : public boost::fast_pool_allocator<T, detail::pcl_pagefile_allocator>
{
};

} // pcl

#endif//__PAGEFILE_ALLOCATOR_H__


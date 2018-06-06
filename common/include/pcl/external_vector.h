/*********************************************************************
* Copyright (c) 2018, CHC Technology Co., Ltd., All rights reserved
* filename: external_vector.h
* created:  2018-06-05  18:44
* author:   Jeffrey
* version:  1.0
* purpose:  
*********************************************************************/
#ifndef __EXTERNAL_VECTOR_H__
#define __EXTERNAL_VECTOR_H__

#include <boost/filesystem/path.hpp>
#include <iterator>

namespace pcl {

template<typename ValueType>
class ExternalVector
{
public:
    typedef std::size_t     size_type;
    typedef std::ptrdiff_t  difference_type;
    typedef ValueType*      pointer;
    typedef const ValueType& const_reference;
    typedef ValueType& reference;

    class const_iterator : public std::iterator<std::random_access_iterator_tag, ValueType>
    {
        friend class ExternalVector;
    public:
        typedef const_iterator ThisIter;
        typedef std::iterator<std::random_access_iterator_tag, ValueType> BaseIterator;

        const_iterator() : vector_(nullptr), position_(size_type(-1)) {}

        const_reference operator*() const;
        pointer   operator->() const;

        ThisIter& operator++();
        ThisIter  operator++(int);
        ThisIter& operator--();
        ThisIter  operator--(int);
        ThisIter& operator+=(difference_type off);
        ThisIter  operator+ (difference_type off) const;
        ThisIter& operator-=(difference_type off);
        ThisIter  operator- (difference_type off) const;
        reference operator[](difference_type off) const;

        difference_type operator+ (const ThisIter& o) const;
        difference_type operator- (const ThisIter& o) const;

        bool operator ==(const ThisIter& o) const;
        bool operator !=(const ThisIter& o) const;
        bool operator < (const ThisIter& o) const;
        bool operator > (const ThisIter& o) const;
        bool operator <=(const ThisIter& o) const;
        bool operator >=(const ThisIter& o) const;

    protected:
        const_iterator(size_type pos, ExternalVector* vec)
            : vector_(vec)
            , position_(pos)
        {
            if (vec && pos >= vec->size())
            {
                vector_ = nullptr;
                position_ = size_type(-1);
            }
        }

    private:
        ExternalVector* vector_;
        size_type position_;
    };

    class iterator : public const_iterator
    {
        friend class ExternalVector;
    public:
        typedef iterator ThisIter;

        iterator() : const_iterator() {}
        iterator(size_type pos, ExternalVector* vec) : const_iterator(pos, vec) {}

        reference operator*() const;
        pointer   operator->() const;

        ThisIter& operator++();
        ThisIter  operator++(int);
        ThisIter& operator--();
        ThisIter  operator--(int);
        ThisIter& operator+=(difference_type off);
        ThisIter  operator+ (difference_type off) const;
        ThisIter& operator-=(difference_type off);
        ThisIter  operator- (difference_type off) const;
        reference operator[](difference_type off) const;

        difference_type operator+ (const ThisIter& o) const;
        difference_type operator- (const ThisIter& o) const;

        bool operator ==(const ThisIter& o) const;
        bool operator !=(const ThisIter& o) const;
        bool operator < (const ThisIter& o) const;
        bool operator > (const ThisIter& o) const;
        bool operator <=(const ThisIter& o) const;
        bool operator >=(const ThisIter& o) const;
    };

    friend const_iterator;
    friend iterator;

    ExternalVector();

    ExternalVector(const boost::filesystem::path& file)
    {
        if (boost::filesystem::exists(file) && boost::filesystem::is_regular_file(file))
        {
            // TODO open
        }
        else
        {
            // create dynamic
        }
    }

    ExternalVector(size_type count)
        : ExternalVector(count, ValueType())
    {
    }

    ExternalVector(size_type count, const ValueType& val)
    {
        file_size_ = count;
        // TODO: create temp path to cache data
    }

    ExternalVector(const ExternalVector& o) = delete;

    ExternalVector& operator = (const ExternalVector& o);

    void push_back(ValueType& v);
    void push_back(const ValueType& val);
    void pop_back();

    void reserve(size_type count);
    size_type capacity() const;
    void resize(size_type newSize);
    void resize(size_type newSize, const ValueType& val);

    size_type size() const;

    const_reference at(size_type pos) const;
    reference at(size_type pos);

    const_reference operator[](size_type pos) const;
    reference operator[](size_type pos);

    const_iterator begin() const;
    iterator begin();
    const_iterator end() const;
    iterator end();

    bool empty() const
    {
        return object_count_ == 0;
    }

    void swap(ExternalVector<ValueType>& o);

    void clear();

    template<class _Iter> void
    assign(_Iter _First, _Iter _Last);

    void assign(size_type _Count, const ValueType& _Val);

    iterator insert(const_iterator _Where, const ValueType& _Val);
    iterator insert(const_iterator _Where, size_type _Count,
        const ValueType& _Val);

    template<class _Iter> void
    insert(const_iterator _Where, _Iter _First, _Iter _Last);

    //iterator erase(const_iterator _Where);

    //iterator erase(const_iterator _First_arg,
    //    const_iterator _Last_arg);

private:
    boost::uintmax_t file_size_ = 0;
    size_type object_count_ = 0;
    boost::filesystem::path file_path_;
};

template <typename ValueType>
typename ExternalVector<ValueType>::difference_type operator - (const typename ExternalVector<ValueType>::iterator& left, const typename ExternalVector<ValueType>::iterator& right);

template <typename ValueType>
typename ExternalVector<ValueType>::difference_type operator -(const typename ExternalVector<ValueType>::const_iterator& left, const typename ExternalVector<ValueType>::const_iterator& right);

}//pcl


#endif//__EXTERNAL_VECTOR_H__

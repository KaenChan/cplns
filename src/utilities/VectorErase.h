#pragma once

#include <memory>
#include <stdexcept>

// 有allocator属性
// 有erase方法

#define VectorE_t VectorE 
// #define VectorE_t std::list 

template <class T> 
class VectorE {
    public:
        // Member types
        typedef T value_type;
        typedef std::allocator<T> allocator_type;
        typedef size_t size_type;
        typedef value_type& reference;
        typedef const value_type& const_reference;
        typedef T* iterator;
        typedef const T* const_iterator;
        typedef std::reverse_iterator<iterator> reverse_iterator;
        typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

        // Member functions
        VectorE() { create(); }
        explicit VectorE(size_type n, const T& val = T{}) { create(n, val); }
        VectorE(std::initializer_list<T> array); // Create from array
        VectorE(const VectorE& v) { create(v.begin(), v.end()); } // copy
        ~VectorE() { uncreate(); }
        VectorE& operator=(const VectorE&);       // copy assignment
        void assign( size_type count, const T& value );
        allocator_type get_allocator() const { return alloc; };

        // Element access
        reference at( size_type i );
        const_reference at( size_type i ) const;
        T& operator[](size_type i) { return data[i]; }
        const T& operator[](size_type i) const { return data[i]; }
        reference front() { return *data; }
        const_reference front() const { return *data; }
        reference back() { return *(avail-1); }
        const_reference back() const { return *(avail-1); }

        // Iterators
        iterator begin() noexcept { return data; }
        const_iterator begin() const noexcept { return data; }
        iterator end() noexcept { return avail; }
        const_iterator end() const noexcept { return avail; }
        reverse_iterator rbegin() noexcept { return reverse_iterator(avail); }
        const_reverse_iterator rbegin() const noexcept { return reverse_iterator(avail); };
        reverse_iterator rend() noexcept { return reverse_iterator(data); }
        const_reverse_iterator rend() const noexcept { return reverse_iterator(data); };

        // Capacity
        bool empty() const { return (begin() == end()); }
        size_type size() const { return avail - data; }
        size_type capacity() const { return limit - data; }
        void reserve( size_type new_cap );
        void shrink_to_fit();

        // Modifiers
        void clear();
        iterator insert( iterator &pos, const T& value );
        iterator insert( iterator &pos, int count, T& value);
        iterator erase( iterator pos );
        iterator erase( iterator first, iterator last );
        void remove(T& del);
        void push_back(const T& val);
	    template<class ... Args> void emplace_back(Args&& ... args);
        void pop_back();
        void resize( size_type count );
        void resize( size_type count, const value_type& value );
        void swap( VectorE<T>& other ) noexcept;

        // Operators
        bool operator==(const VectorE<T>& other) const;
        bool operator!=(const VectorE<T>& other) const;
        bool operator<(const VectorE<T>& other) const;
        bool operator>(const VectorE<T>& other) const;
        bool operator>=(const VectorE<T>& other) const;
        bool operator<=(const VectorE<T>& other) const;
    private:    
        iterator data;        // first element of the VectorErase
        iterator avail;       // first element after the last VectorErase element
        iterator limit;       // first element outside the reserved space

        allocator_type alloc;

        void create();        // set pointers to null
        void create(size_type, const T&);   // create a vec with copies of 'value'
        void create(const_iterator, const_iterator); // create a vec with contents of a range
        void uncreate();      // destroy the vec and deallocate space
        void growTwice();     // increase the reserved space twice
        void unchecked_append(const T&);    // Insert new element at the end
};

// Create a VectorErase from an array
template <class T> 
VectorE<T>::VectorE(std::initializer_list<T> array) {
    data = alloc.allocate(array.size());
    limit = avail = std::uninitialized_copy(array.begin(), array.end(), data);
}

// Create an empty VectorErase
template <class T> 
void VectorE<T>::create() {
    data = avail = limit = nullptr;
}

// Create a VectorErase with copies of 'value' or just reserved space
template <class T> 
void VectorE<T>::create(size_type n, const T& val) {
    data = alloc.allocate(n); 
    limit = avail = data + n;
    std::uninitialized_fill(data, limit, val);
}

// Create a VectorErase with contents of a range
template <class T>
void VectorE<T>::create(const_iterator i, const_iterator j) {    
    data = alloc.allocate(j - i);
    limit = avail = std::uninitialized_copy(i, j, data);
}

// Destroy the VectorErase and deallocate space
template <class T> 
void VectorE<T>::uncreate() {
    if (data) {
        // Destroy elements backwards     
        iterator it = avail;
        while (it != data)
            alloc.destroy(--it);      
        alloc.deallocate(data, limit - data);    
        }
    data = limit = avail = nullptr; // Reset pointers
}

// Assignment operator
template <class T> 
VectorE<T>& VectorE<T>::operator=(const VectorE& rhs) {
    if (&rhs != this) {     
        uncreate();
        create(rhs.begin(), rhs.end());
    }
    return *this;
}

// Increase the reserved space twice
template <class T> 
void VectorE<T>::growTwice() { 
    auto sz = size();
    auto n = sz / 2;
    if(n == 0)
        n = 1;
    size_type new_size = sz + n;
    reserve(new_size);
}

// Insert new element at the end
template <class T> 
void VectorE<T>::unchecked_append(const T& val) {    
    alloc.construct(avail++, val);
}

// Element access funtions
template <class T>
typename VectorE<T>::reference VectorE<T>::at( size_type i ) {
    if (i < size() && i >= 0)
        return data[i]; 
    else throw std::out_of_range {"VectorErase::at"};
}

template <class T>
typename VectorE<T>::const_reference VectorE<T>::at( size_type i ) const { 
    if (i < size() && i >= 0) 
        return data[i]; 
    else throw std::out_of_range {"VectorErase::at"};
}

// Reallocate VectorErase to a larger block of memory
template <class T> 
void VectorE<T>::reserve( size_type new_cap ) {
    if (new_cap > capacity()) {
        iterator new_data = alloc.allocate(new_cap);    
        iterator new_avail = std::uninitialized_copy(data, avail, new_data);   
        uncreate();
        data = new_data;    
        avail = new_avail;     
        limit = data + new_cap;
    }
}

// Release unused memory by the VectorErase
template <class T> 
void VectorE<T>::shrink_to_fit() {
    if (avail != limit) {
        alloc.deallocate(avail, limit - avail);
        limit = avail;
    }
}

// Replace the contents with *count* copies of *value* value
template <class T> 
void VectorE<T>::assign( size_type count, const T& value ) {
    if (count < 1)
        throw std::invalid_argument{ "VectorErase::assign" };

    if (count > capacity()) {
        uncreate();             // Erase all elements and deallocate memory
        create(count, value);   // Create a new VectorErase and fill it in
    } else {
        iterator it = avail;
        while (it != data)            
            alloc.destroy(--it);// Erase all elements
        
        avail = data + count;   // Set new VectorErase size
        std::uninitialized_fill(data, avail, value); // Fill the VectorErase with copies
    }
}

// Delete all elements, don't deallocate space
template <class T> 
void VectorE<T>::clear() { 
    iterator it = avail;
    while (it != data)
        alloc.destroy(--it);
    avail = data;
}

// Insert *value* at *pos* position
template <class T> 
typename VectorE<T>::iterator VectorE<T>::insert( iterator &pos, const T& value ) {
    if (pos < data || pos >= avail)
        throw std::out_of_range{ "VectorErase::insert" };
    int pos_integer = 0;
    for (iterator i = data; i < pos; i++)
        pos_integer++;

    if(true) {
        if(avail == limit)
            growTwice();
        avail++;
        size_type new_size = size();
        for (int i = new_size-1; i > pos_integer; i--)
            data[i] = data[i - 1];
        data[pos_integer] = value;
        pos = data + pos_integer + 1;
    }

    if(false) {
        size_type new_size = size() + 1;
        iterator new_data = alloc.allocate(new_size);
        iterator new_avail = std::uninitialized_copy(data, avail + 1, new_data);

        new_data[pos_integer] = value;
        int after_pos = pos_integer + 1;
        int new_last = size() + 1;

        for (int i = after_pos; i < new_last; i++)
            new_data[i] = data[i - 1];
        
        uncreate();
        data = new_data;
        avail = new_avail;
        limit = data + new_size;
        pos = new_data + after_pos;
    }

    return data + pos_integer;
}

// Insert *count* copies of *value* at *pos* position
template <class T> 
typename VectorE<T>::iterator VectorE<T>::insert( iterator &pos, int count, T& value )
{
    if (pos < data || count < 1 || pos >= avail)
        throw std::out_of_range{ "VectorErase::insert" };
    avail += count;
    int pos_integer = 0;
    for (iterator i = data; i < pos; i++)
        pos_integer ++;
    size_type new_size = size() + count;
    iterator new_data = alloc.allocate(new_size);
    iterator new_avail = std::uninitialized_copy(data, avail, new_data);

    for (int i = 0; i < pos_integer; i++)
        new_data[i] = data[i];

    for (int i = pos_integer; i <= pos_integer + count; i++)
        new_data[i] = value;

    int after_inserted = pos_integer + count;
    int new_last = size() + count;
    for (int i = after_inserted; i < new_last; i++) {
        new_data[i] = data[i - count];
    }
    uncreate();
    data = new_data;
    avail = new_avail;
    limit = data + new_size;

    return data + pos_integer;
}

// Erase element at pos position
template <class T> 
typename VectorE<T>::iterator VectorE<T>::erase( iterator pos ) {
    if (pos < data || pos >= avail)
        throw std::out_of_range{ "VectorErase::erase" };

    for (iterator i = pos; i < avail-1; i ++)
        *i = *(i+1); // Move values by one position to the left

    avail --;        // Reduce the VectorErase size by one
    return pos;
}


template <class T> 
void VectorE<T>::remove(T& del){
    iterator pos = data;
    for (; pos < avail; pos++)
        if(*pos == del) break;
    erase(pos);
}

// Erase elements in a range
template <class T> 
typename VectorE<T>::iterator VectorE<T>::erase( iterator first, iterator last ) {
    if (last < first)
        throw std::invalid_argument{ "VectorErase::erase" };
    if (first < data || last > avail)
        throw std::out_of_range{ "VectorErase::erase" };
    for (iterator i = first; i < avail-1; i ++)
        *i = *(i+(last-first));
    avail -= last - first;
    return last;    
}

// Add an element to the back of the VectorErase
template <class T> 
void VectorE<T>::push_back(const T& val){
    if (avail == limit)        
        growTwice();        // Increase the container capacity twice      
    unchecked_append(val) ; // Insert new element at the end
}
template<class T>
template<class ... Args>
inline void VectorE<T>::emplace_back(Args && ... args)
{
    if (avail == limit)        
        growTwice();        // Increase the container capacity twice      
    // alloc.construct(avail++, val);
	// data[avail] = std::move(T(std::forward<Args>(args)...));
	// ++avail;
    alloc.construct(avail++, std::move(T(std::forward<Args>(args)...)));
}

// Delete the last element of the VectorErase
template <class T> 
void VectorE<T>::pop_back(){
    iterator it = avail;           
    alloc.destroy(--it);
    avail--;
}

// Leave the VectorErase with *count* elements only (count<size)
template <class T> 
void VectorE<T>::resize( size_type count ) {
    if (count < 0 || count > size())
        throw std::invalid_argument{ "VectorErase::resize" };
    while (begin() + count != avail)
        alloc.destroy(--avail);
}

// If the current size is less than count, additional elements are 
// appended and initialized with copies of value
template <class T>
void VectorE<T>::resize( size_type count, const value_type& value ) {
    if (count < 0)
        throw std::invalid_argument{ "VectorErase::resize" };
    if (size() > count)
        resize(count);
    else {
        if (limit == avail)
            reserve(size() + (count - size()));
        std::uninitialized_fill(avail, limit, value);
        avail = limit;
    }
}

// Exchange the contents of the container with those of other
template <class T>
void VectorE<T>::swap( VectorE<T>& other ) noexcept {
    iterator temp = data;
    data = other.data;
    other.data = temp;

    temp = avail;
    avail = other.avail;
    other.avail = temp;

    temp = limit;
    limit = other.limit;
    other.limit = temp;
}

// Check if both vectors have the same size and values
template <class T>
bool VectorE<T>::operator==(const VectorE<T>& other) const {
    if (size() == other.size()) {
        for (int i = 0; i < size(); i ++)
            if (at(i) != other.at(i))   // Search for any mismatch
                return false;           // Return breaks the loop
        return true;
    } else return false;
}

template <class T>
bool VectorE<T>::operator!=(const VectorE<T>& other) const {
    // Use the already implemented == operator
    return *this == other ? false : true;
}

// Compare vectors lexicographically
template <class T>
bool VectorE<T>::operator<(const VectorE<T>& other) const {
    size_type smaller_size;
    if (size() < other.size())
        smaller_size = size();
    else smaller_size = other.size();

    for (size_type i = 0; i < smaller_size; i++)
        if (at(i) != other[i])       // Find the first mismatching element
            return at(i) < other[i];
    // If one range is a prefix of another, the shorter range is 
    // lexicographically less than the other
    return size() < other.size();
}

template <class T>
bool VectorE<T>::operator>(const VectorE<T>& other) const {
    size_type smaller_size;
    if (size() < other.size())
        smaller_size = size();
    else smaller_size = other.size();

    for (size_type i = 0; i < smaller_size; i++)
        if (at(i) != other[i])
            return at(i) > other[i];
    return size() > other.size();
}

template <class T>
bool VectorE<T>::operator<=(const VectorE<T>& other) const {
    // Use the already implemented > operator
    return *this > other ? false : true;
}

template <class T>
bool VectorE<T>::operator>=(const VectorE<T>& other) const {
    // Use the already implemented < operator
    return *this < other ? false : true;
}
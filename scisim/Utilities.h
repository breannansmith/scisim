// Utilities.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef UTILITIES_H
#define UTILITIES_H

#include <vector>
#include <memory>
#include <cassert>
#include <istream>
#include <type_traits>

namespace Utilities
{

  template<typename Test, template<typename...> class Ref>
  struct is_specialization : public std::false_type {};

  template<template<typename...> class Ref, typename... Args>
  struct is_specialization<Ref<Args...>, Ref> : public std::true_type {};

  // Supresses unused variable warnings
  template<typename T>
  void ignoreUnusedVariable( const T& ) {}

  // Returns a 'deep copy' of a vector of unique_ptrs
  template<typename T>
  std::vector<std::unique_ptr<T>> clone( const std::vector<std::unique_ptr<T>>& original )
  {
    using st = typename std::vector<std::unique_ptr<T>>::size_type;
    const st length{ original.size() };
    std::vector<std::unique_ptr<T>> copy( length );
    for( st idx = 0; idx < length; idx++ )
    {
      copy[idx] = original[idx]->clone();
    }
    return copy;
  }

  // Serializes a trivially copyable non-pointer variable
  template<typename T>
  void serialize( const T& var, std::ostream& output_stream )
  {
    static_assert( std::is_trivially_copyable<T>::value, "Error in serialization, type is not trivially copyable." );
    static_assert( !std::is_pointer<T>::value, "Error in serialization, type is a pointer." );
    assert( output_stream.good() );
    output_stream.write( reinterpret_cast<const char*>( &var ), sizeof(T) );
  }

  // Serializes a vector of unique pointers of non-trivially copyable objects that have a custom serialize method
  template<typename T>
  void serialize( const std::vector<std::unique_ptr<T>>& vector, std::ostream& output_stream )
  {
    static_assert( !std::is_trivially_copyable<T>::value, "Error in vector of unique_ptr custom serialization, type is trivially copyable." );
    assert( output_stream.good() );
    // Write out the length of the vector
    serialize( vector.size(), output_stream );
    // Output each element of the vector
    for( typename std::vector<T>::size_type idx = 0; idx < vector.size(); ++idx )
    {
      assert( vector[idx] != nullptr );
      vector[idx]->serialize( output_stream );
    }
  }

  // Serializes a vector of non-pointer and non-trivially copyable objects that have a custom serialize method
  template<typename T>
  typename std::enable_if<!std::is_trivially_copyable<T>::value>::type
  serialize( const std::vector<T>& vector, std::ostream& output_stream )
  {
    static_assert( !std::is_trivially_copyable<T>::value, "Error in vector custom serialization, type is trivially copyable" );
    assert( output_stream.good() );
    // Write out the length of the vector
    serialize( vector.size(), output_stream );
    // Output each element of the vector
    for( typename std::vector<T>::size_type idx = 0; idx < vector.size(); ++idx )
    {
      vector[idx].serialize( output_stream );
    }
  }

  // Serializes a vector of trivially copyable variables
  template<typename T>
  typename std::enable_if<std::is_trivially_copyable<T>::value>::type
  serialize( const std::vector<T>& vector, std::ostream& output_stream )
  {
    static_assert( std::is_trivially_copyable<T>::value, "Error in vector serialization, type is not trivially copyable." );
    static_assert( !std::is_pointer<T>::value, "Error in vector serialization, type is a pointer." );
    assert( output_stream.good() );
    serialize( vector.size(), output_stream );
    output_stream.write( reinterpret_cast<const char*>( vector.data() ), vector.size() * sizeof(T) );
  }

  // Serializes a vector of bools
  template<>
  void serialize<bool>( const std::vector<bool>& vector, std::ostream& output_stream );

  // Deserializes a trivially copyable non-pointer variable
  template<typename T>
  typename std::enable_if<!is_specialization<T, std::vector>::value,T>::type
  deserialize( std::istream& input_stream )
  {
    static_assert( std::is_trivially_copyable<T>::value, "Error in deserialization, type is not trivially copyable." );
    static_assert( !std::is_pointer<T>::value, "Error in deserialization, type is a pointer." );
    assert( input_stream.good() );
    T type_to_read;
    input_stream.read( reinterpret_cast<char*>( &type_to_read ), sizeof(T) );
    return type_to_read;
  }

  // Deserializes a vector of objects with custom deserialize constructor
  template<typename T>
  typename std::enable_if<is_specialization<T, std::vector>::value && !std::is_trivially_copyable<typename T::value_type>::value, T>::type
  deserialize( std::istream& input_stream )
  {
    assert( input_stream.good() );
    T vector;
    const typename T::size_type length{ deserialize<typename T::size_type>( input_stream ) };
    vector.reserve( length );
    for( typename T::size_type idx = 0; idx < length; ++idx )
    {
      vector.emplace_back( input_stream );
      assert( input_stream.good() );
    }
    assert( vector.size() == length );
    return vector;
  }

  // Deserializes a vector of trivially copyable non-pointer variables
  template<class T>
  typename std::enable_if<is_specialization<T, std::vector>::value && std::is_trivially_copyable<typename T::value_type>::value, T>::type
  deserialize( std::istream& input_stream )
  {
    static_assert( !std::is_pointer<typename T::value_type>::value, "Error in vector deserialization, type is a pointer." );
    assert( input_stream.good() );
    T vector;
    const typename T::size_type length{ deserialize<typename T::size_type>( input_stream ) };
    vector.resize( length );
    input_stream.read( reinterpret_cast<char*>( vector.data() ), length * sizeof(typename T::value_type) );
    return vector;
  }

  // Deserializes a vector of bools
  template<>
  std::vector<bool> deserialize<std::vector<bool>>( std::istream& input_stream );

}

#endif

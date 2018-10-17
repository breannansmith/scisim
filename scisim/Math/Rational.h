// Adapted from https://github.com/peoplesbeer/Rational

#ifndef RATIONAL_H
#define RATIONAL_H

#include <algorithm>
#include <cassert>
#include <ostream>
#include <string>

#include "scisim/StringUtilities.h"

// TODO: What happens when denominator == 0 ?
// TODO: Handle unsigned numbers
// TODO: Implement move semantics, etc

// Class representation of a rational number. This rational number will always
// be simplified as far as possible, will always indicate sign on the numerator
// and will always represent 0 as 0 / 1. For example 2 / -8 will be represented as
// -1 / 4 and 0 / -128 will be represented as 0 / 1.
template <typename T>
class Rational final
{

public:

  explicit Rational( const T numerator = 0, const T denominator = 1 )
  : m_numerator( numerator )
  , m_denominator( denominator )
  {
    simplify();
  }

  // Sets the numerator and denominator of this Rational
  Rational& set( const T numerator, const T denominator );

  // Overloaded compound-assignment operators
  Rational& operator+=( const Rational& right )
  {
    return *this = Rational<T>( right.m_denominator * m_numerator + m_denominator * right.m_numerator, m_denominator * right.m_denominator );
  }

  Rational& operator-=( const Rational& right )
  {
    return *this = Rational<T>( right.m_denominator * m_numerator - m_denominator * right.m_numerator, m_denominator * right.m_denominator );
  }

  Rational& operator*=( const Rational& right )
  {
    return *this = Rational<T>( m_numerator * right.m_numerator, m_denominator * right.m_denominator );
  }

  Rational& operator/=( const Rational& right )
  {
    return *this = Rational<T>( right.m_denominator * m_numerator, right.m_numerator * m_denominator );
  }

  T numerator() const
  {
    return m_numerator;
  }

  T denominator() const
  {
    return m_denominator;
  }

  bool positive() const
  {
    assert( m_denominator > 0 );
    return m_numerator > 0;
  }

  bool nonNegative() const
  {
    assert( m_denominator > 0 );
    return m_numerator >= 0;
  }

  bool isInteger() const
  {
    return m_denominator == 1;
  }

  explicit operator double() const
  {
    return double( m_numerator ) / double( m_denominator );
  }


private:

  // Divide by greatest common divisor and makes sure sign is indicated on numerator
  void simplify();

  T m_numerator;
  T m_denominator;

};

template <typename T>
bool operator==( const Rational<T>& lhs, const Rational<T>& rhs )
{
  return ( lhs.numerator() == rhs.numerator() ) && ( lhs.denominator() == rhs.denominator() );
}

template <typename T>
bool operator==( const Rational<T>& lhs, const T rhs )
{
  return lhs == Rational<T>( rhs, 1 );
}

template <typename T>
bool operator==( const T lhs, const Rational<T>& rhs )
{
  return rhs == lhs;
}

template <typename T>
bool operator!=( const Rational<T>& lhs, const Rational<T>& rhs )
{
  return !(lhs == rhs);
}

// Returns the greatest common divisor (Euclidean algorithm)
template <typename T>
T GCD( T numerator, T denominator )
{
  T temp;
  while( denominator != 0 )
  {
    temp = denominator;
    denominator = numerator % denominator;
    numerator = temp;
  }
  return numerator;
}

template <typename T>
void Rational<T>::simplify()
{
  // Always represent 0 as 0 / 1
  if( m_numerator == 0 )
  {
    m_denominator = 1;
    return;
  }

  // Divide by greatest common divisor
  const T gcd = GCD( m_numerator, m_denominator );
  m_numerator /= gcd;
  m_denominator /= gcd;

  // Indicate sign on numerator only
  if( m_denominator < 0 )
  {
    m_numerator = -m_numerator;
    m_denominator = -m_denominator;
  }
}

// TODO: Move this up into the function
template <typename T>
Rational<T>& Rational<T>::set( const T numerator, const T denominator )
{
  m_numerator = numerator;
  m_denominator = denominator;
  simplify();
  return *this;
}

template <typename T>
Rational<T> operator*( const Rational<T>& left, const T right )
{
  return Rational<T>{ left.numerator() * right, left.denominator() };
}

template <typename T>
Rational<T> operator*( const T left, const Rational<T>& right )
{
  return Rational<T>{ right.numerator() * left, right.denominator() };
}

template <typename T>
Rational<T> operator/( const Rational<T>& left, const T right )
{
  return Rational<T>{ left.numerator(), left.denominator() * right };
}

template <typename T>
Rational<T> operator/( const T left, const Rational<T>& right )
{
  return Rational<T>{ right.denominator() * left, right.numerator() };
}

template <typename T>
std::ostream& operator<<( std::ostream& os, const Rational<T>& r )
{
  return os << r.numerator() << " / " << r.denominator();
}

// TODO: This is super adhoc, and will parse in strange things as numbers, but works well enough for now.
// TODO: Once C++ regex are widely available in various compilers, clean this up a lot.
// TODO: Handle, for example, 1.0e-4
template<class T>
bool extractFromString( const std::string& in_string, Rational<T>& output )
{
  const unsigned slash_count{ static_cast<unsigned>( std::count( in_string.cbegin(), in_string.cend(), '/' ) ) };
  const unsigned period_count{ static_cast<unsigned>( std::count( in_string.cbegin(), in_string.cend(), '.' ) ) };

  // Valid input: 1 slash 0 periods, 0 slashes 1 period, 0 slashes 0 periods
  if( slash_count > 2 ) { return false; }
  if( period_count > 2 ) { return false; }
  if( slash_count == 1 && period_count != 0 ) { return false; }
  if( period_count == 1 && slash_count != 0 ) { return false; }

  // TODO: Rename right_substring to mantissa_string and left_substring to characteristic_string
  if( slash_count != 1 )
  {
    // Strip whitespace and split at the period
    std::string left_substring;
    std::string right_substring;
    StringUtilities::splitAtLastCharacterOccurence( StringUtilities::removeWhiteSpace( in_string ), left_substring, right_substring, '.' );
    // Trailing zeros in the mantissa are meaningless, strip them to save computation later
    right_substring = StringUtilities::trimCharacterRight( right_substring, '0' );
    // User might have provided a number of the form .### or + .### or - .###
    if( left_substring.empty() ) { left_substring = "0"; }
    if( left_substring == "+" ) { left_substring = "+0"; }
    if( left_substring == "-" ) { left_substring = "-0"; }
    // Load the left and right portions into integers
    T left_int;
    if( !StringUtilities::extractFromString( left_substring, left_int ) ) { return false; }
    T right_int = 0;
    if( !right_substring.empty() && !StringUtilities::extractFromString( right_substring, right_int ) ) { return false; }
    // Compute the number of digits in the right substring
    const std::string::size_type num_digits{ right_substring.length() };
    // The denominator will be the power of ten needed to shift the decimal place all the way to the right
    T denominator = 1;
    for( std::string::size_type i = 0; i < num_digits; ++i ) { denominator *= 10; }
    // Compute the denominator
    const T numerator{ left_int * denominator + ( left_int < 0 ? -1 : 1 ) * right_int };
    output.set( numerator, denominator );
  }
  else
  {
    // Strip whitespace and split at the period
    std::string left_substring;
    std::string right_substring;
    StringUtilities::splitAtLastCharacterOccurence( StringUtilities::removeWhiteSpace( in_string ), left_substring, right_substring, '/' );
    // Load the left and right portions into integers
    T left_int;
    if( !StringUtilities::extractFromString( left_substring, left_int ) ) { return false; }
    T right_int;
    if( !StringUtilities::extractFromString( right_substring, right_int ) ) { return false; }
    // Left part is the numerator, right part is the denominator
    output.set( left_int, right_int );
  }

  return true;
}

#endif

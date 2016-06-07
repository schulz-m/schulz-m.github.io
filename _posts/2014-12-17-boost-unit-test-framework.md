---
layout: post
title: "Boost Unit Test framework"
tags:
- Boost
- Cpp
---

## Sources

These are notes based upon the Boost Unit Test Framework (UTF) [documentation](http://www.boost.org/doc/libs/1_55_0/libs/test/doc/html/index.html).

## What a test should do

Test cases are chosen to expose as many errors as possible. A defining characteristic of a good test case is that the programmer knows what the unit should do if it is functioning properly. Test cases should be generated to exercise each available execution path. Every path should be tested, since every path is a possible outcome of program execution.

To write a good testing suite the tester must know the structure of the code. The most dependable way to accomplish this is if the original programmer writes tests as part of creating the code.

## Different testing modes

```cpp
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>

int add( int i, int j ) { return i+j; }

BOOST_AUTO_TEST_CASE( my_test ) {

    // seven ways to detect and report the same error:
    BOOST_CHECK( add( 2,2 ) == 4 );        // #1 continues on error

    BOOST_REQUIRE( add( 2,2 ) == 4 );      // #2 throws on error

    if( add( 2,2 ) != 4 )
      BOOST_ERROR( "Ouch..." );            // #3 continues on error

    if( add( 2,2 ) != 4 )
      BOOST_FAIL( "Ouch..." );             // #4 throws on error

    if( add( 2,2 ) != 4 ) throw "Ouch..."; // #5 throws on error

    BOOST_CHECK_MESSAGE( add( 2,2 ) == 4,  // #6 continues on error
                         "add(..) result: " << add( 2,2 ) );

    BOOST_CHECK_EQUAL( add( 2,2 ), 4 );	  // #7 continues on error
}

```

## Example of class and test developement

While developing a new unit of code that encapsulates a concept, e.g. a `class const_string`, start with an empty unit test module.

```cpp
#define BOOST_TEST_MODULE const_string test
#include <boost/test/unit_test.hpp>

// EOF
```

Compile and run and this should return:

```raw
$ No errors detected
```

Now define the interface of `class const_string`. For example it could be (const_string.hpp):

```cpp
class const_string {
public:
    // Constructors
    const_string();
    const_string( std::string const& s )
    const_string( char const* s );
    const_string( char const* s, size_t length );
    const_string( char const* begin, char const* end );

    // Access methods
    char const* data() const;
    size_t      length() const;
    bool        is_empty() const;
    ...
};
```

After implementing this interface a unit test can be written like the following:

```cpp
#define BOOST_TEST_MODULE const_string test
#include <boost/test/unit_test.hpp>

BOOST_AUTO_EST_CASE( constructors_test )
{
	 // Constructor testing
     const_string cs0( "" );                                                 // 1 //
     BOOST_CHECK_EQUAL( cs0.length(), (size_t)0 );
     BOOST_CHECK( cs0.is_empty() );

     const_string cs01( NULL );                                              // 2 //
     BOOST_CHECK_EQUAL( cs01.length(), (size_t)0 );
     BOOST_CHECK( cs01.is_empty() );

     const_string cs1( "test_string" );                                      // 3 //
     BOOST_CHECK_EQUAL( std::strcmp( cs1.data(), "test_string" ), 0 );
     BOOST_CHECK_EQUAL( cs1.length(), std::strlen("test_string") );

     std::string s( "test_string" );                                         // 4 //
     const_string cs2( s );
     BOOST_CHECK_EQUAL( std::strcmp( cs2.data(), "test_string" ), 0 );

     const_string cs3( cs1 );                                                // 5 //
     BOOST_CHECK_EQUAL( std::strcmp( cs1.data(), "test_string" ), 0 );

     const_string cs4( "test_string", 4 );                                   // 6 //
     BOOST_CHECK_EQUAL( std::strncmp( cs4.data(), "test", cs4.length() ), 0 );

     const_string cs5( s.data(), s.data() + s.length() );                    // 7 //
     BOOST_CHECK_EQUAL( std::strncmp( cs5.data(), "test_string", cs5.length() ), 0 );

     const_string cs_array[] = { "str1", "str2" };                           // 8 //
     BOOST_CHECK_EQUAL( cs_array[0], "str1" );
     BOOST_CHECK_EQUAL( cs_array[1], "str2" );
}

// EOF
```

When this test returns no errors one can move to define interfaces for a new feature and implement it. When implmented its unit test can be programmed in order to test the functionality.

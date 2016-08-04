---
layout: post
title: "Short Introduction into Catkin Packages"
tags:
- C++
- Catkin
- ROS
---

## Introduction

The basic structure of a catkin package is very simple, there are two configuration files: the first one `package.xml` is to describe the meta-information about the dependencies [(wiki)](http://wiki.ros.org/catkin/package.xml), the second one `CMakeLists.txt` is the usual configuration file for the CMake build system [(wiki)](http://wiki.ros.org/catkin/CMakeLists.txt). Despite the supposingly clear XML and CMake commands I have found most of them to be rather confusing for regular day to day package building in C++, so this post tries to clear some confusion with these configurations in the context of building in a devel space.

## Example Packages

We will consider three example packages for this post:

* `header_only_package`: Where nothing gets compiled and solely provides headers
* `normal_package`: Simply uses standard components to build a node
* `combined_package`: Builds a node that is dependent on the other two packages

A possible file structure could be as follows:

```
├── catkin_ws/src
│   ├── header_only_package
│   │   ├── include
│   │   │   ├── header_only_package
│   │   │   │   ├── template.hpp
│   │   ├── package.xml
│   │   ├── CMakeLists.xml
│   ├── normal_package
│   │   ├── include
│   │   │   ├── normal_package
│   │   │   │   ├── node.hpp
│   │   ├── src
│   │   │   │   ├── node.cpp
│   │   ├── package.xml
│   │   ├── CMakeLists.xml
│   ├── combined_package
│   │   ├── include
│   │   │   ├── combined_package
│   │   │   │   ├── node.hpp
│   │   ├── src
│   │   │   │   ├── node.cpp
│   │   ├── package.xml
│   │   ├── CMakeLists.xml
```

## Package Configurations

As recommended in the ROS wiki you should use the [package format two](http://www.ros.org/reps/rep-0140.html) as it provides much more consize and clear description tags. For these examples I will miss out the enclosing `<package format="2">` and not dependency-related tags as e.g. `<version>`, `<description>` or `<license>`.

The dependency tags that are explained and used are the following:

* `<buildtool_depend>` Specify a package that provides tools (e.g. python scripts) to build components within the package that uses the tool.
* `<build_depend>` Specify a package that is needed to build this package, e.g. either by headers or pre-compiled libraries.
* `<build_export_depend>` Specify a package that will be needed by another package if it wants to build this package, this is easiest explained e.g. with a header only package.
* `<exec_depend>` Specify a package that you need in order to run the package, this can be the case if you have it specified in your launch files or if you depend on shared libraries provied by this package.
* `<depend>` Catch it all tag that specifies packages that are build, build export and execution dependencies.

### Header Only

This package is only dependent on standard ROS components and is not building anything.

```xml
<name>header_only_package</name>

<build_export_depend>roscpp</build_export_depend>
```

EXPLANATION

### Normal

```xml
<name>normal_package</name>

<buildtool_depend>catkin</buildtool_depend>

<depend>roscpp</depend>
```

EXPLANATION

### Combined

```xml
<name>combined_package</name>

<buildtool_depend>catkin</buildtool_depend>

<build_depend>header_only_package</build_depend>

<depend>roscpp</depend>
<depend>normal_package</depend>
```

EXPLANATION

## CMake Configurations

GENERAL INTRO

Out of the scope of this post is `add_dependencies`, LOOK UP DOC and explain properly.

### Header Only

```CMake
project(header_only_package)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
)
```

EXPLANATION

### Normal

```CMake
project(normal_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    normal_package
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(normal_package
  src/node.cpp
)

target_link_libraries(normal_package
  ${catkin_LIBRARIES}
)
```

EXPLANATION

### Combined

```CMake
project(combined_package)

find_package(catkin REQUIRED COMPONENTS
  header_only_package
  roscpp
  normal_package
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    normal_package
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

EXPLANATION



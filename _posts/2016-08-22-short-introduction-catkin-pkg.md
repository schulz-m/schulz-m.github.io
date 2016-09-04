---
layout: post
comments: true
title: "Short Introduction into Catkin Packages"
tags:
- C++
- Catkin
- ROS
---

## Introduction

The basic structure of a catkin package is very simple, there are two configuration files: the first one, `package.xml`, is to describe the meta-information about the dependencies [(wiki)](http://wiki.ros.org/catkin/package.xml), the second one `CMakeLists.txt` is the usual configuration file for the CMake build system [(wiki)](http://wiki.ros.org/catkin/CMakeLists.txt). Despite the supposingly clear XML and CMake commands I have found some of them to be rather confusing for regular day to day package building in C++, so this post tries to clear some confusion with these configurations in the context of building in a devel space.

## Example Packages

We will consider three example packages for this post:

* `header_only_package`: Nothing gets compiled and solely provides headers
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

As recommended in the ROS wiki you should use the [package format two](http://www.ros.org/reps/rep-0140.html) as it provides much more consize and clear description tags. For these examples I will not include the enclosing `<package format="2">` that defines this usage. Other not dependency-related tags as e.g. `<version>`, `<description>` or `<license>` are also ommited.

The dependency tags that are explained and used are the following:

* `<buildtool_depend>` Specify a package that provides tools (e.g. CMake commands) to build components within the package that uses the tool.
* `<build_depend>` Specify a package that is needed to build this package, e.g. either by headers or pre-compiled libraries.
* `<build_export_depend>` Specify a package that will be needed by another package if it wants to build this package, this is easiest explained e.g. with a header only package.
* `<exec_depend>` Specify a package that you need in order to run the package, this can be the case e.g. if you only use it in your launch files or if you depend on shared libraries provided by this package.
* `<depend>` Catch it all tag that specifies packages that are build, build export and execution dependencies.

### Header Only

This package is only dependent on standard ROS components and is not building anything.

```xml
<name>header_only_package</name>

<buildtool_depend>catkin</buildtool_depend>

<build_export_depend>roscpp</build_export_depend>
```

As you will see, we pretty much always want to have a buildtool dependency on catkin, as it provides the necessary CMake command `catkin_package` explained below to actually define the catkin package. For this example, the package only consists of a template that needs the package `roscpp` and nothing is built, such that `roscpp` is a build dependency for all packages that want to use the `header_only_package`.

### Normal

This package simply consists of a node that is independent of the other packages, but uses `roscpp`.

```xml
<name>normal_package</name>

<buildtool_depend>catkin</buildtool_depend>

<depend>roscpp</depend>
```

For the simple standard case, you will simply have a `<depend>` tag for the package `roscpp` as it provides headers and shared libraries that you need both at compile time and run time for a "standard node".

### Combined

This package uses both the `header_only_package` and the `normal_package` to be built and run.

```xml
<name>combined_package</name>

<buildtool_depend>catkin</buildtool_depend>

<build_depend>header_only_package</build_depend>

<depend>roscpp</depend>
<depend>normal_package</depend>
```

Both `roscpp` and `normal_package` provide headers and shared libraries that we use in this package, so both need a `<depend>` tag, as will also be clearer once you look at the CMake files. For the `header_only_package` we only have a build dependency because the inclusion of the headers is only needed at built time. Note that if another package would built the `combined_package`, the `header_only_package` would also be a build export dependency.

## CMake Configurations

These configurations define the necessary macros, dependencies etc. for the CMake build system. Apart from the self-explanatory command `project()`, the following commands are used in this post:

* `find_package` Finds other packages to build this package, basically listing all your `build` and `buildtoool` dependencies. You pretty much always want to have a dependency on `catkin` which also defines CMake variables such as `${catkin_INCLUDE_DIRS}`.
* `catkin_package` Defines the catkin package as such and is used by the catkin build system to generate the necessary configurations.
* `include_directories` Packages to include, for most cases this will simply need the `${catkin_INCLUDE_DIRS}` variable and any local includes, as all the catkin packages are concatenated in this variable. 
* `add_executable` Defines an executable to be built.
* `add_library` Defines a library (shared by default) built in this package, pretty straightforward.
* `target_link_library` Link the specified library or executable against other libraries, usually you will simply need `${catkin_LIBRARIES}` as again, all shared libraries within your catkin workspace are concatenated in this variable.

### Header Only

```CMake
project(header_only_package)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
)
```

As this package is not building anything, there is no package to be found except `catkin` for the `catkin_package` CMake macro. For everyone using this package we define the `roscpp` dependency such that it's built before someone tries to build this package - this is exactly what a `build export` dependency is.

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

Because we build a library in this package, we have to `find_package` the package `roscpp`, which is also defined at the `CATKIN_DEPENDS` tag, because obviously `roscpp` needs to be built in order to build this package as well. Note that this would not be the case if it were for example a header only package, so to be very precise, you need to know what exactly you are using from the package (is there an automatic way to do this?). In addition, in this package we define the library `normal_package` and therefore define the include directory, source file and libraries to link to (in this case only libraries from catkin).

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

add_executable(combined_package_node src/node.cpp)

target_link_libraries(combined_package_node
  ${catkin_LIBRARIES}
)
```

For the combined package we are not building any library here, but only the executable `combined_package_node` that also has to be linked against `${catkin_LIBRARIES}` to be able to actually use the library provided by `normal_package`. Note that we only need to `find_package()` the `header_only_package` as it doesn't need to be built before building this package - after all we are only including it's headers so simply need the path!



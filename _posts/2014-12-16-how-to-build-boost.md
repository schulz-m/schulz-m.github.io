---
layout: post
title: "How to build Boost"
tags:
- Boost
- Cpp
---


The version of boost at the time of writing is 1.57.0. This document is based upon the getting started Boost [documentation](http://www.boost.org/doc/libs/1_57_0/more/getting_started/) and the help dialog of the `b2` tool.

This document aims to provide a quick reference to allow to build boost.

## Get the source

Download the boost source code form http://www.boost.org/users/download/ or http://sourceforge.net/projects/boost/files/boost/1.57.0/ in your favorite archive format.

Unpack the source code in any folder which will be referred to as `<boost-root-dir>` for the remainder of this text.

## Build boost

Building boost is a two step process:
	1. Build the Boost.Build build engine
	2. Compile the boost libraries themselves

### Build Boost.Build engine

This step is automated through a `.bat` script under windows and a `.sh` script under *NIX. It's preferable to open the command line and navigate to `<boost-root-dir>` through the command line and then execute the script for the current operating system. This is to be able to see the output of the script in case of error.

Remember to supply the compiler as an argument.

Windows
```
$ cd <boost-root-dir>
$ bootstrap.bat gcc
```

*NIX
```
$ cd <boost-root-dir>
$ ./bootstrap.sh gcc
```

If the script succeeded there should be an executable called `b2` in `<boost-root-dir>`.

### Build the boost libraries (static and dynamic linking)

The `b2` executable created in the previous step takes care of building the boost libraries. This tool requires some arguments to achieve the expected result.

Note that invoking `b2` without any arguments under windows defaults to building boost for the msvc compiler.

There are two variants for the boost libraries: static and dynamic. These properties refer to the linking stage of compilation. The two different version can be compiled by using the following commands while in the `<boost-root-dir>` directory.

Static linking (Windows) : `$ b2 toolset=gcc --build-type=complete`
Dynamic linking (Windows) : `$ b2  toolset=gcc --build-type=complete link=shared`

Static linking (NIX) : `$ ./b2 toolset=gcc --build-type=complete`
Dynamic linking (NIX) : `$ ./b2  toolset=gcc --build-type=complete link=shared`


### Add environement variables

Other programs might need to know where the Boost root is for linking against it or to use the shared DLLs.

Windows
```
$ setx BOOST_ROOT <boost-root-dir>
```
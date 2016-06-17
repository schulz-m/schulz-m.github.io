---
layout: post
title: "Sublime with Catkin"
tags:
- Sublime
- Cpp
- Catkin
- ROS
---

## Introduction

This post is about the usage of sublime with the catkin infrastructure used by ROS. This was tested with **Ubuntu 14.04 64** and *ROS Indigo*. The post assumes that you are familiar with both *ROS* and *catkin* and that you have a catkin workspace working with the usual folder structure in `~/catkin_ws`.

## Sublime Installation

For those of you who have never heard of sublime, here a quick and dirty installation guide to have the setup that I am working with. First install sublime 3:

```raw
sudo add-apt-repository ppa:webupd8team/sublime-text-3
sudo apt-get update
sudo apt-get install sublime-text-installer
```

Everything gets installed for you, such that you can open sublime with a simple `subl` command in the terminal. For all real cool features of the program you need to install the so called [package_control](https://packagecontrol.io/installation).

Some useful starting points:

* For some of the common commands, consult the [cheatsheet](http://csnipp.com/s/66/-List-of-commonly-used-sublime-text-commands)
* In general, to edit your preferences (such as tabbing), simply open `Preferences -> Settings - User` and overwrite settings you find in `Preferences -> Settings - Default`
* If you are not satisfied with the syntax detected by sublime you can simply change it with `View -> Syntax -> Open all with current extension as... ->[your syntax choice]`

Some other neat extensions:

* [Markup editing with sublime](http://plaintext-productivity.net/2-04-how-to-set-up-sublime-text-for-markdown-editing.html)

## Set up the catkin project



## Example CPP Code

```cpp
#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>

int add( int i, int j ) { return i+j; }
}
```


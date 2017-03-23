---
layout: post
comments: true
title: "Sublime with Catkin"
tags:
- Sublime
- C++
- Catkin
- ROS
---

## Introduction

This post is about the usage of sublime with the catkin infrastructure used by ROS. This was tested with **Ubuntu 14.04 LTS** and **ROS Indigo**. The post assumes that you are familiar with both **ROS** and **catkin** and that you have a catkin workspace working with the usual folder structure in `~/catkin_ws`. Another nice introduction into using sublime as your IDE can be found at [Linux Sublime Dev](https://chromium.googlesource.com/chromium/src/+/master/docs/linux_sublime_dev.md), this post aims to provide a simple overview and setup for **catkin** specifically.

<!--more-->

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
* [Sublime Linter for Code Style](http://www.sublimelinter.com/en/latest/)
* [Increment Selection](https://github.com/yulanggong/IncrementSelection)

Some of the useful key-bindings to show the power of sublime:

| TEXT EDITING  | |
| :------------- |-------------:|
| `ctrl + d` | multi-select occurrences of pattern |
| `ctrl + click` | multi-cursor selection |

| WINDOW NAVIGATION | |
| :------------- |-------------:|
| `shift + alt + num` | number of split windows in editor |
| `ctrl + page up/down` | to alter between file tabs |

| FILE NAVIGATION |  |
| :------------- |-------------:|
| `ctrl + p` | search box to look for files inside the opened folder |
| `ctrl + o` | to use the OS navigation to open another file in sublime |
| `alt + o` | to switch between source and header files |
| `ctrl + shift + f` | find all occurrences within a certain folder |

## Set up the catkin project

### Create project files

Open sublime and then simply add your `catkin_ws/src` folder by using `Project -> Add Folder to Project`, then save it as a project file in a location of your choice by `Project -> Save Project as...`.

Now when you open sublime, by default you should have your catkin workspace in the file navigator.

### Build tags to jump to definitions

As a next step, you want to use CTags to jump through definitions in your source code. The source of the CTags Plugin is found [here](https://github.com/SublimeText/CTags). You can simply install it by using the package manager. As a refresher: Type `Alt + Shift + P` and type install. Type `ctags` and put enter. For ctags to work properly, ctags also has to be installed on the OS via `sudo apt-get install exuberant-ctags`.

To build the indices just right click on your project folder and click `CTags: Rebuild Tags` in the context menu. In the context menu, a bottom `Navigate to Definition` should now have appeared.

### Build catkin packages from sublime

It is very straightforward to build catkin packages using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/) within sublime. Simply install the [catkin builder](https://packagecontrol.io/packages/Catkin%20Builder) using the package manager. You can then either run it with `ctrl + shift + p` and then `Build with: Catkin` or using `Tools -> Build`.

> Note: If you have your sources symbolically linked into your workspace this package won't work, I should make an issue about this at some point.

### ROS Msg File Syntax

Download the syntax file from [Github](https://gist.github.com/eric-wieser/1cd2919483d18d6b3788a54dec4f165c) and copy the file into `~.config/sublime-text-3/Packages/User`. Now you can simply select *ROS message definition* with `View -> Syntax -> Open all with current extension as...`.
